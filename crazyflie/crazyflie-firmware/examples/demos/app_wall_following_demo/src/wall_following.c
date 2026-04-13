/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * wall_follower.c - App layer application of the wall following demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 *
 * The same wallfollowing strategy was used in the following paper:

 @article{mcguire2019minimal,
  title={Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment},
  author={McGuire, KN and De Wagter, Christophe and Tuyls, Karl and Kappen, HJ and de Croon, Guido CHE},
  journal={Science Robotics},
  volume={4},
  number={35},
  year={2019},
  publisher={Science Robotics}
}
Modified by: Bock Kai Sheng, Hu Linxi, Zhang Zehua (NUS Electrical Engineering students)
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include <math.h>
#include "usec_time.h"

#include "wallfollowing_multiranger_onboard.h"
#include "esp_uart_bridge.h"

#define DEBUG_MODULE "WALLFOLLOWING"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

// ===================== Parameters =====================

static const float spHeight = 0.5f;
// static const float spHeight = 0.8f;
static const uint16_t radius = 300;

static const float REACQUIRE_TIMEOUT = 10.0f;  // Give up searching for wall, go scan
static const float WALLFOLLOW_TIME = 10.0f;  // Cruise along wall, then go scan (changed from 5.0f)
static const float SCAN_TIME = 10.0f;  // changed from 5.0f
static const float TRANSITION_TIME = 0.35f;

// Human detection parameter(s)
static const uint32_t HUMAN_PACKET_STALE_TIMEOUT_MS = 300U;  // Defines how long (in milliseconds) before a received ESP32 packet is considered outdated.

static const float HUMAN_CONFIRM_TIME = 3.0f;  // Param/Variable for Dashboard Reading

// static const float BATTERY_CUTOFF = 2.8f;
static const float BATTERY_CUTOFF = 2.5f;

// Push-style avoidance parameters
// static const float AVOID_RADIUS = 0.4f;   // meters
// static const float AVOID_RADIUS = 0.3f;
static const float AVOID_RADIUS = 0.5f;
// static const float AVOID_VEL_MAX = 0.1f;  // max avoidance velocity
static const float AVOID_VEL_MAX = 0.3f;  // max avoidance velocity

static const float VEL_CLAMP = 0.3f;

// ===================== Wall follow parameters =====================

bool goLeft = false;
// bool goLeft = true;
float distanceToWall = 0.5f;  // Target distance that the Crazyflie tries to maintain from the wall while following it (default is 0.5).
// float distanceToWall = 0.8f;
// float maxForwardSpeed = 0.25f;
float maxForwardSpeed = 0.2f;

// ===================== Python control =====================

uint8_t appActive = 0;

// ===================== Thermal perception =====================

static uint8_t humanDetected = 0U;
static uint8_t humanConfidence = 0U;
static int8_t humanDir = 0;
static uint8_t humanPacketFresh = 0U;
static uint32_t humanAgeMs = 0U;
static int16_t humanMaxTempX100 = 0;
static int16_t humanThermistorX100 = 0;
// static float humanHoldTimeS = 0.0f;
// static uint8_t humanStable = 0U;

static float humanHoldTimeS = 0.0f;  // Param/Variable for Dashboard Reading
static uint8_t humanStable = 0U;  // Param/Variable for Dashboard Reading
static float humanDetectStart = -1.0f;  // Param/Variable for Dashboard Reading
float humanStandOff = 1.0f;  // Desired distance that the drone should keep from a detected human (should NOT be too close for safety)

// ===================== Command variables =====================

float cmdVelX = 0.0f;
float cmdVelY = 0.0f;
float cmdYawRateDeg = 0.0f;
static float cmdHeight = 0.5f; 

static float scanHoldX = 0.0f;  // ADD: XY position locked at scan entry
static float scanHoldY = 0.0f;  // ADD
static float scanEntryYaw = 0.0f;

static uint8_t dashboardMissionState = 0U;  // Param/Variable for Dashboard Reading

// ===================== ESP32 Communication =====================

// ---------- Bridging ---------

// EspUartLogIds struct
// typedef struct {
//   logVarId_t detected;
//   logVarId_t confidence;
//   logVarId_t direction;
//   logVarId_t maxTemp;
//   logVarId_t therm;
//   logVarId_t rxLocalMs;
// } EspUartLogIds;

// ===================== Outer loop =====================

typedef enum
{
  idle,
  unlocked,
  stopping
} StateOuterLoop;  // lowUnlock was removed because we don't want the Crazyflie to start up using the hand unlock anymore (for safety).

static StateOuterLoop stateOuterLoop = idle;

// ===================== Mission FSM =====================

typedef enum
{
  mission_reacquire_wall,
  mission_wallfollow,
  mission_scan,
  mission_approach,
  mission_bob,
  mission_land,
  // mission_track,
  mission_transition,
  mission_align
} MissionState;

static MissionState missionState = mission_reacquire_wall;  // what the drone is doing
static MissionState nextMissionState;  // what it will do next

static float missionStateStart = 0;  // when the current task started
// static float humanDetectStart = -1.0f;  // Related to human detection

// ===================== Wall follower =====================

static StateWF stateInnerLoop = forward;

// ===================== Helpers =====================

static void setVelocitySetpoint(setpoint_t *setpoint,
                                float vx, float vy,
                                float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;

  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

// Position hold + yaw rate setpoint — used during mission_scan
static void setScanSetpoint(setpoint_t *setpoint,
                            float x, float y,
                            float z, float yawrate)
{
  setpoint->mode.z   = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x   = modeAbs;   // hold X position (world frame)
  setpoint->mode.y   = modeAbs;   // hold Y position (world frame)
  setpoint->position.x = x;
  setpoint->position.y = y;

  setpoint->velocity_body = false;  // position is always world frame
}

static void missionTransition(MissionState next, float now)
{
  nextMissionState = next;
  missionState = mission_transition;
  missionStateStart = now;
}

static void resetWallFollower()
{
  stateInnerLoop = forward;
  wallFollowerInit(distanceToWall, maxForwardSpeed, stateInnerLoop);
}

// Checks if all the required ESP32 log IDs have been successfully registered in the system.
// Prevents the drone from reading garbage memory before the ESP32 bridge is fully initialised.

// static bool espUartLogIdsReady(const EspUartLogIds *ids)
// {
//   return logVarIdIsValid(ids->detected) &&
//          logVarIdIsValid(ids->confidence) &&
//          logVarIdIsValid(ids->direction) &&
//          logVarIdIsValid(ids->maxTemp) &&
//          logVarIdIsValid(ids->therm) &&
//          logVarIdIsValid(ids->rxLocalMs);
// }

static uint8_t getDashboardMissionState(MissionState state)
{
  switch(state)
  {
  case mission_reacquire_wall:
    return 0U;
  case mission_wallfollow:
    return 1U;
  case mission_scan:
    return 2U;
  case mission_approach:
    return 3U;
  case mission_bob:
    return 4U;
  case mission_land:
    return 5U;
  case mission_transition:
    return 6U;
  case mission_align:
    return 7U;
  }

  return 0U;
}

// Called every loop tick ('while' loop)
// Reads the ESP32 log variables, calculates how old the data is ( humanAgeMs ),
// checks if it exceeds the 300 ms staleness window, and updates the global perception variables accordingly.

static void updateHumanPerception(uint32_t nowMs)
{
  espUartHumanDetection_t sample = {0};

  humanDetected = 0U;
  humanConfidence = 0U;
  humanDir = 0;
  humanPacketFresh = 0U;
  humanAgeMs = 0U;
  humanMaxTempX100 = 0;
  humanThermistorX100 = 0;
  humanHoldTimeS = 0.0f;
  humanStable = 0U;

  if (!espUartBridgeGetLatestHumanDetectionSample(&sample)) {
    humanDetectStart = -1.0f;
    return;
  }

  humanMaxTempX100 = sample.maxTempX100;
  humanThermistorX100 = sample.thermistorX100;

  if (sample.rxLocalTimeMs == 0U) {
    humanDetectStart = -1.0f;
    return;
  }

  humanAgeMs = nowMs - sample.rxLocalTimeMs;

  if (humanAgeMs > HUMAN_PACKET_STALE_TIMEOUT_MS) {
    humanDetectStart = -1.0f;
    return;
  }

  humanPacketFresh = 1U;
  humanConfidence = sample.confidence;
  humanDir = sample.direction;
  if (sample.detected) {
    humanDetected = 1U;
    if (humanDetectStart < 0.0f) {
      humanDetectStart = (float)nowMs / 1000.0f;
    }
    humanHoldTimeS = ((float)nowMs / 1000.0f) - humanDetectStart;
    if (humanHoldTimeS >= HUMAN_CONFIRM_TIME) {
      humanStable = 1U;
    }
  } else {
    humanDetectStart = -1.0f;
  }
}

// ===================== Main =====================

void appMain()
{
  vTaskDelay(M2T(3000));
  // Getting Logging IDs of the multiranger
  // logVarId_t idUp = logGetVarId("range","up");
  logVarId_t idLeft = logGetVarId("range","left");
  logVarId_t idRight = logGetVarId("range","right");
  logVarId_t idFront = logGetVarId("range","front");
  logVarId_t idBack  = logGetVarId("range","back");

  // Getting the Logging IDs of the state estimates
  logVarId_t idYaw = logGetVarId("stabilizer","yaw");
  logVarId_t idHeight = logGetVarId("stateEstimate","z");

  logVarId_t idVbat = logGetVarId("pm","vbat");

  logVarId_t idX = logGetVarId("stateEstimate","x");
  logVarId_t idY = logGetVarId("stateEstimate","y");

  // EspUartLogIds espUartIds = {
  //   .detected = logGetVarId("espUart", "detected"),
  //   .confidence = logGetVarId("espUart", "confidence"),
  //   .direction = logGetVarId("espUart", "direction"),
  //   .maxTemp = logGetVarId("espUart", "maxTemp"),
  //   .therm = logGetVarId("espUart", "therm"),
  //   .rxLocalMs = logGetVarId("espUart", "rxLocalMs"),
  // };

  // Getting Param IDs of the deck driver initialization
  paramVarId_t idPositioningDeck = paramGetVarId("deck","bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck","bcMultiranger");

  // Intialize the setpoint structure
  setpoint_t setpoint;

  resetWallFollower();  // Initialize the wall follower state machine

  DEBUG_PRINT("Waiting for activation. Press 'Enter' to start.\n");
  while(1)
  {
    vTaskDelay(M2T(10));
    // Check if decks are properly mounted
    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    // // Get the upper range sensor value (used for stopping)
    // uint16_t up = logGetUint(idUp);

    // Get Height estimate
    float heightEstimate = logGetFloat(idHeight);

    float timeNow = usecTimestamp()/1e6f;
    
    const uint32_t nowMs = T2M(xTaskGetTickCount());
    updateHumanPerception(nowMs);

    // Battery check
    float vbat = logGetFloat(idVbat);
    // if(vbat < BATTERY_CUTOFF && vbat > 0.1f && stateOuterLoop == unlocked)
    // {
    //   missionTransition(mission_land, timeNow);
    //   // stateOuterLoop = stopping;
    // }

    // ===================== OUTER LOOP =====================

    // -------- IDLE STATE --------
    if(stateOuterLoop == idle)
    {
      // Start only by Python
      if(appActive && positioningInit && multirangerInit)
      {
        stateOuterLoop = unlocked;
        cmdHeight = spHeight;  // Reset the height from the previous flight (coz cmdHeight is declared with a 'static')
        resetWallFollower();

        missionState = mission_reacquire_wall;
        missionStateStart = usecTimestamp()/1e6f;

        DEBUG_PRINT("Unlocked by Python script\n");
      }
    }


    // -------- NORMAL FLIGHT --------
    else if(stateOuterLoop == unlocked)
    {
      // Python stop
      if(!appActive && missionState != mission_land && missionState != mission_transition)
      {
        missionTransition(mission_land, timeNow);
        // stateOuterLoop = idle;
        DEBUG_PRINT("Stopped by Python script\n");
      }

      // // Emergency stop using hand above sensor
      // if(up < 200)
      // {
      //   stateOuterLoop = stopping;
      //   DEBUG_PRINT("Emergency stop (hand detected)\n");
      // }
    }


    // // -------- STOPPING STATE --------
    // else if(stateOuterLoop == stopping)
    // {
    //   // Wait until hand removed
    //   if(up > 500)
    //   {
    //     stateOuterLoop = idle;

    //     // Force Python restart
    //     appActive = 0;

    //     DEBUG_PRINT("Stopped\n");
    //   }
    // }


    // -------- DISARM MOTORS IF NOT UNLOCKED --------

    dashboardMissionState = (stateOuterLoop == unlocked) ? getDashboardMissionState(missionState) : 0U;  // Param/Variable for Dashboard Reading

    if(stateOuterLoop != unlocked)
    {
      memset(&setpoint,0,sizeof(setpoint));
      commanderSetSetpoint(&setpoint,3);
      continue;
    }

    // ===================== Sensor reads =====================

    float frontRange = logGetUint(idFront)/1000.0f;
    float leftRange  = logGetUint(idLeft)/1000.0f;
    float rightRange = logGetUint(idRight)/1000.0f;
    float backRange  = logGetUint(idBack)/1000.0f;
    // float upRange    = logGetUint(idUp)/1000.0f;

    // ADD: Clamp invalid sensor readings (VL53L1x returns 32766 mm on error)
    // static const float RANGE_MAX_VALID = 3.0f;                          // ADD
    // if (frontRange > RANGE_MAX_VALID) frontRange = RANGE_MAX_VALID;     // ADD
    // if (leftRange  > RANGE_MAX_VALID) leftRange  = RANGE_MAX_VALID;     // ADD
    // if (rightRange > RANGE_MAX_VALID) rightRange = RANGE_MAX_VALID;     // ADD
    // if (backRange  > RANGE_MAX_VALID) backRange  = RANGE_MAX_VALID;     // ADD

    float sideRange;

    if(goLeft)
      sideRange = logGetUint(idRight)/1000.0f;
    else
      sideRange = logGetUint(idLeft)/1000.0f;

    // if (sideRange > RANGE_MAX_VALID) sideRange = RANGE_MAX_VALID;       // ADD
    
    float yawDeg = logGetFloat(idYaw);
    float yawRad = yawDeg * (float)M_PI / 180.0f;

    // float timeNow = usecTimestamp()/1e6f;

    // //Adjust height based on up ranger input
    // uint16_t up_o = radius - MIN(up,radius);
    // float cmdHeight = spHeight - up_o/1000.0f;
    // float cmdHeight = spHeight;

    // if (heightEstimate > 1.5f)
    // {
    //     cmdHeight = spHeight;   // gently correct
    // }
    // if(cmdHeight < spHeight - 0.2f)
    // {
    //   stateOuterLoop = stopping;
    //   continue;
    // }

    cmdVelX = 0.0f;
    cmdVelY = 0.0f;
    cmdYawRateDeg = 0.0f;

    int wallDirection = goLeft ? 1 : -1;

    if (vbat < BATTERY_CUTOFF && vbat > 0.1f && stateOuterLoop == unlocked
        && missionState != mission_land && missionState != mission_transition)
    {
      missionTransition(mission_land, timeNow);
      // stateOuterLoop = stopping;
    }
    
    // ===================== Mission FSM =====================
    // Handle landing outside the height gate so it can complete the descent
    if (missionState == mission_land)
    {
      cmdVelX = 0.0f;
      cmdVelY = 0.0f;
      cmdYawRateDeg = 0.0f;
      cmdHeight -= 0.0025f;
      if (cmdHeight < 0.0f) cmdHeight = 0.0f;

      if (cmdHeight == 0.0f)  // previously 0.10f and 0.05f worked well, but there was still a bit of height which makes the landing not so graceful.
      {
        memset(&setpoint, 0, sizeof(setpoint));
        commanderSetSetpoint(&setpoint, 3);
        stateOuterLoop = idle;
        appActive = 0;
        continue;
      }
    }
    else if (missionState == mission_bob)
    {
      // // Might need to comment out this humanDetected check here coz it may lose track of human during bobbing.
      // if (!humanDetected)
      // {
      //   missionTransition(mission_reacquire_wall, timeNow);
      //   break;
      // }

      // cmdYawRateDeg = 5.0 * humanDir;

      float t = timeNow - missionStateStart;

      // Cycle 1
      if (t < 1.0f) cmdHeight = spHeight + 0.3f;  
      else if (t < 2.0f) cmdHeight = spHeight;  // return to 0.5m
      // Cycle 2
      else if (t < 3.0f) cmdHeight = spHeight + 0.3f;
      else if (t < 4.0f) cmdHeight = spHeight;  // return to 0.5m
      // Done
      else
      {
        cmdHeight = spHeight;
        missionTransition(mission_land, timeNow);
      }
    }
    else if (heightEstimate > spHeight - 0.1f)
    {
      switch(missionState)
      {

      case mission_transition:  // To let the drone hover in between states
      {
        cmdVelX = 0.0f;
        cmdVelY = 0.0f;
        cmdYawRateDeg = 0.0f;
        cmdHeight = spHeight;
        
        if(timeNow - missionStateStart > TRANSITION_TIME)
        {
          if(nextMissionState == mission_reacquire_wall)
            resetWallFollower();

          missionState = nextMissionState;
          missionStateStart = timeNow;
        }
      }
      break;

      // ===================== REACQUIRE WALL =====================
      case mission_reacquire_wall:
      {
        float yawRadCmd;
        cmdHeight = spHeight;

        stateInnerLoop = wallFollower(&cmdVelX, &cmdVelY, &yawRadCmd,
                                      frontRange, sideRange, yawRad,
                                      wallDirection, timeNow);

        cmdYawRateDeg = yawRadCmd * 180.0f / (float)M_PI;

        if (humanDetected)
        {
          missionTransition(mission_approach, timeNow);
        }
        else if (stateInnerLoop == forwardAlongWall)
        {
          missionTransition(mission_wallfollow, timeNow);
        }
        else if (timeNow - missionStateStart > REACQUIRE_TIMEOUT)
        {
          scanHoldX = logGetFloat(idX);   // capture before transitioning
          scanHoldY = logGetFloat(idY);
          scanEntryYaw = logGetFloat(idYaw);
          missionTransition(mission_scan, timeNow);
        }
      }
      break;
      // case mission_reacquire_wall:
      // {
      //   // // If wall already beside us, resume wall following
      //   // if(sideRange < distanceToWall + 0.3f)
      //   // {
      //   //   missionTransition(mission_wallfollow,timeNow);
      //   //   break;
      //   // }

      //   float yawRadCmd;

      //   stateInnerLoop = wallFollower(&cmdVelX,&cmdVelY,&yawRadCmd,
      //                                 frontRange,sideRange,yawRad,
      //                                 wallDirection,timeNow);

      //   cmdYawRateDeg = yawRadCmd * 180.0f/(float)M_PI;

      //   if(stateInnerLoop == forwardAlongWall)  // Wall follower has successfully locked onto a wall
      //   {
      //     missionTransition(mission_wallfollow,timeNow);
      //   }
      // }
      // break;

      // ===================== WALL FOLLOW =====================
      case mission_wallfollow:
      {
        float yawRadCmd;
        cmdHeight = spHeight;

        stateInnerLoop = wallFollower(&cmdVelX,&cmdVelY,&yawRadCmd,
                                      frontRange,sideRange,yawRad,
                                      wallDirection,timeNow);

        cmdYawRateDeg = yawRadCmd * 180.0f/(float)M_PI;

        if (humanDetected)
        {
          missionTransition(mission_approach, timeNow);
        }
        else if (timeNow - missionStateStart > WALLFOLLOW_TIME)
        {
          scanHoldX = logGetFloat(idX);   // capture before transitioning
          scanHoldY = logGetFloat(idY);
          scanEntryYaw = logGetFloat(idYaw);
          missionTransition(mission_scan,timeNow);
        }
      }
      break;

      // // ===================== SCAN =====================
      case mission_scan:
      {
          cmdYawRateDeg = 36.0f;   // always spin
          cmdHeight = spHeight;
          // scanHoldX/Y captured before missionTransition() was called

          float minDist = MIN(
                              MIN(frontRange, backRange),
                              MIN(leftRange, rightRange)
                            );

          // ===== ESCAPE: nudge hold position away from wall if too close =====
          if (minDist < AVOID_RADIUS)
          {
              float factor = AVOID_VEL_MAX / AVOID_RADIUS;
              float f_o = MAX(0.0f, AVOID_RADIUS - frontRange);
              float b_o = MAX(0.0f, AVOID_RADIUS - backRange);
              float l_o = MAX(0.0f, AVOID_RADIUS - leftRange);
              float r_o = MAX(0.0f, AVOID_RADIUS - rightRange);

              // Compute body-frame escape velocity
              float escapeX = (b_o - f_o) * factor;
              float escapeY = (r_o - l_o) * factor;

              // Rotate to world frame and nudge the hold position (dt = 10ms loop rate)
              float cosYaw = cosf(yawRad);
              float sinYaw = sinf(yawRad);
              scanHoldX += (escapeX * cosYaw - escapeY * sinYaw) * 0.01f;
              scanHoldY += (escapeX * sinYaw + escapeY * cosYaw) * 0.01f;
          }

          // Always use position hold — no mode switching, no jerk
          setScanSetpoint(&setpoint, scanHoldX, scanHoldY, cmdHeight, cmdYawRateDeg);
          commanderSetSetpoint(&setpoint, 3);

          if (humanDetected)
          {
              missionTransition(mission_approach, timeNow);
          }
          else if (timeNow - missionStateStart > SCAN_TIME)
          {
              missionTransition(mission_align, timeNow);
          }
      }
      break;

      // case mission_scan:
      // {
      //     cmdYawRateDeg = 36.0f;   // always spin
      //     cmdHeight = spHeight;
      //     // scanHoldX/Y captured before missionTransition() was called

      //     float minDist = MIN(
      //                         MIN(frontRange, backRange),
      //                         MIN(leftRange, rightRange)
      //                       );

      //     // setScanSetpoint(&setpoint, scanHoldX, scanHoldY, cmdHeight, cmdYawRateDeg);
      //     // ===== ESCAPE if any wall is too close =====
      //     if (minDist < AVOID_RADIUS)
      //     {
      //         float factor = AVOID_VEL_MAX / AVOID_RADIUS;
      //         float f_o = MAX(0.0f, AVOID_RADIUS - frontRange);
      //         float b_o = MAX(0.0f, AVOID_RADIUS - backRange);
      //         float l_o = MAX(0.0f, AVOID_RADIUS - leftRange);
      //         float r_o = MAX(0.0f, AVOID_RADIUS - rightRange);

      //         cmdVelX = (b_o - f_o) * factor;
      //         cmdVelY = (r_o - l_o) * factor;
      //         setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdYawRateDeg);
      //     }
      //     // ===== POSITION HOLD when no wall is close =====
      //     else
      //     {
      //         setScanSetpoint(&setpoint, scanHoldX, scanHoldY, cmdHeight, cmdYawRateDeg);
      //     }

      //     commanderSetSetpoint(&setpoint, 3);

      //     if (humanDetected)
      //     {
      //         missionTransition(mission_approach, timeNow);
      //     }
      //     else if (timeNow - missionStateStart > SCAN_TIME)
      //     {
      //         // missionTransition(mission_reacquire_wall, timeNow);
      //         missionTransition(mission_align, timeNow);  // align yaw before re-acquiring
      //     }
      // }
      // break;
      
      // ===================== ALIGN =====================
      case mission_align:
      {
        // Hold the scan position and rotate back to the yaw captured at scan entry.
        // Once aligned (within YAW_ALIGN_TOL degrees), transition to reacquire.
        cmdHeight = spHeight;

        // Compute shortest signed angular error (degrees)
        float yawErr = scanEntryYaw - yawDeg;
        // Wrap to [-180, 180]
        while (yawErr >  180.0f) yawErr -= 360.0f;
        while (yawErr < -180.0f) yawErr += 360.0f;

        static const float YAW_ALIGN_TOL    = 5.0f;   // degrees — tolerance to consider aligned
        static const float YAW_ALIGN_RATE   = 30.0f;  // deg/s — rotation speed during align
        static const float ALIGN_TIMEOUT    = 5.0f;   // s — give up aligning after this long

        if (fabsf(yawErr) < YAW_ALIGN_TOL ||
            timeNow - missionStateStart > ALIGN_TIMEOUT)
        {
          // Aligned (or timed out) — hold still and transition
          cmdYawRateDeg = 0.0f;
          setScanSetpoint(&setpoint, scanHoldX, scanHoldY, cmdHeight, 0.0f);
          commanderSetSetpoint(&setpoint, 3);
          missionTransition(mission_reacquire_wall, timeNow);
        }
        else
        {
          // Rotate toward target yaw, holding XY position
          cmdYawRateDeg = (yawErr > 0.0f) ? YAW_ALIGN_RATE : -YAW_ALIGN_RATE;
          setScanSetpoint(&setpoint, scanHoldX, scanHoldY, cmdHeight, cmdYawRateDeg);
          commanderSetSetpoint(&setpoint, 3);
        }
      }
      break;

      // case mission_scan:
      // {
      //   cmdYawRateDeg = 36.0f;   // always spin
      //   cmdHeight = spHeight;

      //   // Capture the XY position once when this state is first entered.
      //   // missionStateStart is set by missionTransition(), so comparing against
      //   // it detects the first tick in this state.
      //   static float scanHoldX = 0.0f;
      //   static float scanHoldY = 0.0f;
      //   static float lastScanStart = -1.0f;
      //   if (lastScanStart != missionStateStart)
      //   {
      //     scanHoldX = logGetFloat(idX);
      //     scanHoldY = logGetFloat(idY);
      //     lastScanStart = missionStateStart;
      //   }

      //   float minDist = MIN(
      //                       MIN(frontRange, backRange),
      //                       MIN(leftRange, rightRange)
      //                     );

      //   // ===== ESCAPE if any wall is too close =====
      //   // Uses velocity setpoint so the push is in body frame.
      //   // Takes full priority over position hold.
      //   if (minDist < AVOID_RADIUS)
      //   {
      //       float factor = AVOID_VEL_MAX / AVOID_RADIUS;
      //       float f_o = MAX(0.0f, AVOID_RADIUS - frontRange);
      //       float b_o = MAX(0.0f, AVOID_RADIUS - backRange);
      //       float l_o = MAX(0.0f, AVOID_RADIUS - leftRange);
      //       float r_o = MAX(0.0f, AVOID_RADIUS - rightRange);

      //       cmdVelX = (b_o - f_o) * factor;
      //       cmdVelY = (r_o - l_o) * factor;
      //       setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdYawRateDeg);
      //   }
      //   // ===== POSITION HOLD when no wall is close =====
      //   // Locks XY to the position captured on scan entry.
      //   else
      //   {
      //       setScanSetpoint(&setpoint, scanHoldX, scanHoldY, cmdHeight, cmdYawRateDeg);
      //   }

      //   commanderSetSetpoint(&setpoint, 3);

      //   if (humanDetected)
      //   {
      //     missionTransition(mission_approach, timeNow);
      //   }
      //   else if (timeNow - missionStateStart > SCAN_TIME)
      //   {
      //     missionTransition(mission_reacquire_wall, timeNow);
      //   }
      // }
      // break;
      // case mission_scan:
      // {
      //   cmdYawRateDeg = 36.0f;   // always spin
      //   cmdHeight = spHeight;

      //   float minDist = MIN(
      //                       MIN(frontRange, backRange),
      //                       MIN(leftRange, rightRange)
      //                     );

      //   // ===== ESCAPE if too close =====
      //   if (minDist < AVOID_RADIUS)
      //   {
      //       float factor = AVOID_VEL_MAX / AVOID_RADIUS;
      //       float f_o = MAX(0.0f, AVOID_RADIUS - frontRange);
      //       float b_o = MAX(0.0f, AVOID_RADIUS - backRange);
      //       float l_o = MAX(0.0f, AVOID_RADIUS - leftRange);
      //       float r_o = MAX(0.0f, AVOID_RADIUS - rightRange);

      //       cmdVelX = (b_o - f_o) * factor;  // hard override
      //       cmdVelY = (r_o - l_o) * factor;  // hard override
      //   }
      //   // if (minDist < AVOID_RADIUS)
      //   // {
      //   //     // float factor = 0.2f;
      //   //     float factor = 0.1f;
      //   //     float f_o = MAX(0.0f, AVOID_RADIUS - frontRange);
      //   //     float b_o = MAX(0.0f, AVOID_RADIUS - backRange);
      //   //     float l_o = MAX(0.0f, AVOID_RADIUS - leftRange);
      //   //     float r_o = MAX(0.0f, AVOID_RADIUS - rightRange);

      //   //     // cmdVelX = (b_o - f_o) * factor;
      //   //     // cmdVelY = (r_o - l_o) * factor;
      //   //     cmdVelX += (b_o - f_o) * factor;  // additive
      //   //     cmdVelY += (r_o - l_o) * factor;  // additive
      //   // }

      //   if (humanDetected)
      //   {
      //     missionTransition(mission_approach, timeNow);
      //   }
      //   else if (timeNow - missionStateStart > SCAN_TIME)
      //   {
      //     missionTransition(mission_reacquire_wall, timeNow);
      //   }
      // }
      // break;

      // ===================== APPROACH =====================
      case mission_approach:  // Might have to entirely remove this state and just go straight to mission_bob once humanDetected=True
      {
        if(!humanDetected)
        {
          missionTransition(mission_reacquire_wall,timeNow);
          break;
        }
        cmdYawRateDeg = 20 * humanDir;
        cmdHeight = spHeight;

        float margin = 0.15f;

        if (frontRange > humanStandOff + margin)
            cmdVelX = 0.15f;
        else
            missionTransition(mission_bob, timeNow);  // close enough → bob

        // if(frontRange > humanStandOff + margin)
        //   cmdVelX = 0.15f;
        // else if(frontRange < humanStandOff - margin)
        //   cmdVelX = -0.1f;
        // else
        // {
        //   cmdVelX = 0;
        //   missionTransition(mission_bob,timeNow);
        // }
      }
      break;

      // // ===================== BOB =====================
      // case mission_bob:
      // {
      //   // Might need to comment out this humanDetected check here coz it may lose track of human during bobbing.
      //   if (!humanDetected)
      //   {
      //     missionTransition(mission_reacquire_wall, timeNow);
      //     break;
      //   }

      //   cmdYawRateDeg = 5.0 * humanDir;

      //   float t = timeNow - missionStateStart;

      //   // Cycle 1
      //   if (t < 1.0f) cmdHeight = spHeight + 0.5f;  // rise to 1.0m
      //   else if (t < 2.0f) cmdHeight = spHeight + 0.1f;  // return to 0.5m
      //   // Cycle 2
      //   else if (t < 3.0f) cmdHeight = spHeight + 0.5f;  // rise to 1.0m
      //   else if (t < 4.0f) cmdHeight = spHeight + 0.1f;  // return to 0.5m
      //   // Done
      //   else
      //   {
      //     cmdHeight = spHeight;
      //     missionTransition(mission_land, timeNow);
      //   }
      // }
      // break;

      // ===================== LAND =====================
      // case mission_land:
      // {
      //   cmdVelX = 0.0f;
      //   cmdVelY = 0.0f;
      //   cmdYawRateDeg = 0.0f;

      //   cmdHeight -= 0.0025f;  // 0.1 m/s descent at 10ms loop rate

      //   // if (cmdHeight < 0.0f) cmdHeight = 0.0f;

      //   if (cmdHeight < 0.10f)
      //   {
      //     memset(&setpoint, 0, sizeof(setpoint));
      //     commanderSetSetpoint(&setpoint, 3);
      //     stateOuterLoop = idle;
      //     appActive = 0;
      //     continue;
      //   }
      //   // // Descend gradually over 3 seconds
      //   // cmdHeight = landStartHeight * MAX(0.0f, 1.0f - (t / 3.0f));

      //   // // Once low enough, disarm
      //   // if (cmdHeight < 0.05f)
      //   // {
      //   //   memset(&setpoint, 0, sizeof(setpoint));
      //   //   commanderSetSetpoint(&setpoint, 3);
      //   //   stateOuterLoop = idle;
      //   //   appActive = 0;
      //   //   continue;  // skip the rest of the loop
      //   // }
      // }
      // break;

      // // ===================== TRACK =====================
      // case mission_track:
      // {
      //   if(humanDetected)
      //   {
      //     cmdYawRateDeg = 15 * humanDir;
      //     float margin = 0.15f;

      //     if (frontRange > humanStandOff + margin)
      //     {
      //       cmdVelX = 0.08f;
      //     }
      //     else if (frontRange < humanStandOff - margin)
      //     {
      //       cmdVelX = -0.08f;
      //     }  
      //   }
      //   else
      //   {
      //     missionTransition(mission_reacquire_wall, timeNow);
      //   }
      // }
      // break;
      default:
        break;  // mission_land is handled above, outside this switch
      }
    }
    
    
//     /* ===================== OBSTACLE AVOIDANCE + HEIGHT CONTROL (inspired by push.c) ===================== */
// static const float    avoidVelMax    = 1.0f;
// static const uint16_t avoidRadius    = 300;    // mm — matches push.c 'radius'

// float avoidFactor = avoidVelMax / avoidRadius;  // matches: float factor = velMax/radius;

// uint16_t left_mm  = (uint16_t)(leftRange  * 1000.0f);
// uint16_t right_mm = (uint16_t)(rightRange * 1000.0f);
// uint16_t front_mm = (uint16_t)(frontRange * 1000.0f);
// uint16_t back_mm  = (uint16_t)(backRange  * 1000.0f);

// uint16_t left_o  = avoidRadius - MIN(left_mm,  avoidRadius);  // matches: left_o
// uint16_t right_o = avoidRadius - MIN(right_mm, avoidRadius);  // matches: right_o
// uint16_t front_o = avoidRadius - MIN(front_mm, avoidRadius);  // matches: front_o
// uint16_t back_o  = avoidRadius - MIN(back_mm,  avoidRadius);  // matches: back_o

// float l_comp = (-1.0f) * left_o  * avoidFactor;  // matches: float l_comp = (-1) * left_o  * factor;
// float r_comp = right_o * avoidFactor;  // matches: float r_comp = right_o  * factor;
// float velSide = r_comp + l_comp;                 // matches: float velSide = r_comp + l_comp;

// float f_comp = (-1.0f) * front_o * avoidFactor;  // matches: float f_comp = (-1) * front_o * factor;
// float b_comp = back_o  * avoidFactor;  // matches: float b_comp = back_o  * factor;
// float velFront = b_comp + f_comp;                // matches: float velFront = b_comp + f_comp;

// cmdVelX += velFront;  // replaces: setHoverSetpoint(..., velFront, velSide, ...)
// cmdVelY += velSide;

// // // Height clamp (safety addition — not in push.c)
// // if (cmdHeight > 1.0f) cmdHeight = 1.0f;
// // if (cmdHeight < 0.1f) cmdHeight = 0.1f;
// /* ======================================================================================== */

    
    /* ===================== SMOOTH OBSTACLE AVOIDANCE (NEW: Only uses right and back sensor. Front and left sensors will be used from wallFollower() library automatically) ===================== */
    // bool holdStill = (missionState == mission_land || missionState == mission_bob || missionState == mission_scan || missionState == mission_align);

    bool holdStill = (missionState == mission_bob || missionState == mission_scan || missionState == mission_align);
    if (!holdStill)
    {
        // SGBA-style avoidance: fixed per-axis push when sensor is within save_distance.
        // Adapted for maze application: back avoidance added since drone can back into walls
        // during reacquire/wallfollow. Additive on top of wall follower output (same as SGBA).
        float save_distance = AVOID_RADIUS;   // tune: distance at which avoidance activates
        float push_vel = 0.5f;   // fixed push velocity (matches SGBA's 0.5 m/s)

        float temp_vel_x = cmdVelX;   // start from wall follower output
        float temp_vel_y = cmdVelY;

        if (frontRange < save_distance) temp_vel_x -= push_vel;   // front wall → push backward
        if (backRange  < save_distance) temp_vel_x += push_vel;   // back wall  → push forward
        if (leftRange  < save_distance) temp_vel_y -= push_vel;   // left wall  → push right
        if (rightRange < save_distance) temp_vel_y += push_vel;   // right wall → push left

        cmdVelX = temp_vel_x;
        cmdVelY = temp_vel_y;
    }

    // if (!holdStill)
    // {
    //     float factor = AVOID_VEL_MAX / AVOID_RADIUS;

    //     float b_o     = MAX(0.0f, AVOID_RADIUS - backRange);
    //     float front_o = MAX(0.0f, AVOID_RADIUS - frontRange);

    //     float opp_o = goLeft ? MAX(0.0f, AVOID_RADIUS - leftRange)
    //                  : MAX(0.0f, AVOID_RADIUS - rightRange);

    //     float avoidX = (b_o - front_o) * factor;
    //     float avoidY = goLeft ? (-opp_o * factor)   // obstacle on left → push right (-Y)
    //                           : (opp_o * factor);    // obstacle on right → push left (+Y)
    //     // float opp_o = goLeft ? MAX(0.0f, AVOID_RADIUS - rightRange)
    //     //                     : MAX(0.0f, AVOID_RADIUS - leftRange);

    //     // // float opp_o = goLeft ? MAX(0.0f, AVOID_RADIUS - leftRange)
    //     // //                     : MAX(0.0f, AVOID_RADIUS - rightRange);
        

    //     // float avoidX = (b_o - front_o) * factor;   // back pushes forward, front pushes backward (braking)
    //     // // float avoidY = goLeft ? (-opp_o * factor) : (opp_o * factor);
    //     // float avoidY = goLeft ? (opp_o * factor)   // move left (+Y)
    //     //               : (-opp_o * factor); // move right (-Y)

    //     float weight = MAX(MAX(b_o, front_o), opp_o) / AVOID_RADIUS;
    //     weight = MIN(weight, 1.0f);

    //     cmdVelX = (1.0f - weight) * cmdVelX + weight * avoidX;
    //     cmdVelY = (1.0f - weight) * cmdVelY + weight * avoidY;
    // }

    // if (!holdStill)
    // {
    //     float factor = AVOID_VEL_MAX / AVOID_RADIUS;

    //     float b_o = MAX(0.0f, AVOID_RADIUS - backRange);
    //     // float opp_o = goLeft ? MAX(0.0f, AVOID_RADIUS - rightRange)
    //     //                     : MAX(0.0f, AVOID_RADIUS - leftRange);

    //     float opp_o = goLeft ? MAX(0.0f, AVOID_RADIUS - leftRange)
    //                  : MAX(0.0f, AVOID_RADIUS - rightRange);

    //     float avoidX = b_o * factor;
    //     float avoidY = goLeft ? (-opp_o * factor) : (opp_o * factor);

    //     float weight = MAX(b_o, opp_o) / AVOID_RADIUS;
    //     weight = MIN(weight, 1.0f);

    //     cmdVelX = (1.0f - weight) * cmdVelX + weight * avoidX;
    //     cmdVelY = (1.0f - weight) * cmdVelY + weight * avoidY;
    // }


    // /* ===================== SMOOTH OBSTACLE AVOIDANCE (OLD) ===================== */
    // if (!holdStill)
    // {
    //   // Scale factor (same idea as push demo)
    //   float factor = AVOID_VEL_MAX / AVOID_RADIUS;

    //   // Clamp distances to radius
    //   float f = MIN(frontRange, AVOID_RADIUS);
    //   float l = MIN(leftRange,  AVOID_RADIUS);
    //   float r = MIN(rightRange, AVOID_RADIUS);
    //   float b = MIN(backRange,  AVOID_RADIUS);

    //   // Compute "intrusion" (how close obstacle is)
    //   float f_o = AVOID_RADIUS - f;
    //   float l_o = AVOID_RADIUS - l;
    //   float r_o = AVOID_RADIUS - r;
    //   float b_o = AVOID_RADIUS - b;

    //   // Convert to velocities (same logic as push.c)
    //   float avoidX = (-1.0f) * f_o * factor;
    //   avoidX += b_o * factor;
      
    //   float avoidY = (r_o - l_o) * factor;

    //   // === BLENDING WITH EXISTING COMMANDS ===

    //   // Weight avoidance stronger when very close
    //   // float weight = MAX(f_o, MAX(l_o, r_o)) / AVOID_RADIUS;
    //   float weight = MAX(MAX(f_o, b_o), MAX(l_o, r_o)) / AVOID_RADIUS;
    //   weight = MIN(weight, 1.0f);

    //   // Blend instead of hard override
    //   cmdVelX = (1.0f - weight) * cmdVelX + weight * avoidX;
    //   cmdVelY = (1.0f - weight) * cmdVelY + weight * avoidY;

    //   // // Clamp velocity
    //   // cmdVelX = MAX(MIN(cmdVelX, 0.25f), -0.25f);
    //   // cmdVelY = MAX(MIN(cmdVelY, 0.25f), -0.25f);
    // }

    /* =================================================================== */

    // // ================= ACCELERATION LIMIT =================
    // static float prevCmdX = 0.0f;
    // static float prevCmdY = 0.0f;

    // float maxDelta = 0.05f;  // tune this (0.03–0.07 range)

    // // Compute change
    // float dx = cmdVelX - prevCmdX;
    // float dy = cmdVelY - prevCmdY;

    // // Clamp change
    // if (dx > maxDelta) dx = maxDelta;
    // if (dx < -maxDelta) dx = -maxDelta;

    // if (dy > maxDelta) dy = maxDelta;
    // if (dy < -maxDelta) dy = -maxDelta;

    // // Apply limited change
    // cmdVelX = prevCmdX + dx;
    // cmdVelY = prevCmdY + dy;

    // // Update memory
    // prevCmdX = cmdVelX;
    // prevCmdY = cmdVelY;
    // // =====================================================

    // // Smoothing to avoid oscillations
    // static float prevX = 0, prevY = 0;  // 'static' ensures the value is remembered across loop iterations
    
    // // Reset smoothing history when drone should be stationary
    // // if (missionState == mission_land || missionState == mission_bob)
    // // {
    // //     prevX = 0.0f;
    // //     prevY = 0.0f;
    // // }
    // if (missionState == mission_land || missionState == mission_bob ||
    //     stateInnerLoop == rotateInCorner || stateInnerLoop == turnToFindWall)
    // {
    //     prevX = 0.0f;
    //     prevY = 0.0f;
    // }

    // cmdVelX = 0.7f * prevX + 0.3f * cmdVelX;  // 70% old value + 30% new value
    // cmdVelY = 0.7f * prevY + 0.3f * cmdVelY;  // 70% old value + 30% new value
    // prevX = cmdVelX;
    // prevY = cmdVelY;

    // Clamp velocity
    cmdVelX = MAX(MIN(cmdVelX, VEL_CLAMP), -VEL_CLAMP);
    cmdVelY = MAX(MIN(cmdVelY, VEL_CLAMP), -VEL_CLAMP);

    dashboardMissionState = getDashboardMissionState(missionState);  // Param/Variable for Dashboard Reading
    
    // mission_scan dispatches its own setpoint (position hold or escape velocity)
    // so skip the global dispatch for that state to avoid overwriting it.
    if (missionState != mission_scan && missionState != mission_align)
    {
      setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdYawRateDeg);
      commanderSetSetpoint(&setpoint, 3);
    }
    // setVelocitySetpoint(&setpoint,cmdVelX,cmdVelY,cmdHeight,cmdYawRateDeg);
    // commanderSetSetpoint(&setpoint,3);
  }
}

// ===================== Params =====================

PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8,active,&appActive)
PARAM_ADD(PARAM_UINT8,goLeft,&goLeft)
PARAM_ADD(PARAM_FLOAT,distanceWall,&distanceToWall)
PARAM_ADD(PARAM_FLOAT,maxSpeed,&maxForwardSpeed)
PARAM_GROUP_STOP(app)

// ===================== Logs =====================

LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT,cmdVelX,&cmdVelX)
LOG_ADD(LOG_FLOAT,cmdVelY,&cmdVelY)
LOG_ADD(LOG_FLOAT,cmdYawDeg, &cmdYawRateDeg)
LOG_ADD(LOG_UINT8,stateInnerLoop, &stateInnerLoop)
// LOG_ADD(LOG_UINT8,stateOuterLoop, &stateOuterLoop)
LOG_ADD(LOG_UINT8,stateOuter, &stateOuterLoop)  // Param/Variable for Dashboard Reading
LOG_ADD(LOG_UINT8,missionState,&missionState)
LOG_ADD(LOG_UINT8,mission,&dashboardMissionState)  // Param/Variable for Dashboard Reading
// LOG_ADD(LOG_UINT8,humanDetected,&humanDetected)
LOG_ADD(LOG_UINT8,human,&humanDetected)  // Param/Variable for Dashboard Reading
LOG_ADD(LOG_UINT8,humanConf,&humanConfidence)
LOG_ADD(LOG_INT8,humanDir,&humanDir)
LOG_ADD(LOG_UINT8,humanFresh,&humanPacketFresh)
LOG_ADD(LOG_UINT8,humanStable,&humanStable)  // Param/Variable for Dashboard Reading
LOG_ADD(LOG_FLOAT,humanHold,&humanHoldTimeS)  // Param/Variable for Dashboard Reading
LOG_ADD(LOG_UINT32,humanAgeMs,&humanAgeMs)
LOG_ADD(LOG_INT16,humanMax,&humanMaxTempX100)
LOG_ADD(LOG_INT16,humanTherm,&humanThermistorX100)
LOG_GROUP_STOP(app)