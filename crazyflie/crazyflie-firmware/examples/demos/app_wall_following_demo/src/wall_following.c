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

#define DEBUG_MODULE "WALLFOLLOWING"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

// ===================== Parameters =====================

static const float spHeight = 0.5f;
static const uint16_t radius = 300;

static const float WALLFOLLOW_TIME = 10.0f;  // changed from 5.0f
static const float SCAN_TIME = 10.0f;  // changed from 5.0f
static const float TRANSITION_TIME = 0.35f;
static const float HUMAN_LOST_TIMEOUT = 1.5f;

static const float BATTERY_CUTOFF = 2.8f;

static const float OBSTACLE_STOP_DIST = 0.30f;  // 30cm safety distance (to stop scanning and go back to wall-following)

// Push-style avoidance parameters
static const float AVOID_RADIUS = 0.4f;   // meters
static const float AVOID_VEL_MAX = 0.3f;  // max avoidance velocity

// ===================== Wall follow parameters =====================

bool goLeft = false;
float distanceToWall = 0.5f;  // Target distance that the Crazyflie tries to maintain from the wall while following it.
float maxForwardSpeed = 0.25f;

// ===================== Python control =====================

uint8_t appActive = 0;

// ===================== Thermal perception =====================

uint8_t humanDetected = 0;
int8_t humanDir = 0;          // -1 left, 0 center, +1 right
float humanStandOff = 1.0f;  // Desired distance that the drone should keep from a detected human (should NOT be too close for safety)

// ===================== Command variables =====================

float cmdVelX = 0.0f;
float cmdVelY = 0.0f;
float cmdYawRateDeg = 0.0f;

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
  mission_track,
  mission_transition
} MissionState;

static MissionState missionState = mission_reacquire_wall;  // what the drone is doing
static MissionState nextMissionState;  // what it will do next

static float missionStateStart = 0;  // when the current task started
static float lastHumanSeen = -1;  // when the human was last detected

// ===================== Wall follower =====================

static StateWF stateInnerLoop = forward;

// ===================== Detection debounce =====================

static uint8_t humanSeenCount = 0;
#define HUMAN_CONFIRM_FRAMES 3

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

// ===================== Main =====================

void appMain()
{
  vTaskDelay(M2T(3000));
  // Getting Logging IDs of the multiranger
  logVarId_t idUp = logGetVarId("range","up");
  logVarId_t idLeft = logGetVarId("range","left");
  logVarId_t idRight = logGetVarId("range","right");
  logVarId_t idFront = logGetVarId("range","front");
  logVarId_t idBack  = logGetVarId("range","back");

  // Getting the Logging IDs of the state estimates
  logVarId_t idYaw = logGetVarId("stabilizer","yaw");
  logVarId_t idHeight = logGetVarId("stateEstimate","z");

  logVarId_t idVbat = logGetVarId("pm","vbat");

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

    // Get the upper range sensor value (used for stopping)
    uint16_t up = logGetUint(idUp);

    // Get Height estimate
    float heightEstimate = logGetFloat(idHeight);
    
    float vbat = logGetFloat(idVbat);

    if(vbat < BATTERY_CUTOFF && vbat > 0.1f)
      stateOuterLoop = stopping;

    // ===================== OUTER LOOP =====================

    // -------- IDLE STATE --------
    if(stateOuterLoop == idle)
    {
      // Start only by Python
      if(appActive && positioningInit && multirangerInit)
      {
        stateOuterLoop = unlocked;

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
      if(!appActive)
      {
        stateOuterLoop = idle;
        DEBUG_PRINT("Stopped by Python script\n");
      }

      // Emergency stop using hand above sensor
      if(up < 200)
      {
        stateOuterLoop = stopping;
        DEBUG_PRINT("Emergency stop (hand detected)\n");
      }
    }


    // -------- STOPPING STATE --------
    else if(stateOuterLoop == stopping)
    {
      // Wait until hand removed
      if(up > 500)
      {
        stateOuterLoop = idle;

        // Force Python restart
        appActive = 0;

        DEBUG_PRINT("Stopped\n");
      }
    }


    // -------- DISARM MOTORS IF NOT UNLOCKED --------
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

    float sideRange;

    if(goLeft)
      sideRange = logGetUint(idRight)/1000.0f;
    else
      sideRange = logGetUint(idLeft)/1000.0f;

    float yawDeg = logGetFloat(idYaw);
    float yawRad = yawDeg * (float)M_PI / 180.0f;

    float timeNow = usecTimestamp()/1e6f;

    //Adjust height based on up ranger input
    uint16_t up_o = radius - MIN(up,radius);
    float cmdHeight = spHeight - up_o/1000.0f;

    if(cmdHeight < spHeight - 0.2f)
    {
      stateOuterLoop = stopping;
      continue;
    }

    cmdVelX = 0.0f;
    cmdVelY = 0.0f;
    cmdYawRateDeg = 0.0f;

    if(humanDetected)
      lastHumanSeen = timeNow;

    int direction = goLeft ? 1 : -1;

    // ===================== Mission FSM =====================

    if(heightEstimate > spHeight - 0.1f)
    {

      switch(missionState)
      {

      case mission_transition:  // To let the drone hover in between states
      {
        cmdVelX = 0.0f;
        cmdVelY = 0.0f;
        cmdYawRateDeg = 0.0f;

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
        // If wall already beside us, resume wall following
        if(sideRange < distanceToWall + 0.3f)
        {
          missionTransition(mission_wallfollow,timeNow);
          break;
        }

        float yawRadCmd;

        stateInnerLoop = wallFollower(&cmdVelX,&cmdVelY,&yawRadCmd,
                                      frontRange,sideRange,yawRad,
                                      direction,timeNow);

        cmdYawRateDeg = yawRadCmd * 180.0f/(float)M_PI;

        if(stateInnerLoop == forwardAlongWall)  // Wall follower has successfully locked onto a wall
        {
          missionTransition(mission_wallfollow,timeNow);
        }
      }
      break;

      // ===================== WALL FOLLOW =====================

      case mission_wallfollow:
      {
        float yawRadCmd;

        stateInnerLoop = wallFollower(&cmdVelX,&cmdVelY,&yawRadCmd,
                                      frontRange,sideRange,yawRad,
                                      direction,timeNow);

        cmdYawRateDeg = yawRadCmd * 180.0f/(float)M_PI;

        if(humanDetected)
        {
          humanSeenCount++;

          if(humanSeenCount >= HUMAN_CONFIRM_FRAMES)
            missionTransition(mission_approach,timeNow);
        }
        else
          humanSeenCount = 0;

        if(timeNow - missionStateStart > WALLFOLLOW_TIME)
        {
          missionTransition(mission_scan,timeNow);
        }
      }
      break;

      // ===================== SCAN =====================
      case mission_scan:
      {
          float minDist = MIN(
                              MIN(frontRange, backRange),
                              MIN(leftRange, rightRange)
                            );

          // ===== ESCAPE if too close =====
          if (minDist < 0.25f)
          {
              // float factor = 0.2f;
              float factor = 0.1f;

              float f_o = MAX(0.0f, 0.4f - frontRange);
              float b_o = MAX(0.0f, 0.4f - backRange);
              float l_o = MAX(0.0f, 0.4f - leftRange);
              float r_o = MAX(0.0f, 0.4f - rightRange);

              // cmdVelX = (b_o - f_o) * factor;
              // cmdVelY = (r_o - l_o) * factor;
              cmdVelX += (b_o - f_o) * factor;  // additive
              cmdVelY += (r_o - l_o) * factor;  // additive
              cmdYawRateDeg = 0.0f;

              break;
          }

          // ===== Distance-aware rotation =====
          // float safeDist = 0.5f;
          // float stopDist = 0.25f;

          // float scale = (minDist - stopDist) / (safeDist - stopDist);
          // scale = MAX(0.0f, MIN(scale, 1.0f));
          // cmdYawRateDeg = 36.0f * scale;

          cmdYawRateDeg = 36.0f;
          // Optional slight drift
          // cmdVelX = 0.03f;

          // ===== Human detection =====
          if(humanDetected)
          {
              humanSeenCount++;
              if(humanSeenCount >= HUMAN_CONFIRM_FRAMES)
              {
                  humanSeenCount = 0;
                  missionTransition(mission_approach,timeNow);
              }
          }
          else
          {
              humanSeenCount = 0;
          }

          if(timeNow - missionStateStart > SCAN_TIME)
          {
              missionTransition(mission_reacquire_wall,timeNow);
          }
      }
      break;

      // case mission_scan:
      // {
      //   cmdVelX = 0.0f;
      //   cmdVelY = 0.0f;
      //   cmdYawRateDeg = 36.0f;  // changed from 72.0f

      //   // Safety check for obstacles
      //   if(frontRange < OBSTACLE_STOP_DIST || sideRange < OBSTACLE_STOP_DIST)
      //   {
      //     missionTransition(mission_reacquire_wall,timeNow);
      //     break;
      //   }

      //   if(humanDetected)
      //   {
      //     humanSeenCount++;

      //     if(humanSeenCount >= HUMAN_CONFIRM_FRAMES)
      //     {
      //       humanSeenCount = 0;
      //       missionTransition(mission_approach,timeNow);
      //     }
      //   }
      //   else
      //     humanSeenCount = 0;

      //   if(timeNow - missionStateStart > SCAN_TIME)
      //   {
      //     missionTransition(mission_reacquire_wall,timeNow);
      //   }
      // }
      // break;

      // ===================== APPROACH ===================== (should not be triggered yet since thermal sensor is not integrated yet)

      case mission_approach:
      {
        cmdYawRateDeg = 20 * humanDir;

        float margin = 0.15f;

        if(frontRange > humanStandOff + margin)
          cmdVelX = 0.15f;
        else if(frontRange < humanStandOff - margin)
          cmdVelX = -0.1f;
        else
        {
          cmdVelX = 0;
          missionTransition(mission_bob,timeNow);
        }

        if(!humanDetected && timeNow - lastHumanSeen > HUMAN_LOST_TIMEOUT)
        {
          missionTransition(mission_reacquire_wall,timeNow);
        }
      }
      break;

      // ===================== BOB =====================

      case mission_bob:
      {
        float t = timeNow - missionStateStart;

        if(t < 0.4f)
          cmdHeight = spHeight + 0.1f;
        else if(t < 0.8f)
          cmdHeight = spHeight - 0.1f;
        else
        {
          cmdHeight = spHeight;
          missionTransition(mission_track,timeNow);
        }
      }
      break;

      // ===================== TRACK =====================

      case mission_track:
      {
        if(humanDetected)
        {
          cmdYawRateDeg = 15 * humanDir;

          float margin = 0.15f;

          if(frontRange > humanStandOff + margin)
            cmdVelX = 0.08f;
          else if(frontRange < humanStandOff - margin)
            cmdVelX = -0.08f;
        }
        else
        {
          if(timeNow - lastHumanSeen > HUMAN_LOST_TIMEOUT)
            missionTransition(mission_reacquire_wall,timeNow);
        }
      }
      break;

      }
    }

    /* ===================== SMOOTH OBSTACLE AVOIDANCE ===================== */
    // Scale factor (same idea as push demo)
    float factor = AVOID_VEL_MAX / AVOID_RADIUS;

    // Clamp distances to radius
    float f = MIN(frontRange, AVOID_RADIUS);
    float l = MIN(leftRange,  AVOID_RADIUS);
    float r = MIN(rightRange, AVOID_RADIUS);
    float b = MIN(backRange,  AVOID_RADIUS);

    // Compute "intrusion" (how close obstacle is)
    float f_o = AVOID_RADIUS - f;
    float l_o = AVOID_RADIUS - l;
    float r_o = AVOID_RADIUS - r;
    float b_o = AVOID_RADIUS - b;

    // Convert to velocities (same logic as push.c)
    float avoidX = (-1.0f) * f_o * factor;
    avoidX += b_o * factor;
    
    float avoidY = (r_o - l_o) * factor;

    // === BLENDING WITH EXISTING COMMANDS ===

    // Weight avoidance stronger when very close
    // float weight = MAX(f_o, MAX(l_o, r_o)) / AVOID_RADIUS;
    float weight = MAX(MAX(f_o, b_o), MAX(l_o, r_o)) / AVOID_RADIUS;
    weight = MIN(weight, 1.0f);

    // Blend instead of hard override
    cmdVelX = (1.0f - weight) * cmdVelX + weight * avoidX;
    cmdVelY = (1.0f - weight) * cmdVelY + weight * avoidY;

    // Optional: clamp velocity
    cmdVelX = MAX(MIN(cmdVelX, 0.25f), -0.25f);
    cmdVelY = MAX(MIN(cmdVelY, 0.25f), -0.25f);

    /* =================================================================== */

    // ================= ACCELERATION LIMIT =================
    static float prevCmdX = 0.0f;
    static float prevCmdY = 0.0f;

    float maxDelta = 0.05f;  // tune this (0.03–0.07 range)

    // Compute change
    float dx = cmdVelX - prevCmdX;
    float dy = cmdVelY - prevCmdY;

    // Clamp change
    if (dx > maxDelta) dx = maxDelta;
    if (dx < -maxDelta) dx = -maxDelta;

    if (dy > maxDelta) dy = maxDelta;
    if (dy < -maxDelta) dy = -maxDelta;

    // Apply limited change
    cmdVelX = prevCmdX + dx;
    cmdVelY = prevCmdY + dy;

    // Update memory
    prevCmdX = cmdVelX;
    prevCmdY = cmdVelY;
    // =====================================================

    // Smoothing to avoid oscillations
    static float prevX = 0, prevY = 0;  // 'static' ensures the value is remembered across loop iterations
    cmdVelX = 0.7f * prevX + 0.3f * cmdVelX;  // 70% old value + 30% new value
    cmdVelY = 0.7f * prevY + 0.3f * cmdVelY;  // 70% old value + 30% new value
    prevX = cmdVelX;
    prevY = cmdVelY;

    setVelocitySetpoint(&setpoint,cmdVelX,cmdVelY,cmdHeight,cmdYawRateDeg);
    commanderSetSetpoint(&setpoint,3);
  }
}

// ===================== Params =====================

PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8,active,&appActive)
PARAM_ADD(PARAM_UINT8,human,&humanDetected)
PARAM_ADD(PARAM_INT8,humanDir,&humanDir)
PARAM_ADD(PARAM_FLOAT,humanDist,&humanStandOff)
PARAM_ADD(PARAM_UINT8,goLeft,&goLeft)
PARAM_ADD(PARAM_FLOAT,distanceWall,&distanceToWall)
PARAM_ADD(PARAM_FLOAT,maxSpeed,&maxForwardSpeed)
PARAM_GROUP_STOP(app)

// ===================== Logs =====================

LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT,cmdVelX,&cmdVelX)
LOG_ADD(LOG_FLOAT,cmdVelY,&cmdVelY)
LOG_ADD(LOG_UINT8,missionState,&missionState)
LOG_ADD(LOG_UINT8,humanDetected,&humanDetected)
LOG_GROUP_STOP(app)

