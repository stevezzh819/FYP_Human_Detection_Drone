/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * SAR Wall-Following Autopilot
 * Logic: 5s Wall-Follow / 5s 360-degree Scan
 * Features: Keyboard Trigger (app.start), Low Battery Auto-Land
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

#define DEBUG_MODULE "SAR_MISSION"

// Helper to set velocity setpoints
static void setVelocitySetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
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

// States for the mission
typedef enum { idle, searching, unlocked, stopping, scanning } StateOuterLoop;

StateOuterLoop stateOuterLoop = idle;
StateWF stateInnerLoop = forward;

// Mission Control Variables
bool start = false;             // Triggered by Python Script (app.start)
static float stateStartTime = 0.0f;
static const float batteryLowThreshold = 2.8f; // Safe landing voltage
// Change to 2.8 V (for battery level)

// Thresholds for manual hand-unlock (Backup safety)
static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

// Flight Parameters
static const float spHeight = 0.5f;
static const uint16_t radius = 300;
bool goLeft = false;
float distanceToWall = 2.0f;  // Change to 1 or 2 m; max range of the multi-ranger sensor is 4 m
float maxForwardSpeed = 0.5f;

float cmdVelX = 0.0f;
float cmdVelY = 0.0f;
float cmdAngWRad = 0.0f;
float cmdAngWDeg = 0.0f;

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)

void appMain()
{
  vTaskDelay(M2T(3000)); // Wait for sensors to stabilize

  // Getting Logging IDs
  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idStabilizerYaw = logGetVarId("stabilizer", "yaw");
  logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");
  logVarId_t idBattery = logGetVarId("pm", "vbat");

  // Getting Param IDs
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

  wallFollowerInit(distanceToWall, maxForwardSpeed, stateInnerLoop);
  setpoint_t setpoint;

  DEBUG_PRINT("SAR Mission Ready. Waiting for Python 'start' command...\n");

  while (1)
  {
    vTaskDelay(M2T(10)); // 100Hz loop
    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);
    uint16_t up = logGetUint(idUp);
    float heightEstimate = logGetFloat(idHeightEstimate);
    float vbat = logGetFloat(idBattery);

    // --- 1. SAFETY: AUTO-LAND ON LOW BATTERY ---
    if (vbat < batteryLowThreshold && stateOuterLoop != idle) {
        DEBUG_PRINT("LOW BATTERY (%.2fV)! Landing for safety.\n", (double)vbat);
        stateOuterLoop = stopping;
        start = false;
    }

    // --- 2. MISSION EXECUTION ---
    float timeNow = usecTimestamp() / 1e6;
    float timeElapsed = timeNow - stateStartTime;

    // Read side sensor once
    float sideRange = (goLeft) ? (float)logGetUint(idRight)/1000.0f : (float)logGetUint(idLeft)/1000.0f;
    float frontRange = (float)logGetUint(idFront)/1000.0f;
    const float sideValidMin = 0.1f;
    const float sideValidMax = 4.0f;
    bool wallDetected = ((sideRange >= sideValidMin && sideRange <= sideValidMax) ||
                     (frontRange >= sideValidMin && frontRange <= sideValidMax));
    if (stateOuterLoop == searching && start)
    {
      // Move forward at constant speed until wall detected
      // cmdVelX = 0.2f;       // forward speed
      cmdVelX = maxForwardSpeed;  // forward speed
      cmdVelY = 0.0f;       // no lateral motion
      cmdAngWDeg = 0.0f;    // no rotation

      // Check if wall is detected
      if (wallDetected) {
          stateOuterLoop = unlocked;           // start wall-following
          stateStartTime = timeNow;
          DEBUG_PRINT("Wall detected! Starting 5s wall-following.\n");
      }

      // Apply setpoint
      setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, spHeight, cmdAngWDeg);
      commanderSetSetpoint(&setpoint, 3);
    }
    else if ((stateOuterLoop == unlocked || stateOuterLoop == scanning) && start)
    {
      // float sideRange = (goLeft) ? (float)logGetUint(idRight)/1000.0f : (float)logGetUint(idLeft)/1000.0f;
      // float frontRange = (float)logGetUint(idFront) / 1000.0f;
      float estYawDeg = logGetFloat(idStabilizerYaw);
      float estYawRad = estYawDeg * (float)M_PI / 180.0f;

      // Adjust height based on obstacles above (hand or ceiling)
      uint16_t up_o = radius - MIN(up, radius);
      float cmdHeight = spHeight - up_o / 1000.0f;

      // --- 3. STATE MACHINE: 5s SEARCH / 5s SCAN ---
      if (stateOuterLoop == unlocked)
      {
        if (timeElapsed < 5.0f) // 5s Wall-Following
        {
          int direction = (goLeft) ? 1 : -1;
          stateInnerLoop = wallFollower(&cmdVelX, &cmdVelY, &cmdAngWRad, frontRange, sideRange, estYawRad, direction, timeNow);
          cmdAngWDeg = cmdAngWRad * 180.0f / (float)M_PI;
        }
        else 
        {
          stateOuterLoop = scanning; // Switch to Scan phase
        }
      }
      else if (stateOuterLoop == scanning)
      {
        if (timeElapsed < 10.0f) // 5s Scan (Total 10s cycle)
        {
          cmdAngWDeg = 72.0f; // 360 degrees in 5s
          cmdVelX = 0.0f;
          cmdVelY = 0.0f;
        }
        else 
        {
          stateOuterLoop = unlocked;
          stateStartTime = timeNow; // Reset Cycle back to Wall-Following
        }
      }

      // Execute movement if at safe height
      if (heightEstimate > spHeight - 0.1f) {
        setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdAngWDeg);
      } else {
        setVelocitySetpoint(&setpoint, 0.0f, 0.0f, cmdHeight, 0.0f);
      }
      commanderSetSetpoint(&setpoint, 3);

      // Manual stop override (Hand above drone)
      if (cmdHeight < spHeight - 0.2f) {
        stateOuterLoop = stopping;
        start = false;
      }
    }
    else // --- 4. IDLE or STOPPED ---
    {
      // Handle Python 'Stop' command
      if (stateOuterLoop == unlocked || stateOuterLoop == scanning) {
          stateOuterLoop = stopping;
      }

      // Hand-unlock logic (Backup)
      if (stateOuterLoop == stopping && up > stoppedTh) { stateOuterLoop = idle; }
      // Unlocks automatically when Python sends 'start'
      if (stateOuterLoop == idle && start && positioningInit && multirangerInit) {
        stateOuterLoop = searching;
        stateStartTime = usecTimestamp() / 1e6;
        DEBUG_PRINT("Start command received: Searching for wall...\n");
      }

      // Ensure motors are off when idle
      if (stateOuterLoop == idle || stateOuterLoop == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}

// Parameters exposed to Python Script
PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8, start, &start)
PARAM_ADD(PARAM_UINT8, goLeft, &goLeft)
PARAM_ADD(PARAM_FLOAT, distanceWall, &distanceToWall)
PARAM_ADD(PARAM_FLOAT, maxSpeed, &maxForwardSpeed)
PARAM_GROUP_STOP(app)

// Logging for debugging
LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT, vbat, &cmdVelX) 
LOG_ADD(LOG_UINT8, stateOuter, &stateOuterLoop)
LOG_GROUP_STOP(app)
