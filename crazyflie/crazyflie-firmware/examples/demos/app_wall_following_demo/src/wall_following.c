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
 * wall_follower.c - App layer application of the wall following demo. The
 * Crazyflie requires the multiranger and the flowdeck v2.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "commander.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "log.h"
#include "param.h"
#include "task.h"
#include "usec_time.h"
#include "wallfollowing_multiranger_onboard.h"

#define DEBUG_MODULE "WALLFOLLOWING"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

static const float spHeight = 0.5f;
static const uint16_t radius = 300U;

static const float WALLFOLLOW_TIME = 10.0f;
static const float SCAN_TIME = 10.0f;
static const float TRANSITION_TIME = 0.35f;
static const uint32_t HUMAN_PACKET_STALE_TIMEOUT_MS = 300U;
static const float HUMAN_CONFIRM_TIME = 3.0f;
static const float SIGNAL_BOB_AMPLITUDE = 0.05f;
static const float SIGNAL_HALF_CYCLE = 0.45f;
static const uint8_t SIGNAL_BOB_COUNT = 3U;
static const float LAND_DESCENT_RATE = 0.20f;
static const float LAND_COMPLETE_HEIGHT = 0.08f;

static const float BATTERY_CUTOFF = 2.8f;

static const float AVOID_RADIUS = 0.4f;
static const float AVOID_VEL_MAX = 0.3f;
static const float AVOID_VEL_CLAMP = 0.25f;

static bool goLeft = false;
static float distanceToWall = 0.5f;
static float maxForwardSpeed = 0.25f;
static uint8_t appActive = 0U;

static uint8_t humanDetected = 0U;
static uint8_t humanConfidence = 0U;
static int8_t humanDir = 0;
static uint8_t humanPacketFresh = 0U;
static uint32_t humanAgeMs = 0U;
static int16_t humanMaxTempX100 = 0;
static int16_t humanThermistorX100 = 0;
static float humanHoldTimeS = 0.0f;
static uint8_t humanStable = 0U;

static float cmdVelX = 0.0f;
static float cmdVelY = 0.0f;
static float cmdYawRateDeg = 0.0f;

typedef struct {
  logVarId_t detected;
  logVarId_t confidence;
  logVarId_t direction;
  logVarId_t maxTemp;
  logVarId_t therm;
  logVarId_t rxLocalMs;
} EspUartLogIds;

typedef enum
{
  idle,
  unlocked,
  stopping
} StateOuterLoop;

typedef enum
{
  mission_reacquire_wall,
  mission_wallfollow,
  mission_scan,
  mission_signal,
  mission_land,
  mission_transition
} MissionState;

static StateOuterLoop stateOuterLoop = idle;
static MissionState missionState = mission_reacquire_wall;
static MissionState nextMissionState = mission_reacquire_wall;
static StateWF stateInnerLoop = forward;

static float missionStateStart = 0.0f;
static float humanDetectStart = -1.0f;

static void setVelocitySetpoint(setpoint_t *setpoint,
                                float vx,
                                float vy,
                                float z,
                                float yawrate)
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

static void resetWallFollower(void)
{
  stateInnerLoop = forward;
  wallFollowerInit(distanceToWall, maxForwardSpeed, stateInnerLoop);
}

static bool espUartLogIdsReady(const EspUartLogIds *ids)
{
  return logVarIdIsValid(ids->detected) &&
         logVarIdIsValid(ids->confidence) &&
         logVarIdIsValid(ids->direction) &&
         logVarIdIsValid(ids->maxTemp) &&
         logVarIdIsValid(ids->therm) &&
         logVarIdIsValid(ids->rxLocalMs);
}

static void updateHumanPerception(const EspUartLogIds *ids, uint32_t nowMs)
{
  if (!espUartLogIdsReady(ids)) {
    return;
  }

  humanDetected = 0U;
  humanConfidence = 0U;
  humanDir = 0;
  humanPacketFresh = 0U;
  humanAgeMs = 0U;
  humanMaxTempX100 = 0;
  humanThermistorX100 = 0;

  humanMaxTempX100 = (int16_t)logGetInt(ids->maxTemp);
  humanThermistorX100 = (int16_t)logGetInt(ids->therm);

  const uint32_t rxLocalMs = logGetUint(ids->rxLocalMs);
  if (rxLocalMs == 0U) {
    return;
  }

  humanAgeMs = nowMs - rxLocalMs;

  if (humanAgeMs > HUMAN_PACKET_STALE_TIMEOUT_MS) {
    return;
  }

  humanPacketFresh = 1U;
  humanConfidence = (uint8_t)logGetUint(ids->confidence);
  humanDir = (int8_t)logGetInt(ids->direction);
  if (logGetUint(ids->detected) != 0U) {
    humanDetected = 1U;
  }
}

void appMain(void)
{
  vTaskDelay(M2T(3000));

  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");

  logVarId_t idYaw = logGetVarId("stabilizer", "yaw");
  logVarId_t idHeight = logGetVarId("stateEstimate", "z");
  logVarId_t idVbat = logGetVarId("pm", "vbat");
  EspUartLogIds espUartIds = {
    .detected = logGetVarId("espUart", "detected"),
    .confidence = logGetVarId("espUart", "confidence"),
    .direction = logGetVarId("espUart", "direction"),
    .maxTemp = logGetVarId("espUart", "maxTemp"),
    .therm = logGetVarId("espUart", "therm"),
    .rxLocalMs = logGetVarId("espUart", "rxLocalMs"),
  };

  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

  setpoint_t setpoint = {0};

  resetWallFollower();

  DEBUG_PRINT("Waiting for activation via app.active\n");

  while (1) {
    vTaskDelay(M2T(10));

    const uint8_t positioningInit = paramGetUint(idPositioningDeck);
    const uint8_t multirangerInit = paramGetUint(idMultiranger);
    const uint16_t up = logGetUint(idUp);
    const float heightEstimate = logGetFloat(idHeight);
    const float vbat = logGetFloat(idVbat);
    const float timeNow = usecTimestamp() / 1e6f;
    const uint32_t nowMs = T2M(xTaskGetTickCount());

    updateHumanPerception(&espUartIds, nowMs);

    if ((vbat < BATTERY_CUTOFF) && (vbat > 0.1f)) {
      stateOuterLoop = stopping;
    }

    if (stateOuterLoop == idle) {
      if (appActive && positioningInit && multirangerInit) {
        stateOuterLoop = unlocked;
        resetWallFollower();
        missionState = mission_reacquire_wall;
        nextMissionState = mission_reacquire_wall;
        missionStateStart = timeNow;
        humanDetectStart = -1.0f;
        humanHoldTimeS = 0.0f;
        humanStable = 0U;
        DEBUG_PRINT("App activated\n");
      }
    } else if (stateOuterLoop == unlocked) {
      if (!appActive) {
        stateOuterLoop = idle;
        DEBUG_PRINT("App deactivated\n");
      }

      if (up < 200U) {
        stateOuterLoop = stopping;
        DEBUG_PRINT("Emergency stop from top ranger\n");
      }
    } else if (stateOuterLoop == stopping) {
      if (up > 500U) {
        stateOuterLoop = idle;
        appActive = 0U;
        DEBUG_PRINT("Stopped\n");
      }
    }

    if (stateOuterLoop != unlocked) {
      cmdVelX = 0.0f;
      cmdVelY = 0.0f;
      cmdYawRateDeg = 0.0f;
      memset(&setpoint, 0, sizeof(setpoint));
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
      continue;
    }

    const float frontRange = logGetUint(idFront) / 1000.0f;
    const float leftRange = logGetUint(idLeft) / 1000.0f;
    const float rightRange = logGetUint(idRight) / 1000.0f;
    const float backRange = logGetUint(idBack) / 1000.0f;
    const float sideRange = goLeft ? rightRange : leftRange;
    const float yawDeg = logGetFloat(idYaw);
    const float yawRad = yawDeg * (float)M_PI / 180.0f;
    const int direction = goLeft ? 1 : -1;

    const uint16_t upOffset = radius - MIN(up, radius);
    float cmdHeight = spHeight - ((float)upOffset / 1000.0f);

    if (cmdHeight < (spHeight - 0.2f)) {
      stateOuterLoop = stopping;
      continue;
    }

    cmdVelX = 0.0f;
    cmdVelY = 0.0f;
    cmdYawRateDeg = 0.0f;

    if (humanDetected) {
      if (humanDetectStart < 0.0f) {
        humanDetectStart = timeNow;
      }
      humanHoldTimeS = timeNow - humanDetectStart;
      humanStable = (humanHoldTimeS >= HUMAN_CONFIRM_TIME) ? 1U : 0U;
    } else {
      humanDetectStart = -1.0f;
      humanHoldTimeS = 0.0f;
      humanStable = 0U;
    }

    if (heightEstimate > (spHeight - 0.1f)) {
      switch (missionState) {
        case mission_transition:
          if ((timeNow - missionStateStart) > TRANSITION_TIME) {
            if (nextMissionState == mission_reacquire_wall) {
              resetWallFollower();
            }
            missionState = nextMissionState;
            missionStateStart = timeNow;
          }
          break;

        case mission_reacquire_wall: {
          if (sideRange < (distanceToWall + 0.3f)) {
            missionTransition(mission_wallfollow, timeNow);
            break;
          }

          float yawRadCmd = 0.0f;
          stateInnerLoop = wallFollower(&cmdVelX,
                                        &cmdVelY,
                                        &yawRadCmd,
                                        frontRange,
                                        sideRange,
                                        yawRad,
                                        direction,
                                        timeNow);
          cmdYawRateDeg = yawRadCmd * 180.0f / (float)M_PI;

          if (stateInnerLoop == forwardAlongWall) {
            missionTransition(mission_wallfollow, timeNow);
          }
          break;
        }

        case mission_wallfollow: {
          float yawRadCmd = 0.0f;
          stateInnerLoop = wallFollower(&cmdVelX,
                                        &cmdVelY,
                                        &yawRadCmd,
                                        frontRange,
                                        sideRange,
                                        yawRad,
                                        direction,
                                        timeNow);
          cmdYawRateDeg = yawRadCmd * 180.0f / (float)M_PI;

          if ((timeNow - missionStateStart) > WALLFOLLOW_TIME) {
            missionTransition(mission_scan, timeNow);
          }
          break;
        }

        case mission_scan: {
          const float minDist = MIN(MIN(frontRange, backRange), MIN(leftRange, rightRange));

          if (minDist < 0.25f) {
            const float factor = 0.1f;
            const float f_o = MAX(0.0f, 0.4f - frontRange);
            const float b_o = MAX(0.0f, 0.4f - backRange);
            const float l_o = MAX(0.0f, 0.4f - leftRange);
            const float r_o = MAX(0.0f, 0.4f - rightRange);

            cmdVelX += (b_o - f_o) * factor;
            cmdVelY += (r_o - l_o) * factor;
            break;
          }

          cmdYawRateDeg = 36.0f;

          if ((timeNow - missionStateStart) > SCAN_TIME) {
            missionTransition(mission_reacquire_wall, timeNow);
          }
          break;
        }

        case mission_signal: {
          const float signalTime = timeNow - missionStateStart;
          const uint8_t halfCycle = (uint8_t)(signalTime / SIGNAL_HALF_CYCLE);
          if (halfCycle >= (uint8_t)(SIGNAL_BOB_COUNT * 2U)) {
            cmdHeight = spHeight;
            missionTransition(mission_land, timeNow);
          } else if ((halfCycle % 2U) == 0U) {
            cmdHeight = spHeight + SIGNAL_BOB_AMPLITUDE;
          } else {
            cmdHeight = spHeight - SIGNAL_BOB_AMPLITUDE;
          }
          break;
        }

        case mission_land: {
          cmdHeight = spHeight - (LAND_DESCENT_RATE * (timeNow - missionStateStart));
          if (cmdHeight < 0.05f) {
            cmdHeight = 0.05f;
          }

          if ((heightEstimate < LAND_COMPLETE_HEIGHT) || ((up > 0U) && (up < 120U))) {
            appActive = 0U;
            stateOuterLoop = idle;
            missionState = mission_reacquire_wall;
            nextMissionState = mission_reacquire_wall;
            humanDetectStart = -1.0f;
            humanHoldTimeS = 0.0f;
            humanStable = 0U;
            cmdVelX = 0.0f;
            cmdVelY = 0.0f;
            cmdYawRateDeg = 0.0f;
            resetWallFollower();
            memset(&setpoint, 0, sizeof(setpoint));
            commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
            DEBUG_PRINT("Mission landed\n");
            continue;
          }
          break;
        }
      }
    }

    if ((missionState != mission_transition) &&
        (missionState != mission_signal) &&
        (missionState != mission_land) &&
        humanStable) {
      missionTransition(mission_signal, timeNow);
    }

    const bool holdStill = (missionState == mission_signal) || (missionState == mission_land);

    if (!holdStill) {
      const float factor = AVOID_VEL_MAX / AVOID_RADIUS;
      const float f = MIN(frontRange, AVOID_RADIUS);
      const float l = MIN(leftRange, AVOID_RADIUS);
      const float r = MIN(rightRange, AVOID_RADIUS);
      const float b = MIN(backRange, AVOID_RADIUS);

      const float f_o = AVOID_RADIUS - f;
      const float l_o = AVOID_RADIUS - l;
      const float r_o = AVOID_RADIUS - r;
      const float b_o = AVOID_RADIUS - b;

      float avoidX = (-1.0f * f_o * factor) + (b_o * factor);
      float avoidY = (r_o - l_o) * factor;

      float weight = MAX(MAX(f_o, b_o), MAX(l_o, r_o)) / AVOID_RADIUS;
      weight = MIN(weight, 1.0f);

      cmdVelX = ((1.0f - weight) * cmdVelX) + (weight * avoidX);
      cmdVelY = ((1.0f - weight) * cmdVelY) + (weight * avoidY);

      cmdVelX = MAX(MIN(cmdVelX, AVOID_VEL_CLAMP), -AVOID_VEL_CLAMP);
      cmdVelY = MAX(MIN(cmdVelY, AVOID_VEL_CLAMP), -AVOID_VEL_CLAMP);
    } else {
      cmdVelX = 0.0f;
      cmdVelY = 0.0f;
    }

    {
      static float prevCmdX = 0.0f;
      static float prevCmdY = 0.0f;
      if (holdStill) {
        prevCmdX = 0.0f;
        prevCmdY = 0.0f;
        cmdVelX = 0.0f;
        cmdVelY = 0.0f;
      } else {
        const float maxDelta = 0.05f;
        float dx = cmdVelX - prevCmdX;
        float dy = cmdVelY - prevCmdY;

        if (dx > maxDelta) {
          dx = maxDelta;
        } else if (dx < -maxDelta) {
          dx = -maxDelta;
        }

        if (dy > maxDelta) {
          dy = maxDelta;
        } else if (dy < -maxDelta) {
          dy = -maxDelta;
        }

        cmdVelX = prevCmdX + dx;
        cmdVelY = prevCmdY + dy;
        prevCmdX = cmdVelX;
        prevCmdY = cmdVelY;
      }
    }

    {
      static float prevX = 0.0f;
      static float prevY = 0.0f;
      if (holdStill) {
        prevX = 0.0f;
        prevY = 0.0f;
      } else {
        cmdVelX = (0.7f * prevX) + (0.3f * cmdVelX);
        cmdVelY = (0.7f * prevY) + (0.3f * cmdVelY);
        prevX = cmdVelX;
        prevY = cmdVelY;
      }
    }

    setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdYawRateDeg);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
  }
}

PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8, active, &appActive)
PARAM_ADD(PARAM_UINT8, goLeft, &goLeft)
PARAM_ADD(PARAM_FLOAT, distanceWall, &distanceToWall)
PARAM_ADD(PARAM_FLOAT, maxSpeed, &maxForwardSpeed)
PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT, cmdVelX, &cmdVelX)
LOG_ADD(LOG_FLOAT, cmdVelY, &cmdVelY)
LOG_ADD(LOG_FLOAT, cmdYawDeg, &cmdYawRateDeg)
LOG_ADD(LOG_UINT8, stateInner, &stateInnerLoop)
LOG_ADD(LOG_UINT8, stateOuter, &stateOuterLoop)
LOG_ADD(LOG_UINT8, mission, &missionState)
LOG_ADD(LOG_UINT8, human, &humanDetected)
LOG_ADD(LOG_UINT8, humanConf, &humanConfidence)
LOG_ADD(LOG_INT8, humanDir, &humanDir)
LOG_ADD(LOG_UINT8, humanFresh, &humanPacketFresh)
LOG_ADD(LOG_UINT8, humanStable, &humanStable)
LOG_ADD(LOG_FLOAT, humanHold, &humanHoldTimeS)
LOG_ADD(LOG_UINT32, humanAgeMs, &humanAgeMs)
LOG_ADD(LOG_INT16, humanMax, &humanMaxTempX100)
LOG_ADD(LOG_INT16, humanTherm, &humanThermistorX100)
LOG_GROUP_STOP(app)
