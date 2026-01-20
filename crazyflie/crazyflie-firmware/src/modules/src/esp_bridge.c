/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * ESP bridge: streams a compact telemetry frame over the deck I2C bus to an
 * external ESP32-C3 that listens as a slave at address 0x28.
 */

#define DEBUG_MODULE "ESPBRIDGE"

#include "esp_bridge.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "../../deck/drivers/interface/multiranger.h"
#include "system.h"

#define ESP_BRIDGE_I2C_ADDR           (0x28U)
#define ESP_BRIDGE_FRAME_SIZE_BYTES   (24U)
#define ESP_BRIDGE_PAYLOAD_SIZE_BYTES (22U)
#define ESP_BRIDGE_TASK_PERIOD_TICKS  (pdMS_TO_TICKS(10))

#define ESP_BRIDGE_FLAG_VALID_EST     (0x01U)
#define ESP_BRIDGE_FLAG_VALID_RANGER  (0x02U)

typedef struct {
  float yaw_deg;
  float vx_mps;
  float vy_mps;
  bool valid;
} espBridgeEstimatorSample_t;

static StaticSemaphore_t estimatorMutexBuffer;
static SemaphoreHandle_t estimatorMutex = NULL;
static espBridgeEstimatorSample_t estimatorSample = {0};

static bool isInit = false;

static inline void le16(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

static inline void le32(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
  dst[2] = (uint8_t)((value >> 16) & 0xFF);
  dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; ++i) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static int16_t clamp_s16(float value) {
  if (value > 32767.0f) {
    return INT16_MAX;
  }
  if (value < -32768.0f) {
    return INT16_MIN;
  }
  return (int16_t)lrintf(value);
}

bool espBridgeInit(void) {
  if (isInit) {
    return true;
  }

  estimatorMutex = xSemaphoreCreateMutexStatic(&estimatorMutexBuffer);
  if (estimatorMutex == NULL) {
    DEBUG_PRINT("ESP bridge: failed to allocate mutex\n");
    return false;
  }

  const int i2cInitResult = i2cdevInit(I2C1_DEV);
  if (i2cInitResult != 0) {
    DEBUG_PRINT("ESP bridge: deck I2C init failed (%d)\n", i2cInitResult);
    // Continue anyway; the bus might already be initialised by another driver.
  }

  DEBUG_PRINT("ESP I2C bridge enabled: addr=0x%02X (deck bus)\n", ESP_BRIDGE_I2C_ADDR);

  isInit = true;
  return true;
}

void espBridgeFeedState(const state_t *state) {
  if (!isInit || (state == NULL) || (estimatorMutex == NULL)) {
    return;
  }

  if (xSemaphoreTake(estimatorMutex, 0) == pdTRUE) {
    estimatorSample.yaw_deg = state->attitude.yaw;
    estimatorSample.vx_mps = state->velocity.x;
    estimatorSample.vy_mps = state->velocity.y;
    estimatorSample.valid = true;
    xSemaphoreGive(estimatorMutex);
  }
}

void espBridgeTask(void *param) {
  (void)param;

  if (!isInit) {
    DEBUG_PRINT("ESP bridge task started without init\n");
  }

  systemWaitStart();
  vTaskDelay(M2T(500));

  TickType_t lastWakeTime = xTaskGetTickCount();
  uint8_t frame[ESP_BRIDGE_FRAME_SIZE_BYTES] = {0};
  uint32_t errorCount = 0;
  bool infoLogged = false;

  while (true) {
    vTaskDelayUntil(&lastWakeTime, ESP_BRIDGE_TASK_PERIOD_TICKS);

    if (!isInit) {
      continue;
    }

    bool estValid = false;
    float yaw_deg = 0.0f;
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;

    if (xSemaphoreTake(estimatorMutex, 0) == pdTRUE) {
      if (estimatorSample.valid) {
        estValid = true;
        yaw_deg = estimatorSample.yaw_deg;
        vx_mps = estimatorSample.vx_mps;
        vy_mps = estimatorSample.vy_mps;
      }
      xSemaphoreGive(estimatorMutex);
    }

    frame[0] = 1;  // proto_ver
    uint8_t flags = 0;
    if (estValid) {
      flags |= ESP_BRIDGE_FLAG_VALID_EST;
    }

    uint16_t ranges_mm[5] = {0};
    bool haveRanger = false;

    if (multirangerIsInit()) {
      ranges_mm[0] = multirangerGetDistance(multirangerFront);
      ranges_mm[1] = multirangerGetDistance(multirangerBack);
      ranges_mm[2] = multirangerGetDistance(multirangerLeft);
      ranges_mm[3] = multirangerGetDistance(multirangerRight);
      ranges_mm[4] = multirangerGetDistance(multirangerUp);
      haveRanger = true;
    }

    frame[1] = flags;
    if (haveRanger) {
      frame[1] |= ESP_BRIDGE_FLAG_VALID_RANGER;
    }

    const uint32_t timestamp = (uint32_t)usecTimestamp();
    le32(&frame[2], timestamp);

    const float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    const int16_t yaw_mrad = estValid ? clamp_s16(yaw_rad * 1000.0f) : 0;
    const int16_t vx_mmps = estValid ? clamp_s16(vx_mps * 1000.0f) : 0;
    const int16_t vy_mmps = estValid ? clamp_s16(vy_mps * 1000.0f) : 0;

    le16(&frame[6], (uint16_t)yaw_mrad);
    le16(&frame[8], (uint16_t)vx_mmps);
    le16(&frame[10], (uint16_t)vy_mmps);

    for (size_t i = 0; i < 5; ++i) {
      le16(&frame[12 + (i * 2U)], ranges_mm[i]);
    }

    const uint16_t crc = crc16_ccitt(frame, ESP_BRIDGE_PAYLOAD_SIZE_BYTES);
    le16(&frame[22], crc);

    const bool writeOk = i2cdevWrite(I2C1_DEV, ESP_BRIDGE_I2C_ADDR, ESP_BRIDGE_FRAME_SIZE_BYTES, frame);
    if (!writeOk) {
      errorCount++;
      if ((errorCount % 50U) == 0U) {
        DEBUG_PRINT("ESP bridge I2C write failed (%" PRIu32 ")\n", errorCount);
      }
    } else if (!infoLogged) {
      DEBUG_PRINT("ESP bridge streaming telemetry at 100 Hz\n");
      infoLogged = true;
    }
  }
}
