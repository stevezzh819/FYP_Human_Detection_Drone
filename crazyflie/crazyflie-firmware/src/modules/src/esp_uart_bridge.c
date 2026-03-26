/**
 * Crazyflie <-UART-> ESP32 bridge for human-detection packets.
 */

#define DEBUG_MODULE "ESPUART"

#include "esp_uart_bridge.h"

#include <errno.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "debug.h"
#include "log.h"
#include "system.h"
#include "uart2.h"
#include "buzzer.h"
#include "commander.h"
#include "crtp_commander_high_level.h"

#define ESP_UART_BRIDGE_BAUDRATE       (57600U)
#define ESP_UART_HEADER_BYTE0           (0xAAU)
#define ESP_UART_HEADER_BYTE1           (0x55U)
#define ESP_UART_MAX_PAYLOAD            (64U)

#define ESP_UART_TYPE_HUMAN_DETECT      (0x01U)
#define ESP_UART_TYPE_COMMAND           (0x10U)
#define ESP_UART_TYPE_TEST_TEXT         (0x7EU)
#define ESP_UART_TEST_PERIOD_MS         (1000U)
#define ESP_UART_TEST_FROM_CRAZYFLIE    "UART_TEST_FROM_CRAZYFLIE"
#define ESP_UART_TEST_TEXT_BUF_LEN      (ESP_UART_MAX_PAYLOAD + 1U)
#define ESP_UART_RX_BEEP_FREQ_HZ        (2800U)
#define ESP_UART_RX_BEEP_DURATION_MS    (1000U)
#define ESP_UART_CMD_TAKEOFF            (0x20U)
#define ESP_UART_CMD_LAND               (0x21U)

typedef struct {
  uint8_t type;
  uint8_t length;
  uint8_t payload[ESP_UART_MAX_PAYLOAD];
} espUartPacket_t;

typedef enum {
  rxWaitHeader0 = 0,
  rxWaitHeader1,
  rxWaitType,
  rxWaitLength,
  rxWaitPayload,
  rxWaitCrcLo,
  rxWaitCrcHi,
} espUartRxState_t;

static StaticSemaphore_t stateMutexBuffer;
static SemaphoreHandle_t stateMutex = NULL;

static bool isInit = false;
static bool hasHumanSample = false;
static espUartHumanDetection_t latestHumanSample = {0};

static uint8_t humanDetectedLog = 0;
static uint8_t humanConfidenceLog = 0;
static int8_t humanDirectionLog = 0;
static int16_t humanMaxTempLog = 0;
static int16_t humanThermistorLog = 0;
static uint32_t humanTimestampMsLog = 0;
static uint32_t humanRxLocalTimeMsLog = 0;
static uint32_t testRxCountLog = 0;
static uint32_t testTxCountLog = 0;
static uint32_t beepCountLog = 0;
static uint32_t cmdRxCountLog = 0;
static char lastTestMessageRx[ESP_UART_TEST_TEXT_BUF_LEN] = {0};
static bool rxBeepActive = false;
static TickType_t rxBeepStopTick = 0;

static espUartRxState_t rxState = rxWaitHeader0;
static uint8_t rxType = 0;
static uint8_t rxLength = 0;
static uint8_t rxPayloadIndex = 0;
static uint8_t rxPayload[ESP_UART_MAX_PAYLOAD] = {0};
static uint8_t rxCrcLo = 0;

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint32_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8U);
    for (int bit = 0; bit < 8; bit++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      } else {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

static void resetParser(void)
{
  rxState = rxWaitHeader0;
  rxType = 0;
  rxLength = 0;
  rxPayloadIndex = 0;
  rxCrcLo = 0;
}

static bool consumeByte(uint8_t byte, espUartPacket_t *packet)
{
  switch (rxState) {
    case rxWaitHeader0:
      if (byte == ESP_UART_HEADER_BYTE0) {
        rxState = rxWaitHeader1;
      }
      break;

    case rxWaitHeader1:
      if (byte == ESP_UART_HEADER_BYTE1) {
        rxState = rxWaitType;
      } else if (byte == ESP_UART_HEADER_BYTE0) {
        rxState = rxWaitHeader1;
      } else {
        rxState = rxWaitHeader0;
      }
      break;

    case rxWaitType:
      rxType = byte;
      rxState = rxWaitLength;
      break;

    case rxWaitLength:
      rxLength = byte;
      if (rxLength > ESP_UART_MAX_PAYLOAD) {
        DEBUG_PRINT("ESP UART RX: invalid payload len=%u\n", rxLength);
        resetParser();
      } else if (rxLength == 0U) {
        rxState = rxWaitCrcLo;
      } else {
        rxPayloadIndex = 0;
        rxState = rxWaitPayload;
      }
      break;

    case rxWaitPayload:
      rxPayload[rxPayloadIndex++] = byte;
      if (rxPayloadIndex >= rxLength) {
        rxState = rxWaitCrcLo;
      }
      break;

    case rxWaitCrcLo:
      rxCrcLo = byte;
      rxState = rxWaitCrcHi;
      break;

    case rxWaitCrcHi: {
      const uint16_t rxCrc = (uint16_t)rxCrcLo | ((uint16_t)byte << 8U);
      uint8_t crcBuf[2 + ESP_UART_MAX_PAYLOAD] = {0};
      crcBuf[0] = rxType;
      crcBuf[1] = rxLength;
      if (rxLength > 0U) {
        memcpy(&crcBuf[2], rxPayload, rxLength);
      }
      const uint16_t calcCrc = crc16_ccitt(crcBuf, (uint32_t)2U + rxLength);

      if (rxCrc == calcCrc) {
        packet->type = rxType;
        packet->length = rxLength;
        if (rxLength > 0U) {
          memcpy(packet->payload, rxPayload, rxLength);
        }
        resetParser();
        return true;
      }

      DEBUG_PRINT("ESP UART RX: CRC mismatch rx=0x%04X calc=0x%04X\n", rxCrc, calcCrc);
      resetParser();
      break;
    }
  }

  return false;
}

static void handlePacket(const espUartPacket_t *packet)
{
  buzzerOn(ESP_UART_RX_BEEP_FREQ_HZ);
  rxBeepActive = true;
  rxBeepStopTick = xTaskGetTickCount() + pdMS_TO_TICKS(ESP_UART_RX_BEEP_DURATION_MS);
  beepCountLog++;

  if (packet->type == ESP_UART_TYPE_COMMAND) {
    if (packet->length < 1U) {
      return;
    }

    cmdRxCountLog++;
    const uint8_t cmd = packet->payload[0];
    if ((cmd == ESP_UART_CMD_TAKEOFF) || (cmd == ESP_UART_CMD_LAND)) {
      if (packet->length < 5U) {
        DEBUG_PRINT("ESP UART CMD: cmd=0x%02X missing args\n", cmd);
        return;
      }

      const uint16_t height_cm = (uint16_t)packet->payload[1] | ((uint16_t)packet->payload[2] << 8U);
      const uint16_t duration_ms = (uint16_t)packet->payload[3] | ((uint16_t)packet->payload[4] << 8U);
      // Use relative vertical commands to avoid dependence on absolute Z drift
      // during bring-up when no external positioning is available.
      float height_m = (float)height_cm / 100.0f;
      float duration_s = (float)duration_ms / 1000.0f;
      if (height_m < 0.05f) {
        height_m = 0.30f;
      }
      if (duration_s < 0.5f) {
        duration_s = 2.0f;
      }
      const float velocity_m_s = height_m / duration_s;

      // Ensure high-level setpoints can replace any stale higher-priority source.
      commanderRelaxPriority();

      int result = -1;
      int attempts = 0;
      for (attempts = 0; attempts < 6; attempts++) {
        if (cmd == ESP_UART_CMD_TAKEOFF) {
          result = crtpCommanderHighLevelTakeoffWithVelocity(height_m, velocity_m_s, true);
        } else {
          result = crtpCommanderHighLevelLandWithVelocity(height_m, velocity_m_s, true);
        }

        if (result != EBUSY) {
          break;
        }

        // Planner might still be finishing a previous sequence. Force stop,
        // relax priority and try again after a short delay.
        crtpCommanderHighLevelStop();
        commanderRelaxPriority();
        vTaskDelay(pdMS_TO_TICKS(25));
      }

      DEBUG_PRINT("ESP UART CMD: cmd=0x%02X rel_h=%.2fm dur=%.2fs vel=%.2fm/s result=%d attempts=%d\n",
                  cmd,
                  (double)height_m,
                  (double)duration_s,
                  (double)velocity_m_s,
                  result,
                  attempts + 1);
    } else {
      DEBUG_PRINT("ESP UART CMD: unknown cmd=0x%02X\n", cmd);
    }
    return;
  }

  if (packet->type == ESP_UART_TYPE_TEST_TEXT) {
    size_t copyLen = packet->length;
    if (copyLen >= ESP_UART_TEST_TEXT_BUF_LEN) {
      copyLen = ESP_UART_TEST_TEXT_BUF_LEN - 1U;
    }

    if (copyLen > 0U) {
      memcpy(lastTestMessageRx, packet->payload, copyLen);
    }
    lastTestMessageRx[copyLen] = '\0';
    testRxCountLog++;
    DEBUG_PRINT("UART TEST RX <- ESP32: %s\n", lastTestMessageRx);
    return;
  }

  if (packet->type != ESP_UART_TYPE_HUMAN_DETECT) {
    return;
  }

  if (packet->length < 10U) {
    DEBUG_PRINT("ESP UART RX: human packet too short (%u)\n", packet->length);
    return;
  }

  espUartHumanDetection_t sample = {0};
  sample.detected = (packet->payload[0] != 0U);
  sample.confidence = packet->payload[1];
  sample.rxLocalTimeMs = T2M(xTaskGetTickCount());

  if (packet->length >= 11U) {
    sample.direction = (int8_t)packet->payload[2];
    sample.maxTempX100 = (int16_t)((uint16_t)packet->payload[3] |
                                   ((uint16_t)packet->payload[4] << 8U));
    sample.thermistorX100 = (int16_t)((uint16_t)packet->payload[5] |
                                      ((uint16_t)packet->payload[6] << 8U));
    sample.sourceTimestampMs = ((uint32_t)packet->payload[7]) |
                               ((uint32_t)packet->payload[8] << 8U) |
                               ((uint32_t)packet->payload[9] << 16U) |
                               ((uint32_t)packet->payload[10] << 24U);
  } else {
    sample.direction = 0;
    sample.maxTempX100 = (int16_t)((uint16_t)packet->payload[2] |
                                   ((uint16_t)packet->payload[3] << 8U));
    sample.thermistorX100 = (int16_t)((uint16_t)packet->payload[4] |
                                      ((uint16_t)packet->payload[5] << 8U));
    sample.sourceTimestampMs = ((uint32_t)packet->payload[6]) |
                               ((uint32_t)packet->payload[7] << 8U) |
                               ((uint32_t)packet->payload[8] << 16U) |
                               ((uint32_t)packet->payload[9] << 24U);
  }

  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    latestHumanSample = sample;
    hasHumanSample = true;
    humanDetectedLog = sample.detected ? 1U : 0U;
    humanConfidenceLog = sample.confidence;
    humanDirectionLog = sample.direction;
    humanMaxTempLog = sample.maxTempX100;
    humanThermistorLog = sample.thermistorX100;
    humanTimestampMsLog = sample.sourceTimestampMs;
    humanRxLocalTimeMsLog = sample.rxLocalTimeMs;
    xSemaphoreGive(stateMutex);
  }
}

bool espUartBridgeInit(void)
{
  if (isInit) {
    return true;
  }

  stateMutex = xSemaphoreCreateMutexStatic(&stateMutexBuffer);
  if (stateMutex == NULL) {
    DEBUG_PRINT("ESP UART bridge: failed to create mutex\n");
    return false;
  }

  uart2Init(ESP_UART_BRIDGE_BAUDRATE);
  resetParser();

  isInit = true;
  DEBUG_PRINT("ESP UART bridge enabled on UART2 at %u baud\n", ESP_UART_BRIDGE_BAUDRATE);
  return true;
}

void espUartBridgeTask(void *param)
{
  (void)param;

  if (!isInit) {
    DEBUG_PRINT("ESP UART bridge task started without init\n");
  }

  systemWaitStart();
  vTaskDelay(pdMS_TO_TICKS(200));

  TickType_t lastStatusLog = xTaskGetTickCount();
  TickType_t lastTestTx = xTaskGetTickCount();
  espUartPacket_t packet = {0};

  while (true) {
    uint8_t byte = 0;
    if (uart2GetCharWithTimeout(&byte, pdMS_TO_TICKS(10))) {
      if (consumeByte(byte, &packet)) {
        handlePacket(&packet);
      }
    } else {
      taskYIELD();
    }

    const TickType_t now = xTaskGetTickCount();
    if (rxBeepActive && ((int32_t)(now - rxBeepStopTick) >= 0)) {
      buzzerOff();
      rxBeepActive = false;
    }

    if ((now - lastTestTx) >= pdMS_TO_TICKS(ESP_UART_TEST_PERIOD_MS)) {
      const char *testMsg = ESP_UART_TEST_FROM_CRAZYFLIE;
      const uint8_t testLen = (uint8_t)strlen(testMsg);
      if (espUartBridgeSendPacket(ESP_UART_TYPE_TEST_TEXT, (const uint8_t *)testMsg, testLen)) {
        testTxCountLog++;
        DEBUG_PRINT("UART TEST TX -> ESP32: %s\n", testMsg);
      } else {
        DEBUG_PRINT("UART TEST TX failed\n");
      }
      lastTestTx = now;
    }

    if ((now - lastStatusLog) > pdMS_TO_TICKS(2000)) {
      lastStatusLog = now;
      DEBUG_PRINT("ESP UART bridge alive: detected=%u conf=%u dir=%d testRx=%lu testTx=%lu\n",
                  (unsigned)humanDetectedLog,
                  (unsigned)humanConfidenceLog,
                  (int)humanDirectionLog,
                  (unsigned long)testRxCountLog,
                  (unsigned long)testTxCountLog);
    }
  }
}

bool espUartBridgeGetLatestHumanDetectionSample(espUartHumanDetection_t *sample)
{
  if ((!isInit) || (sample == NULL)) {
    return false;
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) != pdTRUE) {
    return false;
  }

  const bool isValid = hasHumanSample;
  if (isValid) {
    *sample = latestHumanSample;
  }

  xSemaphoreGive(stateMutex);
  return isValid;
}

bool espUartBridgeGetLatestHumanDetection(bool *detected,
                                          uint8_t *confidence,
                                          uint32_t *timestamp_ms)
{
  espUartHumanDetection_t sample = {0};
  if (!espUartBridgeGetLatestHumanDetectionSample(&sample)) {
    return false;
  }

  bool freshDetected = sample.detected;
  uint8_t freshConfidence = sample.confidence;
  if ((T2M(xTaskGetTickCount()) - sample.rxLocalTimeMs) > 300U) {
    freshDetected = false;
    freshConfidence = 0U;
  }

  if (detected != NULL) {
    *detected = freshDetected;
  }
  if (confidence != NULL) {
    *confidence = freshConfidence;
  }
  if (timestamp_ms != NULL) {
    *timestamp_ms = sample.sourceTimestampMs;
  }
  return true;
}

bool espUartBridgeSendPacket(uint8_t type, const uint8_t *payload, uint8_t length)
{
  if (!isInit) {
    return false;
  }
  if (length > ESP_UART_MAX_PAYLOAD) {
    return false;
  }

  uint8_t frame[2 + 1 + 1 + ESP_UART_MAX_PAYLOAD + 2] = {0};
  uint32_t idx = 0;

  frame[idx++] = ESP_UART_HEADER_BYTE0;
  frame[idx++] = ESP_UART_HEADER_BYTE1;
  frame[idx++] = type;
  frame[idx++] = length;

  if ((length > 0U) && (payload != NULL)) {
    memcpy(&frame[idx], payload, length);
  }
  idx += length;

  const uint16_t crc = crc16_ccitt(&frame[2], (uint32_t)2U + length);
  frame[idx++] = (uint8_t)(crc & 0xFFU);
  frame[idx++] = (uint8_t)((crc >> 8U) & 0xFFU);

  uart2SendData(idx, frame);
  return true;
}

LOG_GROUP_START(espUart)
LOG_ADD(LOG_UINT8, detected, &humanDetectedLog)
LOG_ADD(LOG_UINT8, confidence, &humanConfidenceLog)
LOG_ADD(LOG_INT8, direction, &humanDirectionLog)
LOG_ADD(LOG_INT16, maxTemp, &humanMaxTempLog)
LOG_ADD(LOG_INT16, therm, &humanThermistorLog)
LOG_ADD(LOG_UINT32, timestampMs, &humanTimestampMsLog)
LOG_ADD(LOG_UINT32, rxLocalMs, &humanRxLocalTimeMsLog)
LOG_ADD(LOG_UINT32, testRxCount, &testRxCountLog)
LOG_ADD(LOG_UINT32, testTxCount, &testTxCountLog)
LOG_ADD(LOG_UINT32, beepCount, &beepCountLog)
LOG_ADD(LOG_UINT32, cmdRxCount, &cmdRxCountLog)
LOG_GROUP_STOP(espUart)
