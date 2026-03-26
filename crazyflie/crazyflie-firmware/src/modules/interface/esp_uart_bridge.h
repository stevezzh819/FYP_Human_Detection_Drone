/**
 * Crazyflie <-UART-> ESP32 human-detection bridge
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ESP_UART_BRIDGE_TASK_STACKWORDS  (1024)
#define ESP_UART_BRIDGE_TASK_PRI         (2)

typedef struct {
  bool detected;
  uint8_t confidence;
  int8_t direction;
  int16_t maxTempX100;
  int16_t thermistorX100;
  uint32_t sourceTimestampMs;
  uint32_t rxLocalTimeMs;
} espUartHumanDetection_t;

bool espUartBridgeInit(void);
void espUartBridgeTask(void *param);

bool espUartBridgeGetLatestHumanDetectionSample(espUartHumanDetection_t *sample);

bool espUartBridgeGetLatestHumanDetection(bool *detected,
                                          uint8_t *confidence,
                                          uint32_t *timestamp_ms);

bool espUartBridgeSendPacket(uint8_t type, const uint8_t *payload, uint8_t length);
