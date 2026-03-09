/**
 * Crazyflie <-UART-> ESP32 human-detection bridge
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ESP_UART_BRIDGE_TASK_STACKWORDS  (1024)
#define ESP_UART_BRIDGE_TASK_PRI         (2)

bool espUartBridgeInit(void);
void espUartBridgeTask(void *param);

bool espUartBridgeGetLatestHumanDetection(bool *detected,
                                          uint8_t *confidence,
                                          uint32_t *timestamp_ms);

bool espUartBridgeSendPacket(uint8_t type, const uint8_t *payload, uint8_t length);
