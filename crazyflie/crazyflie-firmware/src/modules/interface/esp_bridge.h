/**
 * Crazyflie ↔ ESP32 telemetry bridge
 *
 * Streams state estimate and ranger data over the deck I2C bus to an external
 * ESP32-C3 slave at address 0x28.
 */

#pragma once

#include <stdbool.h>

#include "stabilizer_types.h"

#define BRIDGE_STACK_WORDS  (1024)
#define BRIDGE_PRIO         (2)

#define ESP_BRIDGE_TASK_STACKWORDS  BRIDGE_STACK_WORDS
#define ESP_BRIDGE_TASK_PRI          BRIDGE_PRIO

bool espBridgeInit(void);
void espBridgeTask(void *param);
void espBridgeFeedState(const state_t *state);
