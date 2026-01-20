/**
 * Multiranger deck access helpers.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  multirangerFront = 0,
  multirangerBack,
  multirangerLeft,
  multirangerRight,
  multirangerUp,
} multirangerSensor_t;

bool multirangerIsInit(void);
uint16_t multirangerGetDistance(multirangerSensor_t sensor);
