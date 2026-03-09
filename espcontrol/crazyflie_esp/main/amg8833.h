#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define AMG8833_PIXEL_COUNT           64U
#define AMG8833_I2C_ADDRESS_DEFAULT   0x69U
#define AMG8833_I2C_ADDRESS_ALT       0x68U

esp_err_t amg8833_init(i2c_port_t i2c_port,
                       gpio_num_t sda_pin,
                       gpio_num_t scl_pin,
                       uint32_t clock_hz,
                       uint8_t preferred_address);

esp_err_t amg8833_read_pixels(float *pixels, size_t pixel_count);
esp_err_t amg8833_read_thermistor(float *thermistor_c);
uint8_t amg8833_get_i2c_address(void);
