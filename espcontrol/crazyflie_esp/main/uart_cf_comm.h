#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define UART_CF_HEADER_BYTE0       0xAAU
#define UART_CF_HEADER_BYTE1       0x55U
#define UART_CF_MAX_DATA_LEN       64U

#define UART_CF_TYPE_HUMAN_DETECT  0x01U
#define UART_CF_TYPE_COMMAND       0x10U
#define UART_CF_TYPE_TEST_TEXT     0x7EU

#define UART_CF_CMD_SET_THRESHOLD  0x01U
#define UART_CF_CMD_TAKEOFF        0x20U
#define UART_CF_CMD_LAND           0x21U

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t data[UART_CF_MAX_DATA_LEN];
    uint16_t crc;
} uart_cf_packet_t;

esp_err_t uart_cf_init(uart_port_t uart_port,
                       int tx_pin,
                       int rx_pin,
                       int baudrate);

esp_err_t uart_cf_send_packet(uint8_t type,
                              const uint8_t *data,
                              uint8_t length);

bool uart_cf_receive_packet(uart_cf_packet_t *packet, TickType_t timeout_ticks);
void uart_cf_parse_message(const uart_cf_packet_t *packet);

float uart_cf_get_detection_threshold_c(void);
uint32_t uart_cf_get_rx_byte_count(void);
uint32_t uart_cf_get_rx_packet_count(void);
