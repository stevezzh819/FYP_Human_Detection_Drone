#include "uart_cf_comm.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#define UART_CF_RX_BUF_SIZE  1024
#define UART_CF_TX_BUF_SIZE  1024

typedef enum {
    rxWaitHeader0 = 0,
    rxWaitHeader1,
    rxWaitType,
    rxWaitLength,
    rxWaitPayload,
    rxWaitCrcLo,
    rxWaitCrcHi,
} uart_cf_rx_state_t;

static const char *TAG = "uart_cf_comm";

static uart_port_t s_uart_port = UART_NUM_1;
static bool s_is_init = false;
static float s_detection_threshold_c = 30.0f;

static uart_cf_rx_state_t s_rx_state = rxWaitHeader0;
static uint8_t s_rx_type = 0;
static uint8_t s_rx_length = 0;
static uint8_t s_rx_payload[UART_CF_MAX_DATA_LEN] = {0};
static uint8_t s_rx_index = 0;
static uint8_t s_rx_crc_lo = 0;

static uint16_t uart_cf_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void uart_cf_parser_reset(void)
{
    s_rx_state = rxWaitHeader0;
    s_rx_type = 0;
    s_rx_length = 0;
    s_rx_index = 0;
    s_rx_crc_lo = 0;
}

static bool uart_cf_consume_byte(uint8_t byte, uart_cf_packet_t *packet)
{
    switch (s_rx_state) {
        case rxWaitHeader0:
            if (byte == UART_CF_HEADER_BYTE0) {
                s_rx_state = rxWaitHeader1;
            }
            break;

        case rxWaitHeader1:
            if (byte == UART_CF_HEADER_BYTE1) {
                s_rx_state = rxWaitType;
            } else if (byte == UART_CF_HEADER_BYTE0) {
                s_rx_state = rxWaitHeader1;
            } else {
                s_rx_state = rxWaitHeader0;
            }
            break;

        case rxWaitType:
            s_rx_type = byte;
            s_rx_state = rxWaitLength;
            break;

        case rxWaitLength:
            s_rx_length = byte;
            if (s_rx_length > UART_CF_MAX_DATA_LEN) {
                ESP_LOGW(TAG, "Drop packet with invalid length=%u", s_rx_length);
                uart_cf_parser_reset();
            } else if (s_rx_length == 0U) {
                s_rx_state = rxWaitCrcLo;
            } else {
                s_rx_index = 0;
                s_rx_state = rxWaitPayload;
            }
            break;

        case rxWaitPayload:
            s_rx_payload[s_rx_index++] = byte;
            if (s_rx_index >= s_rx_length) {
                s_rx_state = rxWaitCrcLo;
            }
            break;

        case rxWaitCrcLo:
            s_rx_crc_lo = byte;
            s_rx_state = rxWaitCrcHi;
            break;

        case rxWaitCrcHi: {
            const uint16_t rx_crc = (uint16_t)s_rx_crc_lo | ((uint16_t)byte << 8U);

            uint8_t crc_buf[2 + UART_CF_MAX_DATA_LEN] = {0};
            crc_buf[0] = s_rx_type;
            crc_buf[1] = s_rx_length;
            if (s_rx_length > 0U) {
                memcpy(&crc_buf[2], s_rx_payload, s_rx_length);
            }
            const uint16_t calc_crc = uart_cf_crc16_ccitt(crc_buf, (size_t)2U + s_rx_length);

            if (rx_crc == calc_crc) {
                packet->type = s_rx_type;
                packet->length = s_rx_length;
                packet->crc = rx_crc;
                if (s_rx_length > 0U) {
                    memcpy(packet->data, s_rx_payload, s_rx_length);
                }
                uart_cf_parser_reset();
                return true;
            }

            ESP_LOGW(TAG, "CRC mismatch (rx=0x%04X calc=0x%04X)", rx_crc, calc_crc);
            uart_cf_parser_reset();
            break;
        }
    }

    return false;
}

esp_err_t uart_cf_init(uart_port_t uart_port,
                       int tx_pin,
                       int rx_pin,
                       int baudrate)
{
    s_uart_port = uart_port;

    const uart_config_t cfg = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(s_uart_port, &cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin failed");

    esp_err_t drv_err = uart_driver_install(s_uart_port, UART_CF_RX_BUF_SIZE, UART_CF_TX_BUF_SIZE, 0, NULL, 0);
    if (drv_err == ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(uart_driver_delete(s_uart_port), TAG, "uart_driver_delete failed");
        drv_err = uart_driver_install(s_uart_port, UART_CF_RX_BUF_SIZE, UART_CF_TX_BUF_SIZE, 0, NULL, 0);
    }
    ESP_RETURN_ON_ERROR(drv_err, TAG, "uart_driver_install failed");

    uart_cf_parser_reset();
    s_is_init = true;

    ESP_LOGI(TAG, "UART ready: port=%d tx=%d rx=%d baud=%d 8N1", s_uart_port, tx_pin, rx_pin, baudrate);
    return ESP_OK;
}

esp_err_t uart_cf_send_packet(uint8_t type,
                              const uint8_t *data,
                              uint8_t length)
{
    if (!s_is_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (length > UART_CF_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[2 + 1 + 1 + UART_CF_MAX_DATA_LEN + 2] = {0};
    size_t offset = 0;

    frame[offset++] = UART_CF_HEADER_BYTE0;
    frame[offset++] = UART_CF_HEADER_BYTE1;
    frame[offset++] = type;
    frame[offset++] = length;

    if ((length > 0U) && (data != NULL)) {
        memcpy(&frame[offset], data, length);
    }
    offset += length;

    const uint16_t crc = uart_cf_crc16_ccitt(&frame[2], (size_t)2U + length);
    frame[offset++] = (uint8_t)(crc & 0xFFU);
    frame[offset++] = (uint8_t)((crc >> 8U) & 0xFFU);

    const int written = uart_write_bytes(s_uart_port, (const char *)frame, offset);
    if (written != (int)offset) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool uart_cf_receive_packet(uart_cf_packet_t *packet, TickType_t timeout_ticks)
{
    if ((packet == NULL) || !s_is_init) {
        return false;
    }

    uint8_t byte = 0;
    int bytes = uart_read_bytes(s_uart_port, &byte, 1, timeout_ticks);
    while (bytes > 0) {
        if (uart_cf_consume_byte(byte, packet)) {
            return true;
        }
        bytes = uart_read_bytes(s_uart_port, &byte, 1, 0);
    }

    return false;
}

void uart_cf_parse_message(const uart_cf_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    if (packet->type != UART_CF_TYPE_COMMAND) {
        return;
    }

    if (packet->length < 1U) {
        return;
    }

    const uint8_t cmd = packet->data[0];

    if ((cmd == UART_CF_CMD_SET_THRESHOLD) && (packet->length >= 3U)) {
        const int16_t threshold_x10 = (int16_t)((uint16_t)packet->data[1] | ((uint16_t)packet->data[2] << 8U));
        const float threshold_c = ((float)threshold_x10) / 10.0f;

        if ((threshold_c >= 15.0f) && (threshold_c <= 80.0f)) {
            s_detection_threshold_c = threshold_c;
            ESP_LOGI(TAG, "Updated human-detection threshold to %.1fC", s_detection_threshold_c);
        } else {
            ESP_LOGW(TAG, "Ignoring out-of-range threshold command: %.1fC", threshold_c);
        }
    }
}

float uart_cf_get_detection_threshold_c(void)
{
    return s_detection_threshold_c;
}
