/*
 * ESP32-C3 application architecture:
 *   1) AMG8833 thermal sensor over I2C (ESP32 master, AMG8833 slave at 0x69/0x68)
 *   2) Crazyflie STM32 communication over UART (bidirectional packet link)
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "amg8833.h"
#include "uart_cf_comm.h"

#define AMG_I2C_PORT                 I2C_NUM_0
#define AMG_I2C_SDA_PIN              GPIO_NUM_8
#define AMG_I2C_SCL_PIN              GPIO_NUM_9
#define AMG_I2C_CLOCK_HZ             400000U
#define AMG_I2C_PREFERRED_ADDRESS    AMG8833_I2C_ADDRESS_DEFAULT

#define CF_UART_PORT                 UART_NUM_1
#define CF_UART_TX_PIN               GPIO_NUM_20
#define CF_UART_RX_PIN               GPIO_NUM_21
#define CF_UART_BAUDRATE             115200

#define MAIN_LOOP_PERIOD_MS          100U
#define UART_TEST_PERIOD_MS          1000U
#define UART_TEST_TX_ENABLE          0U
#define UART_AUTO_FLIGHT_TEST_ENABLE 1U
#define UART_AUTO_TAKEOFF_DELAY_MS   3000U
#define UART_AUTO_TAKEOFF_HEIGHT_CM  30U
#define UART_AUTO_TAKEOFF_DUR_MS     2000U

static const char *UART_TEST_FROM_ESP32_PREFIX = "UART_TEST_FROM_ESP32";

typedef struct {
    bool human_detected;
    uint8_t confidence;
    float max_temp_c;
    float avg_temp_c;
} inference_result_t;

static const char *TAG = "cf_esp_app";

static void send_flight_command(uint8_t command_id, uint16_t height_cm, uint16_t duration_ms)
{
    uint8_t payload[5] = {0};
    payload[0] = command_id;
    payload[1] = (uint8_t)(height_cm & 0xFFU);
    payload[2] = (uint8_t)((height_cm >> 8U) & 0xFFU);
    payload[3] = (uint8_t)(duration_ms & 0xFFU);
    payload[4] = (uint8_t)((duration_ms >> 8U) & 0xFFU);

    const esp_err_t err = uart_cf_send_packet(UART_CF_TYPE_COMMAND, payload, sizeof(payload));
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "UART FLIGHT CMD TX -> Crazyflie: cmd=0x%02X height=%ucm duration=%ums",
                 command_id,
                 (unsigned)height_cm,
                 (unsigned)duration_ms);
    } else {
        ESP_LOGW(TAG, "UART FLIGHT CMD TX failed (cmd=0x%02X): %s", command_id, esp_err_to_name(err));
    }
}

static inference_result_t run_human_inference(const float *pixels, size_t count, float threshold_c)
{
    inference_result_t result = {0};
    if ((pixels == NULL) || (count == 0U)) {
        return result;
    }

    float max_temp = pixels[0];
    float sum_temp = 0.0f;
    for (size_t i = 0; i < count; i++) {
        if (pixels[i] > max_temp) {
            max_temp = pixels[i];
        }
        sum_temp += pixels[i];
    }

    const float avg_temp = sum_temp / (float)count;
    const float hotspot_delta = max_temp - avg_temp;

    // Lightweight onboard inference placeholder:
    // detect a person-like hotspot when absolute heat and local contrast are both high.
    result.human_detected = (max_temp >= threshold_c) && (hotspot_delta >= 2.0f);
    result.max_temp_c = max_temp;
    result.avg_temp_c = avg_temp;

    float score = ((max_temp - threshold_c) * 8.0f) + (hotspot_delta * 12.0f);
    if (score < 0.0f) {
        score = 0.0f;
    }
    if (score > 100.0f) {
        score = 100.0f;
    }
    result.confidence = (uint8_t)lrintf(score);

    return result;
}

static void send_detection_to_crazyflie(const inference_result_t *result, float thermistor_c)
{
    if (result == NULL) {
        return;
    }

    const uint32_t timestamp_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    const int16_t max_temp_x100 = (int16_t)lrintf(result->max_temp_c * 100.0f);
    const int16_t thermistor_x100 = (int16_t)lrintf(thermistor_c * 100.0f);

    uint8_t payload[10] = {0};
    payload[0] = result->human_detected ? 1U : 0U;
    payload[1] = result->confidence;
    payload[2] = (uint8_t)(max_temp_x100 & 0xFF);
    payload[3] = (uint8_t)((max_temp_x100 >> 8) & 0xFF);
    payload[4] = (uint8_t)(thermistor_x100 & 0xFF);
    payload[5] = (uint8_t)((thermistor_x100 >> 8) & 0xFF);
    payload[6] = (uint8_t)(timestamp_ms & 0xFF);
    payload[7] = (uint8_t)((timestamp_ms >> 8) & 0xFF);
    payload[8] = (uint8_t)((timestamp_ms >> 16) & 0xFF);
    payload[9] = (uint8_t)((timestamp_ms >> 24) & 0xFF);

    const esp_err_t err = uart_cf_send_packet(UART_CF_TYPE_HUMAN_DETECT, payload, sizeof(payload));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_cf_send_packet failed: %s", esp_err_to_name(err));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 dual-protocol firmware starting");

    ESP_ERROR_CHECK(uart_cf_init(
        CF_UART_PORT,
        CF_UART_TX_PIN,
        CF_UART_RX_PIN,
        CF_UART_BAUDRATE));

    ESP_LOGI(TAG,
             "Crazyflie UART config: port=%d TX=%d RX=%d baud=%d 8N1",
             CF_UART_PORT,
             CF_UART_TX_PIN,
             CF_UART_RX_PIN,
             CF_UART_BAUDRATE);

    bool amg_available = false;
    const esp_err_t amg_init_err = amg8833_init(
        AMG_I2C_PORT,
        AMG_I2C_SDA_PIN,
        AMG_I2C_SCL_PIN,
        AMG_I2C_CLOCK_HZ,
        AMG_I2C_PREFERRED_ADDRESS);
    if (amg_init_err == ESP_OK) {
        amg_available = true;
        ESP_LOGI(TAG,
                 "AMG8833 I2C master config: SDA=%d SCL=%d clk=%luHz addr=0x%02X",
                 AMG_I2C_SDA_PIN,
                 AMG_I2C_SCL_PIN,
                 (unsigned long)AMG_I2C_CLOCK_HZ,
                 amg8833_get_i2c_address());
    } else {
        ESP_LOGW(TAG,
                 "AMG8833 init skipped for UART-only bring-up (%s). I2C sensor path disabled until sensor is connected.",
                 esp_err_to_name(amg_init_err));
    }

    float pixels[AMG8833_PIXEL_COUNT] = {0};
    int64_t last_log_ms = 0;
    int64_t last_uart_test_tx_ms = 0;
    int64_t last_uart_rx_stats_ms = 0;
    uint32_t uart_test_seq = 0;
    bool demo_takeoff_sent = false;
    int64_t demo_start_ms = -1;

    while (true) {
        // Process any incoming Crazyflie UART commands first (non-blocking).
        uart_cf_packet_t rx_packet = {0};
        while (uart_cf_receive_packet(&rx_packet, 0)) {
            if (rx_packet.type == UART_CF_TYPE_TEST_TEXT) {
                char test_rx[UART_CF_MAX_DATA_LEN + 1] = {0};
                const size_t copy_len = (rx_packet.length < UART_CF_MAX_DATA_LEN)
                                            ? rx_packet.length
                                            : UART_CF_MAX_DATA_LEN;
                if (copy_len > 0U) {
                    memcpy(test_rx, rx_packet.data, copy_len);
                }
                test_rx[copy_len] = '\0';
                ESP_LOGI(TAG, "UART TEST RX <- Crazyflie: %s", test_rx);
            } else {
                uart_cf_parse_message(&rx_packet);
            }
        }

        const int64_t now_ms = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        if ((now_ms - last_uart_rx_stats_ms) >= 1000) {
            ESP_LOGI(TAG,
                     "UART RX stats: bytes=%lu packets=%lu",
                     (unsigned long)uart_cf_get_rx_byte_count(),
                     (unsigned long)uart_cf_get_rx_packet_count());
            last_uart_rx_stats_ms = now_ms;
        }

        if ((UART_AUTO_FLIGHT_TEST_ENABLE != 0U) && !demo_takeoff_sent) {
            if (demo_start_ms < 0) {
                demo_start_ms = now_ms;
            }

            // Battery endurance mode: send a single takeoff command only.
            if ((now_ms - demo_start_ms) >= UART_AUTO_TAKEOFF_DELAY_MS) {
                send_flight_command(UART_CF_CMD_TAKEOFF, UART_AUTO_TAKEOFF_HEIGHT_CM, UART_AUTO_TAKEOFF_DUR_MS);
                demo_takeoff_sent = true;
            }
        }

        if ((UART_TEST_TX_ENABLE != 0U) && ((now_ms - last_uart_test_tx_ms) >= UART_TEST_PERIOD_MS)) {
            char test_msg[UART_CF_MAX_DATA_LEN + 1] = {0};
            const int test_len_i = snprintf(test_msg,
                                            sizeof(test_msg),
                                            "%s:%lu",
                                            UART_TEST_FROM_ESP32_PREFIX,
                                            (unsigned long)uart_test_seq++);
            const size_t test_len = (test_len_i > 0) ? (size_t)test_len_i : 0U;
            const esp_err_t test_err = uart_cf_send_packet(
                UART_CF_TYPE_TEST_TEXT,
                (const uint8_t *)test_msg,
                (uint8_t)test_len);
            if (test_err == ESP_OK) {
                ESP_LOGI(TAG, "UART TEST TX -> Crazyflie: %s", test_msg);
            } else {
                ESP_LOGW(TAG, "UART TEST TX failed: %s", esp_err_to_name(test_err));
            }
            last_uart_test_tx_ms = now_ms;
        }

        if (amg_available) {
            float thermistor_c = 0.0f;
            esp_err_t read_err = amg8833_read_pixels(pixels, AMG8833_PIXEL_COUNT);
            if (read_err == ESP_OK) {
                read_err = amg8833_read_thermistor(&thermistor_c);
            }

            if (read_err == ESP_OK) {
                const float threshold_c = uart_cf_get_detection_threshold_c();
                const inference_result_t result = run_human_inference(pixels, AMG8833_PIXEL_COUNT, threshold_c);
                send_detection_to_crazyflie(&result, thermistor_c);

                if ((now_ms - last_log_ms) >= 1000) {
                    ESP_LOGI(TAG,
                             "Thermal inference: detect=%u conf=%u%% max=%.2fC avg=%.2fC therm=%.2fC threshold=%.1fC",
                             result.human_detected ? 1U : 0U,
                             result.confidence,
                             result.max_temp_c,
                             result.avg_temp_c,
                             thermistor_c,
                             threshold_c);
                    last_log_ms = now_ms;
                }
            } else {
                ESP_LOGW(TAG, "AMG8833 read failed: %s", esp_err_to_name(read_err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
    }
}
