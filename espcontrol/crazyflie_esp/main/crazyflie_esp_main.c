/*
 * Crazyflie telemetry I2C slave bridge for ESP32-C3.
 *
 * Wiring (ESP32-C3 ↔ Crazyflie 2.x):
 *   - SDA  (GPIO8)  ←→ SDA
 *   - SCL  (GPIO9)  ←→ SCL
 *   - GND  ↔ GND (shared)
 *   Bus speed: 100 kHz. Crazyflie acts as I²C master, ESP32-C3 as slave at address 0x28.
 *
 * Packet format (24 bytes, little-endian):
 *   0  : uint8  proto_ver
 *   1  : uint8  flags (bit0=valid_est, bit1=valid_ranger)
 *   2  : uint32 time_us
 *   6  : int16  yaw_mrad
 *   8  : int16  vx_mmps
 *   10 : int16  vy_mmps
 *   12 : uint16 rng_mm[5] (front, back, left, right, up)
 *   22 : uint16 crc16_ccitt (bytes 0..21)
 *
 * TODO: Crazyflie firmware must perform I²C master writes of exactly 24 bytes at 50–100 Hz.
 *
 * This module configures I2C_NUM_0 as an I²C slave on the ESP32-C3, parses incoming frames,
 * validates CRC16-CCITT, and exposes the latest decoded telemetry sample to other tasks.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define CF_I2C_PORT              I2C_NUM_0
#define CF_I2C_SDA_GPIO          GPIO_NUM_8
#define CF_I2C_SCL_GPIO          GPIO_NUM_9
#define CF_I2C_SLAVE_ADDRESS     0x28
#define CF_I2C_SLAVE_RX_BUF_LEN  512     // Provide ample room for bursts and in-flight transfers.

#define CF_FRAME_SIZE_BYTES      24
#define CF_PAYLOAD_LEN_BYTES     22      // Bytes covered by CRC16.
#define CF_EXPECTED_PROTO_VER    1
#define CF_FLAGS_VALID_MASK      0x03
#define CF_SAMPLE_EVENT_READY_BIT BIT0

#define CF_RX_TASK_STACK_BYTES   4096
#define CF_RX_TASK_PRIORITY      5

static const char *TAG = "cf_i2c_slave";

typedef struct {
    uint8_t  proto_ver;
    uint8_t  flags;
    uint32_t time_us;
    int16_t  yaw_mrad;
    int16_t  vx_mmps;
    int16_t  vy_mmps;
    uint16_t rng_mm[5];  // F, B, L, R, U
} cf_telemetry_t;

static SemaphoreHandle_t s_sample_mutex = NULL;
static EventGroupHandle_t s_sample_events = NULL;
static volatile bool s_sample_valid = false;
static cf_telemetry_t s_latest_sample = {0};
static bool s_logged_first_byte = false;
static int64_t s_last_sample_log_us = 0;
static int64_t s_last_crc_log_us = 0;

static uint16_t cf_crc16_ccitt(const uint8_t *data, size_t len);
static uint16_t read_le_u16(const uint8_t *p);
static int16_t read_le_i16(const uint8_t *p);
static uint32_t read_le_u32(const uint8_t *p);
static void cf_handle_frame(const uint8_t *frame);
static void cf_i2c_rx_task(void *arg);

bool cf_telemetry_get_latest(cf_telemetry_t *out, TickType_t timeout_ticks);

/***********************************************************************************************************************
 * CRC16-CCITT (poly 0x1021, init 0xFFFF, reflect off, xorout 0x0000)
 ***********************************************************************************************************************/
static uint16_t cf_crc16_ccitt(const uint8_t *data, size_t len)
{
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

/***********************************************************************************************************************
 * Little-endian helpers
 ***********************************************************************************************************************/
static uint16_t read_le_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_le_i16(const uint8_t *p)
{
    return (int16_t)read_le_u16(p);
}

static uint32_t read_le_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/***********************************************************************************************************************
 * Frame processing (called from RX task when 24-byte frame assembled)
 ***********************************************************************************************************************/
static void cf_handle_frame(const uint8_t *frame)
{
    const uint16_t crc_expected = read_le_u16(&frame[CF_PAYLOAD_LEN_BYTES]);
    const uint16_t crc_actual = cf_crc16_ccitt(frame, CF_PAYLOAD_LEN_BYTES);

    if (crc_expected != crc_actual) {
    const int64_t now_us = esp_timer_get_time();
    if ((s_last_crc_log_us == 0) || ((now_us - s_last_crc_log_us) > 500000)) {
        ESP_LOGW(TAG, "CRC mismatch (calc=0x%04X, frame=0x%04X) – dropping frame", crc_actual, crc_expected);
        s_last_crc_log_us = now_us;
    } else {
        ESP_LOGD(TAG, "CRC mismatch (calc=0x%04X, frame=0x%04X)", crc_actual, crc_expected);
    }
    return;
  }

    cf_telemetry_t sample = {
        .proto_ver = frame[0],
        .flags = frame[1],
        .time_us = read_le_u32(&frame[2]),
        .yaw_mrad = read_le_i16(&frame[6]),
        .vx_mmps = read_le_i16(&frame[8]),
        .vy_mmps = read_le_i16(&frame[10]),
    };

    for (size_t i = 0; i < 5; ++i) {
        sample.rng_mm[i] = read_le_u16(&frame[12 + (i * 2)]);
    }

    if (sample.proto_ver != CF_EXPECTED_PROTO_VER) {
        ESP_LOGW(TAG, "Unexpected proto_ver=%u (expected %u) – accepting frame anyway",
                 sample.proto_ver, CF_EXPECTED_PROTO_VER);
    }

    if ((sample.flags & ~CF_FLAGS_VALID_MASK) != 0) {
        ESP_LOGW(TAG, "Reserved flag bits set (flags=0x%02X) – accepting frame anyway", sample.flags);
    }

    if (xSemaphoreTake(s_sample_mutex, portMAX_DELAY) == pdTRUE) {
        s_latest_sample = sample;
        s_sample_valid = true;
        xSemaphoreGive(s_sample_mutex);
        (void)xEventGroupSetBits(s_sample_events, CF_SAMPLE_EVENT_READY_BIT);
    } else {
        ESP_LOGE(TAG, "Failed to take sample mutex – dropping frame");
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_sample_log_us) >= 200000) {  // ~5 Hz
        ESP_LOGI(TAG,
                 "CF: t=%" PRIu32 " us | yaw=%+d mrad | vx=%+d vy=%+d mm/s | "
                 "rng=[F:%u,B:%u,L:%u,R:%u,U:%u] flags=0x%02X",
                 sample.time_us,
                 sample.yaw_mrad,
                 sample.vx_mmps,
                 sample.vy_mmps,
                 sample.rng_mm[0],
                 sample.rng_mm[1],
                 sample.rng_mm[2],
                 sample.rng_mm[3],
                 sample.rng_mm[4],
                 sample.flags);
        s_last_sample_log_us = now_us;
    } else {
        ESP_LOGD(TAG,
                 "CF: t=%" PRIu32 " us | yaw=%+d mrad | vx=%+d vy=%+d mm/s | "
                 "rng=[F:%u,B:%u,L:%u,R:%u,U:%u] flags=0x%02X",
                 sample.time_us,
                 sample.yaw_mrad,
                 sample.vx_mmps,
                 sample.vy_mmps,
                 sample.rng_mm[0],
                 sample.rng_mm[1],
                 sample.rng_mm[2],
                 sample.rng_mm[3],
                 sample.rng_mm[4],
                 sample.flags);
    }
}

/***********************************************************************************************************************
 * I²C Rx Task – reassembles fixed-size frames from the slave RX FIFO.
 ***********************************************************************************************************************/
static void cf_i2c_rx_task(void *arg)
{
    uint8_t assembly_buf[CF_FRAME_SIZE_BYTES] = {0};
    size_t assembly_len = 0;
    uint8_t chunk_buf[CF_FRAME_SIZE_BYTES] = {0};

    ESP_LOGI(TAG, "I²C RX task started");

    while (true) {
        const int bytes_read = i2c_slave_read_buffer(
            CF_I2C_PORT, chunk_buf, sizeof(chunk_buf), pdMS_TO_TICKS(50));

        if (bytes_read < 0) {
            ESP_LOGW(TAG, "i2c_slave_read_buffer error (%d)", bytes_read);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read == 0) {
            // No data this interval – keep waiting.
            continue;
        }

        if (!s_logged_first_byte) {
            ESP_LOGI(TAG, "First I2C bytes received (%d)", bytes_read);
            s_logged_first_byte = true;
        }
        ESP_LOGD(TAG, "I2C RX got %d byte(s)", bytes_read);

        size_t processed = 0;
        while (processed < (size_t)bytes_read) {
            const size_t remaining_frame = CF_FRAME_SIZE_BYTES - assembly_len;
            const size_t remaining_chunk = (size_t)bytes_read - processed;
            const size_t to_copy = remaining_chunk < remaining_frame ? remaining_chunk : remaining_frame;

            memcpy(&assembly_buf[assembly_len], &chunk_buf[processed], to_copy);
            assembly_len += to_copy;
            processed += to_copy;

            if (assembly_len == CF_FRAME_SIZE_BYTES) {
                cf_handle_frame(assembly_buf);
                assembly_len = 0;
            }
        }

        if (assembly_len > CF_FRAME_SIZE_BYTES) {
            // Should never happen, but guard against buffer overruns.
            ESP_LOGW(TAG, "Assembly overflow (%u) – resynchronizing", (unsigned)assembly_len);
            assembly_len = 0;
        }
    }
}

/***********************************************************************************************************************
 * Public API – retrieve latest telemetry sample.
 ***********************************************************************************************************************/
bool cf_telemetry_get_latest(cf_telemetry_t *out, TickType_t timeout_ticks)
{
    if (!out) {
        return false;
    }

    if (!s_sample_valid) {
        const EventBits_t bits = xEventGroupWaitBits(
            s_sample_events, CF_SAMPLE_EVENT_READY_BIT, pdFALSE, pdFALSE, timeout_ticks);
        if ((bits & CF_SAMPLE_EVENT_READY_BIT) == 0) {
            return false;  // Timed out waiting for the first valid sample.
        }
    }

    if (xSemaphoreTake(s_sample_mutex, timeout_ticks) != pdTRUE) {
        return false;
    }

    const bool valid = s_sample_valid;
    if (valid) {
        *out = s_latest_sample;
    }
    xSemaphoreGive(s_sample_mutex);
    return valid;
}

/***********************************************************************************************************************
 * I²C slave initialization
 ***********************************************************************************************************************/
static esp_err_t cf_i2c_slave_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = CF_I2C_SDA_GPIO,
        .scl_io_num = CF_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = CF_I2C_SLAVE_ADDRESS,
        },
        .clk_flags = 0,  // Not used in slave mode; master drives the clock (100 kHz on Crazyflie side).
    };

    ESP_ERROR_CHECK(i2c_param_config(CF_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(
        CF_I2C_PORT,
        I2C_MODE_SLAVE,
        CF_I2C_SLAVE_RX_BUF_LEN,
        0,   // No TX buffer needed for slave.
        0));
    ESP_ERROR_CHECK(i2c_reset_rx_fifo(CF_I2C_PORT));

    ESP_LOGI(TAG, "Configured I²C slave: port=%d addr=0x%02X SDA=GPIO%d SCL=GPIO%d (Crazyflie master @100 kHz)",
             CF_I2C_PORT,
             CF_I2C_SLAVE_ADDRESS,
             CF_I2C_SDA_GPIO,
             CF_I2C_SCL_GPIO);

    return ESP_OK;
}

/***********************************************************************************************************************
 * app_main – entry point
 ***********************************************************************************************************************/
void app_main(void)
{
    ESP_LOGI(TAG, "==== Crazyflie ↔ ESP32-C3 I²C telemetry bridge starting ====");
    ESP_LOGI(TAG, "Pinout: SDA=GPIO%d, SCL=GPIO%d | Slave address 0x%02X | Target: ESP32-C3",
             CF_I2C_SDA_GPIO,
             CF_I2C_SCL_GPIO,
             CF_I2C_SLAVE_ADDRESS);

    s_sample_mutex = xSemaphoreCreateMutex();
    configASSERT(s_sample_mutex != NULL);

    s_sample_events = xEventGroupCreate();
    configASSERT(s_sample_events != NULL);

    ESP_ERROR_CHECK(cf_i2c_slave_init());

    BaseType_t task_ok = xTaskCreatePinnedToCore(
        cf_i2c_rx_task,
        "cf_i2c_rx",
        CF_RX_TASK_STACK_BYTES,
        NULL,
        CF_RX_TASK_PRIORITY,
        NULL,
        tskNO_AFFINITY);
    configASSERT(task_ok == pdPASS);

    while (true) {
        cf_telemetry_t sample;
        const bool got_sample = cf_telemetry_get_latest(&sample, pdMS_TO_TICKS(1000));
        if (!got_sample) {
            ESP_LOGW(TAG, "Heartbeat: no telemetry frames received in the last second");
        }
        // Sleep regardless – either we waited inside cf_telemetry_get_latest or we just sleep briefly.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
