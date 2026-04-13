#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_err.h"

#include "amg8833.h"
#include "uart_cf_comm.h"

#define I2C_SDA_PIN                  8
#define I2C_SCL_PIN                  9
#define I2C_FREQ                     400000
#define AMG_ADDR                     0x69

#define CF_UART_PORT                 UART_NUM_1
#define CF_UART_TX_PIN               GPIO_NUM_20
#define CF_UART_RX_PIN               GPIO_NUM_21
#define CF_UART_BAUDRATE             57600

#define STATUS_LED_PIN               GPIO_NUM_7
#define STATUS_LED_BLINK_MS          200U

#define MAIN_LOOP_PERIOD_MS          100U
#define HEATMAP_DEBUG_PERIOD_MS      100U
#define UART_RX_STATS_PERIOD_MS      1000U

typedef struct {
    int size;
    int min_r;
    int max_r;
    int min_c;
    int max_c;
    float c_sum;
    float t_sum;
    float t_max;
    float t_sq_sum;
} blob_stats_t;

typedef struct {
    bool human_detected;
    uint8_t confidence;
    float max_temp_c;
    float avg_temp_c;
    float ambient_c;
    int blob_size;
    int8_t human_dir;
} inference_result_t;

static const char *TAG = "cf_esp_app";
static bool status_led_on = false;
static int64_t status_led_last_toggle_ms = 0;

static void status_led_init(void)
{
    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_PIN, 0);
    status_led_on = false;
    status_led_last_toggle_ms = 0;
}

static void status_led_update(bool human_detected, int64_t now_ms)
{
    if (!human_detected) {
        if (status_led_on) {
            gpio_set_level(STATUS_LED_PIN, 0);
            status_led_on = false;
        }
        status_led_last_toggle_ms = now_ms;
        return;
    }

    if ((now_ms - status_led_last_toggle_ms) >= STATUS_LED_BLINK_MS) {
        status_led_on = !status_led_on;
        gpio_set_level(STATUS_LED_PIN, status_led_on ? 1 : 0);
        status_led_last_toggle_ms = now_ms;
    }
}

static blob_stats_t largest_blob_stats(const bool *mask, const float *pix)
{
    bool vis[64] = {0};
    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    blob_stats_t best;
    memset(&best, 0, sizeof(best));
    best.min_r = best.min_c = 99;
    best.max_r = best.max_c = -99;
    best.t_max = -1e9f;

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            const int k = (r * 8) + c;
            if (!mask[k] || vis[k]) {
                continue;
            }

            int qR[64];
            int qC[64];
            int qh = 0;
            int qt = 0;
            blob_stats_t cur;
            memset(&cur, 0, sizeof(cur));
            cur.min_r = cur.min_c = 99;
            cur.max_r = cur.max_c = -99;
            cur.t_max = -1e9f;

            vis[k] = true;
            qR[qt] = r;
            qC[qt] = c;
            qt++;

            while (qh < qt) {
                const int rr = qR[qh];
                const int cc = qC[qh];
                qh++;
                const int idx = (rr * 8) + cc;

                cur.size++;
                if (rr < cur.min_r) cur.min_r = rr;
                if (rr > cur.max_r) cur.max_r = rr;
                if (cc < cur.min_c) cur.min_c = cc;
                if (cc > cur.max_c) cur.max_c = cc;

                const float t = pix[idx];
                cur.c_sum += (float)cc;
                cur.t_sum += t;
                cur.t_sq_sum += t * t;
                if (t > cur.t_max) cur.t_max = t;

                for (int i = 0; i < 8; i++) {
                    const int nr = rr + dr[i];
                    const int nc = cc + dc[i];
                    if (nr < 0 || nr >= 8 || nc < 0 || nc >= 8) {
                        continue;
                    }
                    const int nk = (nr * 8) + nc;
                    if (mask[nk] && !vis[nk]) {
                        vis[nk] = true;
                        qR[qt] = nr;
                        qC[qt] = nc;
                        qt++;
                    }
                }
            }

            if (cur.size > best.size) {
                best = cur;
            }
        }
    }

    return best;
}

static bool is_human_1m(const blob_stats_t *b, float amb)
{
    if (b->size == 0) {
        return false;
    }

    float mean = b->t_sum / (float)b->size;
    float var = (b->t_sq_sum / (float)b->size) - (mean * mean);
    if (var < 0) {
        var = 0;
    }
    const float std = sqrtf(var);
    const float tmax = b->t_max;

    if (tmax < amb + 3.0f) return false;
    if (mean < amb + 2.0f) return false;
    if (tmax > 33.0f || tmax < 24.5f) return false;
    if (b->size < 15) return false;
    if (b->size > 50) return false;
    if (std <= 0.45f) return false;
    return true;
}

static int cmp_float(const void *a, const void *b)
{
    const float fa = *(const float *)a;
    const float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float ambient_lowk(const float pix[64], int k)
{
    float tmp[64];
    memcpy(tmp, pix, sizeof(tmp));
    qsort(tmp, 64, sizeof(float), cmp_float);
    if (k < 1) k = 1;
    if (k > 64) k = 64;
    float s = 0.0f;
    for (int i = 0; i < k; i++) {
        s += tmp[i];
    }
    return s / (float)k;
}

static inference_result_t run_human_inference(const float pix[64], float bg[64], float thermistor_c)
{
    inference_result_t result = {0};
    bool mask[64];

    static int hits = 0;
    static int misses = 0;
    static bool occupied = false;
    static int h_on = 0;
    static int h_off = 0;
    static bool human_present = false;

    const float alpha = 0.02f;
    const float delta_abs = 2.0f;
    const float delta_rel = 2.0f;

    float max_temp = pix[0];
    float sum_temp = 0.0f;
    for (int i = 0; i < 64; i++) {
        if (pix[i] > max_temp) {
            max_temp = pix[i];
        }
        sum_temp += pix[i];
    }

    const float amb_th = thermistor_c;
    const float amb_low = ambient_lowk(pix, 10);
    int above_th = 0;
    for (int i = 0; i < 64; i++) {
        if (pix[i] > amb_th) {
            above_th++;
        }
    }

    float amb;
    if (amb_low > amb_th + 0.5f || above_th > 48) {
        amb = amb_th;
    } else {
        amb = amb_low;
    }

    for (int i = 0; i < 64; i++) {
        const bool hot_vs_amb = pix[i] >= amb + delta_abs;
        const bool hot_vs_bg = pix[i] >= bg[i] + delta_rel;
        mask[i] = (hot_vs_amb || hot_vs_bg);
        if (!mask[i]) {
            bg[i] = (1.0f - alpha) * bg[i] + alpha * pix[i];
        }
    }

    const blob_stats_t b = largest_blob_stats(mask, pix);
    const bool det = (b.size >= 4);
    if (det) {
        hits++;
        misses = 0;
    } else {
        misses++;
        hits = 0;
    }
    if (!occupied && hits >= 2) occupied = true;
    if (occupied && misses >= 6) occupied = false;

    const bool human_now = is_human_1m(&b, amb);
    if (human_now) {
        h_on++;
        h_off = 0;
    } else {
        h_off++;
        h_on = 0;
    }
    if (!human_present && h_on >= 2) human_present = true;
    if (human_present && h_off >= 6) human_present = false;

    result.human_detected = human_present;
    result.max_temp_c = max_temp;
    result.avg_temp_c = sum_temp / 64.0f;
    result.ambient_c = amb;
    result.blob_size = b.size;
    result.human_dir = 0;

    if (b.size > 0) {
        const float blob_mean = b.t_sum / (float)b.size;
        float score = ((b.t_max - amb) * 18.0f) + ((blob_mean - amb) * 12.0f);
        if (b.size >= 6) {
            score += 10.0f;
        }
        if (score < 0.0f) score = 0.0f;
        if (score > 100.0f) score = 100.0f;
        result.confidence = (uint8_t)lrintf(score);

        if (human_present) {
            const float center_c = b.c_sum / (float)b.size;
            if (center_c <= 2.5f) {
                result.human_dir = -1;
            } else if (center_c >= 4.5f) {
                result.human_dir = 1;
            }
        }
    }

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

    uint8_t payload[11] = {0};
    payload[0] = result->human_detected ? 1U : 0U;
    payload[1] = result->confidence;
    payload[2] = (uint8_t)result->human_dir;
    payload[3] = (uint8_t)(max_temp_x100 & 0xFF);
    payload[4] = (uint8_t)((max_temp_x100 >> 8) & 0xFF);
    payload[5] = (uint8_t)(thermistor_x100 & 0xFF);
    payload[6] = (uint8_t)((thermistor_x100 >> 8) & 0xFF);
    payload[7] = (uint8_t)(timestamp_ms & 0xFF);
    payload[8] = (uint8_t)((timestamp_ms >> 8) & 0xFF);
    payload[9] = (uint8_t)((timestamp_ms >> 16) & 0xFF);
    payload[10] = (uint8_t)((timestamp_ms >> 24) & 0xFF);

    const esp_err_t err = uart_cf_send_packet(UART_CF_TYPE_HUMAN_DETECT, payload, sizeof(payload));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_cf_send_packet failed: %s", esp_err_to_name(err));
    }
}

static void emit_heatmap_frame(int64_t now_ms,
                               const float pix[64],
                               float thermistor_c,
                               const inference_result_t *result)
{
    if ((pix == NULL) || (result == NULL)) {
        return;
    }

    if (!usb_serial_jtag_is_connected()) {
        return;
    }

    char line[1024];
    int offset = snprintf(line,
                          sizeof(line),
                          "AMG_FRAME,%lld,%.2f,%.2f,%.2f,%u,%u,%d",
                          (long long)now_ms,
                          thermistor_c,
                          result->ambient_c,
                          result->max_temp_c,
                          result->human_detected ? 1U : 0U,
                          result->confidence,
                          result->human_dir);
    if ((offset < 0) || ((size_t)offset >= sizeof(line))) {
        return;
    }

    for (int i = 0; i < 64; i++) {
        const int written = snprintf(&line[offset], sizeof(line) - (size_t)offset, ",%.2f", pix[i]);
        if ((written < 0) || ((size_t)written >= (sizeof(line) - (size_t)offset))) {
            return;
        }
        offset += written;
    }

    if ((size_t)offset < sizeof(line) - 1U) {
        line[offset++] = '\n';
    }

    const int written = usb_serial_jtag_write_bytes(line, (size_t)offset, pdMS_TO_TICKS(10));
    (void)written;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Crazyflie + AMG8833 firmware starting");

    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .tx_buffer_size = 2048,
            .rx_buffer_size = 256,
        };
        const esp_err_t usb_serial_err = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
        if (usb_serial_err != ESP_OK) {
            ESP_LOGW(TAG, "USB Serial/JTAG driver install failed: %s", esp_err_to_name(usb_serial_err));
        }
    }

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

    status_led_init();
    bool amg_available = false;
    const esp_err_t amg_init_err = amg8833_init(
        I2C_NUM_0,
        I2C_SDA_PIN,
        I2C_SCL_PIN,
        I2C_FREQ,
        AMG_ADDR);
    if (amg_init_err == ESP_OK) {
        amg_available = true;
        ESP_LOGI(TAG,
                 "AMG8833 I2C config: SDA=%d SCL=%d clk=%dHz addr=0x%02X",
                 I2C_SDA_PIN,
                 I2C_SCL_PIN,
                 I2C_FREQ,
                 amg8833_get_i2c_address());
    } else {
        ESP_LOGW(TAG,
                 "AMG8833 init failed (%s). Heatmap and human detection stay disabled until the sensor responds on I2C.",
                 esp_err_to_name(amg_init_err));
    }
    vTaskDelay(pdMS_TO_TICKS(1200));

    float pix[AMG8833_PIXEL_COUNT] = {0};
    float bg[AMG8833_PIXEL_COUNT] = {0};
    bool bg_ready = false;

    int64_t last_log_ms = 0;
    int64_t last_uart_rx_stats_ms = 0;
    int64_t last_heatmap_frame_ms = 0;
    bool last_human_detected = false;

    while (true) {
        uart_cf_packet_t rx_packet = {0};
        while (uart_cf_receive_packet(&rx_packet, 0)) {
            uart_cf_parse_message(&rx_packet);
        }

        const int64_t now_ms = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

        if ((now_ms - last_uart_rx_stats_ms) >= UART_RX_STATS_PERIOD_MS) {
            ESP_LOGI(TAG,
                     "UART RX stats: bytes=%lu packets=%lu",
                     (unsigned long)uart_cf_get_rx_byte_count(),
                     (unsigned long)uart_cf_get_rx_packet_count());
            last_uart_rx_stats_ms = now_ms;
        }

        if (amg_available) {
            float thermistor_c = 0.0f;
            esp_err_t read_err = amg8833_read_pixels(pix, AMG8833_PIXEL_COUNT);
            if (read_err == ESP_OK) {
                read_err = amg8833_read_thermistor(&thermistor_c);
            }

            if (read_err != ESP_OK) {
                status_led_update(false, now_ms);
                ESP_LOGW(TAG, "AMG8833 read failed: %s", esp_err_to_name(read_err));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }

            if (!bg_ready) {
                memcpy(bg, pix, sizeof(bg));
                bg_ready = true;
                status_led_update(false, now_ms);
                ESP_LOGI(TAG, "AMG8833 background frame captured, heatmap streaming enabled");
                vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
                continue;
            }

            const inference_result_t result = run_human_inference(pix, bg, thermistor_c);

            send_detection_to_crazyflie(&result, thermistor_c);
            if ((now_ms - last_heatmap_frame_ms) >= HEATMAP_DEBUG_PERIOD_MS) {
                emit_heatmap_frame(now_ms, pix, thermistor_c, &result);
                last_heatmap_frame_ms = now_ms;
            }
            status_led_update(result.human_detected, now_ms);

            if (result.human_detected != last_human_detected) {
                ESP_LOGI(TAG,
                         "Human detect transition -> %u (conf=%u dir=%d blob=%d)",
                         result.human_detected ? 1U : 0U,
                         result.confidence,
                         result.human_dir,
                         result.blob_size);
                last_human_detected = result.human_detected;
            }

            if ((now_ms - last_log_ms) >= 1000) {
                ESP_LOGI(TAG,
                         "Human=%u conf=%u%% dir=%d blob=%d max=%.2fC avg=%.2fC amb=%.2fC therm=%.2fC",
                         result.human_detected ? 1U : 0U,
                         result.confidence,
                         result.human_dir,
                         result.blob_size,
                         result.max_temp_c,
                         result.avg_temp_c,
                         result.ambient_c,
                         thermistor_c);
                last_log_ms = now_ms;
            }
        } else {
            status_led_update(false, now_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
    }
}
