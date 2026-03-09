#include "amg8833.h"

#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define AMG8833_REG_PCTL         0x00U
#define AMG8833_REG_RST          0x01U
#define AMG8833_REG_FPSC         0x02U
#define AMG8833_REG_THERMISTOR_L 0x0EU
#define AMG8833_REG_PIXEL_START  0x80U

#define AMG8833_MODE_NORMAL      0x00U
#define AMG8833_RESET_INITIAL    0x3FU
#define AMG8833_FPS_10HZ         0x00U

static const char *TAG = "amg8833";

static i2c_port_t s_i2c_port = I2C_NUM_0;
static uint8_t s_i2c_addr = AMG8833_I2C_ADDRESS_DEFAULT;
static bool s_is_init = false;

static esp_err_t amg8833_read_registers(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(
        s_i2c_port, s_i2c_addr, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

static esp_err_t amg8833_write_register(uint8_t reg, uint8_t value)
{
    const uint8_t payload[2] = {reg, value};
    return i2c_master_write_to_device(
        s_i2c_port, s_i2c_addr, payload, sizeof(payload), pdMS_TO_TICKS(100));
}

static bool amg8833_probe(uint8_t address)
{
    const uint8_t reg = AMG8833_REG_PCTL;
    uint8_t value = 0;
    const esp_err_t err = i2c_master_write_read_device(
        s_i2c_port, address, &reg, 1, &value, 1, pdMS_TO_TICKS(50));
    return (err == ESP_OK);
}

static float amg8833_pixels_raw_to_celsius(uint16_t raw)
{
    int16_t signed12 = (int16_t)(raw & 0x0FFFU);
    if ((signed12 & 0x0800) != 0) {
        signed12 = (int16_t)(signed12 - 0x1000);
    }
    return ((float)signed12) * 0.25f;
}

static float amg8833_thermistor_raw_to_celsius(uint16_t raw)
{
    const int16_t magnitude = (int16_t)(raw & 0x07FFU);
    float value = ((float)magnitude) * 0.0625f;
    if ((raw & 0x0800U) != 0U) {
        value = -value;
    }
    return value;
}

esp_err_t amg8833_init(i2c_port_t i2c_port,
                       gpio_num_t sda_pin,
                       gpio_num_t scl_pin,
                       uint32_t clock_hz,
                       uint8_t preferred_address)
{
    s_i2c_port = i2c_port;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = clock_hz,
        },
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(s_i2c_port, &cfg), TAG, "i2c_param_config failed");

    esp_err_t install_err = i2c_driver_install(s_i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (install_err == ESP_ERR_INVALID_STATE) {
        install_err = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(install_err, TAG, "i2c_driver_install failed");

    const uint8_t fallback_address =
        (preferred_address == AMG8833_I2C_ADDRESS_DEFAULT)
            ? AMG8833_I2C_ADDRESS_ALT
            : AMG8833_I2C_ADDRESS_DEFAULT;

    if (amg8833_probe(preferred_address)) {
        s_i2c_addr = preferred_address;
    } else if (amg8833_probe(fallback_address)) {
        s_i2c_addr = fallback_address;
        ESP_LOGW(TAG, "Preferred address 0x%02X not found, using 0x%02X", preferred_address, s_i2c_addr);
    } else {
        ESP_LOGE(TAG, "AMG8833 probe failed on 0x%02X and 0x%02X", preferred_address, fallback_address);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(amg8833_write_register(AMG8833_REG_PCTL, AMG8833_MODE_NORMAL), TAG, "failed to set normal mode");
    ESP_RETURN_ON_ERROR(amg8833_write_register(AMG8833_REG_RST, AMG8833_RESET_INITIAL), TAG, "failed to reset sensor");
    ESP_RETURN_ON_ERROR(amg8833_write_register(AMG8833_REG_FPSC, AMG8833_FPS_10HZ), TAG, "failed to set frame rate");

    s_is_init = true;
    ESP_LOGI(TAG, "AMG8833 ready on I2C port %d addr 0x%02X (%lu Hz)", s_i2c_port, s_i2c_addr, (unsigned long)clock_hz);
    return ESP_OK;
}

esp_err_t amg8833_read_pixels(float *pixels, size_t pixel_count)
{
    if ((pixels == NULL) || (pixel_count == 0U) || (pixel_count > AMG8833_PIXEL_COUNT) || !s_is_init) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[AMG8833_PIXEL_COUNT * 2] = {0};
    const size_t bytes_to_read = pixel_count * 2U;

    ESP_RETURN_ON_ERROR(amg8833_read_registers(AMG8833_REG_PIXEL_START, raw, bytes_to_read), TAG, "pixel read failed");

    for (size_t i = 0; i < pixel_count; i++) {
        const uint16_t pixel_raw = (uint16_t)raw[(i * 2U)] | ((uint16_t)raw[(i * 2U) + 1U] << 8U);
        pixels[i] = amg8833_pixels_raw_to_celsius(pixel_raw);
    }

    return ESP_OK;
}

esp_err_t amg8833_read_thermistor(float *thermistor_c)
{
    if ((thermistor_c == NULL) || !s_is_init) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[2] = {0};
    ESP_RETURN_ON_ERROR(amg8833_read_registers(AMG8833_REG_THERMISTOR_L, raw, sizeof(raw)), TAG, "thermistor read failed");

    const uint16_t therm_raw = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8U);
    *thermistor_c = amg8833_thermistor_raw_to_celsius(therm_raw);
    return ESP_OK;
}

uint8_t amg8833_get_i2c_address(void)
{
    return s_i2c_addr;
}
