#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us

#include "bme280.h"

static const char *TAG = "BME280_EX";

// ==== Config I2C (schimbă după placă) ====
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    23
#define I2C_SCL_GPIO    22
#define I2C_FREQ_HZ     400 *1000   // 400kHz

// Alege adresa corectă: 0x76 (SDO=GND) sau 0x77 (SDO=VCC)
static uint8_t bme280_i2c_addr = BME280_I2C_ADDR_PRIM; // 0x76

// ==== Callbacks cerute de Bosch API (I2C) ====
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, addr, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    // buffer: reg + payload
    uint8_t buf[1 + 32];
    if (len > 32) return BME280_E_INVALID_LEN; // simplu guard; poți aloca dinamic dacă vrei
    buf[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++) buf[1 + i] = data[i];

    esp_err_t err = i2c_master_write_to_device(I2C_PORT, addr, buf, 1 + len, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

// ==== Init I2C driver ====
static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }    

    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }    
    
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());

    struct bme280_dev dev = {0};
    dev.intf = BME280_I2C_INTF;
    dev.read = bme280_i2c_read;
    dev.write = bme280_i2c_write;
    dev.delay_us = bme280_delay_us;
    dev.intf_ptr = &bme280_i2c_addr;

    int8_t rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_init failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    // Setări: oversampling și IIR (filtru)
    uint8_t settings_sel = 0;
    struct bme280_settings dev_settings = {0};
    dev_settings.osr_t = BME280_OVERSAMPLING_2X;   // temperatură
    dev_settings.osr_p = BME280_OVERSAMPLING_4X;   // presiune
    dev_settings.osr_h = BME280_OVERSAMPLING_1X;   // umiditate
    dev_settings.filter = BME280_FILTER_COEFF_4;   // IIR moderat
    dev_settings.standby_time = BME280_STANDBY_TIME_125_MS; // doar în normal mode

    settings_sel = BME280_SEL_OSR_TEMP | BME280_SEL_OSR_PRESS | BME280_SEL_OSR_HUM |
                   BME280_SEL_FILTER | BME280_SEL_STANDBY;
    rslt = bme280_set_sensor_settings(settings_sel, &dev_settings, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_settings failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    // Pornim în NORMAL mode (autoconversie periodică internă)
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_mode failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    ESP_LOGI(TAG, "BME280 started (I2C addr 0x%02X). Reading every 1s...", bme280_i2c_addr);

    while (1) {
        struct bme280_data comp = {0};
        rslt = bme280_get_sensor_data(BME280_ALL, &comp, &dev);
        if (rslt == BME280_OK) {
            // API Bosch dă valorile în: temp [°C], press [Pa], hum [%rH]
            ESP_LOGI(TAG, "T=%.2f °C  P=%.2f hPa  H=%.2f %%", comp.temperature, comp.pressure / 100.0, comp.humidity);
        } else {
            ESP_LOGW(TAG, "read failed: %d", rslt);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
