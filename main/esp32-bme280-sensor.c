#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "BME280_RAW"

/*
 * ====== Hardware / I2C ======
 * NOTE: If your BME280 module uses address 0x77 instead of 0x76, update BME280_I2C_ADDR accordingly.
 * SDA/SCL are configured with internal pull-ups; for longer I2C buses you should use external ~4.7kΩ pull-ups.
 */

#define BME280_I2C_ADDR     0x76
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO        23
#define I2C_SCL_GPIO        22
#define I2C_FREQ_HZ         400000

/*
 * ====== BME280 Registers ======
 * PRESS_MSB (0xF7) → block of 8 bytes: pressure[3], temperature[3], humidity[2]
 * CALIB00 (0x88..0xA1) → T/P calibration + H1
 * CALIB26 (0xE1..0xE7) → H2..H6 calibration
 */

#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7  // F7..FE (press[3], temp[3], hum[2])

#define BME280_REG_CALIB00      0x88  // 0x88..0xA1 (calibration T/P + H1)
#define BME280_REG_CALIB26      0xE1  // 0xE1..0xE7 (calibration H2..H6)
#define BME280_CHIP_ID_VAL      0x60
#define BME280_SOFT_RESET_CMD   0xB6

// Oversampling configuration
#define OSRS_H_X1   0x01            // Humidity oversampling ×1
#define OSRS_T_X1   (0x01 << 5)     // Temperature oversampling ×1
#define OSRS_P_X1   (0x01 << 2)     // Pressure oversampling ×1
#define MODE_NORMAL 0x03            // Normal mode
#define T_SB_1000MS (0x05 << 5)     // Standby time 1000 ms
#define FILTER_OFF  (0x00 << 2)     // IIR filter disabled

/** \brief Calibration parameters stored inside the BME280. */
typedef struct {
    // Temperature
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    // Pressure
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    // Humidity
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

static bme280_calib_t calib;
static int32_t t_fine = 0;

/**
 * @brief Initialize I2C master interface.
 *
 * Configures SDA/SCL pins, enables pull-ups, and sets clock speed.
 *
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

/**
 * @brief Write a single byte to a BME280 register.
 *
 * @param reg Register address.
 * @param val Value to write.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, BME280_I2C_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(50));
}

/**
 * @brief Read multiple bytes starting from a BME280 register.
 *
 * @param reg Start register address.
 * @param data Output buffer.
 * @param len Number of bytes to read.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, BME280_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

static esp_err_t i2c_read_u8(uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(I2C_PORT, BME280_I2C_ADDR, &reg, 1, val, 1, pdMS_TO_TICKS(50));
}

static esp_err_t wait_im_update_clear(TickType_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    uint8_t st;
    do {
        ESP_ERROR_CHECK(i2c_read_u8(BME280_REG_STATUS, &st));
        if ((st & 0x01) == 0) return ESP_OK;           // im_update cleared
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms));
    return ESP_ERR_TIMEOUT;
}

static esp_err_t wait_measuring_done(TickType_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    uint8_t st;
    do {
        ESP_ERROR_CHECK(i2c_read_u8(BME280_REG_STATUS, &st));
        if ((st & 0x08) == 0) return ESP_OK;           // measuring cleared
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms));
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Read calibration constants from the BME280.
 *
 * Reads calibration data from 0x88..0xA1 and 0xE1..0xE7 and stores in global `calib`.
 *
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t bme280_read_calibration(void) {
    uint8_t buf1[26]; // 0x88..0xA1 (T+P + H1 at A1 separately)
    ESP_ERROR_CHECK(i2c_read(BME280_REG_CALIB00, buf1, sizeof(buf1)));

    calib.dig_T1 = (uint16_t)(buf1[1] << 8 | buf1[0]);
    calib.dig_T2 = (int16_t)(buf1[3] << 8 | buf1[2]);
    calib.dig_T3 = (int16_t)(buf1[5] << 8 | buf1[4]);

    calib.dig_P1 = (uint16_t)(buf1[7] << 8 | buf1[6]);
    calib.dig_P2 = (int16_t)(buf1[9] << 8 | buf1[8]);
    calib.dig_P3 = (int16_t)(buf1[11] << 8 | buf1[10]);
    calib.dig_P4 = (int16_t)(buf1[13] << 8 | buf1[12]);
    calib.dig_P5 = (int16_t)(buf1[15] << 8 | buf1[14]);
    calib.dig_P6 = (int16_t)(buf1[17] << 8 | buf1[16]);
    calib.dig_P7 = (int16_t)(buf1[19] << 8 | buf1[18]);
    calib.dig_P8 = (int16_t)(buf1[21] << 8 | buf1[20]);
    calib.dig_P9 = (int16_t)(buf1[23] << 8 | buf1[22]);

    calib.dig_H1 = buf1[25]; // 0xA1

    uint8_t buf2[7]; // 0xE1..0xE7 (H2..H6)
    ESP_ERROR_CHECK(i2c_read(BME280_REG_CALIB26, buf2, sizeof(buf2)));
    calib.dig_H2 = (int16_t)(buf2[1] << 8 | buf2[0]);
    calib.dig_H3 = buf2[2];
    // H4/H5 are packed across 0xE4, 0xE5, 0xE6
    calib.dig_H4 = (int16_t)((buf2[3] << 4) | (buf2[4] & 0x0F));
    calib.dig_H5 = (int16_t)((buf2[5] << 4) | (buf2[4] >> 4));
    calib.dig_H6 = (int8_t)buf2[6];

    return ESP_OK;
}

/**
 * @brief Temperature compensation function (Bosch reference algorithm).
 *
 * @param adc_T Raw ADC temperature value.
 * @return Temperature in degrees Celsius.
 */
static float compensate_temperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                    ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8; // T * 100
    return T / 100.0f;
}

/**
 * @brief Pressure compensation function (Bosch reference algorithm).
 *
 * @param adc_P Raw ADC pressure value.
 * @return Pressure in Pascals.
 */
static float compensate_pressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0.0f; // avoid div by zero
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (float)p / 256.0f; // Pa
}

/**
 * @brief Humidity compensation function (Bosch reference algorithm).
 *
 * @param adc_H Raw ADC humidity value.
 * @return Relative humidity in %RH.
 */
static float compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r = t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
                    (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                  (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                       (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                     ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r -
                 (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                   ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    float h = (v_x1_u32r >> 12) / 1024.0f; // %RH
    return h;
}

/**
 * @brief Initialize BME280 sensor.
 *
 * Performs reset, checks chip ID, reads calibration constants,
 * and sets oversampling/operating mode.
 *
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t bme280_init(void) {
    // Reset
    ESP_ERROR_CHECK(i2c_write_reg(BME280_REG_RESET, BME280_SOFT_RESET_CMD));
    ESP_ERROR_CHECK(wait_im_update_clear(20));

    // Verify chip ID
    uint8_t id = 0;
    ESP_ERROR_CHECK(i2c_read(BME280_REG_CHIP_ID, &id, 1));
    if (id != BME280_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Chip ID invalid: 0x%02X (expected 0x%02X)", id, BME280_CHIP_ID_VAL);
        return ESP_FAIL;
    }

    // Read calibration constants
    ESP_ERROR_CHECK(bme280_read_calibration());

    // Configuration: oversampling ×1 for all, normal mode, 1000 ms standby, filter off
    ESP_ERROR_CHECK(i2c_write_reg(BME280_REG_CTRL_HUM, OSRS_H_X1));
    ESP_ERROR_CHECK(i2c_write_reg(BME280_REG_CTRL_MEAS, OSRS_T_X1 | OSRS_P_X1 | MODE_NORMAL));
    ESP_ERROR_CHECK(i2c_write_reg(BME280_REG_CONFIG, T_SB_1000MS | FILTER_OFF));

    return ESP_OK;
}

/**
 * @brief Read a measurement set from the BME280.
 *
 * Waits for measurement to complete, reads raw values, applies compensation.
 *
 * @param temperature_c Output: temperature in °C.
 * @param humidity_rh Output: humidity in %RH.
 * @param pressure_pa Output: pressure in Pa.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t bme280_read_measurements(float *temperature_c, float *humidity_rh, float *pressure_pa) {
    ESP_ERROR_CHECK(wait_measuring_done(20));
    uint8_t raw[8];
    ESP_ERROR_CHECK(i2c_read(BME280_REG_PRESS_MSB, raw, sizeof(raw)));
    // raw: press[0..2], temp[3..5], hum[6..7]
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  | (int32_t)raw[7];

    float T = compensate_temperature(adc_T);
    float P = compensate_pressure(adc_P);
    float H = compensate_humidity(adc_H);

    if (temperature_c) *temperature_c = T;
    if (pressure_pa)   *pressure_pa   = P;
    if (humidity_rh)   *humidity_rh   = H;
    return ESP_OK;
}

/**
 * @brief Main application entry.
 *
 * Initializes I2C and the sensor, then continuously reads
 * and logs temperature, humidity, and pressure once per second.
 */
void app_main(void) {
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(bme280_init());

    while (1) {
        float T, H, P;
        esp_err_t err = bme280_read_measurements(&T, &H, &P);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "T = %.2f °C | RH = %.2f %% | P = %.2f Pa", T, H, P);
        } else {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
