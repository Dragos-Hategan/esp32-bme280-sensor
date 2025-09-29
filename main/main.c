/**
 * @file bme280_example.c
 * @brief ESP-IDF example: BME280 over I2C using the new master driver (i2c_master).
 *
 * @details
 * This example shows how to:
 *  - Initialize the ESP-IDF I2C master (new driver API).
 *  - Wire up Bosch Sensortec's BME280 C driver via user-provided I2C callbacks.
 *  - Configure oversampling, IIR filter, and standby time.
 *  - Run the sensor in NORMAL mode and read compensated data periodically.
 *
 * The Bosch BME280 API expects three function pointers:
 *  - `read`   : to read from I2C (register-based).
 *  - `write`  : to write to I2C (register-based).
 *  - `delay_us`: to delay in microseconds.
 *
 * @note
 * - The BME280 API reports: temperature in °C, pressure in Pa, humidity in %rH.
 * - This example uses 7-bit I2C address 0x76 (`BME280_I2C_ADDR_PRIM`).
 *   If SDO is tied to VDDIO, use `BME280_I2C_ADDR_SEC` (0x77).
 * - The code targets ESP-IDF's **new** I2C master driver (`driver/i2c_master.h`).
 *
 * @par Pinout (adjust per board)
 * SDA -> GPIO 23
 * SCL -> GPIO 22
 *
 * @par Build
 * - Add `bme280.c`/`bme280.h` to your component or project.
 * - Link this file into your app, build, and flash with ESP-IDF.
 *
 * @par License
 * Example code provided as-is, for educational purposes.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "bme280.h"

static const char *TAG = "BME280_EX";

/* -------------------------------------------------------------------------- */
/*                         I2C Configuration (edit here)                      */
/* -------------------------------------------------------------------------- */

/** @brief I2C port used by this example. */
#define I2C_PORT        I2C_NUM_0

/** @brief SDA pin. Adjust to match your board wiring. */
#define I2C_SDA_GPIO    23

/** @brief SCL pin. Adjust to match your board wiring. */
#define I2C_SCL_GPIO    22

/** @brief I2C bus frequency (Hz). 400 kHz Fast mode. */
#define I2C_FREQ_HZ     (400 * 1000)

/**
 * @brief Maximum payload bytes we pack into a small stack buffer on write.
 *
 * @details
 * The BME280 register writes are short. 32 bytes is plenty for typical cases
 * (control, config, calibration block writes, etc.). If you need more,
 * either increase this or allocate dynamically.
 */
#define I2C_WRITE_STACK_BUF_MAX   32

/** @brief Handle to the BME280 device on the I2C master bus. */
static i2c_master_dev_handle_t bme280_handle;

/* -------------------------------------------------------------------------- */
/*                  Bosch API: I2C callback implementations                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Bosch API I2C read callback.
 *
 * @param[in]  reg_addr Register address to read from.
 * @param[out] data     Destination buffer for read bytes.
 * @param[in]  len      Number of bytes to read.
 * @param[in]  intf_ptr Unused (Bosch API interface pointer).
 * @return BME280_OK on success, BME280_E_COMM_FAIL on I2C error.
 */
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

    // The new I2C master driver provides a transmit-then-receive helper:
    esp_err_t err = i2c_master_transmit_receive(bme280_handle, &reg_addr, 1, data, len, 100 /*timeout ms*/);
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

/**
 * @brief Bosch API I2C write callback.
 *
 * @param[in] reg_addr Register address to write to.
 * @param[in] data     Source buffer containing payload to write.
 * @param[in] len      Number of bytes to write.
 * @param[in] intf_ptr Unused (Bosch API interface pointer).
 * @return BME280_OK on success,
 *         BME280_E_INVALID_LEN if len exceeds local buffer,
 *         BME280_E_COMM_FAIL on I2C error.
 */
static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

    if (len > I2C_WRITE_STACK_BUF_MAX) {
        return BME280_E_INVALID_LEN;
    }

    // Compose [reg_addr | payload...] into a small stack buffer.
    uint8_t buf[1 + I2C_WRITE_STACK_BUF_MAX];
    buf[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++) {
        buf[1 + i] = data[i];
    }

    esp_err_t err = i2c_master_transmit(bme280_handle, buf, 1 + len, 100 /*timeout ms*/);
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

/**
 * @brief Bosch API delay callback (microseconds).
 *
 * @param[in] period    Delay duration in microseconds.
 * @param[in] intf_ptr  Unused.
 */
static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

/* -------------------------------------------------------------------------- */
/*                            I2C bus initialization                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the I2C master bus and add the BME280 device.
 *
 * @return ESP_OK on success, an ESP_ERR_* code on failure.
 *
 * @details
 * - Sets SDA/SCL pins, enables internal pull-ups, and creates the bus.
 * - Adds a device node for the BME280 at 0x76 with the requested SCL speed.
 *
 * @note
 * If your module uses address 0x77 (SDO=VDDIO), update `device_address`.
 */
static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,                 ///< Small glitch filter (tweak as needed)
        .flags.enable_internal_pullup = true,   ///< Use internal pull-ups if available
        .clk_source = I2C_CLK_SRC_DEFAULT
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t bme280_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR_PRIM,   // 0x76
        .scl_speed_hz = I2C_FREQ_HZ,             // 400 kHz
        .scl_wait_us = 0
    };

    ret = i2c_master_bus_add_device(bus_handle, &bme280_config, &bme280_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                                   app_main                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Main application entry (FreeRTOS).
 *
 * @details
 * Steps performed:
 *  1. Initialize I2C bus and attach the BME280 device.
 *  2. Wire up Bosch API callbacks.
 *  3. Initialize the BME280 (read chip ID, trim params, reset if needed).
 *  4. Configure oversampling, IIR filter, and standby time.
 *  5. Start NORMAL mode (internal periodic conversions).
 *  6. Read and log data once per second.
 *
 * @note
 * NORMAL mode performs measurements periodically according to the
 * oversampling and standby configuration. Alternatively, you can switch
 * to FORCED mode and trigger single conversions on demand if you need
 * lower average power and bursty reads.
 */
void app_main(void)
{
    /* 1) I2C init */
    ESP_ERROR_CHECK(i2c_init());

    /* 2) Hook Bosch API */
    struct bme280_dev dev = {0};
    dev.intf     = BME280_I2C_INTF;
    dev.read     = bme280_i2c_read;
    dev.write    = bme280_i2c_write;
    dev.delay_us = bme280_delay_us;

    /* 3) BME280 init */
    int8_t rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_init failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    /* 4) Sensor configuration */
    struct bme280_settings dev_settings = {0};
    dev_settings.osr_t       = BME280_OVERSAMPLING_2X;            ///< Temperature oversampling
    dev_settings.osr_p       = BME280_OVERSAMPLING_4X;            ///< Pressure oversampling
    dev_settings.osr_h       = BME280_OVERSAMPLING_1X;            ///< Humidity oversampling
    dev_settings.filter      = BME280_FILTER_COEFF_4;             ///< Moderate IIR
    dev_settings.standby_time= BME280_STANDBY_TIME_125_MS;        ///< Only used in NORMAL mode

    uint8_t settings_sel = 0;
    settings_sel = BME280_SEL_OSR_TEMP |
                   BME280_SEL_OSR_PRESS |
                   BME280_SEL_OSR_HUM   |
                   BME280_SEL_FILTER    |
                   BME280_SEL_STANDBY;

    rslt = bme280_set_sensor_settings(settings_sel, &dev_settings, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_settings failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    /* 5) NORMAL mode: automatic periodic conversions */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_mode failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    ESP_LOGI(TAG, "BME280 started (I2C addr 0x%02X). Reading every 1s...", BME280_I2C_ADDR_PRIM);

    /* 6) Periodic reads */
    while (1) {
        struct bme280_data comp = {0};
        rslt = bme280_get_sensor_data(BME280_ALL, &comp, &dev);
        if (rslt == BME280_OK) {
            // Temperature [°C], Pressure [Pa], Humidity [%rH]
            ESP_LOGI(TAG, "T=%.2f °C  P=%.2f hPa  H=%.2f %%", comp.temperature, comp.pressure / 100.0, comp.humidity);
        } else {
            ESP_LOGW(TAG, "bme280_get_sensor_data failed: %d", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
