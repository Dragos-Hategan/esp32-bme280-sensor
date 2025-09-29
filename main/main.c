#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "config.h"
#include "bme280.h"

static const char *TAG = "BME280_EX";

/** @brief Handle to the BME280 device on the I2C master bus. */
static i2c_master_dev_handle_t bme280_handle;

/**
 * @brief I2C read callback for Bosch BME280 driver.
 *
 * @param[in]  reg_addr   Register address to read from.
 * @param[out] data       Pointer to buffer to store read data.
 * @param[in]  len        Number of bytes to read.
 * @param[in]  intf_ptr   Unused, passed by Bosch driver.
 *
 * @return BME280_OK on success, else BME280_E_COMM_FAIL.
 */
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;

    // The new I2C master driver provides a transmit-then-receive helper:
    esp_err_t err = i2c_master_transmit_receive(bme280_handle, &reg_addr, 1, data, len, 100 /*timeout ms*/);
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

/**
 * @brief I2C write callback for Bosch BME280 driver.
 *
 * @param[in] reg_addr    Register address to write to.
 * @param[in] data        Pointer to buffer with payload data.
 * @param[in] len         Number of payload bytes.
 * @param[in] intf_ptr    Unused, passed by Bosch driver.
 *
 * @return BME280_OK on success, else error code.
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
 * @brief Delay callback for Bosch BME280 driver.
 *
 * @param[in] period     Delay duration in microseconds.
 * @param[in] intf_ptr   Unused, passed by Bosch driver.
 */
static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

/**
 * @brief Initialize I2C master bus and attach BME280 device.
 *
 * @return ESP_OK on success, else error code.
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

/**
 * @brief Configure BME280 settings for one-time forced measurement.
 *
 * @param[out] dev_settings   Sensor settings structure to update.
 * @param[out] settings_sel   Bitmask of settings that are modified.
 */
static void config_forced_one_time(struct bme280_settings *dev_settings, uint8_t *settings_sel)
{
    dev_settings->osr_t       = BME280_OVERSAMPLING_2X;            ///< Temperature oversampling
    dev_settings->osr_p       = BME280_OVERSAMPLING_8X;            ///< Pressure oversampling
    dev_settings->osr_h       = BME280_OVERSAMPLING_2X;            ///< Humidity oversampling
    dev_settings->filter      = BME280_FILTER_COEFF_OFF;           ///< No filter

    *settings_sel = BME280_SEL_OSR_TEMP |
                   BME280_SEL_OSR_PRESS |
                   BME280_SEL_OSR_HUM   |
                   BME280_SEL_FILTER;
}

/**
 * @brief Configure BME280 settings for forced burst measurements.
 *
 * @param[out] dev_settings   Sensor settings structure to update.
 * @param[out] settings_sel   Bitmask of settings that are modified.
 */
static void config_forced_burst(struct bme280_settings *dev_settings, uint8_t *settings_sel)
{
    dev_settings->osr_t       = BME280_OVERSAMPLING_2X;
    dev_settings->osr_p       = BME280_OVERSAMPLING_8X;
    dev_settings->osr_h       = BME280_OVERSAMPLING_2X;
    dev_settings->filter      = BME280_FILTER_COEFF_2;

    *settings_sel = BME280_SEL_OSR_TEMP |
                   BME280_SEL_OSR_PRESS |
                   BME280_SEL_OSR_HUM   |
                   BME280_SEL_FILTER;
}

/**
 * @brief Configure BME280 settings for normal mode with periodic reads.
 *
 * @param[out] dev_settings   Sensor settings structure to update.
 * @param[out] settings_sel   Bitmask of settings that are modified.
 */
static void config_normal_periodic(struct bme280_settings *dev_settings, uint8_t *settings_sel)
{
    dev_settings->osr_t       = BME280_OVERSAMPLING_2X;
    dev_settings->osr_p       = BME280_OVERSAMPLING_4X;
    dev_settings->osr_h       = BME280_OVERSAMPLING_2X;
    dev_settings->filter      = BME280_FILTER_COEFF_4;
    dev_settings->standby_time= BME280_STANDBY_TIME_125_MS;       ///< Used only in NORMAL mode

    *settings_sel = BME280_SEL_OSR_TEMP |
                   BME280_SEL_OSR_PRESS |
                   BME280_SEL_OSR_HUM   |
                   BME280_SEL_FILTER    |
                   BME280_SEL_STANDBY;
}

/**
 * @brief Configure BME280 settings for continuous normal operation.
 *
 * @param[out] dev_settings   Sensor settings structure to update.
 * @param[out] settings_sel   Bitmask of settings that are modified.
 */
static void config_normal_continuous(struct bme280_settings *dev_settings, uint8_t *settings_sel)
{
    dev_settings->osr_t       = BME280_OVERSAMPLING_2X;
    dev_settings->osr_p       = BME280_OVERSAMPLING_16X;
    dev_settings->osr_h       = BME280_OVERSAMPLING_2X;
    dev_settings->filter      = BME280_FILTER_COEFF_4;
    dev_settings->standby_time= BME280_STANDBY_TIME_500_MS;        ///< Used only in NORMAL mode

    *settings_sel = BME280_SEL_OSR_TEMP |
                   BME280_SEL_OSR_PRESS |
                   BME280_SEL_OSR_HUM   |
                   BME280_SEL_FILTER    |
                   BME280_SEL_STANDBY;
}

/**
 * @brief Apply sensor configuration based on selected measurement mode.
 *
 * @param[in]  measurement_choice   Desired measurement type.
 * @param[in]  dev                  BME280 device handle.
 *
 * @return BME280_OK on success, else error code.
 */
static int8_t configure_sensor_settings(measurement_choice_t measurement_choice, struct bme280_dev *dev)
{
    struct bme280_settings dev_settings = {0};
    uint8_t settings_sel = 0;

    switch(measurement_choice)
    {
        case FORCED_PERIODIC_ONE_TIME:  config_forced_one_time(&dev_settings, &settings_sel);   break;
        case FORCED_PERIODIC_BURST:     config_forced_burst(&dev_settings, &settings_sel);      break;
        case NORMAL_PERIODIC:           config_normal_periodic(&dev_settings, &settings_sel);   break;
        case NORMAL_CONTINUOUSLY:       config_normal_continuous(&dev_settings, &settings_sel); break;
        default: break;
    }

    return bme280_set_sensor_settings(settings_sel, &dev_settings, dev);
}

/**
 * @brief Wait until measurement is done and read sensor data.
 *
 * @param[in]  dev   BME280 device handle.
 * @param[out] out   Structure to store sensor data.
 *
 * @return BME280_OK on success, else error code.
 */
static int8_t wait_and_read(struct bme280_dev *dev, struct bme280_data *out)
{
    // Poll status until measuring is done
    uint8_t status = 1;
    do {
        dev->delay_us(1000, dev->intf_ptr); // 1 ms delay 
        int8_t r = bme280_get_regs(BME280_REG_STATUS, &status, 1, dev);
        if (r != BME280_OK) {
            ESP_LOGW(TAG, "[NORMAL] status read failed: %d", r);
            break;
        }
    } while (status & BME280_STATUS_MEAS_DONE); // BME280_STATUS_MEAS = 0x08

    if (!(status & BME280_STATUS_MEAS_DONE)) {
        return bme280_get_sensor_data(BME280_ALL, out, dev);
    }

    return -1;
}

/**
 * @brief Trigger and read a single forced measurement.
 *
 * @param[in]  dev   BME280 device handle.
 * @param[out] out   Structure to store sensor data.
 *
 * @return BME280_OK on success, else error code.
 */
static int8_t bme_forced_read_once(struct bme280_dev *dev, struct bme280_data *out)
{
    // Start a FORCED conversion
    int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, dev);
    if (rslt != BME280_OK) return rslt;

    return wait_and_read(dev, out);
}

/**
 * @brief Periodically perform single forced measurements and log results.
 *
 * @param[in] dev   BME280 device handle.
 */
static void forced_one_time_read(struct bme280_dev *dev)
{
    struct bme280_data data = {0};
    while (1) {
        int8_t rslt = bme_forced_read_once(dev, &data);

        if (rslt == BME280_OK){
                ESP_LOGI(TAG, "[FORCED_PERIODIC_ONE_TIME] T=%.2f°C  P=%.2f hPa  H=%.2f%%",
                                    data.temperature, data.pressure / 100.0, data.humidity);
        }else {
                ESP_LOGW(TAG, "[FORCED_PERIODIC_ONE_TIME] read failed: %d", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SECONDS * 1000));
    }
}

/**
 * @brief Comparator for qsort over doubles.
 *
 * @param a Pointer to first element (double).
 * @param b Pointer to second element (double).
 * @return <0 if *a < *b, 0 if equal, >0 if *a > *b.
 */
static int cmp_double(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}

/**
 * @brief Compute the median of a double array in-place.
 *
 * Sorts the array using qsort and returns the median value.
 *
 * @param v Pointer to the array (will be modified by sorting).
 * @param n Number of valid elements in @p v.
 * @return Median value of the first @p n elements.
 */
static double median_inplace(double *v, int n)
{
    qsort(v, n, sizeof(double), cmp_double);
    if (n & 1) {
        return v[n / 2];
    } else {
        return 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }
}

/**
 * @brief Perform a burst of forced measurements and log the median sample.
 *
 * Collects up to @c BURST_COUNT forced-mode samples and computes the median
 * for temperature, pressure, and humidity across the valid readings.
 *
 * @param[in] dev BME280 device handle.
 */
static void forced_burst_read(struct bme280_dev *dev)
{
    while (1) {
        double t[BURST_COUNT] = {0};
        double p[BURST_COUNT] = {0};
        double h[BURST_COUNT] = {0};
        int good = 0;

        for (int i = 0; i < BURST_COUNT; ++i) {
            struct bme280_data s = {0};
            int8_t r = bme_forced_read_once(dev, &s);
            if (r == BME280_OK) {
                t[good] = s.temperature;   /* °C */
                p[good] = s.pressure;      /* Pa (Bosch driver float) */
                h[good] = s.humidity;      /* %rH */
                ++good;
            } else {
                ESP_LOGW(TAG, "[FORCED_PERIODIC_BURST] sample %d failed: %d", i, r);
            }
        }

        if (good > 0) {
            double t_med = median_inplace(t, good);
            double p_med = median_inplace(p, good);
            double h_med = median_inplace(h, good);

            ESP_LOGI(TAG, "[FORCED_PERIODIC_BURST] T=%.2f°C  P=%.2f hPa  H=%.2f%%",
                     good, t_med, p_med / 100.0, h_med);
        } else {
            ESP_LOGW(TAG, "[FORCED_PERIODIC_BURST] no valid samples");
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SECONDS * 1000));
    }
}

/**
 * @brief Perform periodic normal-mode measurements, waking up sensor for each slot.
 *
 * @param[in] dev   BME280 device handle.
 */
static void normal_periodic_read(struct bme280_dev *dev)
{
    while (1) {
        struct bme280_data data = {0};
        // Set NORMAL to let internal conversions run
        int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev);
        if (rslt != BME280_OK) {
            ESP_LOGW(TAG, "[NORMAL_PERIODIC] set NORMAL failed: %d", rslt);
        } else {
           rslt = wait_and_read(dev, &data);
        }

        if (rslt == BME280_OK){
            ESP_LOGI(TAG, "[NORMAL_PERIODIC] T=%.2f°C  P=%.2f hPa  H=%.2f%%",
                    data.temperature, data.pressure / 100.0, data.humidity);
        }else {
            ESP_LOGW(TAG, "[NORMAL_PERIODIC] read failed: %d", rslt);
        }

        // Put in SLEEP to reduce consumption until next slot
        (void)bme280_set_sensor_mode(BME280_POWERMODE_SLEEP, dev);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SECONDS * 1000));
    }
}

/**
 * @brief Continuously read data while sensor runs in normal mode.
 *
 * @param[in] dev   BME280 device handle.
 */
static void normal_continuously_read(struct bme280_dev *dev)
{
    int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_mode failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }    
    
    while (1) {
        struct bme280_data data = {0};
        rslt = bme280_get_sensor_data(BME280_ALL, &data, dev);
        if (rslt == BME280_OK) {
            ESP_LOGI(TAG, "[NORMAL_CONTINUOUSLY] T=%.2f °C  P=%.2f hPa  H=%.2f %%", data.temperature, data.pressure / 100.0, data.humidity);
        } else {
            ESP_LOGW(TAG, "[NORMAL_CONTINUOUSLY] bme280_get_sensor_data failed: %d", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Start measurement loop according to chosen mode.
 *
 * @param[in] measurement_choice   Desired measurement type.
 * @param[in] dev                  BME280 device handle.
 */
static void start_read(measurement_choice_t measurement_choice, struct bme280_dev *dev)
{
    switch(measurement_choice)
    {
        case FORCED_PERIODIC_ONE_TIME:  forced_one_time_read(dev);      break;
        case FORCED_PERIODIC_BURST:     forced_burst_read(dev);         break;
        case NORMAL_PERIODIC:           normal_periodic_read(dev);      break;
        case NORMAL_CONTINUOUSLY:       normal_continuously_read(dev);  break;
        default: break;
    }
}

/**
 * @brief Main entry point of application. Initializes I2C, BME280, applies configuration,
 *        and starts the measurement loop.
 */
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());

    struct bme280_dev dev = {0};
    dev.intf     = BME280_I2C_INTF;
    dev.read     = bme280_i2c_read;
    dev.write    = bme280_i2c_write;
    dev.delay_us = bme280_delay_us;

    int8_t rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_init failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    rslt = configure_sensor_settings(measurement_choice, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "configure_sensor_settings failed: %d", rslt);
        vTaskDelay(portMAX_DELAY);
    }

    start_read(measurement_choice, &dev);
}
