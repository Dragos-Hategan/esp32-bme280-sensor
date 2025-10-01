#include "bme280.h"
#include "bme280_config.h"
#include "bme280_i2c.h"

static const char* TAG = "BME280_CONFIG";

esp_err_t bme280_device_init(struct bme280_dev *dev, i2c_master_dev_handle_t bme280_handle)
{
    if (!dev || !bme280_handle) return ESP_ERR_INVALID_ARG;

    dev->intf     = BME280_I2C_INTF;
    dev->read     = bme280_i2c_read;
    dev->write    = bme280_i2c_write;
    dev->delay_us = bme280_delay_us;
    dev->intf_ptr = (void*) bme280_handle;
    
    int8_t rslt = bme280_init(dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "bme280_init failed: %d", rslt);
        return ESP_ERR_NOT_FINISHED;
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
int8_t configure_sensor_settings(measurement_choice_t measurement_choice, struct bme280_dev *dev)
{
    struct bme280_settings dev_settings = {0};
    uint8_t settings_sel = 0;

    switch(measurement_choice)
    {
        case FORCED_PERIODIC_ONE_TIME:  config_forced_one_time(&dev_settings, &settings_sel);   break;
        case FORCED_PERIODIC_BURST:     config_forced_burst(&dev_settings, &settings_sel);      break;
        case NORMAL_PERIODIC:           config_normal_periodic(&dev_settings, &settings_sel);   break;
        case NORMAL_CONTINUOUS:       config_normal_continuous(&dev_settings, &settings_sel); break;
        default: return BME280_E_NULL_PTR;
    }

    return bme280_set_sensor_settings(settings_sel, &dev_settings, dev);
}