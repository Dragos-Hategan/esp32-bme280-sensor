#ifndef BME280_CONFIG_H
#define BME280_CONFIG_H

#include <stdint.h>
#include "config.h"
#include "esp_err.h"
#include "bme280_defs.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialize and configure the BME280 device structure.
 *
 * This function sets up the BME280 device interface for I2C communication by
 * assigning the function pointers for read, write, and delay operations.
 * It then calls the Bosch API @ref bme280_init to initialize the sensor.
 *
 * @param[in,out] dev   Pointer to the BME280 device structure to initialize.
 * @param[in] bme280_handle   I2C handle for device.
 *
 * @note The caller must provide valid implementations of @ref bme280_i2c_read,
 *       @ref bme280_i2c_write, and @ref bme280_delay_us. The device structure
 *       must remain in scope for the lifetime of the driver usage.
 *
 * @retval esp_err_t
 */
esp_err_t bme280_device_init(struct bme280_dev *dev, i2c_master_dev_handle_t bme280_handle);

/**
 * @brief Apply sensor configuration based on selected measurement mode.
 *
 * @param[in]  measurement_choice   Desired measurement type.
 * @param[in]  dev                  BME280 device handle.
 *
 * @return BME280_OK on success, else error code.
 */
int8_t configure_sensor_settings(measurement_choice_t measurement_choice, struct bme280_dev *dev);


#endif // BME280_CONFIG_H