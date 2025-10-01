#ifndef BME280_I2C_H
#define BME280_I2C_H

#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialize I2C master bus and attach BME280 device.
 *
 * This function sets up the I2C bus and attaches the BME280 as a device.
 *
 * @param[out] bme280_handle  Pointer to I2C device handle where the created
 *                            handle will be stored on success.
 *
 * @return ESP_OK on success, else error code.
 */
esp_err_t i2c_init(i2c_master_dev_handle_t *bme280_handle);

/**
 * @brief I2C read callback for Bosch BME280 driver.
 *
 * @param[in]  reg_addr   Register address to read from.
 * @param[out] data       Pointer to buffer to store read data.
 * @param[in]  len        Number of bytes to read.
 * @param[in]  intf_ptr   Pointer to the I2C device handle (stored in bme280_dev).
 *
 * @return BME280_OK on success, else BME280_E_COMM_FAIL.
 */
int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);

/**
 * @brief I2C write callback for Bosch BME280 driver.
 *
 * @param[in] reg_addr    Register address to write to.
 * @param[in] data        Pointer to buffer with payload data.
 * @param[in] len         Number of payload bytes.
 * @param[in]  intf_ptr   Pointer to the I2C device handle (stored in bme280_dev).
 *
 * @return BME280_OK on success, else error code.
 */
int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

/**
 * @brief Delay callback for Bosch BME280 driver.
 *
 * @param[in] period     Delay duration in microseconds.
 * @param[in] intf_ptr   Unused, passed by Bosch driver.
 */
void bme280_delay_us(uint32_t period, void *intf_ptr);

#endif // BME280_I2C_H