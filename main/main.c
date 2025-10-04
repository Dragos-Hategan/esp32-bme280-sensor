#include <stdio.h>

#include "config.h"
#include "bme280_config.h"
#include "bme280_read.h"
#include "bme280_i2c.h"

#include "esp_log.h"

const char *TAG = "BME280_EX";

/**
 * @brief Main entry point of application. Initializes I2C, BME280, applies configuration,
 *        and starts the measurement loop.
 */
void app_main(void)
{
    i2c_master_dev_handle_t bme280_handle;
    ESP_ERROR_CHECK(i2c_init(&bme280_handle));

    struct bme280_dev dev = {0};
    ESP_ERROR_CHECK(bme280_device_init(&dev, bme280_handle));

    static const measurement_choice_t measurement_choice = FORCED_PERIODIC_BURST;
    int8_t rslt = configure_sensor_settings(measurement_choice, &dev);
    if (rslt != BME280_OK){ ESP_LOGE(TAG, "configure_sensor_settings failed: %d", rslt);}

    start_read(measurement_choice, &dev);
}
