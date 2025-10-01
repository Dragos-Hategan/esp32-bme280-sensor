#ifndef BME280_READ_H
#define BME280_READ_H

#include "config.h"
#include "bme280_defs.h"

/**
 * @brief Start measurement loop according to chosen mode.
 *
 * @param[in] measurement_choice   Desired measurement type.
 * @param[in] dev                  BME280 device handle.
 */
void start_read(measurement_choice_t measurement_choice, struct bme280_dev *dev);

#endif // BME280_READ_H