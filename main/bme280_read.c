#include <stdint.h>
#include "bme280.h"
#include "bme280_read.h"

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BME280_READ";

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
 * @brief CONTINUOUS read data while sensor runs in normal mode.
 *
 * @param[in] dev   BME280 device handle.
 */
static void normal_CONTINUOUS_read(struct bme280_dev *dev)
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
            ESP_LOGI(TAG, "[NORMAL_CONTINUOUS] T=%.2f °C  P=%.2f hPa  H=%.2f %%", data.temperature, data.pressure / 100.0, data.humidity);
        } else {
            ESP_LOGW(TAG, "[NORMAL_CONTINUOUS] bme280_get_sensor_data failed: %d", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void start_read(measurement_choice_t measurement_choice, struct bme280_dev *dev)
{
    switch(measurement_choice)
    {
        case FORCED_PERIODIC_ONE_TIME:  forced_one_time_read(dev);      break;
        case FORCED_PERIODIC_BURST:     forced_burst_read(dev);         break;
        case NORMAL_PERIODIC:           normal_periodic_read(dev);      break;
        case NORMAL_CONTINUOUS:       normal_CONTINUOUS_read(dev);  break;
        default: break;
    }
}