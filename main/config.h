#ifndef CONFIG_H
#define CONFIG_H

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

/* -------------------------------------------------------------------------- */
/*                          Choose A Measurement Type                         */
/* -------------------------------------------------------------------------- */
typedef enum{
    FORCED_PERIODIC_ONE_TIME,
    FORCED_PERIODIC_BURST,
    NORMAL_PERIODIC,
    NORMAL_CONTINUOUS
}measurement_choice_t;

/* -------------------------------------------------------------------------- */
/*                                  Parameters                                */
/* -------------------------------------------------------------------------- */
#define PERIOD_SECONDS          3       // N seconds between reports
#define BURST_COUNT             5       // number of samples in a burst
#define USE_MEDIAN_IN_BURST     1       // 1 = use average of burst
#define NORMAL_PULSE_EXTRA_MS   10      // small margin above t_meas

#endif // CONFIG_H