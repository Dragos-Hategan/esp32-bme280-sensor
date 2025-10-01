#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "config.h"
#include "bme280_i2c.h"
#include "bme280_defs.h"

static const char* TAG = "BME280_I2C";

esp_err_t i2c_init(i2c_master_dev_handle_t *bme280_handle)
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

    ret = i2c_master_bus_add_device(bus_handle, &bme280_config, bme280_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t device_handle = (i2c_master_dev_handle_t)intf_ptr;

    // The new I2C master driver provides a transmit-then-receive helper:
    esp_err_t err = i2c_master_transmit_receive(device_handle, &reg_addr, 1, data, len, 100 /*timeout ms*/);
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t device_handle = (i2c_master_dev_handle_t)intf_ptr;

    if (len > I2C_WRITE_STACK_BUF_MAX) {
        return BME280_E_INVALID_LEN;
    }

    // Compose [reg_addr | payload...] into a small stack buffer.
    uint8_t buf[1 + I2C_WRITE_STACK_BUF_MAX];
    buf[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++) {
        buf[1 + i] = data[i];
    }

    esp_err_t err = i2c_master_transmit(device_handle, buf, 1 + len, 100 /*timeout ms*/);
    return (err == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}
