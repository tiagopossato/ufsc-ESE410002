#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdbool.h>
#include "hdcHAL.h"

static const char *TAG = "i2c-hal";

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 10

uint8_t device_address;

bool hdcHALInit(uint8_t addr)
{
    esp_err_t res;
    device_address = addr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    res = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_param_config %d (%s)", res, esp_err_to_name(res));
        return false;
    }

    res = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_driver_install error %d (%s)", res, esp_err_to_name(res));
        return false;
    }

    return true;
}

uint8_t readReg(uint8_t reg)
{

    esp_err_t err = ESP_OK;
    uint8_t buffer = 0;
    i2c_cmd_handle_t handle;

    // Create and init I2C command link
    handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    // Slave address (W)
    i2c_master_write_byte(handle, (device_address << 1) | I2C_MASTER_WRITE, false);
    // Address
    i2c_master_write_byte(handle, reg, false);
    // Start
    i2c_master_start(handle);
    // Slave address (R)
    i2c_master_write_byte(handle, (device_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(handle, &buffer, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(handle);
    // Perform transaction
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "readReg error %d (%s)", err, esp_err_to_name(err));
        i2c_cmd_link_delete(handle);
        return false;
    }
    i2c_cmd_link_delete(handle);
    return buffer;
}

// https://docs.espressif.com/projects/esp-idf/en/release-v4.1/api-reference/peripherals/i2c.html#i2c-api-master-mode

bool writeReg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t handle;
    esp_err_t err;

    // Create and init I2C command link
    handle = i2c_cmd_link_create();
    // START
    i2c_master_start(handle);
    // Slave address (W)
    i2c_master_write_byte(handle, (device_address << 1) | I2C_MASTER_WRITE, false);
    // Address
    i2c_master_write_byte(handle, reg, true);
    // Data
    i2c_master_write_byte(handle, data, false);
    // Stop
    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "writeReg: error %d (%s)", err, esp_err_to_name(err));
        i2c_cmd_link_delete(handle);
        return false;
    }

    i2c_cmd_link_delete(handle);
    return true;
}
