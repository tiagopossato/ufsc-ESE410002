#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ds18b20.h"
#include "driver/gpio.h"

static const gpio_num_t SENSOR_GPIO = 32;

static const char *TAG = "app_temp_sensor";

DeviceAddress tempSensor = {0x28, 0xFF, 0xBE, 0x67, 0x54, 0x16, 0x04, 0x71};
void app_temp_sensor_init()
{

    ds18b20_init(SENSOR_GPIO);
    // if (search(tempSensor, true))
    // {
    //     ESP_LOGE(TAG, "Sensor não encontrado");
    //     return;
    // }
    ds18b20_setResolution((DeviceAddress *)tempSensor, 1, 12);
    ESP_LOGI(TAG, "Address 0: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", tempSensor[0], tempSensor[1], tempSensor[2], tempSensor[3], tempSensor[4], tempSensor[5], tempSensor[6], tempSensor[7]);
}

float app_temp_sensor_get_temperature()
{
    return ds18b20_get_temp();
}