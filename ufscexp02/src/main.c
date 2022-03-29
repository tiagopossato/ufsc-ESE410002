#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <hdc2080.h>
#include <app_uart.h>
#include <app_temp_sensor.h>

static const char *TAG = "ufscexp02";

void app_main()
{
    float temp01 = 0, temp02 = 0, humd = 0;
    char out[128] = {};
    ESP_LOGI(TAG, "Initializing APP.");

    app_uart_init();
    app_temp_sensor_init();
    setupHDC2080(0x40);
    while (true)
    {
        temp01 = app_temp_sensor_get_temperature();
        temp02 = readTemperature();
        humd = readHumidity();
        snprintf(out, 128, "DS18b20: %.2f°C, HDC2080: %.2f°C, %.2fuR\n",
                 temp01,
                 temp02,
                 humd);
        app_uart_write(out);
        ESP_LOGI(TAG, "%s", out);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
