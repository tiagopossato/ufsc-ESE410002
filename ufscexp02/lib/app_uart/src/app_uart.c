#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include "app_uart.h"

const uart_port_t uart_num = UART_NUM_1;

void app_uart_init()
{
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS, // 8
        .parity = UART_PARITY_DISABLE, // N
        .stop_bits = UART_STOP_BITS_1  // 1
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Install UART driver using an event queue here

    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));

    ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_uart_write(const char *str)
{
    uart_write_bytes(uart_num, (const char *)str, strlen(str));
}