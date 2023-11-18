#include <cstdlib>
#include <cstdio>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define BLE_UART uart0
#define BLE_IRQ UART0_IRQ
#define BLE_TX 0
#define BLE_RX 1
#define BLE_DATABITS 8
#define BLE_STOPBITS 1
#define BLE_PARITY UART_PARITY_NONE



void ble_rx_handle()
{
    while(uart_is_readable(BLE_UART))
    {
        if(uart_getc(BLE_UART) == 'T')
        {
            gpio_xor_mask(1 << 25);
        }
    }
}


int main()
{
    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    uart_init(BLE_UART, 9600);
    gpio_set_function(BLE_TX, GPIO_FUNC_UART);
    gpio_set_function(BLE_RX, GPIO_FUNC_UART);

    irq_set_exclusive_handler(BLE_IRQ, ble_rx_handle);
    irq_set_enabled(BLE_IRQ, true);
    uart_set_irq_enables(BLE_UART, true, false);

    while(true)
    {
        sleep_ms(1000);
        // uart_puts(BLE_UART, "Hello?\n");
    }
}