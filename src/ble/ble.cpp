#include <cstdlib>
#include <cstdio>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "ble.hpp"

#define BLE_UART uart1
#define BLE_IRQ UART1_IRQ
#define BLE_TX 4
#define BLE_RX 5
#define BLE_DATABITS 8
#define BLE_STOPBITS 1
#define BLE_PARITY UART_PARITY_NONE
#define BLE_PIN_CTS 2
#define BLE_PIN_RTS 3

UIParser parser;

void ble_rx_handle()
{
    while (uart_is_readable(BLE_UART))
    {
        char c = uart_getc(BLE_UART);
        parser.newChar(c);
        printf("%c", c);
    }
}


int main()
{
    stdio_init_all();

    gpio_init_mask((1 << BLE_PIN_CTS) | (1 << BLE_PIN_RTS));
    gpio_set_dir(BLE_PIN_CTS, GPIO_OUT);
    gpio_set_dir(BLE_PIN_RTS, GPIO_IN);

    uart_init(BLE_UART, 9600);
    gpio_set_function(BLE_TX, GPIO_FUNC_UART);
    gpio_set_function(BLE_RX, GPIO_FUNC_UART);

    irq_set_exclusive_handler(BLE_IRQ, ble_rx_handle);
    irq_set_enabled(BLE_IRQ, true);
    uart_set_irq_enables(BLE_UART, true, false);

    gpio_clr_mask(1 << BLE_PIN_CTS);

    while (true)
    {
        while(!parser.newData())
        {
            tight_loop_contents();
        }
        printf("%d lat x %d long,  hold %d, drive %d, test drive %d, test avoidance %d\n", 
            parser.userLatitude(), parser.userLongitude(), 
            parser.holdMode(), parser.driveMode(), parser.testDriveMode(), parser.testOAMode()
        );
    }
}