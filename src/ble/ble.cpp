#include <cstdlib>
#include <cstdio>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define BLE_UART uart1
#define BLE_IRQ UART1_IRQ
#define BLE_TX 4
#define BLE_RX 5
#define BLE_DATABITS 8
#define BLE_STOPBITS 1
#define BLE_PARITY UART_PARITY_NONE
#define BLE_PIN_CTS 2
#define BLE_PIN_RTS 3

std::string ui_msg = {""};
#define UI_CODE_POS 'C'
#define UI_CODE_MODE 'M'
#define UI_MODE_HOLD "H"
#define UI_MODE_FOLLOW "F"
#define UI_MODE_TEST_DRIVE "TD"
#define UI_MODE_TEST_OA "TOA"

long latitude;
long longitude;
typedef enum
{
    HOLD,
    FOLLOW,
    DRIVE_TEST,
    OA_TEST
} OpMode;
OpMode mode = HOLD;

void decode_msg(char cmd, std::string data)
{
    printf("Incoming command is '%c'\n", cmd);
    switch (cmd)
    {
    case UI_CODE_POS:
    {
        size_t split = data.find(':');
        latitude = std::stol(data);
        longitude = std::stol(data.substr(split + 1));
        break;
    }

    case UI_CODE_MODE:
    {
        if (data.compare(UI_MODE_HOLD) == 0)
            mode = HOLD;
        else if (data.compare(UI_MODE_FOLLOW) == 0)
            mode = FOLLOW;
        else if (data.compare(UI_MODE_TEST_DRIVE) == 0)
            mode = DRIVE_TEST;
        else if (data.compare(UI_MODE_TEST_OA) == 0)
            mode = OA_TEST;
        else
            printf("Unrecognized mode %s\n", data.c_str());
        break;
    }
    default:
        printf("Unrecognized command %c\n", cmd);
        break;
    }
}

void ble_rx_handle()
{
    while (uart_is_readable(BLE_UART))
    {
        char c = uart_getc(BLE_UART);
        if (c == '\n')
        {
            // Find start of message to handle alignment errors
            auto idx = ui_msg.find(UI_CODE_POS);
            if(idx == std::string::npos)
            {
                idx = ui_msg.find(UI_CODE_MODE);
            }
            if(idx != std::string::npos)
            {
                // Decode input message
                decode_msg(ui_msg[idx], ui_msg.substr(idx+1));
            }
            else
            {
                printf("Invalid command sentence: %s\n", ui_msg.c_str());
            }
            ui_msg = {""};
        }
        else
        {
            ui_msg.append(1, c);
        }
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
        sleep_ms(5000);
        printf("%d lat x %d long, mode #%d\n", latitude, longitude, (int)mode);
    }
}