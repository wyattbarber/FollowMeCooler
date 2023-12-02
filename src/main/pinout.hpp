#pragma once

#define US_PIN_TRIG 10
#define US_PIN_ECHO 11
#define US_PIN_SERVO 20

#define BLE_UART uart1
#define BLE_IRQ UART1_IRQ
#define BLE_TX 4
#define BLE_RX 5
#define BLE_DATABITS 8
#define BLE_STOPBITS 1
#define BLE_PARITY UART_PARITY_NONE

#define GPS_UART uart0
#define GPS_IRQ UART0_IRQ
#define GPS_TX 0
#define GPS_RX 1
#define GPS_DATABITS 8
#define GPS_STOPBITS 1
#define GPS_PARITY UART_PARITY_NONE

#define COMP_SDA 2
#define COMP_SCL 3
#define COMP_I2C i2c1