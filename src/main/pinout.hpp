#pragma once

#define US_PIN_TRIG 10
#define US_PIN_ECHO 11
#define US_PIN_SERVO 20

#define LEFT_PIN_A 18
#define LEFT_PIN_B 16
#define RIGHT_PIN_A 19
#define RIGHT_PIN_B 17

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
#define COMP_ADDR _u(0x1E)
#define COMP_CFG_REG_A _u(0x60)
#define COMP_CFG_A _u(0x8C)
#define COMP_RESET _u(0x40)
#define COMP_REBOOT _u(0x20)
#define COMP_CFG_REG_C _u(0x60)
#define COMP_CFG_C _u(0x10)
#define COMP_X_REG _u(0x68)
#define COMP_Y_REG _u(0x6A)
#define COMP_Z_REG _u(0x6C)