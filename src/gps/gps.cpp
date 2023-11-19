#include <cstdlib>
#include <cstdio>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "MicroNMEA.h"

#define GPS_UART uart0
#define GPS_IRQ UART0_IRQ
#define GPS_TX 0
#define GPS_RX 1
#define GPS_DATABITS 8
#define GPS_STOPBITS 1
#define GPS_PARITY UART_PARITY_NONE

char nmea_in[128]; // Allocate buffer of max NMEA scentence size
MicroNMEA decoder(nmea_in, 128);
bool fix = false;
unsigned long lattitude, longitude = 0;

void gps_rx_handle()
{
    while(uart_is_readable(GPS_UART))
    {
        if(decoder.process(uart_getc(GPS_UART)))
        {
            fix = decoder.getNumSatellites() > 3;
            lattitude = decoder.getLatitude();
            longitude = decoder.getLongitude();
        }
    }
}


int main()
{
    stdio_init_all();

    uart_init(GPS_UART, 9600);
    gpio_set_function(GPS_TX, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX, GPIO_FUNC_UART);

    irq_set_exclusive_handler(GPS_IRQ, gps_rx_handle);
    irq_set_enabled(GPS_IRQ, true);
    uart_set_irq_enables(GPS_UART, true, false);

    while(true)
    {
        if(fix)
        {
            printf("GPS Fix: %d by %d\n", lattitude, longitude);
        }
        else
        {
            printf("Waiting for GPS fix... %d of 4 satellites found...\n", decoder.getNumSatellites());
        }
        sleep_ms(1000);
    }
}