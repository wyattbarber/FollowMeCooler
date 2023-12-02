#pragma once
#include "scanner.hpp"
// #include "gps.hpp"
#include "ble.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include <string.h>
#include "MicroNMEA.h"

#define US_PERIOD_MS 25

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

char nmea_in[128]; // Allocate buffer of max NMEA scentence size
MicroNMEA decoder(nmea_in, 128);
bool new_gps = false;

void gps_rx_handle()
{
    while(uart_is_readable(GPS_UART))
    {
        if(decoder.process(uart_getc(GPS_UART)))
        {
            new_gps = true;
        }
    }
}


ScannerData *scan_data;
uint64_t t_rise;

bool sampler_callback(repeating_timer_t *rt)
{
    scan_data->trigger();
    return true;
}

void echo_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL)
    {
        scan_data->increment(time_us_64() - t_rise);
    }
    else
    {
        t_rise = time_us_64();
    }
}


typedef struct {
    int angle_of_path;
    float min_dist;
} ScanMsg;


typedef struct {
    ScanMsg scan_update;
    bool is_scan_update;

    std::tuple<long, long, bool> robot_gps_update;
    bool is_robot_gps_update;

    std::tuple<long, long, OpMode> user_update;
    bool is_user_update;
} Core1Msg;

queue_t queue_to_core0;


void main_core1()
{
    // stdio_init_all();

    // Initialize multi core communication
    // multicore_fifo_clear_irq();
    // irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_sio_irq);
    // irq_set_enabled(SIO_IRQ_PROC1, true);

    // Initialize scanner data aquisition
    scan_data = new ScannerData(US_PIN_TRIG, US_PIN_ECHO, US_PIN_SERVO);
    repeating_timer_t timer;
    add_repeating_timer_ms(US_PERIOD_MS, sampler_callback, NULL, &timer);
    gpio_set_irq_enabled_with_callback(US_PIN_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_callback);
    
    uart_init(BLE_UART, 9600);
    gpio_set_function(BLE_TX, GPIO_FUNC_UART);
    gpio_set_function(BLE_RX, GPIO_FUNC_UART);
    irq_set_exclusive_handler(BLE_IRQ, ble_rx_handle);
    irq_set_enabled(BLE_IRQ, true);
    uart_set_irq_enables(BLE_UART, true, false);

    uart_init(GPS_UART, 9600);
    gpio_set_function(GPS_TX, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX, GPIO_FUNC_UART);
    irq_set_exclusive_handler(GPS_IRQ, gps_rx_handle);
    irq_set_enabled(GPS_IRQ, true);
    uart_set_irq_enables(GPS_UART, true, false);

    // Run loop endlessly
    while (true)
    {
        if(scan_data->new_data())
        {
            Core1Msg msg;

            msg.is_scan_update = true;
            msg.is_user_update = false;
            msg.is_robot_gps_update = false;

            msg.scan_update.angle_of_path = scan_data->path_angle();
            msg.scan_update.min_dist = scan_data->min();

            queue_add_blocking(&queue_to_core0, &msg);
        }
        else if(new_gps)
        {
            new_gps = false;

            Core1Msg msg;

            msg.is_scan_update = false;
            msg.is_user_update = false;
            msg.is_robot_gps_update = true;

            msg.robot_gps_update = {decoder.getLatitude(), decoder.getLongitude(), decoder.getNumSatellites() >= 3};

            queue_add_blocking(&queue_to_core0, &msg);
        }        
        else if(parser.newData())
        {
            Core1Msg msg;

            msg.is_scan_update = false;
            msg.is_user_update = true;
            msg.is_robot_gps_update = false;

            msg.user_update = {parser.userLatitude(), parser.userLongitude(), parser.currentMode()};

            queue_add_blocking(&queue_to_core0, &msg);
        }
        else
        {
            tight_loop_contents();
        }
    }
}