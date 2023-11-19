#pragma once
#include "scanner.hpp"
#include "gps.hpp"
#include "ble.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include <string.h>

#define US_PERIOD_MS 25

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
} ScanMsg;
ScanMsg msg;


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


    // Run loop endlessly
    while (true)
    {
        if(scan_data->new_data())
        {
            msg.angle_of_path = scan_data->path_angle();
            multicore_fifo_push_blocking((uint32_t)&msg);
        }
        else
        {
            tight_loop_contents();
        }
    }
}