#include <cstdlib>
#include <cstdio>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/rc.h"

const int SAMPLE_PERIOD_MS = 100;
const int SERVO_PIN = 1;
const int TRIG_PIN = 25;
const int ECHO_PIN = 3;

const int SPEED_OF_SOUND_CM_MS = 34;
static rc_servo servo = rc_servo_init(SERVO_PIN);
static uint angle = 0;
static bool sweep_left = false;
static uint64_t t_rise;

std::vector<uint32_t> dist_mm(181);
static bool new_sweep = false;


bool sampler_callback(repeating_timer_t *timer)
{
    rc_servo_set_angle(&servo, angle);
    if(sweep_left)
    {
        --angle;
        if(angle == 0)
        {
            new_sweep = true;
            sweep_left = false;
        }
    } else {
        ++angle;
        if(angle == 180)
        {
            new_sweep = true;
            sweep_left = true;
        }
    }

    gpio_set_mask(1 << TRIG_PIN);
    busy_wait_ms(20);
    gpio_clr_mask(1 << TRIG_PIN);
    return true;
}

void echo_rising_callback(uint gpio, uint32_t events)
{   
    t_rise = time_us_64();
}

void echo_falling_callback(uint gpio, uint32_t events)
{
    uint64_t tof_us = time_us_64() - t_rise;
    dist_mm[angle] = (tof_us / SPEED_OF_SOUND_CM_MS) / 200;
}

int main()
{
    stdio_init_all();
    gpio_init(SERVO_PIN);
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);

    gpio_set_dir(SERVO_PIN, GPIO_OUT);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    printf("GPIO Initialized");

    repeating_timer_t sampler_timer;
    if(!add_repeating_timer_ms(-SAMPLE_PERIOD_MS, sampler_callback, NULL, &sampler_timer))
    {
        printf("Repeating timer failed");
    }

    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE, true, echo_rising_callback);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_FALL, true, echo_falling_callback);

    printf("Interrupts Initialized");

    // Run loop endlessly
    while(true)
    {
        // Wait for new data
        while(!new_sweep)
        {
            sleep_ms(1);
        }
        new_sweep = false;
        
        for(std::size_t i = 0; i < dist_mm.size(); ++i)
        {
            printf("%d:%d,", i, dist_mm[i]);
        }
        printf("\n");
    }
}