#include <cstdlib>
#include <cstdio>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

const int SAMPLE_PERIOD_MS = 100;
const int LED_PIN = 25;
const int SERVO_PIN = 0;
const int TRIG_PIN = 1;
const int ECHO_PIN = 2;

static uint servo_slice, servo_chan;

const int8_t N = 100;
const int8_t MIN_ANGLE = -80;
const int16_t MIN_US = 900;
const int8_t MAX_ANGLE = 80;
const int16_t MAX_US = 2100;

const float SPEED_OF_SOUND_MM_US = 0.343;
static uint n = 0;
static bool sweep_left = false;
static uint64_t t_rise;

std::vector<float> dist_mm(N);
static bool new_sweep = false;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool sampler_callback(repeating_timer_t *timer)
{
    pwm_set_chan_level(servo_slice, servo_chan, map(n, 0, N, MIN_US, MAX_US));

    if(sweep_left)
    {
        --n;
        if(n == 0)
        {
            new_sweep = true;
            sweep_left = false;
        }
    } else {
        ++n;
        if(n == N)
        {
            new_sweep = true;
            sweep_left = true;
        }
    }

    gpio_set_mask((1 << TRIG_PIN) | (1 << LED_PIN));
    busy_wait_ms(20);
    gpio_clr_mask(1 << TRIG_PIN);
    return true;
}


void echo_callback(uint gpio, uint32_t events)
{
    if(events & GPIO_IRQ_EDGE_FALL)
    {   
        float dist_total = static_cast<float>(time_us_64() - t_rise) * SPEED_OF_SOUND_MM_US;
        dist_mm[n] = dist_total / 2.0;
    } else {
        gpio_clr_mask(1 << LED_PIN);
        t_rise = time_us_64();
    }
}


std::vector<float> moving_average(std::vector<float>& in, float alpha)
{
    std::vector<float> out(in.size(), 0.0);
    float accu1, accu2 = 0;
    size_t max_idx = in.size() - 1;
    for(size_t i = 0; i < in.size(); ++i)
    {

        accu1 = (alpha * in[i]) + ((1.0 - alpha) * accu1);
        accu2 = (alpha * in[max_idx-i]) + ((1.0 - alpha) * accu2);

        out[i] += accu1 / 2.0;
        out[max_idx-i] += accu2 / 2.0;
    }
    return out;
}


size_t max_idx(std::vector<float>& in)
{
    size_t idx = 0;
    float max = in[0];
    for(size_t i = 0; i < in.size(); ++i)
    {
        if(in[i] > max)
        {
            idx = i;
            max = in[i];
        }
    }
    return idx;
}


int main()
{
    stdio_init_all();
    gpio_init(SERVO_PIN);
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_init(LED_PIN);

    gpio_set_dir(SERVO_PIN, GPIO_OUT);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    servo_chan = pwm_gpio_to_channel(SERVO_PIN);

    pwm_config config = pwm_get_default_config();
    uint32_t clk = clock_get_hz(clk_sys); // clk_sys
    // aim at 50 Hz with counter running to 20 000 
    uint32_t div = clk / (20000 * 50); 
    // Set divider to get 50 Hz
    pwm_config_set_clkdiv(&config, (float)div);
    // Set wrap to count to 20000, so the period is 20 ms
    pwm_config_set_wrap(&config, 20000); 
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(servo_slice, &config, true);

    printf("GPIO Initialized");

    repeating_timer_t sampler_timer;
    if(!add_repeating_timer_ms(-SAMPLE_PERIOD_MS, sampler_callback, NULL, &sampler_timer))
    {
        printf("Repeating timer failed");
    }

    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_callback);

    printf("Interrupts Initialized");

    // Run loop endlessly
    while(true)
    {
        // Wait for new data
        while(!new_sweep)
        {
            tight_loop_contents();
        }
        new_sweep = false;
        
        for(std::size_t i = 0; i < dist_mm.size(); ++i)
        {
            int theta_d = map(i, 0, N, MIN_ANGLE, MAX_ANGLE);
            printf("%d:%f,", theta_d,  dist_mm[i]);
        }
        auto smoothed = moving_average(dist_mm, 0.7);
        printf(";%d\n", max_idx(smoothed));
    }
}