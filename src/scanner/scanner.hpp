#pragma once

#include <cstdlib>
#include <cstdio>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

class ScannerData
{
public:
    ScannerData(int trig_pin, int echo_pin, int servo_pin, int sample_period = 25,
                uint8_t N = 100, int8_t min_angle = -80, int16_t min_us = 900, int8_t max_angle = 80, int16_t max_us = 2100)
                 : trig_pin(trig_pin), echo_pin(echo_pin), servo_pin(servo_pin),
                 N(N), min_angle(min_angle), min_us(min_us), max_angle(max_angle), max_us(max_us)
    {
        // Configure GPIO
        gpio_init(servo_pin);
        gpio_init(trig_pin);
        gpio_init(echo_pin);

        gpio_set_dir(servo_pin, GPIO_OUT);
        gpio_set_dir(trig_pin, GPIO_OUT);
        gpio_set_dir(echo_pin, GPIO_IN);

        gpio_set_function(servo_pin, GPIO_FUNC_PWM);
        slice = pwm_gpio_to_slice_num(servo_pin);
        channel = pwm_gpio_to_channel(servo_pin);

        // Configure PWM frequency
        config = pwm_get_default_config();
        uint32_t clk = clock_get_hz(clk_sys); // clk_sys
        // aim at 50 Hz with counter running to 20 000
        uint32_t div = clk / (20000 * 50);
        // Set divider to get 50 Hz
        pwm_config_set_clkdiv(&config, (float)div);
        // Set wrap to count to 20000, so the period is 20 ms
        pwm_config_set_wrap(&config, 20000);
        // Load the configuration into our PWM slice, and set it running.
        pwm_init(slice, &config, true);

        // Initialize distance measurements
        dist = std::vector<float>(N, 0.0);
        dist_hold = std::vector<float>(N, 0.0);
        n = 0;
        sweep_left = false;
        new_sweep = false;
    }

    void increment(uint64_t tof_us)
    {
        // Add this distance
        float dist_total = static_cast<float>(tof_us) * SPEED_OF_SOUND_MM_US;
        dist[n] = dist_total / 2.0;
        // Increment servo position
        pwm_set_chan_level(slice, channel, map(n, 0, N, min_us, max_us));
        if (sweep_left)
        {
            --n;
            if (n <= 0)
            {
                n = 0;
                new_sweep = true;
                sweep_left = false;
                dist_hold = dist;
            }
        }
        else
        {
            ++n;
            if (n >= N)
            {
                n = N;
                new_sweep = true;
                sweep_left = true;
                dist_hold = dist;
            }
        }
    }

    void trigger()
    {
        gpio_set_mask(1 << trig_pin);
        busy_wait_ms(20);
        gpio_clr_mask(1 << trig_pin);
    }

    bool new_data()
    {
        if(new_sweep)
        {
            new_sweep = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    int path_angle()
    {
        // moving_average(0.7);
        size_t pos = window(max_idx(), 20);
        return idx_to_angle(pos);
    }

    std::vector<float> data()
    {
        return dist_hold;
    }

    int idx_to_angle(size_t idx)
    {
        return map(idx, 0, N, min_angle, max_angle);
    }

    float min()
    {
        float m = dist_hold[0];
        for (size_t i = 0; i < N; ++i)
        {
            if (dist_hold[i] < m)
            {
                m = dist_hold[i];
            }
        }
        return m;
    }

    protected:
    void moving_average(float alpha)
    {
        std::vector<float> out(N, 0.0);
        float accu1, accu2 = 0;
        size_t max_idx = N - 1;
        for (size_t i = 0; i < N; ++i)
        {

            accu1 = (alpha * dist_hold[i]) + ((1.0 - alpha) * accu1);
            accu2 = (alpha * dist_hold[max_idx - i]) + ((1.0 - alpha) * accu2);

            out[i] += accu1 / 2.0;
            out[max_idx - i] += accu2 / 2.0;
        }
        dist_hold = out;
    }

    size_t max_idx()
    {
        size_t idx = 0;
        float max = dist_hold[0];
        for (size_t i = 0; i < N; ++i)
        {
            if (dist_hold[i] > max)
            {
                idx = i;
                max = dist_hold[i];
            }
        }
        return idx;
    }

    size_t window(size_t center, size_t size)
    {
        size_t max = MIN(N, center+size);
        size_t min = MAX(0, center-size);

        size_t sum = 0;
        size_t weight = 0;
        for (size_t i = min; i < max; ++i)
        {
            sum += dist_hold[i] * i;
            weight += dist_hold[i]; 
        }

        return sum / weight;
    }

    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    const int trig_pin, echo_pin, servo_pin;
    const uint8_t N;
    const int8_t min_angle, max_angle;
    const int16_t min_us, max_us;

    uint slice, channel;
    pwm_config config;

    std::vector<float> dist, dist_hold;
    uint n;
    bool sweep_left, new_sweep;

    const float SPEED_OF_SOUND_MM_US = 0.343;   
};


