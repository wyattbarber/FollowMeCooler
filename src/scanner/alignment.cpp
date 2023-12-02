#include <cstdlib>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define US_MIN 900
#define US_MAX 2100

#define SERVO_PIN 0
#define CENTER_PIN 1
#define DIR_PIN 2

int main()
{    
    // Configure GPIO
    gpio_init(SERVO_PIN);
    gpio_init(CENTER_PIN);
    gpio_init(DIR_PIN);

    gpio_set_dir(SERVO_PIN, GPIO_OUT);
    gpio_set_dir(CENTER_PIN, GPIO_IN);
    gpio_set_dir(DIR_PIN, GPIO_IN);

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint channel = pwm_gpio_to_channel(SERVO_PIN);

    // Configure PWM frequency
    pwm_config config = pwm_get_default_config();
    uint32_t clk = clock_get_hz(clk_sys); // clk_sys
    // aim at 50 Hz with counter running to 20 000
    uint32_t div = clk / (20000 * 50);
    // Set divider to get 50 Hz
    pwm_config_set_clkdiv(&config, (float)div);
    // Set wrap to count to 20000, so the period is 20 ms
    pwm_config_set_wrap(&config, 20000);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice, &config, true);

    while(true)
    {
        if(gpio_get(CENTER_PIN))
        {
            pwm_set_chan_level(slice, channel, (US_MIN + US_MAX) / 2);
        }
        else if(gpio_get(DIR_PIN))
        {
            pwm_set_chan_level(slice, channel, US_MAX);
        }
        else
        {
            pwm_set_chan_level(slice, channel, US_MIN);
        }
    }
}