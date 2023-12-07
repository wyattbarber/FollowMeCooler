#include <cstdlib>
#include <cstdio>
#include <vector>
#include "scanner.hpp"

const int SAMPLE_PERIOD_MS = 50;
const int SERVO_PIN = 20;
const int TRIG_PIN = 10;
const int ECHO_PIN = 11;
ScannerData* data;
uint64_t t_rise;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}   

bool sampler_callback(repeating_timer_t *timer)
{
    data->trigger();
    return true;
}


void echo_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL)
    {
        data->increment(time_us_64() - t_rise);
    }
    else
    {
        t_rise = time_us_64();
    }
}

int main()
{
    stdio_init_all();
    data = new ScannerData(TRIG_PIN, ECHO_PIN, SERVO_PIN);

    printf("Scanner Initialized");

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
        while(!data->new_data())
        {
            tight_loop_contents();
        }
        
        int angle = data->path_angle();
        for(std::size_t i = 0; i < data->data().size(); ++i)
        {
            int theta_d = data->idx_to_angle(i);
            printf("%d:%f,", theta_d,  data->data()[i]);
        }
        printf(";%d\n", angle);
    }
}