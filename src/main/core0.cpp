#include "core1.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "navigation.hpp"

void pop_queue(Core1Msg* obj)
    {
    if(obj->is_scan_update)
    {
        printf(
            "Scanner Update: path %d degrees, object %f mm\n", 
            obj->scan_update.angle_of_path,
            obj->scan_update.min_dist
        );
    }
    else if(obj->is_robot_gps_update)
    {
        printf(
            "Robot Position Update: %d lat x %d long, fix %d\n",
            std::get<0>(obj->robot_gps_update),
            std::get<1>(obj->robot_gps_update),
            std::get<2>(obj->robot_gps_update)
        );
    }
    else if(obj->is_user_update)
    {
        printf(
            "User Update: %d lat x %d long, mode %d\n",
            std::get<0>(obj->user_update),
            std::get<1>(obj->user_update),
            std::get<2>(obj->user_update)
        );
    }
    else
    {
        printf("Invalid update recieved from core 1.\n");
    }
}

int main()
{
    stdio_init_all();
    gpio_set_dir(25, GPIO_OUT);

    // Configure I2C IO
    i2c_init(COMP_I2C, 100000);
    gpio_set_function(COMP_SCL, GPIO_FUNC_I2C);
    gpio_set_function(COMP_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(COMP_SCL);
    gpio_pull_up(COMP_SDA); 

    // Setup core 1 program
    queue_init(&queue_to_core0, sizeof(Core1Msg), 10);
    multicore_launch_core1(main_core1);


    Core1Msg msg;

    while(true)
    {
        queue_remove_blocking(&queue_to_core0, &msg);
        pop_queue(&msg);
    }
}