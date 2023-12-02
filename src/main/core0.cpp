#include "core1.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "navigation.hpp"


bool fix = false;
long robot_lat, robot_long;
long user_lat, user_long;
OpMode mode;
int oa_angle;
float oa_dist;


void pop_queue(Core1Msg* obj)
    {
    if(obj->is_scan_update)
    {
        oa_angle = obj->scan_update.angle_of_path;
        oa_dist = obj->scan_update.min_dist;
        printf(
            "Scanner Update: path %d degrees, object %f mm\n", 
            oa_angle,
            oa_dist
        );
    }
    else if(obj->is_robot_gps_update)
    {
        fix = std::get<2>(obj->robot_gps_update);
        robot_lat = std::get<0>(obj->robot_gps_update);
        robot_long = std::get<1>(obj->robot_gps_update);

        printf(
            "Robot Position Update: %f m from user, %f degrees heading\n",
            nav::dist(robot_lat, user_lat, robot_long, user_long),
            nav::angle(robot_lat, user_lat, robot_long, user_long)
        );
    }
    else if(obj->is_user_update)
    {
        user_lat = std::get<0>(obj->user_update);
        user_long = std::get<1>(obj->user_update);
        mode = std::get<2>(obj->user_update);

        printf(
            "User Update: %d lat x %d long, mode %d\n",
            user_lat,
            user_long,
            mode
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