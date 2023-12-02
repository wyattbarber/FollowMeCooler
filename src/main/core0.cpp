#include "core1.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "navigation.hpp"

#define MOTOR_PERIOD_MS 1000


bool fix = false;
long robot_lat, robot_long;
long robot_heading, robot_speed;
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
        fix = obj->robot_gps_update.fix;
        robot_lat = obj->robot_gps_update.lat;
        robot_long = obj->robot_gps_update.lon;
        robot_heading = obj->robot_gps_update.course;
        robot_speed = obj->robot_gps_update.speed;

        printf(
            "Robot Position Update: fix %d, %f m from user, %d degrees heading, %d speed\n",
            fix,
            nav::dist(robot_lat, user_lat, robot_long, user_long),
            robot_heading, robot_speed
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


uint8_t x_reg_buf[] = {COMP_X_REG};
uint8_t x_reg[] = {0, 0};
uint8_t y_reg_buf[] = {COMP_Y_REG};
uint8_t y_reg[] = {0, 0};
bool motor_callback(repeating_timer_t *rt)
{
    // Read I2C magnetometer data
    i2c_write_blocking(COMP_I2C, COMP_ADDR, x_reg_buf, 1, true);
    i2c_read_blocking(COMP_I2C, COMP_ADDR, x_reg, 2, false);
    i2c_write_blocking(COMP_I2C, COMP_ADDR, y_reg_buf, 1, true);
    i2c_read_blocking(COMP_I2C, COMP_ADDR, y_reg, 2, false);

    int16_t x = (static_cast<int16_t>(x_reg[1]) << 8) | x_reg[0];
    int16_t y = (static_cast<int16_t>(y_reg[1]) << 8) | y_reg[0];
    double heading = atan2(static_cast<double>(y), static_cast<double>(x)) * 180.0 / M_PI;
    while(heading > 360.0) heading -= 360.0;
    while(heading < 0.0) heading += 360.0;
    printf("Orientation Update: %f degrees heading\n", heading);
    return true;
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

    // Setup magnetometer
    uint8_t bufrbt[] = {COMP_CFG_REG_A, COMP_RESET};
    i2c_write_blocking(COMP_I2C, COMP_ADDR, bufrbt, 2, false);
    sleep_ms(100);
    uint8_t bufrst[] = {COMP_CFG_REG_A, COMP_RESET};
    i2c_write_blocking(COMP_I2C, COMP_ADDR, bufrst, 2, false);
    sleep_ms(100);
    uint8_t bufa[] = {COMP_CFG_REG_A, COMP_CFG_A};
    i2c_write_blocking(COMP_I2C, COMP_ADDR, bufa, 2, false);
    uint8_t bufc[] = {COMP_CFG_REG_C, COMP_CFG_C};
    i2c_write_blocking(COMP_I2C, COMP_ADDR, bufc, 2, false);

    // Setup motor update loop
    repeating_timer_t timer;
    add_repeating_timer_ms(MOTOR_PERIOD_MS, motor_callback, NULL, &timer);

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