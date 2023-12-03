#include "core1.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "navigation.hpp"

#define MOTOR_PERIOD_MS 100 //! Motor control update period
#define LEFT_MOTOR_RVS false //! Reverse throttle commands to left motor
#define RIGHT_MOTOR_RVS false //! Reverse throttle commands to right motor

#define DRIVE_TEST_THROTTLE 1000 //! Throttle command for testing drive

#define OA_DIST 100 //! Min clearance (mm) from obstacles, below which obstacle avoiance mode is triggered
#define OA_THROTTLE 500 //! Drive throttle for obstacle avoidanve mode
#define K_OA 0.01 //! Proportional control constance for obstacle avoidance

bool fix = false; //! Robot GPS has a planar position fix
long robot_lat, robot_long; //! Current robot GPS coordinates, in millionths of degrees
float robot_heading, robot_speed; //! Velocity data extrapolted form GPS coordinates
long user_lat, user_long; //! Latest position of user, in millionths of degrees
OpMode mode; //! Current mode of operation selected by user
int oa_angle; //! Angle of most open path determined by scanner (positive is right)
float oa_dist; //! Distance of closest object detected by scanner in mm

uint channel_la, channel_ra; //! PWM channels for motor control
uint slice; //! Motor control PWM slice
int drive_cmd; //! Forward/reverse component of throttle command
int steer_cmd; //! Left/right component of throttle command (left positive)


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
    // // Read I2C magnetometer data
    // i2c_write_blocking(COMP_I2C, COMP_ADDR, x_reg_buf, 1, true);
    // i2c_read_blocking(COMP_I2C, COMP_ADDR, x_reg, 2, false);
    // i2c_write_blocking(COMP_I2C, COMP_ADDR, y_reg_buf, 1, true);
    // i2c_read_blocking(COMP_I2C, COMP_ADDR, y_reg, 2, false);

    // int16_t x = (static_cast<int16_t>(x_reg[1]) << 8) | x_reg[0];
    // int16_t y = (static_cast<int16_t>(y_reg[1]) << 8) | y_reg[0];
    // double heading = atan2(static_cast<double>(y), static_cast<double>(x)) * 180.0 / M_PI;
    // while(heading > 360.0) heading -= 360.0;
    // while(heading < 0.0) heading += 360.0;
    // printf("Orientation Update: %f degrees heading\n", heading);

    // Calculate left/right commands from drive and steer
    int left_cmd = drive_cmd - steer_cmd;
    int right_cmd = drive_cmd + steer_cmd;

    // Set left motor throttle
    if(left_cmd >= 0)
    {   
#if LEFT_MOTOR_RVS
        gpio_clr_mask(1 << LEFT_PIN_B);
#else
        gpio_set_mask(1 << LEFT_PIN_B);
#endif
    }
    else
    {
#if LEFT_MOTOR_RVS
        gpio_set_mask(1 << LEFT_PIN_B);
#else
        gpio_clr_mask(1 << LEFT_PIN_B);
#endif
    }
    pwm_set_chan_level(slice, channel_la, MIN(abs(left_cmd), 2000));

    // Set right motor throttle
    if(right_cmd >= 0)
    {   
#if RIGHT_MOTOR_RVS
        gpio_clr_mask(1 << RIGHT_PIN_B);
#else
        gpio_set_mask(1 << RIGHT_PIN_B);
#endif
    }
    else
    {
#if RIGHT_MOTOR_RVS
        gpio_set_mask(1 << RIGHT_PIN_B);
#else
        gpio_clr_mask(1 << RIGHT_PIN_B);
#endif
    }
    pwm_set_chan_level(slice, channel_ra, MIN(abs(right_cmd), 2000));

    return true; // return true to keep timer running
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

    // Configure motor driver IO
    gpio_init(LEFT_PIN_A);
    gpio_init(LEFT_PIN_B);
    gpio_init(RIGHT_PIN_A);
    gpio_init(RIGHT_PIN_B);

    gpio_set_dir(LEFT_PIN_A, GPIO_OUT);
    gpio_set_dir(LEFT_PIN_B, GPIO_OUT);
    gpio_set_dir(RIGHT_PIN_A, GPIO_OUT);
    gpio_set_dir(RIGHT_PIN_B, GPIO_OUT);

    gpio_set_function(LEFT_PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_PIN_A, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(LEFT_PIN_A); // Left and right share the same slice
    channel_la = pwm_gpio_to_channel(LEFT_PIN_A);
    channel_ra = pwm_gpio_to_channel(RIGHT_PIN_A);

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

    // Setup motor update loop
    repeating_timer_t timer;
    add_repeating_timer_ms(MOTOR_PERIOD_MS, motor_callback, NULL, &timer);

    // Setup core 1 program
    queue_init(&queue_to_core0, sizeof(Core1Msg), 10);
    multicore_launch_core1(main_core1);

    Core1Msg msg;

    while(true)
    {
        if(queue_try_remove(&queue_to_core0, &msg))
        {
            pop_queue(&msg);
    
        }

        switch(mode)
        {
            case HOLD:
            {
                drive_cmd = 0;
                steer_cmd = 0;
            }
            case FOLLOW:
            {

            }
            case DRIVE_TEST:
            {
                drive_cmd = DRIVE_TEST_THROTTLE;
                steer_cmd = 0;
            }
            case OA_TEST:
            {
                if(oa_dist > OA_DIST)
                {
                    // Normal drive test
                    drive_cmd = DRIVE_TEST_THROTTLE;
                    steer_cmd = 0;
                }
                else
                {
                    // Obstacle detected, steer away
                    drive_cmd = OA_THROTTLE;
                    steer_cmd = -oa_angle; // Sign of scanner angles is reversed from drive angles
                    steer_cmd *= K_OA; // Apply control gain
                }
            }
            default:
            {
                drive_cmd = 0;
                steer_cmd = 0;
                mode = HOLD;
            }
        }
    }
}