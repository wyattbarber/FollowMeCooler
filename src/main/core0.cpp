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
#define RIGHT_MOTOR_RVS true //! Reverse throttle commands to right motor

#define DRIVE_TEST_THROTTLE 20000 //! Throttle command for testing drive

#define OA_THROTTLE_MIN 5000 //! Drive throttle for obstacle avoidanve mode
#define OA_THROTTLE_MAX DRIVE_TEST_THROTTLE
#define OA_CLEAR_MIN 300.0
#define OA_CLEAR_MAX 2000.0

#define KS 50 //! Proportional control constant for heading correction

bool fix = false; //! Robot GPS has a planar position fix
long robot_lat, robot_long; //! Current robot GPS coordinates, in millionths of degrees
float robot_heading, robot_speed; //! Velocity data extrapolted form GPS coordinates
unsigned long t_prev_gps; //! Time of last gps update

long user_lat, user_long; //! Latest position of user, in millionths of degrees
int user_heading; //! Direction to user, in degrees from north (westward is positive rotation)
int user_dist; //! Distance to user in meters
ModeMsg mode; //! Current mode of operation selected by user

int oa_angle; //! Angle of most open path determined by scanner (positive is right)
float oa_dist; //! Distance of closest object detected by scanner in mm
float oa_wr, oa_wl = 0.0;

uint channel_la, channel_ra, channel_lb, channel_rb; //! PWM channels for motor control
uint slice_a, slice_b; //! Motor control PWM slice
int drive_cmd; //! Forward/reverse component of throttle command
int target_heading; //! Intended direction of driving


int oa_throttle(float clearance)
{
    if(clearance < OA_CLEAR_MIN)
    {
        return OA_THROTTLE_MIN;
    }
    else if(clearance > OA_CLEAR_MAX)
    {
        return OA_THROTTLE_MAX;
    }
    else{
        float diff = clearance - OA_CLEAR_MIN;
        float slope = static_cast<float>(OA_THROTTLE_MAX - OA_THROTTLE_MIN) / (OA_CLEAR_MAX - OA_CLEAR_MIN);
        return OA_THROTTLE_MIN + static_cast<int>(slope * diff);
    }
}


void pop_queue(Core1Msg* obj)
{
    if(obj->is_scan_update)
    {
        oa_angle = obj->scan_update.angle_of_path;
        oa_dist = obj->scan_update.min_dist;
        oa_wl = obj->scan_update.weight_left;
        oa_wr = obj->scan_update.weight_right;
        // printf(
        //     "Scanner Update: path %d degrees, object %f mm, weights %f l, %f r\n", 
        //     oa_angle,
        //     oa_dist,
        //     oa_wl, oa_wr
        // );
    }
    else if(obj->is_robot_gps_update)
    {
        robot_heading = nav::angle(
            robot_lat, obj->robot_gps_update.lat,
            robot_long, obj->robot_gps_update.lon
        );
        robot_speed = nav::dist(
            robot_lat, obj->robot_gps_update.lat,
            robot_long, obj->robot_gps_update.lon
        ) / static_cast<float>(time_us_64() - t_prev_gps);
        robot_speed *= 1E6; // Convert m/us to m/s   
        
        fix = obj->robot_gps_update.fix;
        robot_lat = obj->robot_gps_update.lat;
        robot_long = obj->robot_gps_update.lon;

        user_heading = static_cast<int>(nav::angle(robot_lat, user_lat, robot_long, user_long));
        user_dist = static_cast<int>(nav::dist(robot_lat, user_lat, robot_long, user_long));

        printf(
            "Robot Position Update: fix %d %dx%d, %d m from user at %d, %f m/s at %f degrees heading\n",
            fix,
            robot_lat, robot_long,
            user_dist, user_heading,
            robot_speed, robot_heading
        );
    }
    else if(obj->is_user_update)
    {
        user_lat = obj->user_update.lat;
        user_long = obj->user_update.lon;
        mode = obj->user_update;

        printf(
            "User Update: %d lat x %d long, hold %d, drive %d, test drive %d, test avoidance %d\n",
            user_lat,
            user_long,
            mode.hold, mode.drive, mode.test_drive, mode.test_oa
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

    // Calculate error in heading
    int heading_error = 0;
    if(drive_cmd != 0)
    {
        heading_error = static_cast<int>(robot_heading) - target_heading;
    }
    int steer_cmd = -(heading_error * KS);

    // Calculate left/right commands from drive and steer
    int left_cmd = drive_cmd - steer_cmd;
    int right_cmd = drive_cmd + steer_cmd;

    // Set left motor throttle
    if(LEFT_MOTOR_RVS ? (left_cmd < 0) : (left_cmd >= 0))
    {   
        pwm_set_chan_level(slice_a, channel_la, 0);
        pwm_set_chan_level(slice_b, channel_lb, MIN(abs(left_cmd), 20000));
    }
    else
    {
        pwm_set_chan_level(slice_a, channel_la, MIN(abs(left_cmd), 20000));
        pwm_set_chan_level(slice_b, channel_lb, 0);
    }

    // Set right motor throttle
    if(RIGHT_MOTOR_RVS ? (right_cmd < 0) : (right_cmd >= 0))
    {   
        pwm_set_chan_level(slice_a, channel_ra, 0);
        pwm_set_chan_level(slice_b, channel_rb, MIN(abs(right_cmd), 20000));
    }
    else
    {
        pwm_set_chan_level(slice_a, channel_ra, MIN(abs(right_cmd), 20000));
        pwm_set_chan_level(slice_b, channel_rb, 0);
    }
    // printf("Motor update; %d left, %d right\n", left_cmd, right_cmd);


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
    gpio_set_function(LEFT_PIN_B, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_PIN_B, GPIO_FUNC_PWM);
    slice_a = pwm_gpio_to_slice_num(LEFT_PIN_A); // Left and right share the same slice
    channel_la = pwm_gpio_to_channel(LEFT_PIN_A);
    channel_ra = pwm_gpio_to_channel(RIGHT_PIN_A);
    slice_b = pwm_gpio_to_slice_num(LEFT_PIN_B); // Left and right share the same slice
    channel_lb = pwm_gpio_to_channel(LEFT_PIN_B);
    channel_rb = pwm_gpio_to_channel(RIGHT_PIN_B);

    // Configure PWM frequency of slices A and B
    pwm_config config = pwm_get_default_config();
    uint32_t clk = clock_get_hz(clk_sys); // clk_sys
    // aim at 50 Hz with counter running to 20 000
    uint32_t div = clk / (20000 * 50);
    // Set divider to get 50 Hz
    pwm_config_set_clkdiv(&config, (float)div);
    // Set wrap to count to 20000, so the period is 20 ms
    pwm_config_set_wrap(&config, 20000);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_a, &config, true);
    pwm_init(slice_b, &config, true);

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
                 
        // Handle modes of operation
        if(mode.hold) // Hold position (idle)
        {
            drive_cmd = 0;
        }
        else if(mode.drive) // Drive towards user
        {
            if(user_dist <= 3.0)
            {
                drive_cmd = 0;
            }
            else
            {
                drive_cmd = 10000;
            }
            target_heading = user_heading;
        }
        else if(mode.test_drive) // Test the drive system
        {
            drive_cmd = DRIVE_TEST_THROTTLE;
            target_heading = robot_heading;
        }
        else if(mode.test_oa) // Test obstacle avoidance system
        {
            drive_cmd = oa_throttle(oa_dist);
            target_heading = static_cast<int>(robot_heading) - oa_angle;
        }
        else
        {
            drive_cmd = 0;
            mode.hold = true;
        }
    }
}