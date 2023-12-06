#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
#include <algorithm>
#include <cmath>

#define MIN_X -354
#define MAX_X 78
#define MIN_Y -258
#define MAX_Y 199
#define MIN_Z -633
#define MAX_Z -329

float normalize(int x, int min, int max)
{
    float m = static_cast<float>(min + max) / 2.0;
    float d = static_cast<float>(x - m);
    return d / static_cast<float>(max - min);
}


int16_t mean(std::vector<int16_t> v)
{
    int s = 0;
    for(auto x = v.begin(); x != v.end(); ++x)
    {
        s += *x;
    }
    return s / v.size(); 
}

uint16_t devsqr(std::vector<int16_t> v)
{
    auto xm = mean(v);
    int s = 0;
    for(auto x = v.begin(); x != v.end(); ++x)
    {   
        int d = *x - xm;
        s += (d * d);
    }
    return s / (v.size() - 1);
    
}


int main()
{
    stdio_init_all();

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

    uint8_t x_reg_buf[] = {COMP_X_REG | 0x80}; // Set MSB to 1 to increment pointer when reding multiple bytes
    uint8_t y_reg_buf[] = {COMP_Y_REG | 0x80};
    uint8_t z_reg_buf[] = {COMP_Z_REG | 0x80};
    int16_t buf[3];

    std::vector<int16_t> x;
    std::vector<int16_t> y;
    std::vector<int16_t> z;
    size_t i = 0;
    while(true)
    {
        // Collect data
        i2c_write_blocking(COMP_I2C, COMP_ADDR, x_reg_buf, 1, true);
        if(i2c_read_blocking(COMP_I2C, COMP_ADDR, (uint8_t*)buf, 6, false) != 6)
        {
            printf("Error reading data\n");
        }

        
        int16_t _x = buf[0];
        int16_t _y = buf[1];
        int16_t _z = buf[2];         
        // Log data
        printf("\tX: %d\n", _x);
        printf("\tY: %d\n", _y);
        printf("\tZ: %d\n", _z);
        printf("\tHeading: %f degrees", atan2(normalize(_y, MIN_Y, MAX_Y), normalize(_x, MIN_X, MAX_X)) * 180.0 / M_PI);
        // printf("\tHeading: %f degrees", atan2(static_cast<float>(_y), static_cast<float>(_x)) * 180.0 / M_PI);
        printf("\n");

        ++i;  
        sleep_ms(10);
    }

}