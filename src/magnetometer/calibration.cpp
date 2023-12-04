#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
#include <algorithm>
#include <cmath>

#define BATCH 10

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
    
    // Collect first point
    i2c_write_blocking(COMP_I2C, COMP_ADDR, x_reg_buf, 1, true);
    if(i2c_read_blocking(COMP_I2C, COMP_ADDR, (uint8_t*)buf, 6, false) != 6)
    {
        printf("Error reading data\n");
    }
    auto x1 = buf[0];
    auto y1 = buf[1];
    auto z1 = buf[2];

    // Collect second point
    i2c_write_blocking(COMP_I2C, COMP_ADDR, x_reg_buf, 1, true);
    if(i2c_read_blocking(COMP_I2C, COMP_ADDR, (uint8_t*)buf, 6, false) != 6)
    {       
        printf("Error reading data\n");
    }
    auto x2 = buf[0];
    auto y2 = buf[1];
    auto z2 = buf[2];

    auto xmin = MIN(x1, x2);
    auto xmax = MAX(x1, x2);
    auto ymin = MIN(y1, y2);
    auto ymax = MAX(y1, y2);
    auto zmin = MIN(z1, z2);
    auto zmax = MAX(z1, z2);

    while(true)
    {
        // Collect data
        i2c_write_blocking(COMP_I2C, COMP_ADDR, x_reg_buf, 1, true);
        if(i2c_read_blocking(COMP_I2C, COMP_ADDR, (uint8_t*)buf, 6, false) != 6)
        {
            printf("Error reading data\n");

        }

        xmin = MIN(xmin, buf[0]);
        xmax = MAX(xmax, buf[0]);
        ymin = MIN(ymin, buf[1]);
        ymax = MAX(ymax, buf[1]);
        zmin = MIN(zmin, buf[2]);
        zmax = MAX(zmax, buf[2]);

        printf("\tX: %d to %d", xmin, xmax);
        printf("\tY: %d to %d", ymin, ymax);
        printf("\tZ: %d to %d", zmin, zmax);
        printf("\n");

        sleep_ms(10);
    }
}