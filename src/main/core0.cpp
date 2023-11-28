#include "core1.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "navigation.hpp"

void core0_msg_handler()
{
    Core1Msg* obj;
    while (multicore_fifo_rvalid())
        obj = (Core1Msg*)multicore_fifo_pop_blocking();
    
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

    // Setup core 1 program
    multicore_launch_core1(main_core1);

    // Initialize multi core communication
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_msg_handler);
    irq_set_enabled(SIO_IRQ_PROC0, true);

    while(true)
    {
        tight_loop_contents();
    }
}