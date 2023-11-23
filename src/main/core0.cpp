#include "core1.hpp"
#include "pinout.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "navigation.hpp"

void core0_msg_handler()
{
    ScanMsg* obj;
    while (multicore_fifo_rvalid())
        obj = (ScanMsg*)multicore_fifo_pop_blocking();
    printf("New Path Direction: %d\n", obj->angle_of_path);
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