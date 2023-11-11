#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"


int main() {


    // Run loop endlessly
    while(true)
        tight_loop_contents();
}