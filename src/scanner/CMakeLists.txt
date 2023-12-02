add_executable(scanner_test scanner_test.cpp)
add_executable(alignment alignment.cpp)

# pull in common dependencies
target_link_libraries(scanner_test 
    pico_stdlib 
    hardware_pwm
)
target_link_libraries(alignment 
    pico_stdlib 
    hardware_pwm
)

# enable usb output, disable uart output
pico_enable_stdio_usb(scanner_test 1)
pico_enable_stdio_uart(scanner_test 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(scanner_test)
pico_add_extra_outputs(alignment)
