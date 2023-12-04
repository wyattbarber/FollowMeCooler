include_directories(../main)

add_executable(test magnetometer.cpp)
add_executable(calibrate calibration.cpp)

# pull in common dependencies
target_link_libraries(test 
    pico_stdlib 
    hardware_i2c
)
target_link_libraries(calibrate 
    pico_stdlib 
    hardware_i2c
)

# enable usb output, disable uart output
pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 1)
pico_enable_stdio_usb(calibrate 1)
pico_enable_stdio_uart(calibrate 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(test)
pico_add_extra_outputs(calibrate)
