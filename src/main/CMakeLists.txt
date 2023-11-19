include_directories(../scanner/ ../gps/ ../ble/)

add_executable(cooler core0.cpp)

# pull in common dependencies
target_link_libraries(cooler 
    pico_stdlib 
    hardware_pwm
    hardware_uart
    pico_multicore
)

# enable usb output, disable uart output
pico_enable_stdio_usb(cooler 1)
pico_enable_stdio_uart(cooler 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(cooler)
