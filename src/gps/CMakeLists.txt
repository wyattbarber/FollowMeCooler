include_directories(../../MicroNMEA/src)
add_executable(gps_test gps.cpp ../../MicroNMEA/src/MicroNMEA.cpp)

# pull in common dependencies
target_link_libraries(gps_test 
    pico_stdlib 
    hardware_uart
)

# enable usb output, disable uart output
pico_enable_stdio_usb(gps_test 1)
pico_enable_stdio_uart(gps_test 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(gps_test)
