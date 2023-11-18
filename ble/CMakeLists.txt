add_executable(ble_test ble.cpp)

# pull in common dependencies
target_link_libraries(ble_test 
    pico_stdlib 
    hardware_uart
)

# enable usb output, disable uart output
pico_enable_stdio_usb(ble_test 1)
pico_enable_stdio_uart(ble_test 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(ble_test)
