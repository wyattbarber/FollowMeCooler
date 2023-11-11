add_executable(scanner_test scanner_test.cpp)

# pull in common dependencies
target_link_libraries(scanner_test 
    pico_stdlib 
    pico_rc
)

# enable usb output, disable uart output
pico_enable_stdio_usb(scanner_test 1)
pico_enable_stdio_uart(scanner_test 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(scanner_test)
