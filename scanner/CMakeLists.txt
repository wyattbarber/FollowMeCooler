add_executable(scanner_test scanner_test.c)

# pull in common dependencies
target_link_libraries(scanner_test pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(scanner_test)
