cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(test_app LANGUAGES C)

# Include the source files we want to test
target_sources(app PRIVATE 
    ../../app/src/gcode.c
    ../../app/src/strutil.c
    src/gcode_test.c
    src/strutil_test.c
)

# Include directories
target_include_directories(app PRIVATE 
    ../../app/src
    ../../include
)
