# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(test_app LANGUAGES C)

# Include the source files we want to test
target_sources(app PRIVATE 
    ../../app/src/gcode_base.c
    ../../app/src/strutil.c
    ../../app/src/motion_base.c
    src/gcode_base_test.c
    src/strutil_test.c
    src/motion_base_test.c
)

# Include directories
target_include_directories(app PRIVATE 
    ../../app/src
    ../../include
)
