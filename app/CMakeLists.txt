cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c src/comm_raw.c src/comm.c src/system.c src/gcode.c src/strutil.c src/settings.c src/motion.c src/motor.c src/pulser.c)

# Include driver (headers & build)
target_include_directories(app PRIVATE ../include)
add_subdirectory(../drivers drivers)
