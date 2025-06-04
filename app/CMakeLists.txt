cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c src/tmc2209.c src/uart1wire.c src/comm_raw.c src/comm.c src/system.c)
