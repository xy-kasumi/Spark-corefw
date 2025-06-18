## Overview
Spark is OSH project for RepRap-style new-generation EDM (electric discharge machining) machine.
This is part of the project.
Firmware for controlling motors, sensors etc, by consuming commands & G-code from the host.

User-visible mental models & specs are located at /spec/*.md. You should check consistency with them when making
decisions that can affect external behavior or design extensibility.

The firmware is built on Zephyr RTOS. /spark is our repository, and it's located under a Zephyr workspace directory ("Zephyr workspace application").

## Code Style
- For C code under, we use Chromium formatting. Format files when you end editing.
  - We don't use doxygen, but we adopt some convention.
  - Use /** */ for definition used from other files. Use @param, @return (but omit them if the content is obvious). Don't use @brief or @file.
  - For normal comments, use //.
- for empty arguments in C, use () instead of (void).
- at end of file: keep one empty line, if it's not already there
- use #pragma once for .h
- when defining file-scope variable in .c, put it before all functions.
- when using field initialization, rely on implicit 0-initialization for non-important (unused) fields
- Always use {} even for single-line if, while, for etc.
- Run ./format_all.sh when you're done editing code.


## Build
- you can use build.sh for "west build"

## Test
- you can verify that test builds by, `west build -p -b octopus_pro -s tests/app`
- you can run unit tests on the real board by  `west twister --device-testing --device-serial=COM3 --platform=octopus_pro --testsuite-root=./tests --board-root=./ --clobber-output --inline-logs`
  - will take about 1 minute and slows you down. don't run unless the user explictly asks for it. if it's end of your task, ask user to run it instead.
  - if the test failed for seemingly no reason, the user might be using serial port.

## Code Tips
- We use own comm.h throughout. Don't use LOG* or printk().

## Safety Guidelines
- Do not use hardware pins if you're unsure about hardware

## Function Call Rules
- When a function calls another function with "(blocking)" comment, the caller must declare itself as "(blocking)" too. ISR cannot call "(blocking)".
