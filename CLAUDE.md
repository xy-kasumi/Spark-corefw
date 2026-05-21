# Overview
Spark is OSH project for RepRap-style new-generation EDM (electric discharge machining) machine.
This is part of the project.
Firmware for controlling motors, sensors etc, by consuming commands & G-code from the host.

Written in Rust on embassy, targeting the BTT Octopus Pro (STM32H723).
* `model/` — portable, host-testable compute (G-code parser, motion, coordinates, protocol).
* `firmware/` — on-device binary: embassy app and hardware drivers.

User-visible mental models & specs are located at /docs/*.md. You should check consistency with them when making
decisions that can affect external behavior or design extensibility.

## Safety Guidelines
- Do not use hardware pins if you're unsure about hardware

# Scripts
Run from the repo root.
* `build.sh`, `test.sh`: fast, run often
* `flash.sh`, `panic_diag.sh`: uses OpenOCD to talk to real board. needs user permission

# Code Style
- Run `cargo fmt --all --manifest-path Cargo.toml` when you're done editing.

# Build, Test
- you can `build.sh` or `test.sh`
  - `build.sh` is much quicker than `test.sh` to check for compilation error.
  - only run `test.sh` after `build.sh` is successful.

# Design
* 1ms central tick loop. Anything needing faster than 1ms is encapsulated as a
  hardware driver with its own hardware timers or DMAs, exposing only a "slow"
  (1ms) API. No other `Timer::after` calls.
