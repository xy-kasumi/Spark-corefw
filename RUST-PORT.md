This firmware is going under C+Zephyr -> Rust rewrite.

Purpose
* Make the firmware more testable, more refactorable, make it faster to do big redesign w/o worrying regression
  * Split of compute vs hardware
* Reduce dependence on zephyr (big & non-hermetic); repo clone -> command to generate firmware binary reproducibly

Plan
* Want both C & Rust code to be buildable until migration is done (or abandoned)
  * Everything Rust under rust/
  * No interdependence; completely separate codebase & scripts which I can cleanly delete one after deciding which to keep
* Want host-side test from the beginning
  * OTOH, I don't care about on-device unit tests anymore.
* Two technical fronts I foresee:
  * A: hardware-side; embassy, re-implementing hardware quirks (like clock tree or peripherals)
  * A': build, flash commands
  * B: finding right system design that's good for Rust
* project scope: either it's 99% feature parity in a few full days or bust

Hardware abstraction level
* For porting completion, only current BTT Octopus (with specific pin assignments) need to work.
* However, these future need to be isolated change (to whatever "hardware layer"), otherwise bust.
  * changing to different pinouts
  * using porting to other STM32-based boards, different motor drivers
* porting to ESP32 or Raspi Pico should be possible w/o re-architecturing or rewriting G-code parser, but can require some update
* will never need to support 8-bit uCs or PC-like hardware with OSes

# Steps

1. DONE: Decide on crates division, establish UART (different from the spec'd protocol) with host on real device.
  * Basics first
2. DONE: Attack TMC UART & stepper driver. Impl minimum comm to check if it's working on real device.
  * Biggest hardware unknown, and probably least testable & least clean part of the code
3. DONE: Big-port: design all the APIs, while impls are mostly not-implemented.
  * Lots of work on API design, test setup back-and-forth
  * Once API is fixed, migration should be "just do" kind of work
  * Should be able to run real dashboard & spooler end-to-end (see `~/repos/Spark` for host-side stack)
4. DOING: Port everything else, while tidying up API and writing tests. Ocassional real-device QA.
5. Feature parity; end-to-end real job finishes.
6. Drop dead features from C for fair comparison.
7. Refactor / re-archictecture.

# Design
* time / data-flow design
  * 1ms central tick loop
    * everything requiring faster than 1ms is encapsulated as a hardware driver with its own hardware timers or DMAs, exposing only "slow" (1ms) API
  * no other Timer::after calls
