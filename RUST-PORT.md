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

Goal criteria ("feature parity")
* I define "feature parity" as:
  * based on live code path on commit `a110d3`
    * Want-to-haves does not need impl.
  * No addition of not currently implemented features (EEPROM serialize, USB, whatever)
  * Do not assume any specs or hardware as frozen. They must be equally / more amenable to change than C code.
  * They should be behaviorally same when seem from the device user (comm host)
    * No need to maintain byte-level same behavior (but doing so would be easier for port task)
    * FW-host comm can accept added latency of ~50ms if ever needed.
    * For device control, degradation is not allowed
      * Most critical one would be pulse step generator (relying on hardware timer + ISR)
      * Other fastest loop is 1ms; should be easy target
* corollary
  * TMC driver code must be ported too, at some point
  * No need to keep same clock tree setup as long as we can achieve same performance
    * I actually prefer less custom configuration and sticking to whatever library default

# Steps

1. DONE: Decide on crates division, establish UART (different from the spec'd protocol) with host on real device.
  * Basics first
2. DONE: Attack TMC UART & stepper driver. Impl minimum comm to check if it's working on real device.
  * Biggest hardware unknown, and probably least testable & least clean part of the code
3. Big-port: design all the APIs, while impls are mostly not-implemented.
  * Lots of work on API design, test setup back-and-forth
  * Once API is fixed, migration should be "just do" kind of work
  * Should be able to run real dashboard & spooler end-to-end (see `~/repos/Spark` for host-side stack)
4. Port everything else, while tidying up API and writing tests. Ocassional real-device QA.

# Design
* time / data-flow design
  * 1ms central tick loop
    * everything requiring faster than 1ms is encapsulated as a hardware driver with its own hardware timers or DMAs, exposing only "slow" (1ms) API
  * no other Timer::after calls
