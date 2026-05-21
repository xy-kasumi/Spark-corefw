# Spark Core

WIP

Firmware for https://github.com/xy-kasumi/Spark — a RepRap-style new-generation
EDM (electric discharge machining) machine. It controls motors, sensors etc. by
consuming commands & G-code from the host.

Written in Rust on [embassy](https://embassy.dev/), targeting the BTT Octopus
Pro (STM32H723).

See
* [docs/gcode.md](docs/gcode.md) Supported G-code
* [docs/settings.md](docs/settings.md) Supported settings
* [docs/protocol.md](docs/protocol.md) Serial communication protocol

## Layout
* `model/` — portable compute (G-code parser, motion, coordinates, protocol).
  Host-testable, no hardware dependency.
* `firmware/` — the on-device binary: embassy app and hardware drivers.

## License
AGPL-3.0-or-later, following the main Spark project policy.

## Building & Testing
```shell
./build.sh   # cross-compile the firmware binary (release)
./test.sh    # run host-side tests for the model crate
```

## Flashing & on-target diagnostics
Flashing uses `openocd` (not probe-rs).
* For the Raspberry Pi Debug Probe, probe-rs requires V2.x firmware, and
  something in V2.x prevents it from working under WSL2.

```shell
./flash.sh        # build + program + verify + reset via OpenOCD
./panic_diag.sh   # post-mortem: read the PANIC_REPORT breadcrumb from a stuck target
```
`panic_diag.sh` also needs `nm`, and only works when the exact ELF matching the
running hardware is available.

## Environment
* Windows PC, Ubuntu in WSL 2, Raspberry Pi Debug Probe connected to the PC.

Attach the probe to WSL2 from an admin PowerShell:
```powershell
winget install usbipd
usbipd list                          # find the BUSID containing "CMSIS-DAP"
usbipd attach --wsl --busid=<BUSID>
```
Then follow Zephyr's [udev rules guide](https://docs.zephyrproject.org/latest/develop/beyond-GSG.html#setting-udev-rules)
to set USB device permissions, and reconnect the probe.

Note: attaching to WSL2 makes the probe disappear from Windows, so use a
terminal emulator inside WSL2. To manually connect and run commands:
```shell
tio --map OCRNL,ONLCRNL,INLCRNL --local-echo /dev/ttyACM0
```

### Board physical connection
See [photo for Octopus Pro](docs/board_pins.png).

## Fun Fact
* This code was written in C + Zephyr up until commit `ce0342d` (2026-05-21).
