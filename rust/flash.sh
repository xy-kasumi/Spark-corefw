#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

OPENOCD="${OPENOCD:-$HOME/zephyr-sdk-0.17.1/sysroots/x86_64-pokysdk-linux/usr/bin/openocd}"
OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-$HOME/zephyr-sdk-0.17.1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts}"
ELF="target/thumbv7em-none-eabihf/release/spark-corefw-rs"

cargo build --release

"$OPENOCD" -s "$OPENOCD_SCRIPTS" \
  -f interface/cmsis-dap.cfg \
  -f target/stm32h7x.cfg \
  -c "program $ELF verify reset exit"
