#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Probe host: a system openocd on PATH by default (apt install openocd, or an
# xpack build). Set OPENOCD_SCRIPTS only for a relocated scripts dir; a normal
# install resolves its bundled cmsis-dap/stm32h7x configs on its own.
OPENOCD="${OPENOCD:-openocd}"
ELF="firmware/target/thumbv7em-none-eabihf/release/firmware"

oocd=(-f interface/cmsis-dap.cfg -f target/stm32h7x.cfg)
[ -n "${OPENOCD_SCRIPTS:-}" ] && oocd=(-s "$OPENOCD_SCRIPTS" "${oocd[@]}")

(cd firmware && cargo build --release)

"$OPENOCD" "${oocd[@]}" -c "program $ELF verify reset exit"
