#!/usr/bin/env bash
# Post-mortem for a stuck/silent firmware: halt the target over SWD, read the
# PANIC_REPORT breadcrumb left by the diagnostic panic handler, and print where
# the panic happened. Does NOT reset the target (so state is preserved); resumes
# it on exit.
#
# Usage: ./panic_diag.sh
set -uo pipefail
cd "$(dirname "$0")"

# Host tools, not the Zephyr SDK: openocd from PATH (apt install openocd, or an
# xpack build), and any nm to read the PANIC_REPORT symbol. Set OPENOCD_SCRIPTS
# only for a relocated scripts dir.
OPENOCD="${OPENOCD:-openocd}"
NM="${NM:-nm}"
ELF="${ELF:-firmware/target/thumbv7em-none-eabihf/release/firmware}"
OOCD=(-f interface/cmsis-dap.cfg -f target/stm32h7x.cfg)
[ -n "${OPENOCD_SCRIPTS:-}" ] && OOCD=(-s "$OPENOCD_SCRIPTS" "${OOCD[@]}")
MAGIC=0x50414e31

ADDR=$("$NM" "$ELF" | awk '/ PANIC_REPORT$/{print "0x"$1}')
if [ -z "$ADDR" ]; then echo "PANIC_REPORT symbol not found in $ELF"; exit 1; fi

# read_memory returns a TCL list of words; echo them space-separated.
# Note: $1/$2 expand in bash (double quotes); the Tcl $v is kept literal
# (single quotes) so bash's `set -u` doesn't try to expand it.
read4() {
  "$OPENOCD" "${OOCD[@]}" -c "init; halt" \
    -c "set v [read_memory $1 32 4]" \
    -c 'echo "REPORT $v"' \
    -c "resume; exit" 2>&1 | sed -n 's/^REPORT //p'
}
readstr() {  # addr len -> bytes
  "$OPENOCD" "${OOCD[@]}" -c "init; halt" \
    -c "set v [read_memory $1 8 $2]" \
    -c 'echo "STR $v"' \
    -c "resume; exit" 2>&1 | sed -n 's/^STR //p'
}

words=($(read4 "$ADDR"))
# read_memory returns 0x-prefixed words; normalize to decimal for bash arithmetic.
magic=$(( ${words[0]:-0} )); line=$(( ${words[1]:-0} ))
fptr=$(( ${words[2]:-0} )); flen=$(( ${words[3]:-0} ))
if [ "$magic" -ne $(( MAGIC )) ]; then
  printf 'No panic recorded (magic=0x%08x). Target may be hung without a panic, or never panicked.\n' "$magic"
  exit 0
fi

file="?"
if [ "$fptr" -ne 0 ] && [ "$flen" -gt 0 ] && [ "$flen" -lt 256 ]; then
  bytes=($(readstr "$(printf '0x%x' "$fptr")" "$flen"))
  file=$(printf '%b' "$(printf '\\x%02x' "${bytes[@]}")")
fi
echo "PANIC recorded at: $file:$line"
