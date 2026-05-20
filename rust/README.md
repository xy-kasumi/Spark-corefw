# Spark firmware (Rust)

(omitted b/c repo root README already covers it)

## Flashing & on-target diagnostics

`openocd` is necessary for `flash.sh`.
`panic_diag.sh` also needs `nm`.

note: `panic_diag.sh` only works when exactly same ELF binary is available as the running hardware.

## Testing w/o proper host

Use `tio --map OCRNL,ONLCRNL,INLCRNL --local-echo /dev/ttyACM0` (change device to what you have)
to manually connect and run commands.
