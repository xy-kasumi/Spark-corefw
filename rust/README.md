# Spark firmware (Rust)

(omitted b/c repo root README already covers it)

## Flashing & on-target diagnostics

`openocd` is necessary for `flash.sh`.
`panic_diag.sh` also needs `nm`.

note: `panic_diag.sh` only works when exactly same ELF binary is available as the running hardware.
