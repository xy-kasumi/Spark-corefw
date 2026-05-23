# Serial protocol
Simplifying assumptions
* Host-core channel properties
  * bandwidth must be >= 10KB/sec (100kbaud/sec)
  * latency < 25 ms, low jitter (direct or USB-serial connection)
  * low noise (0 ~ 1 bit / MB level)
* Host has abundant compute, memory, and storage

## Transport
LF-delimited line-based comm in both directions.
* printable chars (ASCII 0x20~0x7e): append to linebuf
* LF (0x0a): finish line buffer

Other bytes are free to use for interactive terminal.
However, programmatic host should not send them, and should ignore when receiving one.

## Command / Response
Commands (host->core) and responses (core->host) are not explicitly linked.
note: we're considering move to more request->response half-duplex format.

Host sends commands (by observing queue), while core sends p-states.

Command summary
* `G...` or `M...`: [G-code](./gcode.md)
* `!`: cancel any execution & empty all queue (not-queued)
* `?pos`, `?edm`, `?queue`: request latest p-state (not-queued)
* `set <key> <val>`: set single config (queued)
  * e.g. `set m.6.microstep 32`
* `get`: dump all settings as p-state (queued)
* `fset <fs-key> <val>`: set single config quickly (not-queued)
* `stat`: dump all debug states as p-state (queued)

Response: everything is  `<pstate-tag> <pstate>`.
* pstate-tag is finite (e.g. `queue`, `stat`). See [pstate.md](./pstate.md).
* pstate is a dictionary where
  * key is hierachical name like "motor.0.step"
  * value is one of bool, float32 (finite), uint32 (hex), string

Multiple p-state of different tags can be interleaved as multiple lines.

### Command details
G-code, when streamed quickly connects back-and-forth.
This matter for `G1` (feed), as EDM control constantly need retractions.

e.g.
```
; X==0
G1 X1  ; X: 0->1
G1 X2  ; X: 1->2. when this gets stuck at X=1.1, it can go back to X=0.9, if this is considered "connected".
```

`!` causes the core to enter cancel window of approx 500ms.
Everything other than `?...` will be ignored in this window.

`set` & `get` shares same settings namespace (addressed by hierarchical key).
See [settings.md](./settings.md).

`fset` operates on special fs namespace.
Currently only one is
```
fset ov.pump_en true ; true enables pump, false respects M8/M9. default: false
```

### Response (P-State)
pstate examples
```
<m.1.microstep:1 m.2.microstep:2 m.3.microstep:3 m.4.microstep:3 m.4.microstep:5>
< >
```

interleaved pstate examples on wire
```
stg < motor.step:1
edm < eff_duty:0.5 >
stg motor.thresh:0 >
```

p-state

rought BNF (not whitespace spec is not clearly defined)
```
pstate-tag = id
pstate = "<" { key ":" val } ">"

key = *(id ".") id
val = val-bool | val-float | val-hex | val-string

id = /[A-Za-z0-9_]+/
val-bool = "true" | "false"
val-float = /-?[0-9]*(.[0-9]+)?/
val-hex = /0x[0-9a-fA-F]*/
val-string = TBD...
```
