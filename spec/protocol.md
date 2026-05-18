# Protocol V2
This protocol design aims
* simple implemenation
* human-usable: usable from serial terminal
* core-friendly: assumes core has relatively lower compute power than host

Simplifying assumptions
* Host-core channel properties
  * bandwidth must be >= 10KB/sec (100kbaud/sec)
  * latency is at most a few ms
  * will never reorder or duplicate data
* Host has abundant compute & memory


## Terminology & Syntax
We use up and down throughout, to denote each direction.
* "up": "towards human" direction (e.g. core -> host)
* "down": "from human" direction (e.g. host -> core)

We use [RFC7405](https://datatracker.ietf.org/doc/html/rfc7405) ABNF to describe formats.

## Transport Layer & Line Contents
Transport layer is full-duplex. Line is as follows.
Each `line` is at most 106 byte (10.6ms or less).

```
line = payload LF
payload = 1*100VCHAR
```

Implementations SHOULD silently ignore CR (0x0D) present on the channel.
Receivers MUST silently discard non-conformant or checksum-error lines silently as channel error.


## Application Layer
Downlink payload has two types:
* signal: single-character line starting from special character like "!" or "?"
* command: anything that is not a signal

Commands will be always queued internally, and executed sequentially.
Signals will execute immediately, regardless of whether a command is running or not.

Uplink payload is always parallel state (p-state). P-state is semi-structured data about current state of the core.
P-state is designed to allow concurrent state reporing to be interleaved.

P-state data format
```
p-state = id [ SP "<" ] *(SP key ":" val) [ SP ">" ]

key = *(id ".") id
val
  = val-bool
  / val-float
  / val-hex
  / val-string

id = ALPHA *(ALPHA / DIGIT / "_")
val-bool = "true" / "false"
val-float = ?regex -?[0-9.]+?
val-hex = "0x" 1*HEXDIG
val-string = DQUOTE *string-char DQUOTE
string-char
  = %0x21 / %0x23-0x5b / %0x5d-7e  ; VCHAR other than " or \
  / '\\'
  / '\"'
```

P-state value can be bool, float32, uint32, string.
uint32 always is `number-hex` format. Other numbers are treated as float32.
String cannot contain newlines.

Valid Examples
```
stg < m.1.microstep:1 m.2.microstep:2 m.3.microstep:3
stg m.4.microstep:3 m.4.microstep:5 >
stg < >
```

"<" indicates start of a new p-state. ">" indicates completion.
Note that ">" that corresponds to "<" might be not present when the operation is canceled.

Core SHOULD NOT insert processing delay within a line.
Core MAY insert arbirary processing delay between lines.

Host MAY choose to present incomplete p-state to the user.

### Signals
If core receives same signal while the previous signal is being processed,
core SHOULD drop the new one to avoid resource starvation.
When dropping a signal, transport layer MUST still ack it.

#### "!": Cancel
Stop execution of current command (if any), and clear the queue.

#### "?": Query
Respond with current status.

* `?queue`: query queue
* `?pos`: query pos

### Commands
#### "set": Set Setting
Set single setting entry. See settings.md for list of settings.

```
set-command = "set" space key space value
space = { " " }
```

Example
```
set m.6.microstep 32
```

#### "get": Get Settings
Get all setting entries. See settings.md for list of settings.

#### "stat": Dump Status
Dump software & hardware internal status useful for debugging the firmware or hardware.
Unlike "?" signal which responds immediately, stat can take time to query peripherals, run self-check etc.

#### "test": Execute Hardware Tests
Execute potentially unsafe operation to test the hardware.
See main.c for details.

#### G-Code
Commands starting with "G" or "M". See gcode.md for details.

### Parallel States
* Event-driven: Reported in pre-defined ocassions
* Signal-driven: Reported in response to certain signals
* Command-driven: Reported in response to certain commands

In future extensions, periodic time-series data SHOULD use dedicated signal-driven p-states.
Manually triggered data retrieval SHOULD extend "stat" if possible.

#### `queue`: Signal-driven ("?queue")
`queue` reports current command queue status.
Host SHOULD query queue fast enough to achieve 75% fill rate (= num / cap) for stable command streaming.

Keys
* `cap`: total capacity of the queue
* `num`: number of items in the queue, including currently running commands

`num == 0` means that the core is idle.

Example
```
queue < cap:100 num:54 >
```

#### `pos`: Signal-driven ("?pos")
`pos` reports current coordinates and coordinate systems.

Keys
* `sys`: current coordinate system ("machine", "grinder", "work", "toolsupply")
* `m`: machine coordinate
* `g`: grinder coordinate
* `t`: tool supply coordinate
* `w`: work coordinate

`m` will always be present. `g` or `t` or `w` will be present iff it's current coordinate system as defined by `sys`.

#### "edm": Signal-driven ("?edm")
`edm` reports current EDM status.

Keys
* `eff_duty` (float | undef): [0, 1] effective duty ratio (successful pulse duration ratio). Undef if unavailable.
* `open` (float | undef): [0, 1] open time ratio. Undef if data is unavailable (not in EDM mode).
* `short` (float | undef): [0, 1] short time ratio. Undef if data is unavailable (not in EDM mode).
* `pb_f` (float | undef): Remaining position buffer length (mm) in forward direction. Undef if unavailable (not moving).
* `pb_b` (float | undef): Remaining position buffer length (mm) in backward direction. Undef if unavailable (not moving).
* `dist` (float | undef): Cumulative distance from current current continuous motion start. Undef if unavailable (not moving).
* `dist_max` (float | undef): Furthest traveled cumulative distance from current continuous motion start. Undef if unavailable (not moving).

#### "init": Event-driven
`init` reports core initialization status.
Triggered just once after every boot.

Keys
* `ok`: whether the entire core was succesfully initialized
* `<module>.ok` (bool): whether the module was succesfully initialized
* `<module>.msg` (string or undef): error or warning message if available

Example
```
init < ok:false pulser.ok:true motor.ok:false motor.msg:"Failed to change pin XXX" >
```

#### "stat": Command-driven
`stat` reports various modules stats for manual debugging.

Keys
* `<module>.<anything>`: Parameters of each module

#### "stg": Command-driven
`stg` reports current snapshot of all settings.

Example
```
stg m.5.microstep:32 m.6.microstep:16
```

#### "error": Event-driven
`error` reports latest error.

Keys
* `src` (optional): line content (w/o newline or hash) that caused the error. Not populated if non-identifiable.
* `msg`: human-readable error

Note checksum errors are handled by lower-layer, and *not* reported as "error".
