# Protocol V2
This protocol design aims
* simple implemenation
* human-usable: usable from serial terminal
* reliable streaming: can remedy communication loss or error
* core-friendly: assumes core has relatively lower compute power than host

Simplifying assumptions
* Host-core channel properties
  * bandwidth must be >= 10KB/sec (100kbaud/sec)
  * latency is at most a few ms
  * will never reorder or duplicate data
* Host has abundant compute & memory

## Terminology
We use up and down throughout, to denote each direction.
* "up": "towards human" direction (e.g. core -> host)
* "down": "from human" direction (e.g. host -> core)

## Transport Layer & Line Contents
Transport layer is full-duplex. Line is as follows.

```
line
  = payload "\n"
  | payload seq checksum "\n"
  | "ack" seq checksum "\n"

payload = ?printable characters?
seq = "*" (* 0 *) | "+" (* 1 *)
checksum = ?regex [0-9a-f]{4}?
```

`payload` cannot be "ack". "\n" is LF (0x0A).
Implementations SHOULD silently ignore CR (0x0D) present on the channel.

`payload` MUST be 100 byte or less. Making each `line` at most 106 byte (10.6ms or less).
`payload` SHOULD be ASCII printable characters (0x20 to 0x7e, inclusive). However, implementation MAY use UTF-8.

`checksum` is calculated from `payload`+`seq` or "ack"+`seq`, using CRC-16/CCITT-FALSE.
Receivers MUST silently discard non-conformant or checksum-error lines silently as channel error.

### Transport Upgrade
There are two modes of tranport:
* Interactive mode: both ends will not compute checksum nor send ack. Assume no comm error. Never resend.
* Machine mode: both ends compute checksum & sends ack. Both ends respect ack and resend.

The protocol starts from interactive mode.
Core implementation MAY use control characters to provide shell-like interactive line editing.
In this case the protocol assumes `payload` is post-edit content.

Host can initiate upgrade to machine mode by sending ack.
Upon reception of ack, core enters machine mode and sends ack immediately.
When host receives ack within 50ms, transport enters machine mode. Otherwise, host must resend ack.
Until transport upgrade is complete, host MUST NOT send other payloads.
During upgrade, ack with "*" is used, resulting in `ack*3B65`.
When upgrade is complete, both ends expects/sends "+".

There's no transition from machine mode to interactive mode.

### Resend & Flow Control
Sender-side pseudo-code
```
const num_retries: int
curr_seq: bool

fn on_payload_to_send_available(payload):
  loop(num_retries):
    send(payload, curr_seq)
    wait:
      receive ack(curr_seq):
        curr_seq = !curr_seq
        return
      pass 50ms:
        continue
  
  error("channel is broken")
```

Receiver-side pseudo-code
```
expect_seq: bool

fn on_incoming_valid_payload(payload, seq):
  if seq == expect_seq:
    # accept must be immediate
    if accept(payload):
      # accepted
      send_ack(seq)
      expect_seq = !expect_seq
    else:
      # ignore if receiver is full
  else:
    # duplicate payload due to resend (must have been processed in previous cycle)
    # re-send previous ack, but do not accept
    send_ack(seq)
```

Receiver MUST send ack immediately when channel become available.

Seen from the sender, worst case expected delay is 11ms = 10.6ms (full line in transit) + 0.4ms (ack).
With 50ms wait, implementation and channel latency is allowed to have 39ms in total to do processing.


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
p-state = id [ " <" ] { " " key ":" value } [ " >" ]

id = ?regex [A-Za-z0-9_]+?

key = id | { id "." } id
value
  = "true" | "false"
  | number-float
  | number-hex
  | '"' [{ string-char }] '"'

string-char = ?ascii char other than " or \ or control chars.? | '\\' | '\"'

number-float = ?regex -?[0-9.]+?
number-hex = ?regex 0x[0-9a-f]+?
```

P-state value can be bool, float32, uint32, string.
uint32 always is `number-hex` format. Other numbers are treated as float32.
String cannot contain newlines.

Valid Examples
```
stg < m.1.microstep:1 m.2.microstep:2 m.3.microstep:3
stg m.4.microstep:3 m.4.microstep:5 >
stg <>
```

Invalid Examples
```
stg a > b
stg < a b > < c d >
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

#### "download": Download Latest Available Data
Download latest available log data.

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
* `num`: number of items in the queue

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
* `open` (float | undef): [0, 1] open time ratio. Undef if data is unavailable (not in EDM mode).
* `short` (float | undef): [0, 1] short time ratio. Undef if data is unavailable (not in EDM mode).
* `pb_f` (float | undef): Remaining position buffer length (mm) in forward direction. Undef if unavailable (not moving).
* `pb_b` (float | undef): Remaining position buffer length (mm) in backward direction. Undef if unavailable (not moving).
* `dist` (float | undef): Cumulative distance from current current continuous motion start. Undef if unavailable (not moving).
* `dist_max` (float |undef): Furthest traveled cumulative distance from current continuous motion start. Undef if unavailable (not moving).

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

#### "blob": Command-driven
`blob` reports a single blob.

* `0`,`1`,...: N-th payload (urlsafe base64 w/o "=")

Must be concatenated to form a single blob.
