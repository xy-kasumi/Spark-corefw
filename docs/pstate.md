
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
* `retr_rem` (float | undef): Distance (mm) available to retract before hitting the history limit. Undef if unavailable (not moving).
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
