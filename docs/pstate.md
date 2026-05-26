### Tier-1
Stable formats & timings semantics, useable for driving events.

#### `sys`: Event-driven
`sys` reports fundamental system state change.

Example
```
sys <ev:"boot">
sys <ev:"fault">
```

* `boot`: indicates firmware has booted and host can send commands & signals.
* `fault` indicates firmware has "fault mode" by encountered critical error.
  * This state can only be reset by power-cycle.
  * Upon encouter, hardware shifts to "safe" default modes (like `!`), but tries to preserve queryable state for diagnosis.
  * Read commands would still function: `?...`, `get`, `stat`.
  * Cancel (`!`) is no-op.
  * All write commands (G-codes, `set`, `fset`) will be silently ignored.

`boot` and `fault` can only happen at most once in a single power cycle,
and the order will be always `boot` -> `fault`.


#### `queue`: Signal-driven ("?queue")
`queue` reports current command queue status.
Host SHOULD query queue fast enough to achieve 75% fill rate (= num / cap) for stable command streaming.
`?queue` is also suitable for use as heartbeat.

Keys
* `cap`: total capacity of the queue
* `num`: number of items in the queue, including currently running commands

`num == 0` means that the core is idle.

Example
```
queue < cap:100 num:54 >
```

#### "stg": Command-driven
`stg` reports current snapshot of all settings.

Example
```
stg < m.5.microstep:32 m.6.microstep:16 ... >
```


### Tier-2
Format is relatively stable, but timing is not.
Suitable for having fixed UI.

#### `pos`: Signal-driven ("?pos")
`pos` reports current coordinates and coordinate systems.

Keys
* `sys`: current coordinate system ("machine", "grinder", "work", "toolsupply")
* `m`: machine coordinate
* `g`: grinder coordinate
* `t`: tool supply coordinate
* `w`: work coordinate

`m` will always be present. `g` or `t` or `w` will be present iff it's current coordinate system as defined by `sys`.


### Tier-3
Content semantics is in flux. User should be flexible about field change.

#### "edm": Signal-driven ("?edm")
`edm` reports current EDM status.

Keys
* `eff_duty` (float | undef): [0, 1] effective duty ratio (successful pulse duration ratio) (1sec-smoothed). Undef if unavailable.
* `open` (float | undef): [0, 1] open time ratio (1sec-smoothed). Undef if data is unavailable (not in EDM mode).
* `short` (float | undef): [0, 1] short time ratio (1sec-smoothed). Undef if data is unavailable (not in EDM mode).
* `retr_rem` (float | undef): Distance (mm) available to retract before hitting the history limit. Undef if unavailable (not moving).
* `dist` (float | undef): Cumulative distance from current current continuous motion start. Undef if unavailable (not moving).
* `dist_max` (float | undef): Furthest traveled cumulative distance from current continuous motion start. Undef if unavailable (not moving).

#### "error": Event-driven
`error` reports an error.

Keys
* `msg`: human-readable error, target 40 chars or less.


#### "stat": Command-driven
`stat` reports various modules stats for manual debugging.

Keys
* `<module>.<anything>`: Parameters of each module
