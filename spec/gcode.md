# Supported G-Codes

### G0: Fast move
Parameters: X, Y, Z (all optional, but at least one required)

Examples:
```
G0 X12.3
G0 Z123.5 Y-23.5

G0  ; error
```

### G28: Home
Parameters: X, Y, Z (none or just one parameter allowed)

Examples:
```
G28  ; home all-axis according to the settings
G28 X  ; home X-axis

G28 X Y  ; error
```

Coordinates of homed axes will be set to origin value configured by
`a.{x,y,z}.home.origin`.

When all-axis homing (`G28`) is instructed, `a.{x,y,z}.home.phase` will
be used for grouping and ordering of axes.

# Supported M-Codes

### M3: Energize, tool negative voltage
Parameters: P (pulse time in µs), Q (current in A), R (duty cycle %)

TODO: Document expected behavior, safety limits, interaction with motion

Default values:
- P: 500µs
- Q: 1.0A  
- R: 25%

Examples:
```
M3              ; Use all defaults
M3 P750 Q1.5    ; 750µs pulses, 1.5A current, default duty
M3 Q2.0 R30     ; 2A current, 30% duty, default pulse time
```

### M4: Energize, tool positive voltage
Parameters: P (pulse time in µs), Q (current in A), R (duty cycle %)

TODO: Document expected behavior, safety limits, interaction with motion

Default values same as M3.

Examples:
```
M4              ; Use all defaults
M4 P1000 Q0.8   ; 1000µs pulses, 0.8A current, default duty
```

### M5: De-energize
Parameters: None

TODO: Document shutdown behavior, safety procedures

Examples:
```
M5              ; Stop all EDM activity
```

TODO: Document M-code interaction with motion system, error handling, safety interlocks

### (future) M8: Start pump
### (future) M9: Stop pump
### (future) M10: Start grinder wire feeding
Parameters: R (feed rate in mm/min, required)

Examples:
```
M10 R0.1 ; start with wire feed rate of 15mm/min
```

### (future) M11: Stop grinder wire feeding
Parameters: None

Examples:
```
M11  ; Stop wire feed
```
