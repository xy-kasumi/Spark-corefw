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
