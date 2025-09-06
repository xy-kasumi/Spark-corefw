## Settings Description
Settings composes of (key, value) pairs.
A key is a string, a value is a float.

Keys are organized in hierarchical ways, splitted by `.`.

Example of valid keys:
```
m.0.thresh
a.x.home.side
cs.w.pos.x
ts.servo.openms
```

## Key List
note: Keys marked with `(future)` will be candidates for future addition, but currently unavailable.

### **m.{0,1,2,3,4,5,6}** Motor
0~6 is stepper motor index.
It's displayed as `mot0`, `mot1` etc in `stat motor`.

* **thresh** Stall detection threshold for StallGuard
	* valid: >= 0
	* lower value: more load needed for stall detection
* **unitsteps** Fullsteps for moving +1 unit (1 mm or 1 rotation)
	* valid: != 0
	* negative values are allowed (invert direction)
* **current** Motor current
	* valid: [1, 100]
	* % of current to apply to motor
* **microstep** Microsteps
	* valid: 1, 2, 4, ..., 128, 256
* **idlems** Motor idle shutdown wait
    * valid: [-1, 1000]
	* duration (msec) to wait before de-energizing motor when not moving
		* negative: always keep energized (use -1)
		* 0 or positive: msec to wait (max is 1000)

### **a.{x,y,z}** Axis
* **home.{side,origin,phase}**
	* **side**
		* valid: -1, 1
		* -1: home towards negative side
		* 1: home towards positive side
	* **origin**
		* valid: any
		* value (home position's coordinate)
	* **phase**
		* valid: 0, 1, 2... (natural number)
		* when auto-homing, phase is executed sequentially
		* same-phase axes are homed simultaneously
* (future) **maxtravel**
	* mm
	* 0: infinite
	* violation of this is serious error (results in auto-cancel)
* (future) **maxvel**
	* mm/sec
* (future) **maxacc**
	* mm/sec2

### **cs.{w,g}** Coordinates
`w` is work coordinates, `g` is grinder coordinates.

* **pos.{x,y,z}** origin of specified (work or grinder) coordinates in machine coordinates
	* valid: any
	* must be configured by user to match machine's spec
* (future) **ofs.{x,y,z}**
	* grinder's origin offset in machine coordinates
	* auto-adjusted by calibration motion (G-code TBD)
	* offset must be at most a few mm; otherwise calibration will fail

### **ts** Tool supply
* **servo.{openms,closems}**
	* valid: [0, 20]
	* Configure tool supply's servo position (on-cycle time) in msec (float).
		* For SG90, valid value is 0.5~2.4.
	* Period is 20ms (50Hz).
