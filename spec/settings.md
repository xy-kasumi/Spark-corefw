
## Supported settings
Keys marked with `(future)` will be candidates for future addition,
but currently unavailable.

* m.{0,1,2}.{thresh,microstep,current,unitsteps}
	* unitsteps = fullsteps for moving +1 unit (1 mm or 1 rotation)
		* can be negative
* (future) a.{x,y,z}.{maxtravel}
	* mm
	* 0: infinite
	* violation of this is serious error (results in auto-cancel)
* (future) a.{x,y,z}.{maxvel}
	* mm/sec
* (future) a.{x,y,z}.{maxacc}
	* mm/sec2
* a.{x,y,z}.home.{side,origin, (future)phase}
	* side
		* -1: home towards negative side
		* 1: home towards positive side
	* origin
		* value (home position's coordinate)
	* phase
		* 0, 1, 2... (nat-number)
		* when auto-homing, phase is executed sequentially
		* same-phase axes are homed simultaneously
