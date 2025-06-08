
## Supported settings
Keys marked with `(future)` will be candidates for future addition,
but currently unavailable.

* m.{0,1,2}.{thresh,microstep,current,unitsteps,idlems}
	* thresh = Stall detection threshold for StallGuard
	    * >= 0
		* lower value = more load needed for stall detection
	* unitsteps = fullsteps for moving +1 unit (1 mm or 1 rotation)
		* any non-zero value
		* make it negative to indicate inverse direction
	* current = % of current to apply to motor
	   * 1~100
	* microstep = how many microstep
	   * 1, 2, 4, ..., 128, 256
	* idlems = how long (msec) to wait before de-energizing motor when not moving
	* negative value: always keep energized (use -1)
	* 0~positive value: msec to wait (max is 1000)
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
