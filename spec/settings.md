

* m.{0,1,2}.{thresh,microstep,current,unitsteps}
	* unitsteps = fullsteps for moving +1 unit (1 mm or 1 rotation)
		* can be negative
* a.{x,y,z}.{maxtravel}
	* mm
	* 0: infinite
	* violation of this is serious error (results in auto-cancel)
* a.{x,y,z}.{maxvel}
	* mm/sec
* a.{x,y,z}.{macacc}
	* mm/sec2
* a.{x,y,z}.home.{side,origin,phase}
	* side
		* -1: min side is the home switch
		* 1: max side is the home switch
	* origin
		* value (home position's coordinate)
	* phase
		* 0, 1, 2... (nat-number)
		* when auto-homing, phase is executed sequential
		* same-phase axes are homed simultaneously
