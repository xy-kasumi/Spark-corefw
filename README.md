# Spark Core

WIP

Zephyr-based firmware for https://github.com/xy-kasumi/Spark.


## Licenses
* Main app code (app/src, tests/app/src): AGPL-3.0-or-later
  * Follows main Spark project policy
* Driver code (drivers/, include/drivers): Apache-2.0
  * Since Zephyr is Apache-2.0, this will make it easier if someone want to put this upstream
* Config files: CC0
  * Don't care, just copy-paste them around

## Environment Setup
* Windows PC
* Ubuntu running in WSL 2
* Raspberry Pi Debug Probe connected to the PC

### Building
Setup Zephyr to WSL2 Ubuntu by following https://docs.zephyrproject.org/latest/develop/getting_started/index.html

Init a new workspace for Spark-corefw by
```shell
west init -m https://github.com/xy-kasumi/Spark-corefw --mr main ~/zephyr-ws-spark
cd ~/zephyr-ws-spark
west update
```

Confirm that it builds
```shell
cd ~/zephyr-ws-spark/Spark-corefw
west build -b octopus_pro app
```

### Flashing
Run powershell as admin.
`usbpid list` will list USB devices on windows. Find the `BUSID` of the probe, which contains "CMSIS-DAP".

```powershell
winget install usbipd
usbipd list
usbipd attach --wsl --busid=<BUSID>
```

Follow this in WSL2 to setup USB device permission.
* https://docs.zephyrproject.org/latest/develop/beyond-GSG.html#setting-udev-rules

Reconnect the USB device.

`west flash` should now work.
```shell
cd ~/zephyr-ws-spark/Spark-corefw
west flash
```

Note: this process makes the probe disappear from Windows land.
You need to use terminal emulator inside WSL2.

I'm using `tio /dev/ttyACM0 --local-echo`

### Board Physical Connection
See [photo for Octopus Pro](boards/btt/octopus_pro/board_pins.png)

### Tips
* To make VSCode IntelliSense work, open `~/zephyr-ws-spark` folder
* I'm running claude at `~/zephyr-ws-spark/Spark-corefw` rather than the workspace.
