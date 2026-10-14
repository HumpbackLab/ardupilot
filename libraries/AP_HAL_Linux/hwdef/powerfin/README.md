# PowerFin Linux board

This target uses the PowerFin OpenHarmony musl userspace and the kernel interfaces
used by the `humpback/powerfin` PX4 target. It has four FlexBus DShot outputs,
an SC7U22 IMU through IIO, an I2C QMC5883P compass and SPA06 barometer, a
MAX7456-compatible SPI OSD, and a battery voltage IIO channel. Current
sensing is unavailable.

Build a static Copter binary with the PowerFin toolchain:

```sh
./waf configure --board powerfin --static
./waf copter
```

The executable is `build/powerfin/bin/arducopter`. Copy it and `start.sh` to
the same writable directory on the board. Stop the PX4 process so it releases
the sensor and serial devices, then run as root:

```sh
./start.sh
```

The script loads the `sc7u22` kernel module, creates the log and terrain
directories, and starts ArduPilot with the PowerFin serial map. UDP port
14550 accepts a ground station connection. The device tree must already
expose the sensor and DShot interfaces. Pass extra ArduPilot options to the
script as arguments.

To start it at boot, register this script with the PowerFin image's init
system after bench validation; the repository does not install a service.

The startup script maps `SERIAL1` through `SERIAL5` to `/dev/ttyS1` through
`/dev/ttyS5` using `--serialN` arguments. Configure their protocols and baud
rates through the ground station. Extra script arguments can override the
device paths.
For an ELRS receiver using CRSF on UART3, set `SERIAL3_PROTOCOL=23`,
`SERIAL3_OPTIONS=0`, and enable bit 13 of `RC_OPTIONS` for 420000 baud,
preserving any other selected bits. RC protocols control their own baud rate.
Restart ArduPilot after changing the serial configuration.
`SERIAL2_PROTOCOL=42` and `OSD_TYPE2=5` assign `/dev/ttyS2` to MSP
DisplayPort alongside the on-board SPI OSD.

The defaults select DShot300 and voltage-only battery monitoring. The battery
IIO device must be named `battery-voltage`; its `in_voltage0_scale` is in
millivolts per count and already includes the hardware divider. The driver
finds its device number at startup. Reboot ArduPilot after changing the IIO
scale or offset. Compare `BAT.Volt` against `(raw + offset) * scale / 1000`
and a voltmeter before relying on battery failsafes.
Set the battery cell count and voltage thresholds for the installed pack.

The SC7U22 IIO adapter requires the seven scan elements and 24-byte layout
used by PX4's `sc7u22_iio` driver, with monotonic timestamps. ArduPilot
refuses to initialize the IMU if that interface is absent or differs. Verify
IMU axes, compass heading, barometer pressure, battery voltage, RC input,
each motor's channel and direction, and disarm behavior on the bench before
flight. The four DShot output numbers follow the kernel driver's channel
order; the physical connector mapping must be checked against the board.
