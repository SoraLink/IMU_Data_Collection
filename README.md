# IMU Data Collection

Real-time pipeline and recording tools for a 17-sensor Xsens MTw Awinda
system. No MVN Analyze required -- everything here talks to the sensors
directly through the Xsens MT SDK (`xsensdeviceapi`).

## Platform requirement -- Windows x64 only

The Xsens SDK (`xsensdeviceapi`) ships Windows binaries only (Win32/x64).
There is no macOS build, and the underlying library reportedly cannot be
compiled for ARM at all -- so even a Windows-on-ARM VM (e.g. Parallels on
Apple Silicon) is unverified and may not work at the driver level. See the
project history for the reasoning; the short version is: **use a real x64
Windows machine, or an x64 Windows VM with USB passthrough, for anything
that opens the Awinda dongle.**

Everything else (reading this code, editing it) works anywhere.

## Setup

1. Install the MT Software Suite (get the SDK wheel matching your Python
   version from `MT SDK/Python/x64/xsensdeviceapi-*.whl`).
2. `pip install numpy matplotlib <path-to-wheel>`
3. Pair your sensors with the Awinda dongle once using MT Manager (records
   the pairing on the dongle itself). Close MT Manager before running
   anything here -- it holds the COM port exclusively.
4. Edit `SENSOR_MAP` in `pipeline.py` with your own sensors' Device IDs
   and body-segment assignment (use `list_sensors.py` to discover IDs).

## Files

- **`list_sensors.py`** -- connection test only. Opens the dongle, enables
  the radio, prints every Device ID that connects. Use this first, before
  touching `pipeline.py`, to confirm the SDK/driver path works and to
  learn your sensors' Device IDs.
- **`pipeline.py`** -- real-time 3D skeleton viewer. Pure geometry, no
  learned model: `global bone orientation = q_sensor(t) * q_sensor(T-pose)^-1`.
  Press `c` in the plot window (you get an 8s countdown with audible beeps
  to get into T-pose, since you can't be at the keyboard and in T-pose at
  the same time), `q` to quit.
- **`record_9axis.py`** -- records sensor-frame 9-axis data (calibrated
  accelerometer, gyroscope, magnetometer) plus orientation, for all mapped
  sensors, to CSV + `metadata.json` + a `.mtb` log. See the module
  docstring for exact units, frames, and why the 60Hz samples are interval
  integrals rather than instantaneous.
- **`smoke_test.py`** -- headless data-path check: connects, streams for a
  few seconds, prints quaternions per segment. No plotting.
- **`probe_packet.py`** -- inspects what fields an actual MTw data packet
  contains and whether output configuration is settable per-device.
  Debugging tool, not part of the normal workflow.

## Known SDK quirks (this MT Software Suite build, 2026.2)

- `XsPortInfoArray` / `XsDevicePtrArray` have no `.at()` -- use `arr[i]`
  and `arr.size()`.
- `control.openPort(portName, baudrate)` times out ("no data received")
  on this Awinda dongle. Pass the whole `XsPortInfo` object instead:
  `control.openPort(port)`.
- The Awinda wireless master rejects `setOutputConfiguration()`. Calibrated
  sensor-frame data is enabled via
  `control.setOptions(xda.XSO_Calibrate | xda.XSO_Orientation, xda.XSO_None)`
  instead.
- `scanPorts()` only probes for ~100ms and occasionally misses the master
  on the first attempt -- retry a few times before giving up.
- MTw data callbacks must be attached to the individual MTw device
  instances (available only after `gotoMeasurement()`), not to the
  wireless master -- the master only emits connectivity events.
