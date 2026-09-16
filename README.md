# IMU Data Collection

Real-time pipeline and recording tools for a 17-sensor Xsens MTw Awinda
system. No MVN Analyze required -- everything here talks to the sensors
directly through the Xsens MT SDK (`xsensdeviceapi`).

## Platform requirement -- x86_64, Windows or Linux

The Xsens SDK (`xsensdeviceapi`) ships **Windows x86/x64 and Linux x86_64**
binaries. Awinda support is frozen at MT Software Suite **2022.2**, whose
stated requirement is an Intel/AMD processor. There is no macOS build and no
ARM build of any kind, and the open-source XDA does not help: it registers
MTi and DOT device classes only, so it cannot drive a wireless master however
you compile it.

The Linux SDK is a free download (no account) and ships Python wheels for
CPython 3.7-3.10:

    https://www.xsens.com/hubfs/Downloads/Software/MTSS/Releases/2022.2.0-stable-Awinda/MT_Software_Suite_linux-x64_2022.2_b7381_r124627.tar.gz

### Running on an Apple Silicon Mac -- verified working

Apple Silicon cannot virtualise x86_64, so the route is full CPU emulation:
a QEMU x86_64 Ubuntu guest with the dongle passed through over USB. That
sounds too slow to work, but the data rate is only ~50 kB/s and it holds up.
**Measured on an M4 Pro: all 17 sensors, a sustained 60.1 Hz per sensor, and
0.166 % worst-case packet loss over a 20 s take** -- indistinguishable from
the hardware's nominal behaviour.

`vm/` contains the whole setup; see `vm/README.md`. The short version:

    brew install qemu
    cd vm && ./prepare-disk.sh && ./run-vm.sh

Two things make this work, and both are worth knowing:

- The Awinda dongle is an **FTDI FT232R** behind Xsens' own USB vendor ID
  (`0x2639:0x0102`, product `AW-DNG2-ANT`). Linux's in-tree `ftdi_sio` has
  carried that ID since kernel 3.18, so the guest binds it to `/dev/ttyUSB0`
  with no driver work at all.
- macOS binds *no* driver to VID `0x2639` (Apple's `AppleUSBFTDI` matches
  only FTDI's own `0x0403`), which is exactly what USB passthrough wants --
  the host is not holding the device, so QEMU can claim it cleanly.

The Windows-on-ARM alternative (Parallels + Prism emulating x64 Python) is
*not* recommended: it additionally needs a hand-modified FTDI ARM64 driver
INF and permanent Windows test-signing mode, because no signed ARM64 driver
binds VID `0x2639`.

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

## Linux SDK quirks (MT Software Suite 2022.2)

- The bundled wheels are built against the **numpy 1.x C ABI**. A default
  `pip install` pulls numpy 2.x and the import dies with "module compiled
  against ABI version 1000009". Pin `numpy<2`.
- Ubuntu cloud images ship a kernel without `ftdi_sio`. Install
  `linux-modules-extra-$(uname -r)`, then `modprobe ftdi_sio`, and add
  `ftdi_sio` to `/etc/modules` so it survives a reboot.
- `XsDevice` spells the full-scale-range getters **without** the `actual`
  prefix here -- `accelerometerRange()` / `gyroscopeRange()`, not
  `actualAccelerometerRange()`. `record_9axis.py` tries both spellings.
- `supportedUpdateRates()` on this 17-sensor setup returns
  `[120, 100, 80, 60, 40]`. Note the radio cannot actually sustain the top
  entries with 17 MTws: the User Manual caps 11-20 sensors at 60 Hz, which
  is what `DESIRED_RATE` already targets.

## Radio and update-rate behaviour -- measured, and worth knowing

Two behaviours cost real debugging time here, so they are written down:

- **Disabling the master's radio powers every MTw off.** A take that ended
  with `disableRadio()` left all 17 sensors dead, and the next run would sit
  there reporting "Only 0 sensors connected" until someone walked over and
  pressed 17 buttons. That looks exactly like a hardware or passthrough fault
  and is neither. Both scripts now leave the radio up on exit; pass
  `--release-radio` to `record_9axis.py` when you actually want the sensors
  powered down. Switched-on MTws rejoin in about two seconds.
- **`setUpdateRate()` is ignored while the radio is live, and reports it only
  through its return value.** The original code called it before dropping the
  radio and did not check the result, so a second consecutive take silently
  recorded at 40 Hz while `metadata.json` claimed 60. Both scripts now change
  the rate only when it differs (dropping the radio just for that), check the
  return value, and record `master.updateRate()` -- the rate actually in
  force -- rather than the one that was requested.
