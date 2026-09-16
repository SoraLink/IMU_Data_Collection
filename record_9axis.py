"""
Sensor-frame 9-axis recording for all 17 MTw sensors -> CSV + metadata + .mtb.

What this records, and why it is not what MVNX datasets publish
--------------------------------------------------------------
Awinda MTws stream SDI (strap-down integrated) data. The wireless master
rejects setOutputConfiguration(); calibrated sensor-frame channels are instead
produced by the XDA processing layer, switched on via XSO_Calibrate.

  acc_[xyz]   calibrated specific force, SENSOR frame, m/s^2  (includes gravity)
  gyr_[xyz]   calibrated angular velocity, SENSOR frame, rad/s
  mag_[xyz]   calibrated magnetic field, SENSOR frame, arbitrary units
  q_[wxyz]    XKF orientation estimate, sensor-to-global

The acc/gyr/mag triplet is the sensor-frame raw signal that MVNX exports do not
contain -- they only ship world-frame gravity-removed solver output.

Timing
------
At 60 Hz each sample is the integral over a 16.7 ms interval, NOT an
instantaneous sample. Clipping at the 1000 Hz internal rate is averaged away and
is not observable in this data.

Use `counter` (the MTw packet counter) as the authoritative timeline -- never
host arrival time, which carries radio jitter and retransmission delay. The
counter is 16-bit and wraps at 65536; unwrap it during analysis.

Usage
-----
    python record_9axis.py --subject S01 --trial sprint_01 --seconds 20
    python record_9axis.py --subject S01 --trial longjump_01      # Ctrl+C to stop
"""

import argparse
import csv
import json
import os
import threading
import time
from datetime import datetime, timezone

import xsensdeviceapi as xda

from skeleton import SENSOR_MAP
from pipeline import RADIO_CHANNEL, DESIRED_RATE, find_master

OUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

FIELDS = ["counter", "sample_time_fine",
          "acc_x", "acc_y", "acc_z",
          "gyr_x", "gyr_y", "gyr_z",
          "mag_x", "mag_y", "mag_z",
          "q_w", "q_x", "q_y", "q_z"]


class Recorder(xda.XsCallback):
    """Buffers packets in memory. Writing to disk inside the callback would
    stall the SDK receive thread, so the buffer is drained after the take."""

    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.rows = []

    def onLiveDataAvailable(self, dev, packet):
        if packet is None:
            return

        counter = packet.packetCounter() if packet.containsPacketCounter() else None
        stf = packet.sampleTimeFine() if packet.containsSampleTimeFine() else None

        if packet.containsCalibratedAcceleration():
            a = packet.calibratedAcceleration()
            ax, ay, az = a[0], a[1], a[2]
        else:
            ax = ay = az = None

        if packet.containsCalibratedGyroscopeData():
            g = packet.calibratedGyroscopeData()
            gx, gy, gz = g[0], g[1], g[2]
        else:
            gx = gy = gz = None

        if packet.containsCalibratedMagneticField():
            m = packet.calibratedMagneticField()
            mx, my, mz = m[0], m[1], m[2]
        else:
            mx = my = mz = None

        if packet.containsOrientation():
            q = packet.orientationQuaternion()
            qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        else:
            qw = qx = qy = qz = None

        row = (counter, stf, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz)
        with self.lock:
            self.rows.append(row)

    def drain(self):
        with self.lock:
            rows, self.rows = self.rows, []
            return rows

    def count(self):
        with self.lock:
            return len(self.rows)


def device_property(dev, names):
    """Read the first of `names` this SDK build actually implements.

    The Windows build spells the full-scale-range getters actualAccelerometerRange
    / actualGyroscopeRange; the Linux 2022.2 build drops the prefix. Trying both
    keeps one metadata.json schema across platforms.
    """
    missing = []
    for name in names:
        try:
            value = getattr(dev, name)()
        except AttributeError as e:
            missing.append(str(e))
            continue
        except Exception as e:
            return f"unavailable ({e})"
        if isinstance(value, (int, float)):
            # numpy scalars pass this check but are not json-serialisable.
            return value.item() if hasattr(value, "item") else value
        # XsVersion and friends str() to a useless SWIG proxy repr.
        for render in ("toSimpleString", "toXsString"):
            if hasattr(value, render):
                return str(getattr(value, render)())
        return str(value)
    return f"unavailable ({'; '.join(missing)})"


def packet_loss(rows):
    """Gaps in the 16-bit packet counter -> dropped samples that never arrived."""
    counters = [r[0] for r in rows if r[0] is not None]
    if len(counters) < 2:
        return None
    gaps, expected = 0, 0
    for prev, cur in zip(counters, counters[1:]):
        step = (cur - prev) % 65536
        expected += step
        if step > 1:
            gaps += step - 1
    # int() because the counters arrive as numpy scalars, which json rejects.
    return {"received": len(counters), "expected_span": int(expected) + 1,
            "missing": int(gaps),
            "loss_pct": round(100.0 * int(gaps) / max(int(expected) + 1, 1), 4)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--subject", default="unknown", help="subject id, e.g. S01")
    ap.add_argument("--trial", default=None, help="trial name, e.g. sprint_01")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop automatically after this long (default: Ctrl+C)")
    ap.add_argument("--notes", default="", help="free-text note stored in metadata")
    ap.add_argument("--no-mtb", action="store_true", help="skip the .mtb log")
    ap.add_argument("--release-radio", action="store_true",
                    help="disable the radio on exit, which powers every MTw "
                         "down. Off by default so consecutive takes don't need "
                         "all 17 sensors switched on again by hand; use it when "
                         "you're finished and want to save their batteries")
    ap.add_argument("--connect-timeout", type=float, default=300.0,
                    help="seconds to wait for the MTws to join (default 300). "
                         "They join within seconds of being switched on, so this "
                         "is really budget for you to walk over and switch them on")
    args = ap.parse_args()

    trial = args.trial or datetime.now().strftime("%H%M%S")
    stamp = datetime.now().strftime("%Y%m%d")
    out_dir = os.path.join(OUT_ROOT, args.subject, f"{stamp}_{trial}")
    os.makedirs(out_dir, exist_ok=True)

    control, port = find_master()
    # Awinda has no configurable output list; this is what produces the
    # calibrated sensor-frame channels.
    control.setOptions(xda.XSO_Calibrate | xda.XSO_Orientation, xda.XSO_None)
    print(f"Master {port.deviceId()} on {port.portName()}")
    if not control.openPort(port):
        raise RuntimeError(f"openPort failed: {control.lastResultText()}")

    master = control.device(port.deviceId())
    master.gotoConfig()

    rates = master.supportedUpdateRates()
    rates = [rates[i] for i in range(rates.size())]
    rate = min(rates, key=lambda r: abs(r - DESIRED_RATE)) if rates else DESIRED_RATE

    # The master only accepts a new update rate while its radio is down, and
    # setUpdateRate() reports failure by return value rather than raising -- so
    # an unchecked call on a live radio silently leaves the old rate in place.
    # Only cycle the radio when the rate actually has to change, since dropping
    # it powers every MTw off.
    if master.updateRate() != rate:
        if master.isRadioEnabled():
            print(f"  changing update rate to {rate} Hz -- this drops the radio, "
                  f"so the MTws will need switching on again")
            master.disableRadio()
        if not master.setUpdateRate(rate):
            raise RuntimeError(f"setUpdateRate({rate}) failed")

    if not master.isRadioEnabled():
        if not master.enableRadio(RADIO_CHANNEL):
            raise RuntimeError(f"enableRadio({RADIO_CHANNEL}) failed")

    rate = master.updateRate()   # what the master will actually deliver
    print(f"Update rate: {rate} Hz  (supported: {rates})")

    expected = len(SENSOR_MAP)
    print(f"Radio on (channel {RADIO_CHANNEL}). Waiting for {expected} sensors...")
    print(f"  SWITCH THE MTws ON NOW if they aren't already -- they join within "
          f"seconds of powering up.\n  Giving up after {args.connect_timeout:.0f}s.")
    count, last_change, start = 0, time.time(), time.time()
    while True:
        now = time.time()
        n = master.children().size()
        if n != count:
            count, last_change = n, now
            print(f"  connected: {n}/{expected}")
        if n >= expected:
            break
        if now - last_change > 4.0 and n > 0:
            print(f"  settled at {n}/{expected}")
            break
        if now - start > args.connect_timeout:
            raise RuntimeError(
                f"Only {n} sensors connected after {args.connect_timeout:.0f}s")
        # Nothing joining yet looks identical to a hang otherwise, which is how
        # a too-short timeout gets mistaken for dead hardware.
        if n == 0 and int(now - start) % 10 == 0 and now - start >= 10:
            print(f"  still waiting... {now - start:.0f}s", flush=True)
            time.sleep(1.0)
        time.sleep(0.2)

    mtb_path = os.path.join(out_dir, "session.mtb")
    if not args.no_mtb:
        if master.createLogFile(mtb_path) != xda.XRV_OK:
            raise RuntimeError(f"createLogFile failed: {mtb_path}")

    if not master.gotoMeasurement():
        raise RuntimeError("Could not enter measurement mode")

    # Device instances (and their reported ranges) are only available once
    # measuring. actualAccelerometerRange/actualGyroscopeRange give the FSR
    # per sensor, which C3 requires us to publish alongside the signals.
    recorders, sensors = {}, {}
    ids = control.deviceIds()
    for i in range(ids.size()):
        did = ids[i]
        if not did.isMtw():
            continue
        key = str(did)
        seg = SENSOR_MAP.get(key)
        if seg is None:
            print(f"  ignoring unmapped sensor {key}")
            continue
        dev = control.device(did)
        rec = Recorder()
        dev.addCallbackHandler(rec)
        recorders[seg] = rec

        info = {"device_id": key}
        for label, names in (
                ("acc_range_g", ("actualAccelerometerRange", "accelerometerRange")),
                ("gyr_range_dps", ("actualGyroscopeRange", "gyroscopeRange")),
                ("product_code", ("productCode",)),
                ("firmware", ("firmwareVersion",))):
            info[label] = device_property(dev, names)
        sensors[seg] = info

    print(f"Recording {len(recorders)} sensors.")
    rng = {s: (i.get("acc_range_g"), i.get("gyr_range_dps"))
           for s, i in sensors.items()}
    uniq = set(rng.values())
    print(f"  FSR (acc g / gyr dps): {uniq if len(uniq) > 1 else next(iter(uniq), '?')}")

    if not args.no_mtb:
        master.startRecording()

    t0_wall = datetime.now(timezone.utc).isoformat()
    t0 = time.time()
    try:
        print("\nRecording... Ctrl+C to stop.\n")
        while True:
            time.sleep(1.0)
            el = time.time() - t0
            total = sum(r.count() for r in recorders.values())
            print(f"  {el:6.1f} s   {total:8d} samples   "
                  f"{total / max(el, 1e-9) / max(len(recorders), 1):5.1f} Hz/sensor",
                  flush=True)
            if args.seconds and el >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\nStopped by user.")
    duration = time.time() - t0
    t1_wall = datetime.now(timezone.utc).isoformat()

    print("\nStopping...")
    if not args.no_mtb:
        master.stopRecording()
        master.closeLogFile()
    master.gotoConfig()
    if args.release_radio:
        master.disableRadio()
        print("  radio off -- all MTws powered down")
    else:
        # Dropping the radio powers every MTw off, so the next take would need
        # all 17 switched on by hand again. Leave the link up by default.
        master.setGotoConfigOnClose(False)
        print("  radio left on -- MTws stay powered for the next take "
              "(--release-radio to power them down)")

    print(f"\nWriting to {out_dir}")
    stats = {}
    for seg in sorted(recorders):
        rows = recorders[seg].drain()
        path = os.path.join(out_dir, f"{seg}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(FIELDS)
            w.writerows(rows)
        loss = packet_loss(rows)
        stats[seg] = {"samples": len(rows), "packet_loss": loss}
        lp = f"{loss['loss_pct']:.3f}%" if loss else "n/a"
        print(f"  {seg:<16} {len(rows):6d} samples   loss {lp}")

    meta = {
        "subject": args.subject,
        "trial": trial,
        "notes": args.notes,
        "started_utc": t0_wall,
        "ended_utc": t1_wall,
        "duration_s": round(duration, 3),
        "update_rate_hz": rate,
        "supported_rates_hz": rates,
        "radio_channel": RADIO_CHANNEL,
        "master_id": str(port.deviceId()),
        "frames": {
            "acc": "sensor frame, m/s^2, specific force (gravity included)",
            "gyr": "sensor frame, rad/s",
            "mag": "sensor frame, arbitrary units",
            "q": "sensor-to-global, XKF estimate, (w,x,y,z)",
        },
        "timing_note": ("60 Hz samples are 16.7 ms interval integrals (SDI), not "
                        "instantaneous. Use `counter` (16-bit, wraps at 65536) as "
                        "the timeline, not host arrival time."),
        "sensors": sensors,
        "stats": stats,
    }
    with open(os.path.join(out_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    control.close()

    counts = [s["samples"] for s in stats.values()]
    worst = max((s["packet_loss"]["loss_pct"] for s in stats.values()
                 if s["packet_loss"]), default=0.0)
    print(f"\nDuration {duration:.1f} s | {len(stats)} sensors | "
          f"samples {min(counts) if counts else 0}-{max(counts) if counts else 0} | "
          f"worst packet loss {worst:.3f}%")
    print(f"metadata.json written. Done.")


if __name__ == "__main__":
    main()
