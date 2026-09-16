"""Session recorder for IMU takes that must be hand-aligned to 8-camera video.

Runs inside the x86_64 guest, where the Xsens SDK lives, and serves viewer.html
over HTTP. The browser on the host does the 3D drawing, which keeps rendering
off the emulated CPU, and sends stage commands back as small POSTs.

Protocol per take
-----------------
Every take is one continuous recording containing, in order:

    T-POSE  ->  JUMP IN  ->  ACTION  ->  JUMP OUT

The T-pose gives a per-take calibration reference. The two jumps bracket the
action with a signal that is unmistakable in both the IMU stream and the video:
free-fall drops |acc| toward zero, landing spikes it. Having one at each end
means the clip can be aligned at its start and checked at its end, which also
exposes any drift across the take.

The operator advances each stage by hand, so the stage marks are approximate.
That is fine -- they only need to bracket the search. Within each jump window
the exact free-fall minimum and landing peak are found from the pelvis
accelerometer and written to metadata as packet counters, which is what you
actually align against.

Timing
------
`counter` (the MTw packet counter) is the authoritative timeline, not host
arrival time, which carries radio jitter. It is 16-bit and wraps at 65536;
unwrap it during analysis. All 17 sensors share the master's counter, so a
counter value identifies the same instant across every sensor.

Usage
-----
    python3 -u session_record.py --subject S01
    python3 -u session_record.py --subject S01 --port 9000
"""

import argparse
import csv
import json
import os
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import xsensdeviceapi as xda

from skeleton import (SENSOR_MAP, REFERENCE_SEGMENT, SEGMENT_LABELS,
                      SEGMENT_TO_DEVICE, SKELETON, TIPS, BONES, TIP_BONES,
                      limb_colour, joint_positions, q_mean, yaw_of,
                      bone_rotations)

HERE = os.path.dirname(os.path.abspath(__file__))

OUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

FIELDS = ["counter", "sample_time_fine",
          "acc_x", "acc_y", "acc_z",
          "gyr_x", "gyr_y", "gyr_z",
          "mag_x", "mag_y", "mag_z",
          "q_w", "q_x", "q_y", "q_z"]

RADIO_CHANNEL = 11
DESIRED_RATE = 60

# The recorded stages of a take, in order.
STAGES = ["T_POSE", "JUMP_IN", "ACTION", "JUMP_OUT"]

# Between stages the take sits here until the subject presses the button. They
# are recording themselves, so the pacing has to be theirs -- but once a stage
# is under way they are out in the capture volume and cannot press anything,
# which is what the countdown and the fixed duration are for.
WAITING = "WAITING"

# Both jumps are the same movement -- jump in place. They are numbered rather
# than named differently so the subject is never left wondering whether the
# second one is supposed to be different. Only their role differs: one brackets
# the start of the action, the other the end.
STAGE_HINT = {
    "T_POSE":   "摆 T-pose 站稳不动",
    "JUMP_IN":  "原地跳一下 ①（起始同步点）",
    "ACTION":   "执行动作",
    "JUMP_OUT": "原地跳一下 ②（结束同步点）",
}

# Seconds of countdown after the button is pressed, before the stage starts.
# 5s covers walking in from the keyboard in practice; adjustable on the page.
DEFAULT_LEADS = {
    "T_POSE":    5.0,
    "JUMP_IN":   5.0,
    "ACTION":    5.0,
    "JUMP_OUT":  5.0,
}

DEFAULT_DURATIONS = {
    "T_POSE":    5.0,
    "JUMP_IN":   5.0,
    "ACTION":   20.0,
    "JUMP_OUT":  5.0,
}


class Recorder(xda.XsCallback):
    """Buffers packets in memory. Writing to disk inside the callback would
    stall the SDK receive thread, so the buffer is drained after the take."""

    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.rows = []
        self.latest_quat = None
        self.latest_counter = None

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
            self.latest_counter = counter
            if qw is not None:
                self.latest_quat = (qw, qx, qy, qz)

    def start_take(self):
        with self.lock:
            self.rows = []

    def snapshot(self):
        with self.lock:
            return list(self.rows)

    def drain(self):
        with self.lock:
            rows, self.rows = self.rows, []
            return rows

    def live(self):
        with self.lock:
            return self.latest_quat, self.latest_counter

    def count(self):
        with self.lock:
            return len(self.rows)


def device_property(dev, names):
    """Read the first of `names` this SDK build actually implements.

    The Windows build spells the full-scale-range getters actualAccelerometerRange
    / actualGyroscopeRange; the Linux 2022.2 build drops the prefix.
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
        for render in ("toSimpleString", "toXsString"):
            if hasattr(value, render):
                return str(getattr(value, render)())
        return str(value)
    return f"unavailable ({'; '.join(missing)})"


def _angle_deg(qa, qb):
    """Rotation angle separating two orientations, degrees."""
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


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


def find_jump(rows, lo, hi):
    """Locate a jump inside a counter window on the reference sensor.

    Returns the counters of the free-fall minimum and the landing peak of
    |acc|. Free-fall is the cleaner of the two to align against -- it is a
    sustained dip rather than a single-sample spike -- but landing is the
    louder event in video, so both are reported.
    """
    samples = []
    for r in rows:
        counter, _, ax, ay, az = r[0], r[1], r[2], r[3], r[4]
        if counter is None or ax is None:
            continue
        if lo is not None and hi is not None:
            # The counter wraps, so compare positions relative to the window start.
            if (counter - lo) % 65536 > (hi - lo) % 65536:
                continue
        samples.append((counter, (ax * ax + ay * ay + az * az) ** 0.5))
    if len(samples) < 5:
        return None

    magnitudes = [m for _, m in samples]
    i_min = min(range(len(samples)), key=lambda i: magnitudes[i])
    i_max = max(range(len(samples)), key=lambda i: magnitudes[i])
    # The SDK hands back numpy scalars; np.bool_ and np.int64 are not subclasses
    # of bool/int, so json.dump rejects them. Coerce before they reach metadata.
    return {
        "freefall_counter": int(samples[i_min][0]),
        "freefall_acc": round(float(magnitudes[i_min]), 3),
        "landing_counter": int(samples[i_max][0]),
        "landing_acc": round(float(magnitudes[i_max]), 3),
        "detected": bool(magnitudes[i_min] < 4.0 and magnitudes[i_max] > 15.0),
    }


class Session:
    """Owns the hardware, the take state machine, and what the viewer sees."""

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()

        # The viewer may attach before the sensors have joined, so the session
        # starts in a reportable state rather than existing only once connected.
        self.state = "CONNECTING"
        self.action = ""
        self.take_index = 0
        self.phase_started = None
        self.take_started = None
        self.counting = False
        self.pending = None          # stage the button press will start next
        self.auto = not args.manual
        self.leads = ({s: args.lead for s in DEFAULT_LEADS} if args.lead is not None
                      else dict(DEFAULT_LEADS))
        self.durations = dict(DEFAULT_DURATIONS)
        self.durations["ACTION"] = args.action_seconds
        self._stop = threading.Event()
        self.events = []
        self.takes = []
        self.message = "正在连接硬件..."
        self.tpose_offsets = {}
        # Heading = this take's pelvis yaw minus the session reference, so the
        # first (correctly-facing) take defines forward and later takes are
        # rotated onto it -- see bone_rotations. The reference is the pelvis yaw
        # captured at the first take's T-pose; None until then.
        self.session_ref_yaw = None
        self.heading = 0.0
        self.view_offset = 0.0      # manual nudge on top, persists across takes
        self.tpose_sway = None

        stamp = datetime.now().strftime("%Y%m%d")
        self.session_name = f"{stamp}_{args.session}"
        # Only create the directory once a subject is known; otherwise starting
        # the recorder would litter data/ with an empty folder named "".
        self.out_dir = (os.path.join(OUT_ROOT, args.subject, self.session_name)
                        if args.subject else None)
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)

        self.control = None
        self.master = None
        self.recorders = {}
        self.sensors = {}
        self.rate = None

    # -- hardware ---------------------------------------------------------
    def connect(self):
        control = xda.XsControl_construct()
        port = None
        for attempt in range(5):
            ports = xda.XsScanner_scanPorts()
            for i in range(ports.size()):
                p = ports[i]
                if p.deviceId().isWirelessMaster():
                    port = p
                    break
            if port:
                break
            print(f"  scan attempt {attempt + 1}/5: no master, retrying...")
            time.sleep(1.0)
        if port is None:
            raise RuntimeError("No Awinda master found. Is the dongle plugged in?")

        control.setOptions(xda.XSO_Calibrate | xda.XSO_Orientation, xda.XSO_None)
        print(f"Master {port.deviceId()} on {port.portName()}")
        if not control.openPort(port):
            raise RuntimeError(f"openPort failed: {control.lastResultText()}")

        master = control.device(port.deviceId())
        master.gotoConfig()

        rates = master.supportedUpdateRates()
        rates = [rates[i] for i in range(rates.size())]
        rate = min(rates, key=lambda r: abs(r - DESIRED_RATE)) if rates else DESIRED_RATE

        # The rate can only be changed with the radio down, and dropping the
        # radio powers every MTw off -- so only cycle it if it must change.
        if master.updateRate() != rate:
            if master.isRadioEnabled():
                print(f"  changing update rate to {rate} Hz -- this drops the "
                      f"radio, so the MTws will need switching on again")
                master.disableRadio()
            if not master.setUpdateRate(rate):
                raise RuntimeError(f"setUpdateRate({rate}) failed")

        if not master.isRadioEnabled():
            if not master.enableRadio(RADIO_CHANNEL):
                raise RuntimeError(f"enableRadio({RADIO_CHANNEL}) failed")

        self.rate = master.updateRate()
        expected = len(SENSOR_MAP)
        print(f"Update rate: {self.rate} Hz. Radio on (channel {RADIO_CHANNEL}).")
        print(f"Waiting for {expected} sensors -- SWITCH THE MTws ON NOW if they "
              f"aren't already.")

        count, start = 0, time.time()
        while True:
            n = master.children().size()
            if n != count:
                count = n
                print(f"  connected: {n}/{expected}")
            with self.lock:
                self.message = (f"等待传感器 {n}/{expected} — 请打开 MTw "
                                f"({time.time() - start:.0f}s)")
            if n >= expected:
                break
            if time.time() - start > self.args.connect_timeout:
                raise RuntimeError(f"Only {n} sensors connected")
            time.sleep(0.2)

        if not master.gotoMeasurement():
            raise RuntimeError("Could not enter measurement mode")

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
            self.recorders[seg] = rec
            self.sensors[seg] = {
                "device_id": key,
                "acc_range_g": device_property(
                    dev, ("actualAccelerometerRange", "accelerometerRange")),
                "gyr_range_dps": device_property(
                    dev, ("actualGyroscopeRange", "gyroscopeRange")),
                "product_code": device_property(dev, ("productCode",)),
                "firmware": device_property(dev, ("firmwareVersion",)),
            }

        self.control, self.master = control, master
        with self.lock:
            self.state = "IDLE"
            self.message = "按 n 新建一个 take"
        print(f"Streaming from {len(self.recorders)} sensors.\n")

    def shutdown(self):
        if self.master is not None:
            self.master.gotoConfig()
            if self.args.release_radio:
                self.master.disableRadio()
                print("radio off -- all MTws powered down")
            else:
                # Dropping the radio powers every MTw off; keep them alive for
                # the next session.
                self.master.setGotoConfigOnClose(False)
                print("radio left on -- MTws stay powered")
        if self.control is not None:
            self.control.close()

    # -- state machine ----------------------------------------------------
    def reference_counter(self):
        rec = self.recorders.get(REFERENCE_SEGMENT)
        if rec is None:
            return None
        return rec.live()[1]

    def start_take(self, action):
        with self.lock:
            if self.state != "IDLE":
                return
            self.action = action or f"take{self.take_index + 1}"
            self.take_index += 1
            self.events = []
            self.tpose_offsets = {}
            self.tpose_sway = None
            for rec in self.recorders.values():
                rec.start_take()
            self.take_started = time.time()
            self.mark("take_start")
            print(f"\n=== take {self.take_index:02d}  {self.action} ===")
            self._enter(STAGES[0])

    def mark(self, name):
        """Record an event against the packet counter, the real timeline."""
        counter = self.reference_counter()
        self.events.append({
            "event": name,
            "counter": None if counter is None else int(counter),
            "t_since_take_start": round(time.time() - self.take_started, 3),
            "wall_utc": datetime.now(timezone.utc).isoformat(),
        })

    def _enter(self, stage):
        """Start a stage's countdown. Caller holds the lock.

        The countdown is what makes an unattended take workable: it is the gap
        between pressing the button and the window opening, so the subject can
        walk in and be ready. Jumping a second early would put the impact
        outside the window the detector searches.
        """
        self.state = stage
        self.pending = None
        self.counting = self.leads.get(stage, 0) > 0
        self.phase_started = time.time()
        self.message = STAGE_HINT[stage]
        if self.counting:
            print(f"  [{stage}] 倒计时 {self.leads[stage]:.0f}s — {self.message}")
        else:
            self._begin_stage()

    def _begin_stage(self):
        """Countdown finished: the stage is now live. Caller holds the lock."""
        self.counting = False
        self.phase_started = time.time()
        self.mark(f"{self.state.lower()}_start")
        print(f"  [{self.state}] {self.message}")

    def _finish_stage(self):
        """Close the stage and park until the next button press.

        Returns True when the take is over and needs saving.
        """
        self.mark(f"{self.state.lower()}_end")
        if self.state == "T_POSE":
            self._capture_tpose()
        idx = STAGES.index(self.state)
        if idx + 1 < len(STAGES):
            self.pending = STAGES[idx + 1]
            self.state = WAITING
            self.counting = False
            self.phase_started = time.time()
            self.message = f"准备好后点击开始：{STAGE_HINT[self.pending]}"
            print(f"  [等待] {self.message}")
            return False
        self.state = "SAVING"
        self.pending = None
        self.counting = False
        self.message = "保存中..."
        return True

    def advance(self):
        """The button. Starts the next stage, or cuts a countdown/stage short."""
        saving = False
        with self.lock:
            if self.state == WAITING and self.pending:
                self._enter(self.pending)
            elif self.counting:
                self._begin_stage()
            elif self.state in STAGES:
                saving = self._finish_stage()
        if saving:
            self.save_take()

    def tick(self):
        """Run the clock *within* a stage only.

        A countdown runs down to the stage, and the stage runs out its duration
        -- both happen while the subject is out of reach of the keyboard. Moving
        on to the next stage is never automatic; that is their button press.
        """
        while not self._stop.is_set():
            time.sleep(0.05)
            saving = False
            with self.lock:
                if self.state not in STAGES:
                    continue
                if self.counting:
                    limit = self.leads.get(self.state, 0)
                elif self.auto:
                    limit = self.durations.get(self.state, 0)
                else:
                    continue
                if limit <= 0 or time.time() - self.phase_started < limit:
                    continue
                if self.counting:
                    self._begin_stage()
                else:
                    saving = self._finish_stage()
            if saving:
                self.save_take()

    def remaining(self):
        """Seconds left in the current countdown or stage; None when untimed."""
        if self.state not in STAGES:
            return None
        if self.counting:
            limit = self.leads.get(self.state, 0)
        elif self.auto:
            limit = self.durations.get(self.state, 0)
        else:
            return None
        if limit <= 0:
            return None
        return max(0.0, limit - (time.time() - self.phase_started))

    def _capture_tpose(self):
        """Average each sensor's orientation over the held T-pose.

        Stored per take rather than applied to the data: the CSVs stay raw, and
        analysis can undo or redo the calibration however it likes.
        """
        # Average only the T-pose window. The recording also contains the
        # walk-to-position lead-in, which would drag the reference off badly.
        start = next((e["counter"] for e in self.events
                      if e["event"] == "t_pose_start"), None)
        worst = 0.0
        for seg, rec in self.recorders.items():
            rows = rec.snapshot()
            if start is not None:
                rows = [r for r in rows if r[0] is not None
                        and (r[0] - start) % 65536 < 32768]
            quats = [(r[11], r[12], r[13], r[14]) for r in rows
                     if r[11] is not None]
            # Skip the first moments: the subject is still settling into pose.
            quats = quats[len(quats) // 3:]
            mean = q_mean(quats) if quats else None
            if mean is not None:
                self.tpose_offsets[seg] = [round(float(v), 6) for v in mean]
                # How much this sensor moved across the hold. A held T-pose
                # sways a degree or two; tens of degrees means the subject was
                # still getting into position and the reference is poisoned --
                # which then renders as limbs pointing in nonsense directions.
                sway = max(_angle_deg(mean, q) for q in quats)
                worst = max(worst, sway)
        self.tpose_sway = round(worst, 1)
        # The first take of the session defines "forward": its pelvis yaw becomes
        # the reference, and every take (including this first one, heading 0) is
        # rotated onto it. This keeps the render facing-independent across takes
        # without needing to know where the screen is -- see bone_rotations.
        ref = self.tpose_offsets.get(REFERENCE_SEGMENT)
        pelvis_yaw = yaw_of(ref) if ref is not None else 0.0
        if self.session_ref_yaw is None:
            self.session_ref_yaw = pelvis_yaw
        self.heading = pelvis_yaw - self.session_ref_yaw
        quality = "OK" if worst < 10 else "SUSPECT -- subject was moving, redo"
        print(f"  T-pose captured for {len(self.tpose_offsets)} sensors, "
              f"max sway {worst:.1f} deg ({quality}), "
              f"heading {np.degrees(self.heading):+.0f} deg vs session ref")

    def set_subject(self, subject):
        """Point the session at a different person.

        Take numbering restarts and a fresh directory is used -- continuing the
        old numbering under a new name would interleave two people's data in one
        folder. Anything already written stays where it is.
        """
        subject = (subject or "").strip()
        if not subject:
            return
        with self.lock:
            if self.state != "IDLE" or subject == self.args.subject:
                return
            self.args.subject = subject
            self.take_index = 0
            self.takes = []
            # New person = sensors re-donned, so the pelvis mounting (part of the
            # heading reference) changes; the next take re-establishes forward.
            self.session_ref_yaw = None
            self.out_dir = os.path.join(OUT_ROOT, subject, self.session_name)
            os.makedirs(self.out_dir, exist_ok=True)
            self.message = "按 n 新建一个 take"
        print(f"\nsubject set to {subject} -- writing to {self.out_dir}")
        self.write_session_index()

    def redo(self):
        with self.lock:
            if self.state in ("IDLE", "SAVING", "CONNECTING"):
                return
            print(f"  take {self.take_index:02d} discarded")
            self.take_index -= 1
            self.state = "IDLE"
            self.counting = False
            self.pending = None
            self.phase_started = None
            self.events = []
            self.tpose_offsets = {}
            self.tpose_sway = None
            self.message = "已丢弃，按 n 重录"

    # -- persistence ------------------------------------------------------
    def save_take(self):
        slug = "".join(c if c.isalnum() or c in "-_" else "_" for c in self.action)
        take_dir = os.path.join(self.out_dir,
                                f"take{self.take_index:02d}_{slug}")
        os.makedirs(take_dir, exist_ok=True)

        stats, ref_rows = {}, None
        for seg in sorted(self.recorders):
            rows = self.recorders[seg].drain()
            if seg == REFERENCE_SEGMENT:
                ref_rows = rows
            with open(os.path.join(take_dir, f"{seg}.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(FIELDS)
                w.writerows(rows)
            stats[seg] = {"samples": len(rows), "packet_loss": packet_loss(rows)}

        # take_start was stamped from the last packet seen *before* the buffers
        # were cleared, so it lands one sample ahead of the first recorded row.
        # Alignment is computed relative to it, so a stale value would bias every
        # take by one frame. Pin it to the first sample that actually exists.
        first = next((r[0] for r in (ref_rows or []) if r[0] is not None), None)
        if first is not None:
            for e in self.events:
                if e["event"] == "take_start":
                    e["counter"] = int(first)

        events = {e["event"]: e for e in self.events}
        jumps = {}
        if ref_rows:
            for name, a, b in (("jump_in", "jump_in_start", "jump_in_end"),
                               ("jump_out", "jump_out_start", "jump_out_end")):
                lo = events.get(a, {}).get("counter")
                hi = events.get(b, {}).get("counter")
                found = find_jump(ref_rows, lo, hi)
                if found:
                    jumps[name] = found

        counts = [s["samples"] for s in stats.values()]
        worst = max((s["packet_loss"]["loss_pct"] for s in stats.values()
                     if s["packet_loss"]), default=0.0)

        meta = {
            "subject": self.args.subject,
            "session": self.session_name,
            "take": self.take_index,
            "action": self.action,
            "notes": self.args.notes,
            "duration_s": round(time.time() - self.take_started, 3),
            "update_rate_hz": self.rate,
            "radio_channel": RADIO_CHANNEL,
            "reference_segment": REFERENCE_SEGMENT,
            "events": self.events,
            "jump_sync": jumps,
            "tpose_quat": self.tpose_offsets,
            "tpose_sway_deg": self.tpose_sway,
            # Render heading (relative to the session's first take) and manual
            # nudge. The CSVs are raw; these only affect the live skeleton view.
            "heading_deg": round(float(np.degrees(self.heading)), 1),
            "view_offset_deg": round(float(np.degrees(self.view_offset))) % 360,
            "frames": {
                "acc": "sensor frame, m/s^2, specific force (gravity included)",
                "gyr": "sensor frame, rad/s",
                "mag": "sensor frame, arbitrary units",
                "q": "sensor-to-global, XKF estimate, (w,x,y,z)",
            },
            "timing_note": (f"{self.rate} Hz samples are interval integrals (SDI), "
                            "not instantaneous. Use `counter` (16-bit, wraps at "
                            "65536) as the timeline, not host arrival time."),
            "sync_note": ("Align video to jump_sync counters. freefall_counter is "
                          "the |acc| minimum (most robust); landing_counter is the "
                          "impact peak (easiest to see in video)."),
            "sensors": self.sensors,
            "stats": stats,
        }
        with open(os.path.join(take_dir, "metadata.json"), "w",
                  encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        summary = {
            "take": self.take_index,
            "action": self.action,
            "dir": os.path.basename(take_dir),
            "duration_s": meta["duration_s"],
            "samples": [min(counts), max(counts)] if counts else [0, 0],
            "worst_loss_pct": worst,
            "jump_sync": jumps,
        }
        self.takes.append(summary)
        self.write_session_index()

        for name in ("jump_in", "jump_out"):
            j = jumps.get(name)
            if not j:
                print(f"  {name}: no reference data")
            elif j["detected"]:
                print(f"  {name}: freefall@{j['freefall_counter']} "
                      f"({j['freefall_acc']} m/s2)  landing@{j['landing_counter']} "
                      f"({j['landing_acc']} m/s2)")
            else:
                print(f"  {name}: WEAK -- min {j['freefall_acc']} / max "
                      f"{j['landing_acc']} m/s2. Was the jump real? "
                      f"Consider [r] to redo.")
        print(f"  saved {os.path.basename(take_dir)}  "
              f"{meta['duration_s']:.1f}s  samples {min(counts)}-{max(counts)}  "
              f"worst loss {worst:.3f}%")

        with self.lock:
            self.state = "IDLE"
            self.message = f"take {self.take_index:02d} 已保存，按 n 继续"

    def write_session_index(self):
        if not self.out_dir:
            return
        index = {
            "subject": self.args.subject,
            "session": self.session_name,
            "update_rate_hz": self.rate,
            "sensor_map": SENSOR_MAP,
            "takes": self.takes,
        }
        with open(os.path.join(self.out_dir, "session.json"), "w",
                  encoding="utf-8") as f:
            json.dump(index, f, indent=2, ensure_ascii=False)

    # -- what the viewer sees --------------------------------------------
    def bones(self, quats):
        """Resolve the skeleton to drawable line segments.

        Done here rather than in the browser so the geometry has exactly one
        definition -- skeleton.py -- instead of a JavaScript copy that would
        quietly drift out of step with it.

        The heading conjugation is what makes limbs move the right way
        regardless of which direction the subject faced at T-pose. Without it
        the render is only correct for one lucky facing -- measured on real
        data: at heading 210 deg, raised arms rendered pointing DOWN.
        """
        bone_R = bone_rotations(quats, self.tpose_offsets,
                                self.heading + self.view_offset)
        joints = joint_positions(bone_R)
        segments = []
        for colour_seg, a, b in BONES:
            p, q_ = joints[a], joints[b]
            segments.append([round(float(v), 4) for v in (*p, *q_)]
                            + [limb_colour(colour_seg), colour_seg])
        for colour_seg, a, _ in TIP_BONES:
            p = joints[a]
            q_ = p + bone_R[a] @ TIPS[a]
            segments.append([round(float(v), 4) for v in (*p, *q_)]
                            + [limb_colour(colour_seg), colour_seg])
        return segments

    def status(self):
        quats, live = {}, 0
        for seg, rec in self.recorders.items():
            q, _ = rec.live()
            if q is not None:
                quats[seg] = [round(float(v), 5) for v in q]
                live += 1
        with self.lock:
            elapsed = (time.time() - self.phase_started) if self.phase_started else 0
            remaining = self.remaining()
            return {
                "state": self.state,
                "counting": self.counting,
                "pending": self.pending,
                "remaining": None if remaining is None else round(remaining, 2),
                "auto": self.auto,
                "leads": self.leads,
                "durations": self.durations,
                "stage_elapsed": round(elapsed, 1),
                "take": self.take_index,
                "action": self.action,
                "message": self.message,
                "subject": self.args.subject,
                "session": self.session_name,
                "live": live,
                "expected": len(SENSOR_MAP),
                "rate_hz": self.rate,
                "takes_done": len(self.takes),
                "calibrated": bool(self.tpose_offsets),
                "tpose_sway": self.tpose_sway,
                "view_offset_deg": round(float(np.degrees(self.view_offset))) % 360,
                "missing": sorted(set(SENSOR_MAP.values()) - set(quats)),
                "bones": self.bones(quats),
                "last_take": self.takes[-1] if self.takes else None,
            }

    def handle(self, cmd):
        name = cmd.get("cmd")
        if name == "set_config":
            with self.lock:
                if "view_offset_deg" in cmd:
                    # A manual nudge added on top of the automatic heading.
                    # Render-only, so adjustable mid-take -- exactly when you
                    # notice the view is turned. Persists across takes.
                    self.view_offset = float(np.radians(float(cmd["view_offset_deg"])))
                if self.state in ("IDLE", "CONNECTING"):
                    if "auto" in cmd:
                        self.auto = bool(cmd["auto"])
                    for stage, value in (cmd.get("leads") or {}).items():
                        if stage in self.leads:
                            self.leads[stage] = max(0.0, float(value))
                    for stage, value in (cmd.get("durations") or {}).items():
                        if stage in self.durations:
                            self.durations[stage] = max(0.0, float(value))
        elif name == "set_subject":
            self.set_subject(cmd.get("subject", ""))
        elif name == "start":
            if not self.args.subject:
                return True          # the page blocks this; guard anyway
            self.start_take(cmd.get("action", ""))
        elif name == "advance":
            self.advance()
        elif name == "redo":
            self.redo()
        elif name == "quit":
            return False
        return True


def make_handler(session, stop):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass                      # the console belongs to the session log

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                try:
                    with open(os.path.join(HERE, "viewer.html"), "rb") as f:
                        body = f.read()
                except OSError as e:
                    self._send(500, str(e).encode(), "text/plain")
                    return
                self._send(200, body, "text/html; charset=utf-8")
            elif self.path == "/roster":
                # Static, so it is fetched once at load rather than riding along
                # with every status frame.
                roster = [{"segment": seg, "label": label,
                           "device": SEGMENT_TO_DEVICE.get(seg, "?")}
                          for seg, label in SEGMENT_LABELS]
                self._send(200, json.dumps(roster).encode(),
                           "application/json; charset=utf-8")
            elif self.path == "/events":
                self.stream_events()
            else:
                self._send(404, b"not found", "text/plain")

        def stream_events(self):
            """Server-sent events: a plain HTTP stream the browser reconnects
            to on its own. Avoids hand-rolling WebSocket framing for what is a
            one-way feed."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            print("viewer connected")
            try:
                while not stop.is_set():
                    payload = json.dumps(session.status())
                    self.wfile.write(f"data: {payload}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(1.0 / 30)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                print("viewer disconnected")

        def do_POST(self):
            if self.path != "/cmd":
                self._send(404, b"not found", "text/plain")
                return
            length = int(self.headers.get("Content-Length", 0))
            try:
                cmd = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                self._send(400, b'{"ok":false}', "application/json")
                return
            if not session.handle(cmd):
                stop.set()
            self._send(200, b'{"ok":true}', "application/json")

    return Handler


def serve(session, port, stop):
    """Start serving before the hardware is up, so a browser opened early sees
    "waiting for sensors" rather than a connection error."""
    httpd = ThreadingHTTPServer(("127.0.0.1", port), make_handler(session, stop))
    httpd.daemon_threads = True
    httpd.timeout = 0.5          # so the stop flag is noticed between requests
    print(f"Viewer at http://localhost:{port}")
    print(f"  From the Mac, open the tunnel then load that URL in Chrome:")
    print(f"  ssh -p 2222 -i ~/.ssh/xsens_vm -L {port}:localhost:{port} "
          f"-N xsens@127.0.0.1")
    while not stop.is_set():
        httpd.handle_request()
    httpd.server_close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--subject", default="",
                    help="subject id, e.g. S01. Optional -- it can also be "
                         "typed into the page, which is the usual way")
    ap.add_argument("--session", default="session1", help="session label")
    ap.add_argument("--notes", default="", help="free-text note stored per take")
    ap.add_argument("--port", type=int, default=9000, help="viewer socket port")
    ap.add_argument("--manual", action="store_true",
                    help="end each stage on a button press rather than after "
                         "its set duration. Only workable with someone else at "
                         "the keyboard -- a subject recording themselves is out "
                         "in the capture volume once a stage is running")
    ap.add_argument("--lead", type=float, default=None,
                    help="override the countdown before every stage "
                         "(default 5s each; per-stage values adjustable on "
                         "the page)")
    ap.add_argument("--action-seconds", type=float, default=20.0,
                    help="how long the action stage runs in timed mode "
                         "(default 20); also settable from the page")
    ap.add_argument("--connect-timeout", type=float, default=300.0,
                    help="seconds to wait for the MTws to join")
    ap.add_argument("--release-radio", action="store_true",
                    help="power the MTws down on exit instead of keeping the "
                         "radio up for the next session")
    args = ap.parse_args()

    session = Session(args)
    stop = session._stop
    threading.Thread(target=serve, args=(session, args.port, stop),
                     daemon=True).start()
    threading.Thread(target=session.tick, daemon=True).start()
    try:
        session.connect()
        print(f"Session {session.session_name}")
        if session.out_dir:
            print(f"Subject {args.subject}, writing to {session.out_dir}\n")
        else:
            print("Type the subject id into the page before recording.\n")
        stop.wait()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        stop.set()
        print("\nShutting down...")
        session.write_session_index()
        session.shutdown()
        print(f"{len(session.takes)} takes saved"
              + (f" to {session.out_dir}" if session.out_dir else ""))


if __name__ == "__main__":
    main()
