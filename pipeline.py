"""
Minimal real-time sparse-IMU pose pipeline for Xsens MTw Awinda (17 sensors).

No learning, no translation estimation. Pure geometry:
    global bone orientation = q_sensor(t) * q_sensor(T-pose)^-1

Root (pelvis) is pinned at the origin, so there is no drift in position --
only orientation error, which is what you want for a first pipeline check.

Controls (with the plot window focused):
    c    calibrate -- stand in T-pose first
    q    quit

Run with:
    C:\\ProgramData\\Anaconda3\\envs\\xsens\\python.exe pipeline.py
"""

import time
import threading

try:
    import winsound  # Windows-only; used for the calibration countdown beeps
except ImportError:
    winsound = None

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

import xsensdeviceapi as xda


from skeleton import (SENSOR_MAP, SKELETON, TIPS, BONES, TIP_BONES,
                      limb_colour, joint_positions, yaw_of, bone_rotations)


DESIRED_RATE = 60      # Hz; the closest supported rate is picked automatically
RADIO_CHANNEL = 11     # match what you used in MT Manager
COUNTDOWN = 8.0        # seconds between pressing 'c' and the T-pose snapshot


def beep(freq, ms):
    """Audible cue -- you can't watch the screen while standing in T-pose."""
    if winsound is None:
        return
    try:
        winsound.Beep(freq, ms)
    except RuntimeError:
        pass


# ---------------------------------------------------------------------------
# One callback per MTw. The official SDK example attaches handlers to the
# individual MTw devices, not to the wireless master -- the master only emits
# connectivity events, never live data.
# ---------------------------------------------------------------------------
class MtwCallback(xda.XsCallback):
    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.quat = None

    def onLiveDataAvailable(self, dev, packet):
        if packet is None or not packet.containsOrientation():
            return
        q = packet.orientationQuaternion()
        with self.lock:
            self.quat = np.array([q[0], q[1], q[2], q[3]])

    def get(self):
        with self.lock:
            return self.quat


def find_master(retries=5, delay=1.0):
    """scanPorts() only probes for 100 ms and occasionally misses, so retry."""
    control = xda.XsControl_construct()
    for attempt in range(retries):
        ports = xda.XsScanner_scanPorts()
        for i in range(ports.size()):
            p = ports[i]
            if p.deviceId().isWirelessMaster():
                return control, p
        print(f"  scan attempt {attempt + 1}/{retries}: no master, retrying...")
        time.sleep(delay)
    raise RuntimeError(
        "No Awinda master found. Is the dongle plugged in, and is MT Manager "
        "fully closed (it holds the COM port exclusively)?"
    )


def connect(settle=4.0, timeout=300.0):
    """Follow the official MTw startup sequence. Returns (control, master, callbacks)
    where callbacks maps segment name -> MtwCallback."""
    control, port = find_master()
    print(f"Master {port.deviceId()} on {port.portName()} @ {port.baudrate()} baud")

    if not control.openPort(port):
        raise RuntimeError(f"openPort failed: {control.lastResultText()}")

    master = control.device(port.deviceId())
    if not master.gotoConfig():
        raise RuntimeError("Could not enter config mode")

    rates = master.supportedUpdateRates()
    rates = [rates[i] for i in range(rates.size())]
    rate = min(rates, key=lambda r: abs(r - DESIRED_RATE)) if rates else DESIRED_RATE

    # The rate can only be changed with the radio down, and dropping the radio
    # powers every MTw off -- so only cycle it when the rate really must change.
    if master.updateRate() != rate:
        if master.isRadioEnabled():
            print(f"  changing update rate to {rate} Hz -- this drops the radio, "
                  f"so the MTws will need switching on again")
            master.disableRadio()
        if not master.setUpdateRate(rate):
            raise RuntimeError(f"setUpdateRate({rate}) failed")

    if not master.isRadioEnabled():
        if not master.enableRadio(RADIO_CHANNEL):
            raise RuntimeError(
                f"enableRadio({RADIO_CHANNEL}) failed -- try another channel")

    rate = master.updateRate()
    print(f"Supported update rates: {rates} -> using {rate} Hz")

    print(f"Radio on (channel {RADIO_CHANNEL}). Waiting for {len(SENSOR_MAP)} sensors...")
    print(f"  SWITCH THE MTws ON NOW if they aren't already -- they join within "
          f"seconds of powering up.\n  Giving up after {timeout:.0f}s.")
    expected = len(SENSOR_MAP)
    count, last_change, start = 0, time.time(), time.time()
    while True:
        now = time.time()
        n = master.children().size()
        if n != count:
            count, last_change = n, now
            print(f"  connected: {n}/{expected}")
        if n >= expected:
            break
        if now - last_change > settle and n > 0:
            print(f"  settled at {n}/{expected} -- continuing without the rest")
            break
        if now - start > timeout:
            raise RuntimeError(f"Only {n} sensors connected after {timeout:.0f}s")
        # Silence while nothing joins is indistinguishable from a hang, which is
        # how a too-short timeout gets misread as a hardware fault.
        if n == 0 and int(now - start) % 10 == 0 and now - start >= 10:
            print(f"  still waiting... {now - start:.0f}s", flush=True)
            time.sleep(1.0)
        time.sleep(0.2)

    if not master.gotoMeasurement():
        raise RuntimeError("Could not enter measurement mode")

    # MTw device instances only become available once measuring.
    callbacks = {}
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
        mtw = control.device(did)
        cb = MtwCallback()
        mtw.addCallbackHandler(cb)
        callbacks[seg] = cb

    print(f"Streaming from {len(callbacks)} mapped sensors.")
    missing = set(SENSOR_MAP.values()) - set(callbacks)
    if missing:
        print(f"  no data for: {', '.join(sorted(missing))}")
    return control, master, callbacks


def main():
    control, master, callbacks = connect()

    offsets = {}   # segment -> orientation captured at T-pose
    state = {"run": True, "calib_at": None, "beeped": None,
             "heading": 0.0}

    fig = plt.figure(figsize=(7, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.1, 0.9)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=8, azim=-88)
    ax.set_xlabel("X (right)")
    ax.set_ylabel("Y (forward)")
    ax.set_zlabel("Z (up)")

    artists = []
    for colour_seg, a, b in BONES:
        (ln,) = ax.plot([], [], [], lw=4, color=limb_colour(colour_seg),
                        solid_capstyle="round")
        artists.append((ln, a, b))
    for colour_seg, a, _ in TIP_BONES:
        (ln,) = ax.plot([], [], [], lw=4, color=limb_colour(colour_seg),
                        solid_capstyle="round")
        artists.append((ln, a, None))

    title = ax.set_title("stand in T-pose, then press 'c'")

    def do_calibrate():
        n = 0
        for seg, cb in callbacks.items():
            q = cb.get()
            if q is not None:
                offsets[seg] = q
                n += 1
        # No heading correction: render = R(q_now . q_Tpose^-1), the original
        # algorithm. Deriving heading from pelvis yaw (magnetic-north-referenced,
        # not screen-referenced) rotated correct renders off, so it was dropped.
        # A session that comes out reversed is the known limitation of the
        # heading-free approach; live it up to the operator, not an auto-guess.
        state["heading"] = 0.0
        print(f"Calibrated {n} sensors.")
        beep(1400, 120)
        beep(1800, 200)

    def on_key(event):
        if event.key == "c":
            state["calib_at"] = time.time() + COUNTDOWN
            state["beeped"] = None
            print(f"Calibrating in {COUNTDOWN:.0f}s -- get into T-pose!")
        elif event.key == "q":
            state["run"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)
    print(f"\nPress 'c' in the plot window, then you have {COUNTDOWN:.0f} s to get "
          f"into T-pose.\nOne beep per second, two high beeps = captured. 'q' quits.")

    frames, t0 = 0, time.time()
    try:
        while state["run"] and plt.fignum_exists(fig.number):
            quats = {seg: cb.get() for seg, cb in callbacks.items()}
            quats = {seg: q for seg, q in quats.items() if q is not None}
            live = sum(1 for seg in quats if seg in offsets)
            bone_R = bone_rotations(quats, offsets, state["heading"])

            joints = joint_positions(bone_R)

            for ln, a, b in artists:
                start = joints[a]
                end = joints[b] if b is not None else start + bone_R[a] @ TIPS[a]
                ln.set_data_3d([start[0], end[0]], [start[1], end[1]],
                               [start[2], end[2]])

            due = state["calib_at"]
            if due is not None:
                remain = due - time.time()
                if remain <= 0:
                    state["calib_at"] = None
                    title.set_text("capturing T-pose...")
                    fig.canvas.draw_idle()
                    plt.pause(0.001)
                    do_calibrate()
                else:
                    sec = int(remain) + 1
                    title.set_text(f"STAND IN T-POSE   ---  {sec}  ---")
                    if state["beeped"] != sec:
                        state["beeped"] = sec
                        beep(700 if sec > 3 else 1000, 70)
            else:
                frames += 1
                if frames % 30 == 0:
                    fps = frames / (time.time() - t0)
                    title.set_text(f"{live}/{len(callbacks)} bones live  |  {fps:4.1f} fps"
                                   f"  |  'c' recalibrate, 'q' quit")
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down...")
        master.gotoConfig()
        # Disabling the radio powers every MTw off, so the next run would need
        # all 17 switched on by hand again. Leave the link up.
        master.setGotoConfigOnClose(False)
        control.close()
        print("Closed (radio left on, MTws still powered).")


if __name__ == "__main__":
    main()
