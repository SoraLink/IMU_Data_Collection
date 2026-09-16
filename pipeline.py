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


# ---------------------------------------------------------------------------
# Sensor -> body segment mapping (17-sensor full body).
# ---------------------------------------------------------------------------
SENSOR_MAP = {
    "00B4F79F": "head",             # 头
    "00B4F79C": "sternum",          # 前胸
    "00B4F7A1": "pelvis",           # 骨盆
    "00B4F79B": "right_shoulder",   # 右肩胛
    "00B4F773": "right_upperarm",   # 右大臂
    "00B4F7A8": "right_forearm",    # 右小臂
    "00B4F79D": "right_hand",       # 右手
    "00B4F775": "left_shoulder",    # 左肩胛
    "00B4F7A3": "left_upperarm",    # 左大臂
    "00B4F7B6": "left_forearm",     # 左小臂
    "00B4F7A0": "left_hand",        # 左手
    "00B4F7A7": "right_thigh",      # 右大腿
    "00B4F7A2": "right_shank",      # 右小腿
    "00B4F184": "right_foot",       # 右脚
    "00B4F7A5": "left_thigh",       # 左大腿
    "00B4F7A6": "left_shank",       # 左小腿
    "00B4F79E": "left_foot",        # 左脚
}

# ---------------------------------------------------------------------------
# Skeleton. Axes: +X = subject's right, +Y = forward, +Z = up.
#
# Each entry is (parent, offset), where offset is the vector from the parent's
# proximal joint to this segment's proximal joint. Because calibration forces
# every bone rotation to identity at T-pose time, these offsets ARE the T-pose:
# arms straight out sideways, legs straight down.
# ---------------------------------------------------------------------------
SKELETON = {
    "pelvis":          (None,             np.array([0.00,  0.00,  0.00])),
    "sternum":         ("pelvis",         np.array([0.00,  0.00,  0.22])),
    "head":            ("sternum",        np.array([0.00,  0.00,  0.30])),

    "right_shoulder":  ("sternum",        np.array([0.03,  0.00,  0.26])),
    "right_upperarm":  ("right_shoulder", np.array([0.15,  0.00,  0.00])),
    "right_forearm":   ("right_upperarm", np.array([0.28,  0.00,  0.00])),
    "right_hand":      ("right_forearm",  np.array([0.26,  0.00,  0.00])),

    "left_shoulder":   ("sternum",        np.array([-0.03, 0.00,  0.26])),
    "left_upperarm":   ("left_shoulder",  np.array([-0.15, 0.00,  0.00])),
    "left_forearm":    ("left_upperarm",  np.array([-0.28, 0.00,  0.00])),
    "left_hand":       ("left_forearm",   np.array([-0.26, 0.00,  0.00])),

    "right_thigh":     ("pelvis",         np.array([0.09,  0.00, -0.06])),
    "right_shank":     ("right_thigh",    np.array([0.00,  0.00, -0.44])),
    "right_foot":      ("right_shank",    np.array([0.00,  0.00, -0.43])),

    "left_thigh":      ("pelvis",         np.array([-0.09, 0.00, -0.06])),
    "left_shank":      ("left_thigh",     np.array([0.00,  0.00, -0.44])),
    "left_foot":       ("left_shank",     np.array([0.00,  0.00, -0.43])),
}

# Leaf segments need an explicit end point so they render as a bone, not a dot.
TIPS = {
    "head":       np.array([0.00,  0.00,  0.22]),
    "right_hand": np.array([0.18,  0.00,  0.00]),
    "left_hand":  np.array([-0.18, 0.00,  0.00]),
    "right_foot": np.array([0.00,  0.20, -0.04]),
    "left_foot":  np.array([0.00,  0.20, -0.04]),
}

# Colour by body part so left/right are distinguishable at a glance.
TORSO = "#3E4C59"
RIGHT = "#D06A12"
LEFT = "#1F7A99"


def limb_colour(seg):
    if seg.startswith("right_"):
        return RIGHT
    if seg.startswith("left_"):
        return LEFT
    return TORSO


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
# Quaternion helpers (w, x, y, z)
# ---------------------------------------------------------------------------
def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def q_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


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


def connect(settle=4.0, timeout=60.0):
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
    print(f"Supported update rates: {rates} -> using {rate} Hz")
    if not master.setUpdateRate(rate):
        raise RuntimeError(f"setUpdateRate({rate}) failed")

    if master.isRadioEnabled():
        master.disableRadio()
    if not master.enableRadio(RADIO_CHANNEL):
        raise RuntimeError(f"enableRadio({RADIO_CHANNEL}) failed -- try another channel")

    print(f"Radio on (channel {RADIO_CHANNEL}). Waiting for {len(SENSOR_MAP)} sensors...")
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


def joint_positions(bone_R):
    """Walk the skeleton, returning {segment: world position of its proximal joint}."""
    joints = {}

    def resolve(seg):
        if seg in joints:
            return joints[seg]
        parent, offset = SKELETON[seg]
        if parent is None:
            joints[seg] = np.zeros(3)
        else:
            joints[seg] = resolve(parent) + bone_R[parent] @ offset
        return joints[seg]

    for seg in SKELETON:
        resolve(seg)
    return joints


# Every drawn line: (segment providing the colour, parent joint, child joint or tip)
BONES = [(seg, parent, seg) for seg, (parent, _) in SKELETON.items() if parent]
TIP_BONES = [(seg, seg, None) for seg in TIPS]


def main():
    control, master, callbacks = connect()

    offsets = {}   # segment -> q0^-1
    state = {"run": True, "calib_at": None, "beeped": None}

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
                offsets[seg] = q_conj(q)
                n += 1
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
            bone_R = {seg: np.eye(3) for seg in SKELETON}
            live = 0
            for seg, cb in callbacks.items():
                q = cb.get()
                if q is None or seg not in offsets:
                    continue
                bone_R[seg] = q_to_R(q_mul(q, offsets[seg]))
                live += 1

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
        master.disableRadio()
        control.close()
        print("Closed.")


if __name__ == "__main__":
    main()
