"""Sensor map, skeleton geometry and quaternion maths.

Deliberately free of any `xsensdeviceapi` import: the live viewer runs on the
macOS host, where the SDK does not exist and cannot be installed, while the
recorder runs inside the x86_64 guest. Both need this geometry, so it lives
apart from anything that touches hardware.
"""

import numpy as np


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

# The sensor whose signal is used for jump detection and as the event-counter
# reference. The pelvis moves with the whole body and never leaves the trunk.
REFERENCE_SEGMENT = "pelvis"

# Shown on the strap-up screen next to each device id. Ordered head-down so the
# list reads in the order the sensors actually get put on.
SEGMENT_LABELS = [
    ("head",           "头"),
    ("sternum",        "前胸"),
    ("pelvis",         "骨盆"),
    ("right_shoulder", "右肩胛"),
    ("right_upperarm", "右大臂"),
    ("right_forearm",  "右小臂"),
    ("right_hand",     "右手"),
    ("left_shoulder",  "左肩胛"),
    ("left_upperarm",  "左大臂"),
    ("left_forearm",   "左小臂"),
    ("left_hand",      "左手"),
    ("right_thigh",    "右大腿"),
    ("right_shank",    "右小腿"),
    ("right_foot",     "右脚"),
    ("left_thigh",     "左大腿"),
    ("left_shank",     "左小腿"),
    ("left_foot",      "左脚"),
]

# segment -> device id, the inverse of SENSOR_MAP, which is keyed by device.
SEGMENT_TO_DEVICE = {seg: dev for dev, seg in SENSOR_MAP.items()}

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

# Every drawn line: (segment providing the colour, parent joint, child joint)
BONES = [(seg, parent, seg) for seg, (parent, _) in SKELETON.items() if parent]
TIP_BONES = [(seg, seg, None) for seg in TIPS]


def limb_colour(seg):
    if seg.startswith("right_"):
        return RIGHT
    if seg.startswith("left_"):
        return LEFT
    return TORSO


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


def q_mean(quats):
    """Average orientation over a hold, e.g. the T-pose window.

    Quaternions double-cover rotations, so q and -q are the same pose; summing
    them naively cancels out. Flipping each sample onto the same hemisphere as
    the first fixes that, and for the few degrees of sway in a held T-pose the
    normalised sum is indistinguishable from the proper eigenvector solution.
    """
    quats = np.asarray(quats, dtype=float)
    if len(quats) == 0:
        return None
    reference = quats[0]
    aligned = np.where((quats @ reference)[:, None] < 0, -quats, quats)
    mean = aligned.mean(axis=0)
    norm = np.linalg.norm(mean)
    return (mean / norm) if norm > 1e-9 else None


def yaw_of(q):
    """Heading of an orientation, in radians about the global Z (up) axis."""
    R = q_to_R(np.asarray(q, dtype=float))
    return np.arctan2(R[1, 0], R[0, 0])


def Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def bone_rotations(quats, tpose, heading=0.0):
    """Per-segment world orientation for rendering.

    Per segment: the earth-frame rotation since T-pose, re-expressed in the
    skeleton frame by conjugating with the facing::

        D_i = R(q_i) . R(q_i,Tpose)^-1              # earth-frame, mounting-free
        W_i = Rz(-heading) . D_i . Rz(heading)      # viewed in the skeleton frame

    D_i is the true rotation of segment i and reproduces the physical joint
    angles (verified: rendered knee and elbow track the inter-sensor angle to a
    few degrees). Its one flaw is that it lives in the sensors' earth frame,
    which is tied to magnetic north, not the screen -- so raise an arm facing
    one way vs another and D_i points a different way on screen. The heading
    conjugation rotates that earth frame onto the skeleton frame.

    `heading` must be measured against a per-session reference, NOT the raw
    pelvis yaw: the pelvis yaw is relative to magnetic north (plus the pelvis
    sensor's mounting), so using it raw spins every take by that constant and
    breaks the ones that were already right. The caller passes
    heading = yaw_of(pelvis at this take's T-pose) - session_reference_yaw,
    where the reference is the first (correctly-facing) take of the session.
    Then the reference take gets heading 0 (identical to the original,
    heading-free algorithm) and the rest are rotated onto it -- which makes the
    render facing-independent (rotating every quaternion by any yaw leaves it
    identical, 0.000 m drift) while preserving every joint angle.
    """
    align = Rz(-heading)
    align_T = align.T
    out = {seg: np.eye(3) for seg in SKELETON}
    for seg, q in quats.items():
        q0 = tpose.get(seg)
        if q0 is None or seg not in out:
            continue
        delta = q_to_R(q_mul(np.asarray(q, dtype=float),
                             q_conj(np.asarray(q0, dtype=float))))
        out[seg] = align @ delta @ align_T
    return out


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
