"""Turn a recorded take into model-ready calibrated data for ALL 17 sensors.

The raw CSVs this repo records are the *unprocessed* signal -- per-sensor
calibrated acc (gravity included) + gyro + mag + a sensor->earth orientation
quaternion, on the packet-counter timeline. This script produces the
*processed* form every sparse-IMU pose model (TransPose, DIP/Deep Inertial
Poser, and the many that inherit their convention) expects: orientation as a
rotation matrix in the body/SMPL frame, gravity-free acceleration, on a clean
60 Hz grid with the small packet-loss gaps interpolated.

We compute this for all 17 segments (not just the 6 a sparse model reads), so
the processed dataset is complete; the 6-sensor TransPose input vector is then
just a subset of it.

Produced per take, in `processed/`:
  - calibrated.npz : ori_smpl (T,17,3,3), acc_free_smpl (T,17,3),
    gyr (T,17,3), the segment order, the 60 Hz counter grid, and the
    calibration used -- model-agnostic, so any model's own normaliser applies.
  - transpose_input.npy : (T,72) = [18 acc | 54 ori], the 6-sensor TransPose
    live-demo layout (root-relative, self-contained, no external stats).

Format verified against TransPose `live_demo.py` and DIP `inference_server.py`
/ `read_mtw.cs` / `read_TC_data.py`.

Calibration choices (the honest weak points, documented):
  - Gravity removed analytically: a_earth = R(q).a_sensor - [0,0,9.81]. Each
    take's static T-pose check gives earth gravity ~ [0,0,+9.81] (Xsens ENU,
    Z up), so +Z / 9.81 fits this data; re-verify per rig.
  - device2bone uses the TransPose assumption that each bone is at identity in
    the SMPL frame at T-pose: device2bone[s] = (smpl2imu . R_Tpose[s])^T. Any
    T-pose imperfection becomes a constant per-sensor offset.
  - smpl2imu maps earth (ENU) -> SMPL (x=Left,y=Up,z=Forward) and yaws the
    pelvis T-pose facing onto SMPL +Z. Model inputs are root-relative and
    cancel smpl2imu, so this only sets the root's absolute orientation.

NOT validated: whether a given pretrained model yields good poses from this --
that needs the model's weights/runtime. This guarantees FORMAT + physics
(units, frame, gravity, gaps, 60 Hz), not model fit.

Usage:
    python3 postprocess.py data/S01/20260916_session1/take01_squat
    python3 postprocess.py --session data/S01/20260916_session1
"""

import argparse
import csv
import json
import math
import os

import numpy as np

from skeleton import SENSOR_MAP, REFERENCE_SEGMENT

# All 17 segments, pelvis (the root) LAST so ALL_SEGMENTS[-1] is the root.
ALL_SEGMENTS = [s for s in SENSOR_MAP.values() if s != REFERENCE_SEGMENT] \
    + [REFERENCE_SEGMENT]
ROOT = len(ALL_SEGMENTS) - 1

# The 6 sensors a TransPose/DIP model reads, in their fixed order (root last).
MODEL_SENSORS = ["left_forearm", "right_forearm", "left_shank",
                 "right_shank", "head", "pelvis"]

RATE = 60
GRAVITY = np.array([0.0, 0.0, 9.81])   # earth ENU, Z up (verified per take)
ACC_SCALE = 30.0                       # TransPose config.acc_scale
# Earth ENU (x=East,y=North,z=Up) -> SMPL (x=Left,y=Up,z=Forward); the exact
# horizontal mapping is re-settled by the T-pose yaw, so only Up must line up.
ENU_TO_SMPL = np.array([[1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [0.0, 1.0, 0.0]])


def q_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_only(R):
    yaw = math.atan2(R[1, 0], R[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def load_take(take_dir):
    """Return {seg: (counters, quats, accs, gyrs)} in ACQUISITION order.

    Rows are kept in CSV (time) order, never sorted by counter value: the
    16-bit counter wraps mid-take (e.g. 65535 -> 0), and sorting numerically
    would interleave post-wrap samples ahead of pre-wrap ones and destroy the
    timeline.
    """
    meta = json.load(open(os.path.join(take_dir, "metadata.json")))
    data = {}
    for seg in ALL_SEGMENTS:
        path = os.path.join(take_dir, seg + ".csv")
        if not os.path.exists(path):
            raise SystemExit(f"missing sensor CSV: {path}")
        counters, quats, accs, gyrs = [], [], [], []
        for r in csv.DictReader(open(path)):
            if not (r["counter"] and r["q_w"] and r["acc_x"] and r["gyr_x"]):
                continue
            counters.append(int(r["counter"]))
            quats.append([float(r["q_w"]), float(r["q_x"]),
                          float(r["q_y"]), float(r["q_z"])])
            accs.append([float(r["acc_x"]), float(r["acc_y"]), float(r["acc_z"])])
            gyrs.append([float(r["gyr_x"]), float(r["gyr_y"]), float(r["gyr_z"])])
        data[seg] = (np.array(counters), np.array(quats),
                     np.array(accs), np.array(gyrs))
    return data, meta


def unwrap(counters):
    prev, off, out = counters[0], 0, []
    for c in counters:
        if c < prev - 30000:
            off += 65536
        prev = c
        out.append(c + off)
    return np.array(out)


def build_grid(data):
    """One continuous 60 Hz grid on the shared master counter, covering the
    range every sensor spans.

    All 17 sensors share the master's counter, so unwrapping each in
    acquisition order already puts them on the same absolute scale (they all
    start within a couple of counts and increment together). The grid is the
    overlap [max first, min last]."""
    abs_un = {}
    lo, hi = -(10**18), 10**18
    for seg in ALL_SEGMENTS:
        a = unwrap(data[seg][0])
        abs_un[seg] = a
        lo, hi = max(lo, a[0]), min(hi, a[-1])
    grid = np.arange(lo, hi + 1)
    return grid, abs_un


def interp_sensor(seg_data, abscnt, grid):
    counters, quats, accs, gyrs = seg_data
    quats = quats.copy()
    for i in range(1, len(quats)):        # keep quaternion hemisphere consistent
        if quats[i] @ quats[i - 1] < 0:
            quats[i] = -quats[i]
    q = np.stack([np.interp(grid, abscnt, quats[:, k]) for k in range(4)], 1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    a = np.stack([np.interp(grid, abscnt, accs[:, k]) for k in range(3)], 1)
    g = np.stack([np.interp(grid, abscnt, gyrs[:, k]) for k in range(3)], 1)
    return q, a, g


def calibrate(q_grid, acc_grid, tpose_quat):
    """q_grid/acc_grid (N,K,..) over ALL_SEGMENTS. Returns ori_smpl (K,N,3,3),
    acc_free_smpl (K,N,3)."""
    N, K = q_grid.shape[0], q_grid.shape[1]
    R_T = np.stack([q_to_R(np.asarray(tpose_quat[ALL_SEGMENTS[s]]))
                    for s in range(N)])
    pelvis_T_smpl = ENU_TO_SMPL @ R_T[ROOT]
    smpl2imu = yaw_only(pelvis_T_smpl).T @ ENU_TO_SMPL
    d2b = np.stack([(smpl2imu @ R_T[s]).T for s in range(N)])

    ori = np.empty((K, N, 3, 3))
    acc = np.empty((K, N, 3))
    for s in range(N):
        for t in range(K):
            R = q_to_R(q_grid[s, t])
            ori[t, s] = smpl2imu @ R @ d2b[s]
            acc[t, s] = smpl2imu @ (R @ acc_grid[s, t] - GRAVITY)
    return ori, acc, smpl2imu


def transpose_input(ori_all, acc_all):
    """TransPose 72-dim from the 6-sensor subset (root-relative, heading-free)."""
    idx = [ALL_SEGMENTS.index(s) for s in MODEL_SENSORS]
    ori = ori_all[:, idx]          # (K,6,3,3), root last
    acc = acc_all[:, idx]
    K = ori.shape[0]
    R_root = ori[:, 5]
    acc6 = acc.copy()
    acc6[:, :5] = acc[:, :5] - acc[:, 5:6]
    acc6 = np.einsum("ksj,kjl->ksl", acc6, R_root) / ACC_SCALE
    ori6 = ori.copy()
    ori6[:, :5] = np.einsum("kij,ksjl->ksil",
                            np.transpose(R_root, (0, 2, 1)), ori[:, :5])
    return np.concatenate([acc6.reshape(K, 18), ori6.reshape(K, 54)], axis=1)


def process(take_dir):
    data, meta = load_take(take_dir)
    tpose = meta.get("tpose_quat")
    if not tpose or any(s not in tpose for s in ALL_SEGMENTS):
        raise SystemExit("metadata has no tpose_quat for all 17 segments")

    grid, grids = build_grid(data)
    N, K = len(ALL_SEGMENTS), len(grid)
    q_grid = np.empty((N, K, 4))
    acc_grid = np.empty((N, K, 3))
    gyr_grid = np.empty((N, K, 3))
    for s, seg in enumerate(ALL_SEGMENTS):
        q_grid[s], acc_grid[s], gyr_grid[s] = interp_sensor(
            data[seg], grids[seg], grid)

    ori, acc, smpl2imu = calibrate(q_grid, acc_grid, tpose)
    gyr = np.transpose(gyr_grid, (1, 0, 2))        # (K,N,3)
    nn = transpose_input(ori, acc)

    out_dir = os.path.join(take_dir, "processed")
    os.makedirs(out_dir, exist_ok=True)
    np.savez(os.path.join(out_dir, "calibrated.npz"),
             ori_smpl=ori.astype(np.float32),
             acc_free_smpl=acc.astype(np.float32),
             gyr=gyr.astype(np.float32),
             segment_order=np.array(ALL_SEGMENTS),
             model_sensors=np.array(MODEL_SENSORS),
             counter_grid=grid.astype(np.int64),
             rate_hz=RATE, gravity=GRAVITY, acc_scale=ACC_SCALE,
             smpl2imu=smpl2imu.astype(np.float32))
    np.save(os.path.join(out_dir, "transpose_input.npy"), nn.astype(np.float32))
    return grid, ori, acc, nn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="a take dir, or a session dir with --session")
    ap.add_argument("--session", action="store_true",
                    help="target is a session dir; process every take under it")
    args = ap.parse_args()

    takes = []
    if args.session:
        for name in sorted(os.listdir(args.target)):
            d = os.path.join(args.target, name)
            if os.path.isdir(d) and os.path.exists(os.path.join(d, "metadata.json")):
                takes.append(d)
    else:
        takes = [args.target]

    for d in takes:
        grid, ori, acc, nn = process(d)
        still = int(np.argmin(np.linalg.norm(acc.reshape(len(grid), -1), axis=1)))
        print(f"{os.path.basename(d)}: {len(grid)} frames @ {RATE}Hz  "
              f"| 17-sensor ori {ori.shape} acc {acc.shape}  "
              f"| transpose_input {nn.shape}  "
              f"| min|free-acc|/frame ~{np.linalg.norm(acc[still]):.2f} m/s^2")


if __name__ == "__main__":
    main()
