#
# OFFLINE PnP validation: estimate each gate's 3D pose from the CAMERA alone and
# check it against ground truth - no flying needed.
#
# Why: the pursuit/trajectory controllers steer on a gate's body-frame geometry
# (forward/right/down), which today comes from the sim's ground truth
# (active_gate_relative). The real race has no ground truth - we must get the SAME
# numbers from the camera: detect the gate's 4 corners -> cv2.solvePnP against the
# known 2.72 m gate + focal length -> gate pose in the camera -> rotate into body
# frame. This script runs that pipeline on recorded frames and compares to the
# ground-truth relative pose (from the synced odometry + gates.json), so we can
# measure the error BEFORE trusting it in flight.
#
# Usage:  python pnp_offline.py [session_dir]   (defaults to newest with gates.json+frames)
#

import glob
import json
import math
import os
import sys

import cv2
import numpy as np

# Allow running this file directly (python analysis/pnp_offline.py) by putting the
# src/ package root on sys.path, not just this subpackage's folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.gate_geometry import relative_gate
from common.paths import DATASETS_DIR
from perception.vision_pose import gate_corners, pnp_pose_body   # shared camera->gate-pose pipeline


def find_session(arg):
    if arg:
        return arg
    cands = sorted(glob.glob(os.path.join(DATASETS_DIR, "*", "gates.json")), key=os.path.getmtime)
    for g in reversed(cands):
        d = os.path.dirname(g)
        if os.path.isdir(os.path.join(d, "frames")):
            return d
    raise SystemExit("no session with gates.json + frames found")


def load_gates(sess):
    """Gates for ground truth: prefer this session's own; else the most recent cached
    layout (the track is static and shares the odometry NED frame, so a flight session
    that missed the live broadcast can still be validated against the cached gates)."""
    own = os.path.join(sess, "gates.json")
    if os.path.exists(own):
        return json.load(open(own))["gates"]
    cached = sorted(glob.glob(os.path.join(DATASETS_DIR, "*", "gates.json")), key=os.path.getmtime)
    if not cached:
        raise SystemExit("no gates.json anywhere to use as ground truth")
    print(f"  (session has no gates.json; using cached {os.path.relpath(cached[-1])})")
    return json.load(open(cached[-1]))["gates"]


def load_odometry(path):
    """Return list of (recv_time_ns, pos_xyz, quat_wxyz) sorted by time."""
    rows = []
    with open(os.path.join(path, "telemetry.jsonl")) as f:
        for line in f:
            o = json.loads(line)
            if o.get("kind") == "odometry":
                rows.append((o["recv_time_ns"],
                             (o["x"], o["y"], o["z"]),
                             (o["qw"], o["qx"], o["qy"], o["qz"])))
    rows.sort(key=lambda r: r[0])
    return rows


def nearest_odo(odo, t):
    # binary-ish nearest by time
    import bisect
    times = [r[0] for r in odo]
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return odo[0]
    if i >= len(odo):
        return odo[-1]
    return odo[i] if abs(odo[i][0] - t) < abs(odo[i - 1][0] - t) else odo[i - 1]


def main():
    sess = find_session(sys.argv[1] if len(sys.argv) > 1 else None)
    print(f"session: {os.path.relpath(sess)}")
    gates = load_gates(sess)
    odo = load_odometry(sess)
    frames = [json.loads(l) for l in open(os.path.join(sess, "frames.jsonl"))]
    frames = [fr for fr in frames if os.path.exists(os.path.join(sess, fr["file"]))]
    step = max(1, len(frames) // 60)
    print(f"{len(frames)} frames with images, sampling every {step}\n")
    print("  frame   PnP(fwd,right,down)        GT(fwd,right,down)      dist err  pos err")

    errs, dist_errs = [], []
    for fr in frames[::step]:
        img = cv2.imread(os.path.join(sess, fr["file"]))
        if img is None:
            continue
        h, w = img.shape[:2]
        corners = gate_corners(img)
        if corners is None:
            continue
        est = pnp_pose_body(corners, w, h)
        if est is None:
            continue
        odo_t, pos, quat = nearest_odo(odo, fr["recv_time_ns"])
        # the gate the detector locks onto = the nearest one that's in front of the
        # camera (allow ones right at/just past us, forward > -3), nearest by distance
        gts = [relative_gate(pos, quat, g) for g in gates]
        gts = [g for g in gts if g["forward"] > -3.0 and abs(g["azimuth_deg"]) < 55]
        if not gts:
            continue
        gt = min(gts, key=lambda g: g["distance"])
        gt_v = np.array([gt["forward"], gt["right"], gt["down"]])
        est_v = np.array(est)
        pos_err = float(np.linalg.norm(est_v - gt_v))
        dist_err = abs(float(np.linalg.norm(est_v)) - float(np.linalg.norm(gt_v)))
        errs.append(pos_err)
        dist_errs.append(dist_err)
        dt_ms = abs(odo_t - fr["recv_time_ns"]) / 1e6
        print(f"  {fr['frame_id']:6d}  ({est[0]:5.1f},{est[1]:5.1f},{est[2]:5.1f})    "
              f"({gt['forward']:5.1f},{gt['right']:5.1f},{gt['down']:5.1f})  g{gt['gate_id']} "
              f"sync{dt_ms:3.0f}ms  {dist_err:5.1f}m   {pos_err:5.1f}m")

    if errs:
        print(f"\n{len(errs)} matched.  pos err: median {np.median(errs):.2f} m, "
              f"mean {np.mean(errs):.2f} m, p90 {np.percentile(errs, 90):.2f} m")
        print(f"               dist err: median {np.median(dist_errs):.2f} m, mean {np.mean(dist_errs):.2f} m")
    else:
        print("no matches - detector/PnP found nothing usable")


if __name__ == "__main__":
    main()
