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

from gate_detector import gate_mask, MIN_GATE_AREA_FRAC
from gate_geometry import relative_gate

# --- camera model (measured; see gate_detector.py / controller.py) ---
FOCAL_PX = 229.0
GATE_SIZE_M = 2.72          # gate width/height from gates.json (square)
CAM_UPTILT_RAD = 0.446      # camera tilted UP ~26 deg from body forward


def find_session(arg):
    here = os.path.dirname(os.path.abspath(__file__))
    if arg:
        return arg
    cands = sorted(glob.glob(os.path.join(here, "datasets", "*", "gates.json")), key=os.path.getmtime)
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
    here = os.path.dirname(os.path.abspath(__file__))
    cached = sorted(glob.glob(os.path.join(here, "datasets", "*", "gates.json")), key=os.path.getmtime)
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


def gate_corners(img):
    """Return the 4 corners (ordered TL,TR,BR,BL) of the largest orange gate ring,
    or None. Uses a 4-point polygon approx of the ring's convex hull; falls back to
    the min-area rotated rectangle."""
    h, w = img.shape[:2]
    mask = gate_mask(img)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_GATE_AREA_FRAC * h * w]
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    hull = cv2.convexHull(c)
    peri = cv2.arcLength(hull, True)
    quad = None
    for k in (0.02, 0.04, 0.06, 0.08, 0.10):
        approx = cv2.approxPolyDP(hull, k * peri, True)
        if len(approx) == 4:
            quad = approx.reshape(4, 2).astype(np.float32)
            break
    if quad is None:
        quad = cv2.boxPoints(cv2.minAreaRect(c)).astype(np.float32)
    # order TL, TR, BR, BL
    s = quad.sum(axis=1)
    d = quad[:, 0] - quad[:, 1]
    return np.array([quad[np.argmin(s)], quad[np.argmax(d)],
                     quad[np.argmax(s)], quad[np.argmin(d)]], dtype=np.float32)


def pnp_pose_body(corners, w, h):
    """solvePnP the gate -> (forward, right, down) of the gate centre in BODY frame."""
    half = GATE_SIZE_M / 2.0
    obj = np.array([[-half, -half, 0], [half, -half, 0],
                    [half, half, 0], [-half, half, 0]], dtype=np.float32)  # x right, y down
    K = np.array([[FOCAL_PX, 0, w / 2.0], [0, FOCAL_PX, h / 2.0], [0, 0, 1]], dtype=np.float32)
    # ITERATIVE (not IPPE_SQUARE - that gave ~0.4x the true distance on these gates)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    t = np.asarray(tvec).reshape(3)
    x, y, z = float(t[0]), float(t[1]), float(t[2])   # OpenCV cam: x right, y down, z fwd
    # camera axes -> body-aligned (fwd=z, right=x, down=y), then rotate by the up-tilt
    fwd, right, down = z, x, y
    c, s = math.cos(CAM_UPTILT_RAD), math.sin(CAM_UPTILT_RAD)
    return (fwd * c + down * s, right, -fwd * s + down * c)


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
