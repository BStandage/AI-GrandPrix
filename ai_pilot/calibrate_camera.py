#
# Estimate the camera focal length (and FOV) from a straight-in gate approach,
# using only the gate detector + odometry - NO ground truth needed.
#
# As the drone flies straight at a gate, the gate's pixel height follows
#   px_height = f * H / distance         (H = real gate height, 2.7 m)
# and distance = (x_drone - x_gate) along the approach axis. So
#   1/px_height = (1/(f*H)) * x_drone - x_gate/(f*H)
# is LINEAR in x_drone -> a line fit gives f*H and x_gate, hence f.
#
# Usage: python calibrate_camera.py <session_dir>
#

import csv
import json
import math
import os
import sys

import cv2

from gate_detector import detect_gates

GATE_REAL_HEIGHT_M = 2.7
SUBSAMPLE = 6


def load_odometry(session):
    odo = []
    with open(os.path.join(session, "telemetry.jsonl")) as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("kind") == "odometry":
                odo.append((r["recv_time_ns"], r["x"], r["y"], r["z"]))
    odo.sort()
    return odo


def nearest_odo(odo, t):
    # binary-ish nearest by recv_time_ns
    best = min(odo, key=lambda o: abs(o[0] - t))
    return best


def main():
    session = sys.argv[1] if len(sys.argv) > 1 else "datasets/session_20260605_221219"
    odo = load_odometry(session)
    if not odo:
        print("no odometry in session")
        return

    frames = []
    with open(os.path.join(session, "frames.jsonl")) as f:
        for line in f:
            try:
                frames.append(json.loads(line))
            except ValueError:
                pass

    pts = []  # (x_drone, px_height, center_y_norm, w, h)
    for i, fr in enumerate(frames):
        if i % SUBSAMPLE:
            continue
        img = cv2.imread(os.path.join(session, fr["file"]))
        if img is None:
            continue
        dets, _ = detect_gates(img)
        if not dets:
            continue
        g = dets[0]                      # biggest = the gate we're approaching
        _, _, bw, bh = g["bbox"]
        _, x_drone, _, _ = nearest_odo(odo, fr["recv_time_ns"])
        pts.append((x_drone, bh, g["offset_y"], fr["width"], fr["height"]))

    if len(pts) < 8:
        print(f"only {len(pts)} usable frames - need a cleaner approach")
        return

    # linear fit of 1/px_height vs x_drone
    xs = [p[0] for p in pts]
    inv = [1.0 / p[1] for p in pts]
    n = len(xs)
    mx, my = sum(xs) / n, sum(inv) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, inv))
    slope = sxy / sxx
    intercept = my - slope * mx

    K = 1.0 / slope                      # = f * H
    x_gate = -intercept * K
    f_px = K / GATE_REAL_HEIGHT_M
    W = pts[0][3]
    hfov = math.degrees(2 * math.atan(W / (2 * f_px)))
    mean_offy = sum(p[2] for p in pts) / n

    print(f"frames used: {n}")
    print(f"focal length f ~ {f_px:.0f} px")
    print(f"horizontal FOV ~ {hfov:.0f} deg  (image width {W}px)")
    print(f"gate at x ~ {x_gate:.1f} (drone x ran {min(xs):.1f}..{max(xs):.1f})")
    print(f"mean gate offset_y over approach ~ {mean_offy:+.2f}  "
          f"(persistent + => camera tilted UP / drone flying below gate centre)")
    print("\nUse f for PnP distance:  distance = f * 2.7 / gate_pixel_height")


if __name__ == "__main__":
    main()
