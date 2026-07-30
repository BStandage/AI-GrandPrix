"""
Reconstruct the REAL flown track of a tape flight from its recorded detections + the map,
and overlay it against the plan in three projections.

    python -m analysis.tape_track datasets/ace_dbg_<run>.csv datasets/session_<run>

Per frame: every located OPENING gives a metric world vector camera->gate (PnP dist + image
angles + logged commanded attitude/heading, elevation bias-calibrated). Matching a detection
to a map gate implies a drone position fix: pos = gate - vec. Fixes are chained: each frame's
match is chosen against the PREVIOUS position estimate (nearest predicted gate), so the track
follows through the course without trusting any dynamics model.

Writes datasets/tape_track_view.png (plan blue, real track red, gates green).
"""

import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common.camera import HALF_TAN_X, HALF_TAN_Y

UPTILT = math.radians(20.0)
EL_BIAS = math.radians(3.70)


def main():
    dbg_path, sess = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(dbg_path)))
    frames = [json.loads(l) for l in open(os.path.join(sess, "vision_frames.jsonl"))]
    t0_ns = frames[0]["sim_time_ns"]
    dur_ns = max(frames[-1]["sim_time_ns"] - t0_ns, 1)

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    gates = json.load(open(os.path.join(here, "pilots", "ace_pilot", "course_map.json")))["gates"]
    traj = json.load(open(os.path.join(here, "pilots", "ace_pilot", "trajectory.json")))

    def dbg_at(frac):
        return rows[min(int(frac * len(rows)), len(rows) - 1)]

    pos = (0.0, 0.0, 0.0)
    track = []
    for f in frames:
        if not f.get("dets"):
            continue
        frac = (f["sim_time_ns"] - t0_ns) / dur_ns
        r = dbg_at(frac)
        try:
            pitch = math.radians(float(r["pitch_deg"]))
            yaw = math.radians(float(r["yaw_deg"]))
        except (ValueError, KeyError):
            continue
        fixes = []
        for det in f["dets"]:
            dist = det.get("pnp_dist") or det.get("pinhole_dist")
            if not dist or not (2.5 < dist < 30.0):
                continue
            if not (det.get("has_opening") and det.get("bbox")):
                continue
            bx, by, bw, bh = det["bbox"]
            ox = ((bx + bw / 2.0) - 320.0) / 320.0
            oy = ((by + bh / 2.0) - 180.0) / 180.0
            bear = yaw - math.atan(ox * HALF_TAN_X)
            el = (UPTILT - pitch) - math.atan(oy * HALF_TAN_Y) - EL_BIAS
            v = (dist * math.cos(el) * math.cos(bear),
                 dist * math.cos(el) * math.sin(bear),
                 dist * math.sin(el))
            # match to the map gate that makes the implied fix closest to the running estimate
            best = None
            for g in gates:
                fx, fy, fz = g["x"] - v[0], g["y"] - v[1], g["z"] - v[2]
                d = math.hypot(fx - pos[0], fy - pos[1])
                if best is None or d < best[0]:
                    best = (d, (fx, fy, fz))
            if best and best[0] < 8.0:
                fixes.append(best[1])
        if fixes:
            fx = sorted(p[0] for p in fixes)[len(fixes) // 2]
            fy = sorted(p[1] for p in fixes)[len(fixes) // 2]
            fz = sorted(p[2] for p in fixes)[len(fixes) // 2]
            pos = (0.6 * pos[0] + 0.4 * fx, 0.6 * pos[1] + 0.4 * fy, 0.6 * pos[2] + 0.4 * fz)
            track.append((frac, *pos))

    print(f"{len(track)} position fixes")
    px = [t[1] for t in track]
    py = [t[2] for t in track]
    pz = [t[3] for t in track]
    tx = [r[2] for r in traj["samples"]]
    ty = [r[3] for r in traj["samples"]]
    tz = [r[4] for r in traj["samples"]]
    gx = [g["x"] for g in gates]
    gy = [g["y"] for g in gates]
    gz = [g["z"] for g in gates]
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))
    for ax, (a1, a2, b1, b2, c1, c2, n1, n2) in zip(axes, [
            (tx, ty, px, py, gx, gy, "x", "y"),
            (tx, tz, px, pz, gx, gz, "x", "z"),
            (ty, tz, py, pz, gy, gz, "y", "z")]):
        ax.plot(a1, a2, "b-", lw=1.2, label="plan")
        ax.plot(b1, b2, "r-", lw=1.2, label="REAL (vision fixes)")
        ax.scatter(c1, c2, c="g", s=50, zorder=5)
        ax.set_xlabel(n1)
        ax.set_ylabel(n2)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="datalim")
    axes[0].legend()
    out = os.path.join(here, "datasets", "tape_track_view.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main()
