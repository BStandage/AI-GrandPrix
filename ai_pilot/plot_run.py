#
# Visualise a flown run: the drone's actual path vs the gates vs the planned racing
# line, from top / side / 3D. Saves a PNG to analyse the line instead of guessing
# from raw numbers.
#
# Usage:  python plot_run.py [session_dir]   (defaults to newest with telemetry)
#
# Frames: NED (x fwd-ish/north, y right/east, z DOWN). We plot ALTITUDE = -z so up is up.
#

import glob
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from trajectory import Trajectory

HALF = 1.36   # gate half-opening (m), gate is 2.72 m


def newest_session():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = sorted(glob.glob(os.path.join(here, "datasets", "session_*")), key=os.path.getmtime)
    for s in reversed(cands):
        if os.path.exists(os.path.join(s, "telemetry.jsonl")):
            return s
    raise SystemExit("no session with telemetry found")


def load_gates(sess):
    own = os.path.join(sess, "gates.json")
    if os.path.exists(own):
        return json.load(open(own))["gates"], "own live track"
    here = os.path.dirname(os.path.abspath(__file__))
    cached = sorted(glob.glob(os.path.join(here, "datasets", "*", "gates.json")), key=os.path.getmtime)
    return json.load(open(cached[-1]))["gates"], "cached " + os.path.basename(os.path.dirname(cached[-1]))


def main():
    sess = sys.argv[1] if len(sys.argv) > 1 else newest_session()
    gates, gsrc = load_gates(sess)
    traj = Trajectory(gates, v_max=8.5, apex_max=0.6)

    odo_rows = [(o["recv_time_ns"], o["x"], o["y"], o["z"]) for o in
                (json.loads(l) for l in open(os.path.join(sess, "telemetry.jsonl")))
                if o.get("kind") == "odometry"]
    od = np.array([[r[1], r[2], r[3]] for r in odo_rows])
    odo_t = np.array([r[0] for r in odo_rows])
    # collisions: (time, id, impact). 1001 = gate, 1002 = environment.
    colls = [(o["recv_time_ns"], o.get("collision_id"), o.get("impact", 0.0)) for o in
             (json.loads(l) for l in open(os.path.join(sess, "telemetry.jsonl")))
             if o.get("kind") == "collision"]
    # drone position at each collision (nearest odometry by time) - BEFORE any path cut
    coll_pts = []
    for ct, cid, imp in colls:
        i = int(np.argmin(np.abs(odo_t - ct)))
        coll_pts.append((od[i, 0], od[i, 1], -od[i, 2], cid, imp))
    # cut at the first big jump (sim reset after a crash) so we see only the first attempt
    if len(od) > 2:
        jumps = np.where(np.linalg.norm(np.diff(od, axis=0), axis=1) > 5.0)[0]
        if len(jumps):
            od = od[:jumps[0] + 1]
    # keep only collisions that occurred on the shown (first-attempt) segment
    if len(od):
        coll_pts = [c for c in coll_pts if c[0] >= od[:, 0].min() - 2]

    dx, dy, dalt = od[:, 0], od[:, 1], -od[:, 2]
    lx, ly, lalt = traj.pts[:, 0], traj.pts[:, 1], -traj.pts[:, 2]
    gx = np.array([g["position_ned"][0] for g in gates])
    gy = np.array([g["position_ned"][1] for g in gates])
    galt = np.array([-g["position_ned"][2] for g in gates])

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"{os.path.basename(sess)}   gates: {gsrc}   (drone ended x={dx[-1]:.0f})", fontsize=11)

    def gate_markers(ax, hx, hv, vx, vv, horiz_is_y):
        for i in range(len(gx)):
            # draw each gate's opening as a bar in this projection
            if horiz_is_y:   # top view: gate spans in Y at its X
                ax.plot([gx[i], gx[i]], [gy[i] - HALF, gy[i] + HALF], "r-", lw=3, alpha=0.5)
            else:            # side view: gate spans in ALT at its X
                ax.plot([gx[i], gx[i]], [galt[i] - HALF, galt[i] + HALF], "r-", lw=3, alpha=0.5)
            ax.annotate(str(i), (hx[i], hv[i]), color="red", fontsize=12, fontweight="bold")

    # TOP view: X vs Y
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(lx, ly, "c--", lw=1.5, label="planned line")
    ax1.plot(dx, dy, "b-", lw=2, label="drone path")
    for i in range(len(gx)):
        ax1.plot([gx[i], gx[i]], [gy[i] - HALF, gy[i] + HALF], "r-", lw=4, alpha=0.5)
        ax1.annotate(str(i), (gx[i], gy[i]), color="red", fontsize=12, fontweight="bold")
    ax1.set_title("TOP-DOWN  (X vs Y) - red bars = gate openings sideways")
    ax1.set_xlabel("x (forward, m)"); ax1.set_ylabel("y (right, m)")
    ax1.legend(); ax1.grid(alpha=0.3); ax1.invert_xaxis()

    # SIDE view: X vs Altitude
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(lx, lalt, "c--", lw=1.5, label="planned line")
    ax2.plot(dx, dalt, "b-", lw=2, label="drone path")
    for i in range(len(gx)):
        ax2.plot([gx[i], gx[i]], [galt[i] - HALF, galt[i] + HALF], "r-", lw=4, alpha=0.5)
        ax2.annotate(str(i), (gx[i], galt[i]), color="red", fontsize=12, fontweight="bold")
    ax2.set_title("SIDE  (X vs altitude) - red bars = gate openings vertically")
    ax2.set_xlabel("x (forward, m)"); ax2.set_ylabel("altitude (m, up)")
    ax2.legend(); ax2.grid(alpha=0.3); ax2.invert_xaxis()

    # FRONT-ish view: Y vs Altitude
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(ly, lalt, "c--", lw=1.5, label="planned line")
    ax3.plot(dy, dalt, "b-", lw=2, label="drone path")
    ax3.scatter(gy, galt, c="red", s=60, marker="s", label="gate centres")
    for i in range(len(gx)):
        ax3.annotate(str(i), (gy[i], galt[i]), color="red", fontsize=11, fontweight="bold")
    ax3.set_title("FRONT  (Y vs altitude)")
    ax3.set_xlabel("y (right, m)"); ax3.set_ylabel("altitude (m, up)")
    ax3.legend(); ax3.grid(alpha=0.3)

    # 3D
    ax4 = fig.add_subplot(2, 2, 4, projection="3d")
    ax4.plot(lx, ly, lalt, "c--", lw=1.5, label="planned line")
    ax4.plot(dx, dy, dalt, "b-", lw=2, label="drone path")
    ax4.scatter(gx, gy, galt, c="red", s=60, marker="s")
    for i in range(len(gx)):
        ax4.text(gx[i], gy[i], galt[i], str(i), color="red", fontsize=11)
    ax4.set_xlabel("x"); ax4.set_ylabel("y"); ax4.set_zlabel("alt")
    ax4.set_title("3D"); ax4.legend()

    # COLLISION markers (big black X) on every view
    for cx, cy, calt, cid, imp in coll_pts:
        lbl = "GATE HIT" if cid == 1001 else "ENV HIT"
        ax1.scatter([cx], [cy], c="k", s=220, marker="x", lw=3, zorder=6)
        ax1.annotate(f"{lbl} ({imp:.1f})", (cx, cy), color="k", fontsize=9, fontweight="bold")
        ax2.scatter([cx], [calt], c="k", s=220, marker="x", lw=3, zorder=6)
        ax3.scatter([cy], [calt], c="k", s=220, marker="x", lw=3, zorder=6)
        ax4.scatter([cx], [cy], [calt], c="k", s=140, marker="x")
    print(f"{len(coll_pts)} collision(s) on the shown segment")

    out = os.path.join(sess, "racing_line.png")
    plt.tight_layout()
    plt.savefig(out, dpi=90)
    print("wrote", out)


if __name__ == "__main__":
    main()
