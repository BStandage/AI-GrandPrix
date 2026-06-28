#
# Visualise a flown run: the drone's actual path vs the gates vs the planned racing
# line, from top / side / 3D. Saves a PNG to analyse the line instead of guessing
# from raw numbers.
#
# Usage:  python -m analysis.plot_run [session_dir]   (defaults to newest with telemetry)
#
# Frames: NED (x fwd-ish/north, y right/east, z DOWN). We plot ALTITUDE = -z so up is up.
#

import glob
import json
import os
import sys

# Allow running this file directly (python analysis/plot_run.py) by putting the
# src/ package root on sys.path, not just this subpackage's folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common.trajectory import Trajectory
from common.paths import DATASETS_DIR

HALF = 1.36   # gate half-opening (m), gate is 2.72 m


def newest_session():
    cands = sorted(glob.glob(os.path.join(DATASETS_DIR, "session_*")), key=os.path.getmtime)
    for s in reversed(cands):
        if os.path.exists(os.path.join(s, "telemetry.jsonl")):
            return s
    raise SystemExit("no session with telemetry found")


# The gate course is DETERMINISTIC - the SAME fixed positions every run (gate 0 at x=-23.3 ... gate 5 at
# x=-159.2). Some cached gates.json are corrupt - a different frame / junk (gates at x=+88, +1700, etc.).
# Do NOT frame-match by centroid: on a short/crashed path a corrupt file's centroid can sit closer than the
# real course's, drawing gates in the wrong place and making a clean PASS look like a clip. Lock onto the
# fixed course instead - accept only a gates.json whose gate 0 matches the known x, reject everything else.
CANON_G0X = -23.3   # gate 0 x of the fixed course; the signature that rejects corrupt cached tracks


def _is_fixed_course(gs):
    return len(gs) >= 6 and abs(gs[0]["position_ned"][0] - CANON_G0X) < 3.0


def load_gates(sess, od=None):
    own = os.path.join(sess, "gates.json")
    if os.path.exists(own):
        gs = json.load(open(own)).get("gates") or []
        if _is_fixed_course(gs):
            return gs, "own live track"
    # the run's own track is missing/corrupt - find any cached track that matches the fixed course (they are
    # all identical), ignoring corrupt ones. Newest first is fine since the positions never change.
    for fp in sorted(glob.glob(os.path.join(DATASETS_DIR, "*", "gates.json")), key=os.path.getmtime, reverse=True):
        gs = json.load(open(fp)).get("gates") or []
        if _is_fixed_course(gs):
            return gs, "fixed course (" + os.path.basename(os.path.dirname(fp)) + ")"
    raise RuntimeError("no gates.json matching the fixed course (gate 0 @ x=-23.3) found")


def main():
    sess = sys.argv[1] if len(sys.argv) > 1 else newest_session()
    odo_rows = [(o["recv_time_ns"], o["x"], o["y"], o["z"]) for o in
                (json.loads(l) for l in open(os.path.join(sess, "telemetry.jsonl")))
                if o.get("kind") == "odometry"]
    od = np.array([[r[1], r[2], r[3]] for r in odo_rows])
    odo_t = np.array([r[0] for r in odo_rows])
    gates, gsrc = load_gates(sess, od)
    traj = Trajectory(gates, v_max=8.5, apex_max=0.6)
    # real measured roll (rad) for the roll-timing panel - interpolated onto the odometry times below.
    att_rows = [(o["recv_time_ns"], o["roll"]) for o in
                (json.loads(l) for l in open(os.path.join(sess, "telemetry.jsonl")))
                if o.get("kind") == "attitude"]
    att = np.array(att_rows) if att_rows else np.zeros((0, 2))
    # collisions: (time, id, impact). 1001 = gate, 1002 = environment.
    colls = [(o["recv_time_ns"], o.get("collision_id"), o.get("impact", 0.0)) for o in
             (json.loads(l) for l in open(os.path.join(sess, "telemetry.jsonl")))
             if o.get("kind") == "collision"]
    # drone position at each collision (nearest odometry by time) - BEFORE any path cut
    coll_pts = []
    for ct, cid, imp in colls:
        i = int(np.argmin(np.abs(odo_t - ct)))
        coll_pts.append((od[i, 0], od[i, 1], -od[i, 2], cid, imp))
    # Split the run at big position jumps (sim resets/teleports between attempts) and keep the one
    # segment that actually flew the course. The OLD code kept the prefix before the first jump,
    # which on a clean run is the pre-race "parked" teleport blob (x~-2000, alt~800) - so the plot
    # autoscaled to that junk and the real path + gates fell off-screen (looked empty). Instead:
    # drop segments parked far from the course, then keep the one reaching furthest down-track (-x).
    if len(od) > 2:
        bnd = [0] + [i + 1 for i in np.where(np.linalg.norm(np.diff(od, axis=0), axis=1) > 5.0)[0]] + [len(od)]
        segs = list(zip(bnd, bnd[1:]))
        near = [(a, b) for (a, b) in segs if np.median(np.abs(od[a:b, 0])) < 500.0]  # course is x in [-160,0]
        cand = near or segs
        a, b = min(cand, key=lambda ab: od[ab[0]:ab[1], 0].min())   # furthest down-track = the real lap
        od = od[a:b]
        odo_t = odo_t[a:b]
    # measured roll (deg) at each shown odometry sample (+ = bank right / toward +y)
    roll_deg = np.degrees(np.interp(odo_t, att[:, 0], att[:, 1])) if len(att) else np.zeros(len(od))
    # keep only collisions that occurred on the shown (first-attempt) segment
    if len(od):
        coll_pts = [c for c in coll_pts if c[0] >= od[:, 0].min() - 2]

    dx, dy, dalt = od[:, 0], od[:, 1], -od[:, 2]
    lx, ly, lalt = traj.pts[:, 0], traj.pts[:, 1], -traj.pts[:, 2]
    gx = np.array([g["position_ned"][0] for g in gates])
    gy = np.array([g["position_ned"][1] for g in gates])
    galt = np.array([-g["position_ned"][2] for g in gates])

    # ESTIMATE vs TRUTH. TRUE range/lateral to the nearest gate ahead is computed per REAL odometry point
    # (vs real x). The pilot's ESTIMATE (dbg rng + cur_y-lf_y) is plotted against the pilot's OWN
    # dead-reckoned x (lf_x) - NOT cross-clock time (the loop runs slower than real-time, so dbg tick-time
    # != wall-time and a time-align is garbage). Small drift offsets the two in x; jumpy-magenta vs
    # smooth-green still shows the range estimate breaking (lateral = range x bearing, so it jumps too).
    Gp = np.array([g["position_ned"] for g in gates])
    true_dist, true_lat, true_vert = (np.full(len(od), np.nan) for _ in range(3))
    for i in range(len(od)):
        ahead = Gp[Gp[:, 0] < od[i, 0] - 1.0]                      # gates further forward (-x)
        if not len(ahead):
            continue
        g = ahead[np.argmin(np.linalg.norm(ahead - od[i], axis=1))]
        true_dist[i] = np.linalg.norm(g - od[i])
        true_lat[i] = g[1] - od[i, 1]
        true_vert[i] = od[i, 2] - g[2]                             # gate altitude above drone (+ = above)
    est_x = est_dist = est_lat = est_vert = None
    dbg_path = os.path.join(DATASETS_DIR, "vision_pilot_dbg.csv")
    if os.path.exists(dbg_path):
        import csv as _csv
        D = list(_csv.DictReader(open(dbg_path)))
        def _f(col):
            return np.array([float(r[col]) if r.get(col) not in (None, "") else np.nan for r in D])
        est_x, est_dist, est_lat = _f("lf_x"), _f("rng"), _f("cur_y") - _f("lf_y")
        est_vert = _f("lf_z") - _f("cur_z")                        # estimated gate altitude above drone

    fig = plt.figure(figsize=(16, 21))
    gs = fig.add_gridspec(6, 2, height_ratios=[1.0, 1.0, 0.5, 0.5, 0.5, 0.5])
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
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(lx, ly, "c--", lw=1.5, label="planned line")
    ax1.plot(dx, dy, "b-", lw=2, label="drone path")
    for i in range(len(gx)):
        ax1.plot([gx[i], gx[i]], [gy[i] - HALF, gy[i] + HALF], "r-", lw=4, alpha=0.5)
        ax1.annotate(str(i), (gx[i], gy[i]), color="red", fontsize=12, fontweight="bold")
    ax1.set_title("TOP-DOWN  (X vs Y) - red bars = gate openings sideways")
    ax1.set_xlabel("x (forward, m)"); ax1.set_ylabel("y (right, m)")
    ax1.legend(); ax1.grid(alpha=0.3); ax1.invert_xaxis()

    # SIDE view: X vs Altitude
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(lx, lalt, "c--", lw=1.5, label="planned line")
    ax2.plot(dx, dalt, "b-", lw=2, label="drone path")
    for i in range(len(gx)):
        ax2.plot([gx[i], gx[i]], [galt[i] - HALF, galt[i] + HALF], "r-", lw=4, alpha=0.5)
        ax2.annotate(str(i), (gx[i], galt[i]), color="red", fontsize=12, fontweight="bold")
    ax2.set_title("SIDE  (X vs altitude) - red bars = gate openings vertically")
    ax2.set_xlabel("x (forward, m)"); ax2.set_ylabel("altitude (m, up)")
    ax2.legend(); ax2.grid(alpha=0.3); ax2.invert_xaxis()

    # FRONT-ish view: Y vs Altitude
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(ly, lalt, "c--", lw=1.5, label="planned line")
    ax3.plot(dy, dalt, "b-", lw=2, label="drone path")
    ax3.scatter(gy, galt, c="red", s=60, marker="s", label="gate centres")
    for i in range(len(gx)):
        ax3.annotate(str(i), (gy[i], galt[i]), color="red", fontsize=11, fontweight="bold")
    ax3.set_title("FRONT  (Y vs altitude)")
    ax3.set_xlabel("y (right, m)"); ax3.set_ylabel("altitude (m, up)")
    ax3.legend(); ax3.grid(alpha=0.3)

    # 3D
    ax4 = fig.add_subplot(gs[1, 1], projection="3d")
    ax4.plot(lx, ly, lalt, "c--", lw=1.5, label="planned line")
    ax4.plot(dx, dy, dalt, "b-", lw=2, label="drone path")
    ax4.scatter(gx, gy, galt, c="red", s=60, marker="s")
    for i in range(len(gx)):
        ax4.text(gx[i], gy[i], galt[i], str(i), color="red", fontsize=11)
    ax4.set_xlabel("x"); ax4.set_ylabel("y"); ax4.set_zlabel("alt")
    ax4.set_title("3D"); ax4.legend()

    # ROLL-TIMING panel: measured bank vs x with gates marked - shows if the roll LEADS or TRAILS each
    # turn. A bank that only starts AT the gate's x (not before) is the late-roll / outside-overshoot.
    ax5 = fig.add_subplot(gs[2, :])
    ax5.plot(dx, roll_deg, "b-", lw=1.5)
    ax5.axhline(0, color="k", lw=0.8, alpha=0.5)
    for i in range(len(gx)):
        if dx.min() - 3 <= gx[i] <= dx.max() + 3:
            ax5.axvline(gx[i], color="r", ls=":", alpha=0.6)
            ax5.annotate(str(i), (gx[i], ax5.get_ylim()[1]), color="red", fontsize=11,
                         fontweight="bold", va="top")
    ax5.set_title("ROLL (deg) vs x  -  bank should LEAD each turn, not trail it  (+ = bank right / +y)")
    ax5.set_ylabel("roll (deg)")
    ax5.grid(alpha=0.3); ax5.invert_xaxis()

    # DISTANCE panel: TRUE range to the next gate (green) vs the pilot's ESTIMATED range (magenta).
    # Smooth-green vs jumpy-magenta = the depth/range estimate breaking - the thing that whips the roll.
    ax6 = fig.add_subplot(gs[3, :])
    ax6.plot(dx, true_dist, "g-", lw=1.6, label="TRUE dist to next gate")     # true vs REAL x
    if est_dist is not None:
        ax6.plot(est_x, est_dist, "m-", lw=1.2, label="pilot ESTIMATE")       # estimate vs dead-reckoned x
    ax6.legend(loc="upper right", fontsize=8); ax6.set_ylim(0, 35)
    for i in range(len(gx)):
        if dx.min() - 3 <= gx[i] <= dx.max() + 3:
            ax6.axvline(gx[i], color="r", ls=":", alpha=0.5)
    ax6.set_title("DISTANCE to target gate: TRUE (green) vs pilot ESTIMATE (magenta)")
    ax6.set_ylabel("distance (m)"); ax6.grid(alpha=0.3); ax6.invert_xaxis()

    # LATERAL panel: TRUE y-offset to the next gate (green) vs the pilot's ESTIMATE (magenta). When the
    # range estimate jumps, lateral = range x bearing jumps with it - this is what the strafe chases.
    ax7 = fig.add_subplot(gs[4, :])
    ax7.plot(dx, true_lat, "g-", lw=1.6, label="TRUE lateral to next gate")   # true vs REAL x
    if est_lat is not None:
        ax7.plot(est_x, est_lat, "m-", lw=1.2, label="pilot ESTIMATE")        # estimate vs dead-reckoned x
    ax7.axhline(0, color="k", lw=0.8, alpha=0.5)
    ax7.legend(loc="upper right", fontsize=8)
    for i in range(len(gx)):
        if dx.min() - 3 <= gx[i] <= dx.max() + 3:
            ax7.axvline(gx[i], color="r", ls=":", alpha=0.5)
    ax7.set_title("LATERAL offset to target gate: TRUE (green) vs pilot ESTIMATE (magenta)  (+ = gate to +y)")
    ax7.set_ylabel("lateral (m)")
    ax7.grid(alpha=0.3); ax7.invert_xaxis()

    # VERTICAL panel: TRUE altitude of the next gate above the drone (green) vs the pilot's ESTIMATE
    # (magenta). On the steep descent the estimate flips - it thinks a low gate is ABOVE and climbs into it.
    ax8 = fig.add_subplot(gs[5, :])
    ax8.plot(dx, true_vert, "g-", lw=1.6, label="TRUE gate altitude vs drone")  # true vs REAL x
    if est_vert is not None:
        ax8.plot(est_x, est_vert, "m-", lw=1.2, label="pilot ESTIMATE")         # estimate vs dead-reckoned x
    ax8.axhline(0, color="k", lw=0.8, alpha=0.5)
    ax8.legend(loc="upper right", fontsize=8)
    for i in range(len(gx)):
        if dx.min() - 3 <= gx[i] <= dx.max() + 3:
            ax8.axvline(gx[i], color="r", ls=":", alpha=0.5)
    ax8.set_title("VERTICAL: gate altitude above drone: TRUE (green) vs ESTIMATE (magenta)  (+ = gate higher)")
    ax8.set_xlabel("x (forward, m)"); ax8.set_ylabel("alt offset (m)")
    ax8.grid(alpha=0.3); ax8.invert_xaxis()

    # COLLISION markers (big black X) on every view
    for cx, cy, calt, cid, imp in coll_pts:
        lbl = "GATE HIT" if cid == 1001 else "ENV HIT"
        ax1.scatter([cx], [cy], c="k", s=220, marker="x", lw=3, zorder=6)
        ax1.annotate(f"{lbl} ({imp:.1f})", (cx, cy), color="k", fontsize=9, fontweight="bold")
        ax2.scatter([cx], [calt], c="k", s=220, marker="x", lw=3, zorder=6)
        ax3.scatter([cy], [calt], c="k", s=220, marker="x", lw=3, zorder=6)
        ax4.scatter([cx], [cy], [calt], c="k", s=140, marker="x")
        for axb in (ax5, ax6, ax7, ax8):
            axb.axvline(cx, color="k", lw=1.5, alpha=0.7)
    print(f"{len(coll_pts)} collision(s) on the shown segment")

    out = os.path.join(sess, "racing_line.png")
    plt.tight_layout()
    plt.savefig(out, dpi=90)
    print("wrote", out)


if __name__ == "__main__":
    main()
