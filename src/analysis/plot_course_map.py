"""
Plot the ace course map (gates + reconstructed source path) in three projections (x-y, x-z,
y-z), optionally overlaying a flown ace_dbg estimate track.

    python -m analysis.plot_course_map [datasets/ace_dbg_YYYYMMDD_HHMMSS.csv]

Writes datasets/course_map_view.png.
"""

import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    cmap = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "course_map.json")))
    gates = cmap["gates"]
    path = cmap.get("path", [])
    gx = [g["x"] for g in gates]
    gy = [g["y"] for g in gates]
    gz = [g["z"] for g in gates]
    px = [p[1] for p in path]
    py = [p[2] for p in path]
    pz = [p[3] for p in path]

    traj = None
    tfile = os.path.join(HERE, "pilots", "ace_pilot", "trajectory.json")
    if os.path.exists(tfile):
        t = json.load(open(tfile))
        traj = ([r[2] for r in t["samples"]], [r[3] for r in t["samples"]],
                [r[4] for r in t["samples"]])

    flown = None
    if len(sys.argv) > 1:
        rows = list(csv.DictReader(open(sys.argv[1])))
        flown = ([float(r["x"]) for r in rows], [float(r["y"]) for r in rows],
                 [float(r["z"]) for r in rows])

    fig, axes = plt.subplots(1, 3, figsize=(21, 7))
    views = [("x", "y", gx, gy, px, py, 0, 1), ("x", "z", gx, gz, px, pz, 0, 2),
             ("y", "z", gy, gz, py, pz, 1, 2)]
    for ax, (nx, ny, gxx, gyy, pxx, pyy, i0, i1) in zip(axes, views):
        ax.plot(pxx, pyy, "-", color="0.7", lw=1, label="steady flown (map source)")
        if traj:
            ax.plot(traj[i0], traj[i1], "-", color="tab:blue", lw=1.2, label="solved trajectory")
        if flown:
            ax.plot(flown[i0], flown[i1], "-", color="tab:red", lw=1.2, label="ace flown (estimate)")
        ax.scatter(gxx, gyy, c="tab:green", s=60, zorder=5, label="gates")
        for g, xx, yy in zip(gates, gxx, gyy):
            ax.annotate(str(g["gate_id"]), (xx, yy), fontsize=8,
                        textcoords="offset points", xytext=(4, 4))
        ax.set_xlabel(nx)
        ax.set_ylabel(ny)
        ax.set_title(f"{nx} vs {ny}")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="datalim")
    axes[0].legend(loc="best", fontsize=8)
    out = os.path.join(HERE, "datasets", "course_map_view.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main()
