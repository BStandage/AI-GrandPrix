"""Speed-colored plan render: top-down course + path colored by planned
speed, plus the v(s) profile with its pointwise ceiling. Eyeball tool only -
the planner never depends on it.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

from raceline import course as course_bridge  # noqa: E402


def _colored_line(ax, x, y, c, cmap, norm, lw=2.5):
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, norm=norm, linewidths=lw)
    lc.set_array(c[:-1])
    ax.add_collection(lc)
    return lc


def render(p, out_path, course=None):
    if course is None:
        course = course_bridge.load_course()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, (ax, axv) = plt.subplots(
        2, 1, figsize=(14, 12), gridspec_kw={"height_ratios": [3, 1]})

    norm = plt.Normalize(0.0, float(np.max(p.v)))
    lc = _colored_line(ax, p.pos[:, 0], p.pos[:, 1], p.v, "viridis", norm)
    fig.colorbar(lc, ax=ax, label="planned speed (m/s)", shrink=0.8)

    # gates: outer-frame bar along each body's yaw axis
    seen_xy = set()
    for g in course.gates:
        bx, by = math.cos(g.yaw_rad), math.sin(g.yaw_rad)
        h = 1.35
        ax.plot([g.x - h * bx, g.x + h * bx], [g.y - h * by, g.y + h * by],
                color="crimson", lw=3, alpha=0.8)
        key = (round(g.x, 1), round(g.y, 1))
        if key not in seen_xy:
            seen_xy.add(key)
            name = f"g{g.gate_order}"
            ax.annotate(name, (g.x, g.y), textcoords="offset points",
                        xytext=(6, 6), fontsize=9, color="crimson")
    for c in course.cones:
        ax.add_patch(plt.Circle((c.x, c.y), c.radius, color="orange",
                                alpha=0.6))

    # speed-profile minimum between first and last crossing
    s0, s1 = p.events[0]["s"], p.events[-1]["s"]
    idx = np.flatnonzero((p.s >= s0) & (p.s <= s1))
    imin = idx[np.argmin(p.v[idx])]
    ax.plot(p.pos[imin, 0], p.pos[imin, 1], "x", ms=12, mew=3, color="red")
    ax.annotate(f"v_min {p.v[imin]:.2f} m/s", p.pos[imin, :2],
                textcoords="offset points", xytext=(8, -12), color="red")
    ax.plot(0, 0, "k^", ms=10)
    ax.annotate("spawn", (0, 0), textcoords="offset points", xytext=(6, -12))

    ax.set_aspect("equal")
    ax.autoscale()
    ax.margins(0.05)
    ax.set_title(f"racing-line plan - {p.meta['path_length_m']:.0f} m, "
                 f"predicts {p.total_s:.1f} s (model prediction, unverified)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    axv.plot(p.s, p.v_lim, color="silver", lw=1, label="v_lim (ceiling)")
    axv.plot(p.s, p.v, color="tab:blue", lw=1.5, label="v (profile)")
    axv.plot(p.s[imin], p.v[imin], "rx", ms=8, mew=2)
    for e in p.events:
        axv.axvline(e["s"], color="crimson", lw=0.5, alpha=0.4)
        axv.annotate(e["label"], (e["s"], axv.get_ylim()[1]), fontsize=6,
                     rotation=90, va="top", color="crimson")
    axv.set_xlabel("arc length s (m)")
    axv.set_ylabel("m/s")
    axv.legend(loc="lower right", fontsize=8)
    axv.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path
