"""Debrief cam: what the drone THOUGHT, drawn on the plan it was flying, and
the two errors it can measure about itself with NO ground truth.

Built for the real airframe. There is no truth on the Archer - no motion
capture, no position sensor - so nothing here reads one, and the same tool
gives the same numbers on a sim trace and on a hardware log. (A sim trace
does carry truth columns; this tool ignores them on purpose. Tuning against
them is tuning against the sim's own noise model, which is how the per-gate
pose knobs in planner.py got searched to 29 s and then had to be switched off.)

The two measurements, both logged by the estimator itself:

1. THE OFFSET IT BELIEVED IT HAD AS IT WENT THROUGH A GATE (`cross_lat`, +
   left of centre looking along the crossing). Reality answers this one: the
   drone physically fitted through a 1.5 m opening, so the TRUE offset was
   inside +-0.75 m. A belief outside that band is estimator error proven
   without measuring anything - `proven_err` below. A belief inside it is not
   proof on its own, but the line is planned through the gate centre, so the
   same non-zero belief at the same gate run after run is a bias.

2. THE CAMERA-VERSUS-DEAD-RECKONING DISAGREEMENT AT EACH FIX (the innovation),
   split the way the fix itself is split:
       fix_cross  (+ left)    the camera says we are left of where DR thought
       fix_along  (+ nearer)  the camera says we are nearer the gate than DR thought
   Summed in the estimator and differenced here, so the leg mean is exact at
   any log rate. One sign everywhere is a calibration error, not drift: cross
   is the camera's boresight or the yaw reference, along is its range scale
   (--fy, or the gate width the range divides by).

Legs are cut by the estimator's own gate count, so "g4->g5" is exactly the
blind hairpin, and a leg with no fixes is flagged blind from the data.

    python -m raceline.debrief                      # newest log in out/flightlogs
    python -m raceline.debrief ../out/flightlogs/hw_follower_20260919_141233.csv
    python -m raceline.debrief --all                # every log
    python -m raceline.debrief --aggregate          # the table across runs + a PNG

Offline analysis only. Nothing in the flight path imports it.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import re
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from raceline import planner as plan_io  # noqa: E402
from raceline.config import AIGP_REPO  # noqa: E402

FLIGHTLOGS = AIGP_REPO / "out" / "flightlogs"
DEBRIEF_DIR = AIGP_REPO / "out" / "debrief"
DRIFT_LOG = DEBRIEF_DIR / "drift_log.csv"

# the gate opening: the drone got through it, so the true offset was inside
# this. The same half width the referee checks against (pq_course half_w).
OPENING_HALF_W = 0.75
OPENING_HALF_H = 0.75
# a bias worth acting on: bigger than the noise it sits in, and bigger than
# the 0.1 m the crossing fix pulls out for free. One or two runs prove nothing.
CONSISTENT_SD_RATIO = 2.0
CONSISTENT_MIN_M = 0.10
MIN_RUNS = 3

LOG_COLS = ["run", "trace", "plan", "leg", "leg_label", "lap", "n", "t0", "t1",
            "fixes", "rejects", "unmatched", "blind",
            "fix_cross", "fix_along", "cross_lat", "cross_dz", "proven_err"]


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def _f(row, key, default=np.nan):
    v = row.get(key, "")
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


_EV_RE = re.compile(r"ev(\d+)")


def _ev(row):
    """The gate count at this sample. The sim trace keeps it in `ev`; the
    hardware log writes it as the label `ev3/lm2`."""
    v = row.get("ev", None)
    if v is None:
        v = row.get("phase_or_event", "")
    if v in (None, ""):
        return -1
    try:
        return int(float(v))
    except ValueError:
        m = _EV_RE.search(str(v))
        return int(m.group(1)) if m else -1


def load_trace(path) -> dict:
    """A flight log into arrays. Sim trace or hardware log, same columns used;
    truth columns, where they exist, are not read."""
    path = Path(path)
    with open(path, newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.DictReader(fh) if r.get("t") not in (None, "")]
    if not rows:
        raise SystemExit(f"{path.name}: no rows")
    cols = set(rows[0].keys())
    return {
        "path": path,
        "name": path.stem,
        "t": np.array([_f(r, "t") for r in rows]),
        "p": np.array([[_f(r, "x"), _f(r, "y"), _f(r, "z")] for r in rows]),
        "ev": np.array([_ev(r) for r in rows], dtype=int),
        "fixes": np.array([_f(r, "fixes", 0.0) for r in rows]),
        "rejects": np.array([_f(r, "rej", 0.0) for r in rows]),
        "unmatched": np.array([_f(r, "unm", 0.0) for r in rows]),
        "cross_ev": np.array([_f(r, "cross_ev", -1.0) for r in rows]),
        "cross_lat": np.array([_f(r, "cross_lat") for r in rows]),
        "cross_dz": np.array([_f(r, "cross_dz") for r in rows]),
        "cx_sum": np.array([_f(r, "fix_cx_sum") for r in rows]),
        "al_sum": np.array([_f(r, "fix_al_sum") for r in rows]),
        # a log written before the estimator kept these: path only, no numbers
        "has_debrief_cols": {"cross_ev", "fix_cx_sum"} <= cols,
    }


def _plan_for(trace_path: Path, explicit=None):
    """The plan the run flew. Explicit wins; otherwise the newest plan JSON
    not newer than the log."""
    if explicit:
        return Path(explicit)
    cands = [Path(p) for p in glob.glob(str(AIGP_REPO / "out" / "plans" / "*.json"))]
    cands += [Path(p) for p in glob.glob(str(AIGP_REPO / "config" / "ladder" / "plans" / "*.json"))]
    if not cands:
        return None
    t_trace = trace_path.stat().st_mtime
    older = [c for c in cands if c.stat().st_mtime <= t_trace]
    return max(older or cands, key=lambda c: c.stat().st_mtime)


# ---------------------------------------------------------------------------
# the two measurements
# ---------------------------------------------------------------------------

def crossings(trace) -> dict:
    """The offset the drone believed it had at each counted crossing, one per
    event. The logs sample slower than the flight loop, but `cross_ev` is
    sticky, so a crossing is picked up on the next logged row whatever the
    rate; deduping on the event index is what makes that safe."""
    out = {}
    if not trace["has_debrief_cols"]:
        return out
    for i, ev in enumerate(trace["cross_ev"]):
        if not np.isfinite(ev) or ev < 0 or int(ev) in out:
            continue
        lat, dz = trace["cross_lat"][i], trace["cross_dz"][i]
        if not (np.isfinite(lat) and np.isfinite(dz)):
            continue
        out[int(ev)] = {
            "lat": float(lat), "dz": float(dz), "t": float(trace["t"][i]),
            # what is proven with no truth: the drone got through a 1.5 m
            # opening, so anything it believed beyond the half width is error
            "proven_err": round(max(0.0, abs(float(lat)) - OPENING_HALF_W), 3),
            "inside": abs(float(lat)) <= OPENING_HALF_W and abs(float(dz)) <= OPENING_HALF_H,
        }
    return out


def _innovation(trace, i0, i1):
    """Mean camera-vs-dead-reckoning disagreement over rows [i0, i1]. Exact at
    any log rate: the estimator sums it, we difference the sums."""
    base = max(0, i0 - 1)
    dn = trace["fixes"][i1] - trace["fixes"][base]
    if not np.isfinite(dn) or dn <= 0:
        return (np.nan, np.nan, 0)
    dcx = trace["cx_sum"][i1] - trace["cx_sum"][base]
    dal = trace["al_sum"][i1] - trace["al_sum"][base]
    if not (np.isfinite(dcx) and np.isfinite(dal)):
        return (np.nan, np.nan, int(dn))
    return (float(dcx / dn), float(dal / dn), int(dn))


def leg_rows(trace, plan) -> list[dict]:
    """One row per leg flown: what the camera and dead reckoning disagreed
    about along it, and the offset it believed it had at the gate that ends
    it."""
    labels = [e["label"] for e in plan["events"]]
    laps = [e.get("lap", 0) for e in plan["events"]]
    cross = crossings(trace)
    run = f"{trace['name']}@{int(trace['path'].stat().st_mtime)}"
    out = []
    for k in sorted(set(int(v) for v in trace["ev"])):
        rows = np.flatnonzero(trace["ev"] == k)
        if rows.size < 2 or not (0 <= k < len(labels)):
            continue
        i0, i1 = int(rows[0]), int(rows[-1])
        fix_cross, fix_along, n_fix = _innovation(trace, i0, i1)
        c = cross.get(k, {})
        out.append({
            "run": run, "trace": trace["name"],
            "plan": Path(plan.get("_path", "?")).name,
            "leg": k, "leg_label": f"{'start' if k == 0 else labels[k - 1]}->{labels[k]}",
            "lap": int(laps[k]) + 1, "n": int(rows.size),
            "t0": round(float(trace["t"][i0]), 2), "t1": round(float(trace["t"][i1]), 2),
            "fixes": n_fix,
            "rejects": int(trace["rejects"][i1] - trace["rejects"][max(0, i0 - 1)]),
            "unmatched": int(trace["unmatched"][i1] - trace["unmatched"][max(0, i0 - 1)]),
            "blind": int(n_fix == 0),
            "fix_cross": None if not np.isfinite(fix_cross) else round(fix_cross, 3),
            "fix_along": None if not np.isfinite(fix_along) else round(fix_along, 3),
            "cross_lat": c.get("lat"),
            "cross_dz": c.get("dz"),
            "proven_err": c.get("proven_err"),
        })
    return out


# ---------------------------------------------------------------------------
# the per-run picture
# ---------------------------------------------------------------------------

def _draw_gates(ax, plan):
    seen = set()
    for e in plan["events"]:
        h = float(e.get("heading_rad", 0.0)) + math.pi / 2.0
        bx, by = math.cos(h), math.sin(h)
        ax.plot([e["x"] - OPENING_HALF_W * bx, e["x"] + OPENING_HALF_W * bx],
                [e["y"] - OPENING_HALF_W * by, e["y"] + OPENING_HALF_W * by],
                color="crimson", lw=3, alpha=0.75, zorder=3)
        key = (round(e["x"], 1), round(e["y"], 1))
        if key not in seen:
            seen.add(key)
            ax.annotate(e["label"], (e["x"], e["y"]), textcoords="offset points",
                        xytext=(6, 6), fontsize=8, color="crimson")


def render(trace, plan, legs, out_path) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    have = [lg for lg in legs if lg["cross_lat"] is not None]
    panels = bool(have) or any(lg["fix_cross"] is not None for lg in legs)
    if panels:
        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1.0])
        ax = fig.add_subplot(gs[:, 0])
        axi = fig.add_subplot(gs[0, 1])
        axc = fig.add_subplot(gs[1, 1])
    else:
        fig, ax = plt.subplots(figsize=(9, 11))

    ax.plot(plan["pos"][:, 0], plan["pos"][:, 1], color="silver", lw=2.0,
            label="plan", zorder=1)
    ax.plot(trace["p"][:, 0], trace["p"][:, 1], color="tab:blue", lw=1.4,
            label="where it thought it was", zorder=5)
    _draw_gates(ax, plan)
    jump = np.flatnonzero(np.diff(trace["fixes"]) > 0) + 1
    if jump.size:                      # under the estimate: the eyes-open stretches
        ax.plot(trace["p"][jump, 0], trace["p"][jump, 1], ".", ms=2.5,
                color="darkorange", alpha=0.35, zorder=2,
                label=f"camera fixes ({int(trace['fixes'][-1])})")
    for lg in legs:
        sel = np.flatnonzero(trace["ev"] == lg["leg"])
        if lg["blind"] and sel.size:
            mid = sel[len(sel) // 2]
            ax.annotate(f"blind {lg['leg_label']}", trace["p"][mid, :2],
                        textcoords="offset points", xytext=(8, -14),
                        fontsize=7, color="dimgray")
        if lg["cross_lat"] is not None and abs(lg["cross_lat"]) > OPENING_HALF_W:
            e = plan["events"][lg["leg"]]
            ax.plot(e["x"], e["y"], "o", ms=9, mfc="none", mew=2,
                    color="tab:red", zorder=7)
    ax.set_aspect("equal")
    ax.margins(0.05)
    ax.grid(alpha=0.2)
    ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("circled = it believed itself outside an opening it flew through",
                 fontsize=9)

    fig.suptitle(
        f"debrief {trace['name']} on {Path(plan.get('_path', '?')).name} - "
        f"{int(trace['fixes'][-1])} fixes, {int(trace['rejects'][-1])} rejected, "
        f"{len(have)} crossings, no ground truth used", fontsize=11)

    if panels:
        x = np.arange(len(legs))
        axi.bar(x - 0.2, [lg["fix_cross"] or 0.0 for lg in legs], 0.4,
                color="tab:purple", label="camera says left of DR (+)")
        axi.bar(x + 0.2, [lg["fix_along"] or 0.0 for lg in legs], 0.4,
                color="tab:olive", label="camera says nearer than DR (+)")
        axi.axhline(0.0, color="k", lw=0.8)
        axi.set_ylabel("m")
        axi.set_title("fix innovation per leg: one sign everywhere is calibration, "
                      "not drift", fontsize=9)
        lat = [lg["cross_lat"] if lg["cross_lat"] is not None else np.nan for lg in legs]
        axc.bar(x, lat, 0.6, color=["tab:red" if np.isfinite(v) and abs(v) > OPENING_HALF_W
                                    else "tab:blue" for v in lat])
        for s in (-OPENING_HALF_W, OPENING_HALF_W):
            axc.axhline(s, color="crimson", ls="--", lw=1.0)
        axc.axhline(0.0, color="k", lw=0.8)
        axc.set_ylabel("m")
        axc.set_title("offset it believed it had at each gate; outside the dashed "
                      "opening is proven error", fontsize=9)
        for a in (axi, axc):
            for i, lg in enumerate(legs):
                if lg["blind"]:
                    a.axvspan(i - 0.5, i + 0.5, color="gray", alpha=0.12)
            a.set_xticks(x)
            a.set_xticklabels([lg["leg_label"] for lg in legs], rotation=60,
                              ha="right", fontsize=6)
            a.grid(alpha=0.3, axis="y")
        axi.legend(fontsize=7)        # axc is explained by its title and the band

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# the log across runs
# ---------------------------------------------------------------------------

def append_log(rows, log_path=DRIFT_LOG, force=False) -> int:
    """Append leg rows, skipping a run already logged, so the tool is safe to
    re-run over a whole flightlogs directory."""
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    seen = set()
    if log_path.exists():
        with open(log_path, newline="", encoding="utf-8") as fh:
            seen = {r["run"] for r in csv.DictReader(fh)}
    rows = [r for r in rows if force or r["run"] not in seen]
    if not rows:
        return 0
    new = not log_path.exists()
    with open(log_path, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=LOG_COLS)
        if new:
            w.writeheader()
        for r in rows:
            w.writerow({k: ("" if r.get(k) is None else r.get(k)) for k in LOG_COLS})
    return len(rows)


def _stat(vals):
    v = np.array([x for x in vals if x is not None and np.isfinite(x)], dtype=float)
    if v.size == 0:
        return (np.nan, np.nan, 0)
    return (float(np.mean(v)), float(np.std(v, ddof=1)) if v.size > 1 else np.nan, int(v.size))


def _consistent(mean, sd, n):
    """Same way every time, big enough to chase, and over enough runs."""
    if n < MIN_RUNS or not np.isfinite(sd) or not np.isfinite(mean):
        return False
    return abs(mean) >= CONSISTENT_MIN_M and abs(mean) >= CONSISTENT_SD_RATIO * sd


def aggregate(log_path=DRIFT_LOG, out_png=None):
    """Per leg, across every logged run: does it believe the same wrong thing
    every time?"""
    log_path = Path(log_path)
    if not log_path.exists():
        raise SystemExit(f"no drift log at {log_path}: debrief some flights first")
    with open(log_path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    legs = {}
    for r in rows:
        legs.setdefault((int(r["leg"]), r["leg_label"]), []).append(r)

    def col(rs, k):
        return [None if r[k] in ("", None) else float(r[k]) for r in rs]

    out = []
    for (k, label), rs in sorted(legs.items()):
        lat, lat_sd, n_lat = _stat(col(rs, "cross_lat"))
        cx, cx_sd, _ = _stat(col(rs, "fix_cross"))
        al, al_sd, _ = _stat(col(rs, "fix_along"))
        proven, _, _ = _stat(col(rs, "proven_err"))
        out.append({
            "leg": k, "leg_label": label, "lap": int(rs[0].get("lap", 1) or 1),
            "runs": len(rs), "n_cross": n_lat,
            "cross_lat": lat, "cross_lat_sd": lat_sd,
            "fix_cross": cx, "fix_cross_sd": cx_sd,
            "fix_along": al, "fix_along_sd": al_sd,
            "proven_err": proven,
            "blind": sum(int(r["blind"]) for r in rs) == len(rs),
            "consistent": _consistent(lat, lat_sd, n_lat),
        })
    if out_png:
        _aggregate_png(out, out_png)
    return out


def calibration(summary) -> dict:
    """The whole-course means. A fix innovation with the same sign at every
    gate is not drift, it is the camera being wrong about where it points
    (cross) or how far things are (along)."""
    cx, cx_sd, n = _stat([s["fix_cross"] for s in summary])
    al, al_sd, _ = _stat([s["fix_along"] for s in summary])
    return {"fix_cross": cx, "fix_cross_sd": cx_sd, "fix_along": al,
            "fix_along_sd": al_sd, "legs": n}


def _aggregate_png(summary, out_png):
    out_png = Path(out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    labels = [s["leg_label"] for s in summary]
    x = np.arange(len(labels))
    fig, (ax, axi) = plt.subplots(2, 1, figsize=(max(10, 0.55 * len(labels)), 9))
    ax.bar(x, [s["cross_lat"] for s in summary], 0.6,
           yerr=[0.0 if not np.isfinite(s["cross_lat_sd"]) else s["cross_lat_sd"] for s in summary],
           capsize=2, color="tab:blue")
    for s in (-OPENING_HALF_W, OPENING_HALF_W):
        ax.axhline(s, color="crimson", ls="--", lw=1.0)
    for i, s in enumerate(summary):
        if not s["consistent"]:
            continue
        v = s["cross_lat"]                     # label above the bar, outside it
        sd = 0.0 if not np.isfinite(s["cross_lat_sd"]) else s["cross_lat_sd"]
        ax.annotate("same way\nevery run", (i, v + (sd if v >= 0 else -sd)),
                    textcoords="offset points", xytext=(0, 6 if v >= 0 else -20),
                    ha="center", fontsize=7, color="tab:red", fontweight="bold")
    ax.axhline(0.0, color="k", lw=0.8)
    ax.margins(y=0.25)              # headroom for the "same way every run" labels
    ax.set_ylabel("m")
    n = max((s["runs"] for s in summary), default=0)
    ax.set_title(f"offset it believed it had at each gate, {n} runs "
                 "(dashed = the opening it actually flew through)")
    axi.bar(x - 0.2, [s["fix_cross"] for s in summary], 0.4, color="tab:purple",
            yerr=[0.0 if not np.isfinite(s["fix_cross_sd"]) else s["fix_cross_sd"] for s in summary],
            capsize=2, label="camera says left of DR (+)")
    axi.bar(x + 0.2, [s["fix_along"] for s in summary], 0.4, color="tab:olive",
            yerr=[0.0 if not np.isfinite(s["fix_along_sd"]) else s["fix_along_sd"] for s in summary],
            capsize=2, label="camera says nearer than DR (+)")
    axi.axhline(0.0, color="k", lw=0.8)
    axi.set_ylabel("m")
    axi.set_title("fix innovation per leg: one sign everywhere is calibration")
    axi.legend(fontsize=8)
    for a in (ax, axi):
        for i, s in enumerate(summary):
            if s["blind"]:
                a.axvspan(i - 0.5, i + 0.5, color="gray", alpha=0.12)
        a.set_xticks(x)
        a.set_xticklabels(labels, rotation=60, ha="right", fontsize=8)
        a.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return out_png


def _fmt(mean, sd):
    if not np.isfinite(mean):
        return f"{'-':>18}"
    return f"{mean:>+8.2f}" + (f" +-{sd:.2f}" if np.isfinite(sd) else " " * 8)


def print_aggregate(summary) -> str:
    """The table, in flight order, with no ground truth anywhere in it."""
    w = max([len(s["leg_label"]) for s in summary] + [9])
    lines = [f"{'leg'.ljust(w)}  lap  runs  {'believed offset':>18}  "
             f"{'cam-DR cross':>18}  {'cam-DR along':>18}  flag",
             "-" * (w + 76)]
    for s in summary:
        flag = "CONSISTENT" if s["consistent"] else ""
        if np.isfinite(s["proven_err"]) and s["proven_err"] > 0.01:
            flag = (f"PROVEN >={s['proven_err']:.2f}m " + flag).strip()
        if s["blind"]:
            flag = ("blind " + flag).strip()
        lines.append(f"{s['leg_label'].ljust(w)}  {s['lap']:>3}  {s['runs']:>4}  "
                     f"{_fmt(s['cross_lat'], s['cross_lat_sd'])}  "
                     f"{_fmt(s['fix_cross'], s['fix_cross_sd'])}  "
                     f"{_fmt(s['fix_along'], s['fix_along_sd'])}  {flag}")

    cal = calibration(summary)
    lines += ["", "Whole course, every leg pooled (a calibration error shows here, "
                  "drift does not):"]
    lines.append(f"  camera vs dead reckoning, across the line of sight: {cal['fix_cross']:+.3f} m"
                 + (f" +-{cal['fix_cross_sd']:.3f}" if np.isfinite(cal["fix_cross_sd"]) else ""))
    lines.append(f"  camera vs dead reckoning, along the line of sight:  {cal['fix_along']:+.3f} m"
                 + (f" +-{cal['fix_along_sd']:.3f}" if np.isfinite(cal["fix_along_sd"]) else ""))
    if np.isfinite(cal["fix_cross"]) and abs(cal["fix_cross"]) >= CONSISTENT_MIN_M:
        lines.append(f"  -> the camera puts the drone {abs(cal['fix_cross']):.2f} m to the "
                     f"{'left' if cal['fix_cross'] > 0 else 'right'} of dead reckoning at EVERY "
                     "gate. That is the boresight or the yaw reference (camcal --cam-tilt, "
                     "--map-north), not drift.")
    if np.isfinite(cal["fix_along"]) and abs(cal["fix_along"]) >= CONSISTENT_MIN_M:
        lines.append(f"  -> the camera reads {'short' if cal['fix_along'] > 0 else 'long'} against "
                     f"dead reckoning at every gate by {abs(cal['fix_along']):.2f} m. That is the "
                     "range scale: re-run camcal for --fy, and check the gate width the range "
                     "divides by.")

    hot = [s for s in summary if s["consistent"]]
    proven = [s for s in summary if np.isfinite(s["proven_err"]) and s["proven_err"] > 0.01]
    lines.append("")
    if proven:
        lines.append("Proven wrong with no truth needed - it believed it was outside an "
                     "opening it flew through:")
        for s in proven:
            lines.append(f"  lap {s['lap']} {s['leg_label']}: at least {s['proven_err']:.2f} m of "
                         f"position error at that gate, over {s['runs']} runs")
    if hot:
        lines.append("Same way every run, so it is a bias and not noise:")
        for s in hot:
            lines.append(f"  lap {s['lap']} {s['leg_label']}: it believes it crosses "
                         f"{abs(s['cross_lat']):.2f} m to the "
                         f"{'left' if s['cross_lat'] > 0 else 'right'} of centre, "
                         f"{s['runs']} runs, sd {s['cross_lat_sd']:.2f}")
        lines.append("  The line is planned through the gate centre, so a repeatable offset is")
        lines.append("  the estimate, the map, or the line - in that order of suspicion.")
    if not hot and not proven:
        lines.append(f"Nothing to chase yet: no leg is off the same way over {MIN_RUNS}+ runs "
                     f"by more than {CONSISTENT_MIN_M} m, and nothing is outside an opening.")
    return "\n".join(lines)


# ---------------------------------------------------------------------------

def debrief_one(trace_path, plan_path=None, out_dir=DEBRIEF_DIR,
                log_path=DRIFT_LOG, force=False) -> dict:
    """One flight: the PNG, and the leg rows appended to the drift log."""
    trace = load_trace(trace_path)
    pp = _plan_for(trace["path"], plan_path)
    if pp is None or not Path(pp).is_file():
        raise SystemExit("no plan found: pass --plan")
    plan = plan_io.load_plan(pp)
    plan["_path"] = str(pp)
    legs = leg_rows(trace, plan)
    png = render(trace, plan, legs, Path(out_dir) / f"debrief_{trace['name']}.png")
    added = append_log(legs, log_path, force) if legs else 0
    return {"png": png, "legs": legs, "added": added, "plan": pp,
            "has_debrief_cols": trace["has_debrief_cols"]}


def _logs_in(d):
    return sorted(glob.glob(str(Path(d) / "dr_*.csv")) + glob.glob(str(Path(d) / "hw_*.csv")),
                  key=os.path.getmtime)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trace", nargs="?", help="a flight log (default: the newest in out/flightlogs)")
    ap.add_argument("--plan", help="the plan it flew (default: the newest plan not newer than the log)")
    ap.add_argument("--all", action="store_true", help="every log in out/flightlogs")
    ap.add_argument("--aggregate", action="store_true", help="the table across logged runs")
    ap.add_argument("--out-dir", default=str(DEBRIEF_DIR))
    ap.add_argument("--log", default=str(DRIFT_LOG), help="the drift log to append to / read")
    ap.add_argument("--force", action="store_true", help="log a run again even if it is already in")
    args = ap.parse_args(argv)

    if args.aggregate:
        png = Path(args.out_dir) / "drift_by_leg.png"
        print(print_aggregate(aggregate(args.log, png)))
        print(f"\n{png}")
        return 0

    if args.all:
        traces = _logs_in(FLIGHTLOGS)
    elif args.trace:
        traces = [args.trace]
    else:
        traces = _logs_in(FLIGHTLOGS)[-1:]
    if not traces:
        raise SystemExit(f"no flight logs in {FLIGHTLOGS}")

    for t in traces:
        r = debrief_one(t, args.plan, args.out_dir, args.log, args.force)
        worst = max((lg for lg in r["legs"] if lg["cross_lat"] is not None),
                    key=lambda x: abs(x["cross_lat"]), default=None)
        print(f"{Path(t).name} -> {r['png']}"
              + (f"; {r['added']} legs logged" if r["added"] else "")
              + (f"; widest believed offset {worst['cross_lat']:+.2f} m at {worst['leg_label']}"
                 if worst else "")
              + ("" if r["has_debrief_cols"] else
                 "; NOTE this log predates the debrief columns, so it is the path only"))
    if len(traces) > 1:
        print("\n" + print_aggregate(aggregate(args.log)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
