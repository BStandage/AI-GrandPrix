"""
Parse the sysid tab CSVs into an engineering report (SYSID_REPORT.md).

Pure post-processing - no sim needed - so it is the smoke-testable core of the campaign:
    python sysid_report.py [sysid_dir]      # defaults to the latest datasets/sysid_* dir

It emits the master envelope summary plus the three blueprint deliverables:
  1. the MPC No-Go-Zone boundary  dz_recovery(vz)  regressed from the recovery grid,
  2. the maneuver-efficiency audit (continuous vs zero-thrust-snap),
  3. actuator-saturation warnings (a motor pinned > 400 ms),
and the kinematic feasibility cone (flown vs analytically derived).

All "force" quantities are accelerations (m/s^2): the sim exposes no vehicle mass, so absolute
Newtons are not recoverable - documented in the report itself.
"""

import csv
import math
import os
import sys
import time

import numpy as np

# Allow running this file directly (python flight_test/sysid_report.py) by putting
# the src/ package root on sys.path, not just this subpackage's folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common import dynamics
from common.paths import DATASETS_DIR

SAT_WARN_MS = 400.0


# ---- loading -------------------------------------------------------------------------------
def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, key):
    """Numeric column as a list, dropping blanks/non-numeric."""
    out = []
    for r in rows:
        v = _num(r.get(key))
        if v is not None:
            out.append(v)
    return out


# ---- per-tab analysis ----------------------------------------------------------------------
def analyze_rotational(rows):
    """Per axis: omega_max, alpha_max, command->response lag, cross-axis coupling."""
    out = {}
    # Only the 'step' segment is the real measurement; the 'recover' (re-level) segment reverses
    # the rate and would inflate alpha and pollute omega_max / coupling.
    rows = [r for r in rows if r.get("seg") != "recover"]
    for axis in ("roll", "pitch", "yaw"):
        ar = [r for r in rows if r.get("axis") == axis]
        if not ar:
            continue
        omega = [abs(v) for v in col(ar, "measured_omega")]
        alpha = [abs(v) for v in col(ar, "alpha")]
        # coupling: cross-axis drift relative to the achieved primary rate, on the strongest cmd
        cmds = sorted({_num(r.get("commanded_rate")) for r in ar if _num(r.get("commanded_rate"))})
        coupling = None
        lag = None
        if cmds:
            top = max(cmds)
            top_rows = [r for r in ar if _num(r.get("commanded_rate")) == top
                        and r.get("seg") == "step"]
            drift = [abs(v) for v in col(top_rows, "cross_axis_drift")]
            prim = [abs(v) for v in col(top_rows, "measured_omega")]
            if drift and prim and max(prim) > 0.1:
                coupling = max(drift) / max(prim)
            lag = _lag(top_rows)
        out[axis] = {
            "omega_max": max(omega) if omega else None,
            "alpha_max": max(alpha) if alpha else None,
            "coupling": coupling,
            "lag_ms": lag,
        }
    return out


def _lag(step_rows):
    """Time (ms) to reach 63% of the steady achieved rate after the step onset."""
    if len(step_rows) < 3:
        return None
    t = col(step_rows, "t")
    w = [abs(v) for v in col(step_rows, "measured_omega")]
    if len(t) != len(w) or not w:
        return None
    steady = np.mean(w[len(w) // 2:])      # back half = steady state
    if steady <= 0:
        return None
    target = 0.63 * steady
    t0 = t[0]
    for ti, wi in zip(t, w):
        if wi >= target:
            return (ti - t0) * 1000.0
    return None


def analyze_drag(rows):
    """Quadratic drag fit a = k*v^2 on the coast phase, plus terminal velocities seen."""
    coast = [r for r in rows if r.get("phase") == "coast"]
    v = np.array([x for x in col(coast, "vh")])
    a = np.array([x for x in (col(coast, "drag_accel"))])
    k = None
    if len(v) >= 3:
        mask = v > 1.0
        v, a = v[mask], a[mask]
        if len(v) >= 3 and np.sum(v ** 4) > 0:
            k = float(np.sum(a * v ** 2) / np.sum(v ** 4))   # least squares through origin
    fwd = [r for r in rows if r.get("trial") == "drag_fwd"]
    lat = [r for r in rows if r.get("trial") == "drag_lat"]
    dive = [r for r in rows if "dive" in (r.get("trial") or "")]
    term_fwd = max(col(fwd, "vh"), default=None)
    term_lat = max(col(lat, "vh"), default=None)
    # inverted-dive terminal vz = steady tail of the world-frame downward speed
    dive_vz = col(dive, "vz_w")
    term_dive = float(np.mean(dive_vz[-int(len(dive_vz) * 0.3):])) if len(dive_vz) > 5 else None
    return {"drag_k": k, "term_fwd": term_fwd, "term_lat": term_lat, "term_dive_vz": term_dive}


def analyze_recovery(rows):
    """No-Go-Zone fit per strategy + efficiency audit + saturation warnings."""
    gate_failed = any(r.get("trial") == "GATE_FAILED" for r in rows)
    grid = [r for r in rows if r.get("trial") not in (None, "GATE_FAILED")]
    fits = {}
    for strat in ("continuous", "snap"):
        sr = [r for r in grid if r.get("strategy") == strat and r.get("arrested") in ("True", "1")]
        vz = np.array(col(sr, "entry_vz_actual"))
        dz = np.array(col(sr, "delta_z_loss"))
        n = min(len(vz), len(dz))
        if n >= 3:
            vz, dz = vz[:n], dz[:n]
            deg = 2 if n >= 4 else 1
            coeffs = np.polyfit(vz, dz, deg).tolist()
            fits[strat] = {"coeffs": coeffs, "deg": deg, "n": n}
    warnings = [r for r in grid if (_num(r.get("max_actuator_saturation_ms")) or 0) > SAT_WARN_MS]
    return {"gate_failed": gate_failed, "fits": fits, "grid": grid, "warnings": warnings}


def analyze_feasibility(rows, drag_k):
    """Flown lateral offsets + the analytically derived cone from the measured limits."""
    flown = []
    for r in rows:
        v = _num(r.get("entry_speed_actual")) or _num(r.get("entry_speed_target"))
        off = _num(r.get("lateral_offset"))
        if v is not None and off is not None:
            flown.append((v, off))
    # derived: max lateral accel from the max strafe bank; offset over 15 m from rest-lateral
    a_lat = dynamics.G_ACC * math.tan(0.5)        # 0.5 rad bank, matches the flown test
    derived = []
    for v, _ in flown or [(4.0, 0), (7.0, 0), (9.0, 0)]:
        if v > 0:
            tt = 15.0 / v
            derived.append((v, 0.5 * a_lat * tt ** 2))
    return {"flown": flown, "derived": derived, "a_lat": a_lat}


# ---- rendering -----------------------------------------------------------------------------
def _fmt(x, nd=2):
    return f"{x:.{nd}f}" if isinstance(x, (int, float)) else "n/a"


def _poly_str(coeffs):
    deg = len(coeffs) - 1
    parts = []
    for i, c in enumerate(coeffs):
        p = deg - i
        mag = (f"{abs(c):.4f}·vz²" if p == 2 else f"{abs(c):.3f}·vz" if p == 1 else f"{abs(c):.3f}")
        if i == 0:
            parts.append(("−" if c < 0 else "") + mag)
        else:
            parts.append((" − " if c < 0 else " + ") + mag)
    return "".join(parts)


def _cell(r, key, nd=2):
    """A table cell: numeric rounded to nd places, else the raw string (or '?')."""
    v = _num(r.get(key))
    return f"{v:.{nd}f}" if v is not None else (r.get(key) or "?")


def _render(rot_rows, drag_rows, rec_rows, feas_rows, source_label, out_path, provenance=None):
    rot = analyze_rotational(rot_rows)
    drag = analyze_drag(drag_rows)
    rec = analyze_recovery(rec_rows)
    feas = analyze_feasibility(feas_rows, drag.get("drag_k"))

    L = []
    L.append("# Flight Dynamics System-Identification Report")
    L.append("")
    L.append(f"Source: {source_label}")
    if provenance:
        L.append("")
        L.append("Per-tab provenance (best clean run chosen for each, since long sessions degrade):")
        for line in provenance:
            L.append(f"- {line}")
    L.append("")
    L.append("> All force quantities are **accelerations (m/s²)**. The sim exposes no vehicle "
             "mass, so absolute Newtons / drag coefficients are not recoverable; the controller "
             "only needs accelerations regardless. Motor columns are **observed normalized "
             "outputs** (saturation proxy), not commanded RPM — the interface commands body "
             "rates + collective thrust, not the motor mixer.")
    L.append("")

    # ---- master summary ----
    L.append("## Master envelope summary")
    L.append("")
    L.append("| Quantity | Measured | dynamics.py (current) |")
    L.append("|---|---|---|")
    L.append(f"| Hover thrust | (see drag/vertical runs) | {dynamics.HOVER_THRUST:.3f} |")
    for axis in ("roll", "pitch", "yaw"):
        a = rot.get(axis, {})
        L.append(f"| {axis} ω_max (rad/s) | {_fmt(a.get('omega_max'))} | "
                 f"MAX_RATE={dynamics.MAX_RATE:.1f} (clamp) |")
    for axis in ("roll", "pitch", "yaw"):
        a = rot.get(axis, {})
        L.append(f"| {axis} α_max (rad/s²) | {_fmt(a.get('alpha_max'), 1)} | — |")
    L.append(f"| Terminal fwd speed (m/s) | {_fmt(drag.get('term_fwd'))} | "
             f"SPEED_LEAN_TABLE top {dynamics.SPEED_LEAN_TABLE[-1][0]:.1f} |")
    L.append(f"| Terminal lateral speed (m/s) | {_fmt(drag.get('term_lat'))} | — |")
    L.append(f"| Inverted-dive terminal vz (m/s, +down) | {_fmt(drag.get('term_dive_vz'))} | "
             f"free-fall ≈ {abs(dynamics.THRUST_CLIMB_TABLE[0][1]):.1f} |")
    L.append(f"| Quadratic drag k (a=k·v², 1/m) | {_fmt(drag.get('drag_k'), 4)} | — |")
    L.append("")

    # ---- rotational detail ----
    L.append("## 1. Rotational dynamics")
    L.append("")
    L.append("| Axis | ω_max (rad/s) | α_max (rad/s²) | Lag to 63% (ms) | Cross-axis coupling |")
    L.append("|---|---|---|---|---|")
    for axis in ("roll", "pitch", "yaw"):
        a = rot.get(axis)
        if not a:
            L.append(f"| {axis} | — | — | — | — |")
            continue
        L.append(f"| {axis} | {_fmt(a['omega_max'])} | {_fmt(a['alpha_max'], 1)} | "
                 f"{_fmt(a['lag_ms'], 0)} | {_fmt(a['coupling'])} |")
    L.append("")

    # ---- No-Go-Zone ----
    L.append("## 2. MPC No-Go-Zone (minimum recovery airspace)")
    L.append("")
    L.append("Constraint for the planner:  `Z_drone − Z_floor > Δz_recovery(vz)`  — the altitude "
             "that must be reserved to arrest a descent of downward speed `vz`.")
    L.append("")
    if rec["gate_failed"]:
        L.append("**Tier-4 gate FAILED: the sim did not support inverted flight.** The recovery "
                 "grid was skipped; no Δz_recovery curve could be measured. Aerobatic Split-S "
                 "recoveries are not available on this build — the planner must stay upright.")
    elif not rec["fits"]:
        L.append("_No arrested recovery trials with enough points to fit. Run `--recovery` against "
                 "a live sim, or check that trials are arresting before the floor._")
    else:
        for strat, fit in rec["fits"].items():
            L.append(f"- **{strat}**: `Δz_recovery = {_poly_str(fit['coeffs'])}`  (n={fit['n']})")
    L.append("")

    # ---- efficiency audit ----
    L.append("## 3. Maneuver efficiency audit (continuous vs zero-thrust snap)")
    L.append("")
    if rec["grid"]:
        L.append("| entry_vz | strategy | axis | Δz_loss (m) | horizon-cross (s) | arrested |")
        L.append("|---|---|---|---|---|---|")
        for r in rec["grid"]:
            L.append(f"| {_cell(r, 'entry_vz_actual', 1)} | {r.get('strategy','?')} | "
                     f"{r.get('axis','?')} | {_cell(r, 'delta_z_loss', 2)} | "
                     f"{_cell(r, 'horizon_crossing_time', 2)} | {r.get('arrested','?')} |")
    else:
        L.append("_No recovery grid data._")
    L.append("")

    # ---- saturation warnings ----
    L.append("## 4. Actuator saturation warnings (motor pinned > 400 ms)")
    L.append("")
    if rec["warnings"]:
        for r in rec["warnings"]:
            L.append(f"- `{r.get('trial')}` — {r.get('max_actuator_saturation_ms')} ms at saturation "
                     f"(structural loss of attitude authority).")
    else:
        L.append("None flagged (or no recovery data).")
    L.append("")

    # ---- feasibility cone ----
    L.append("## 5. Kinematic feasibility cone (15 m gate spacing)")
    L.append("")
    L.append(f"Max lateral acceleration from a {math.degrees(0.5):.0f}° bank: "
             f"**{feas['a_lat']:.1f} m/s²**.")
    L.append("")
    L.append("| Entry speed (m/s) | Lateral offset, flown (m) | Lateral offset, derived (m) |")
    L.append("|---|---|---|")
    flown_map = {round(v, 1): o for v, o in feas["flown"]}
    for v, d in feas["derived"]:
        L.append(f"| {v:.1f} | {_fmt(flown_map.get(round(v, 1)))} | {d:.2f} |")
    L.append("")
    L.append("> NB: this airframe tops out ≈ 9 m/s (SPEED_LEAN_TABLE), far below the blueprint's "
             "15–35 m/s assumption. The cone is mapped over realistic speeds; high-speed gate "
             "displacement that would force an aerobatic flip does not arise at these speeds.")
    L.append("")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    print(f"  wrote {out_path}", flush=True)
    return out_path


def generate(out_dir):
    """Report from a single run directory."""
    return _render(load(os.path.join(out_dir, "tab1_rotational.csv")),
                   load(os.path.join(out_dir, "tab2_drag.csv")),
                   load(os.path.join(out_dir, "tab3_recovery.csv")),
                   load(os.path.join(out_dir, "tab4_feasibility.csv")),
                   f"`{os.path.basename(out_dir)}`",
                   os.path.join(out_dir, "SYSID_REPORT.md"))


TABS = ["tab1_rotational", "tab2_drag", "tab3_recovery", "tab4_feasibility"]


def _best_source(base, tab):
    """Across all sysid_* runs, pick the dir with the most REAL data rows for this tab (a
    GATE_FAILED-only recovery file counts as zero real rows). Most-rows tracks best-quality
    here because the clean fresh runs are also the complete ones; degraded/partial runs are
    shorter."""
    best_dir, best_rows, best_n = None, [], -1
    for d in sorted(os.listdir(base)):
        if not d.startswith("sysid_") or d.startswith("sysid_merged"):
            continue
        p = os.path.join(base, d, tab + ".csv")
        if not os.path.exists(p):
            continue
        rows = load(p)
        real = [r for r in rows if r.get("trial") != "GATE_FAILED"]
        if len(real) > best_n:
            best_dir, best_rows, best_n = d, rows, len(real)
    return best_dir, best_rows


def generate_merged(base):
    """Combine the best clean run of each tab into one master report + a self-contained dir."""
    import shutil
    picked = {tab: _best_source(base, tab) for tab in TABS}
    out_dir = os.path.join(base, time.strftime("sysid_merged_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    provenance = []
    for tab in TABS:
        src_dir, _ = picked[tab]
        if src_dir:
            shutil.copy(os.path.join(base, src_dir, tab + ".csv"), os.path.join(out_dir, tab + ".csv"))
            provenance.append(f"`{tab}` ← `{src_dir}`")
        else:
            provenance.append(f"`{tab}` ← (no data)")
    return _render(picked["tab1_rotational"][1], picked["tab2_drag"][1],
                   picked["tab3_recovery"][1], picked["tab4_feasibility"][1],
                   f"`{os.path.basename(out_dir)}` (merged)",
                   os.path.join(out_dir, "SYSID_REPORT.md"), provenance=provenance)


def main():
    base = DATASETS_DIR
    if len(sys.argv) > 1 and sys.argv[1] == "merge":
        if not os.path.isdir(base):
            print("No datasets/ directory found.", flush=True)
            return 1
        generate_merged(base)
        return 0
    if len(sys.argv) > 1:
        out_dir = sys.argv[1]
    else:
        cands = sorted(d for d in os.listdir(base)
                       if d.startswith("sysid_")) if os.path.isdir(base) else []
        if not cands:
            print("No sysid_* directory found. Run the campaign first.", flush=True)
            return 1
        out_dir = os.path.join(base, cands[-1])
    generate(out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
