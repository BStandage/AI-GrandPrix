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


def _parse_pt_name(name):
    """Nominal (target_dist, entry_speed) from a 'pt_d<d>_v<v>' trial name."""
    d = v = None
    for part in (name or "").split("_"):
        if part.startswith("d"):
            d = _num(part[1:])
        elif part.startswith("v"):
            v = _num(part[1:])
    return d, v


def analyze_point_tracking(rows):
    """Per-trial closed-loop summary re-derived from the per-tick Tab 5 rows.

    Station hold -> drift stats. Point approaches -> braking distance (forward travel from loop
    closure to the furthest point reached), peak overshoot, settling time (loop closure to
    |err|<0.5 m & |v|<0.5 m/s) and final error. Everything is computed from the logged err_*/vh/t
    columns, so the report stays a pure function of the CSVs (same as tabs 1-2)."""
    trials = {}
    for r in rows:
        trials.setdefault(r.get("trial"), []).append(r)

    station = None
    approaches = []
    for name, trows in trials.items():
        if not name:
            continue
        if name.startswith("station"):
            drift = [math.hypot(_num(r.get("err_fwd")) or 0.0, _num(r.get("err_lat")) or 0.0)
                     for r in trows]
            altd = [abs(_num(r.get("err_alt")) or 0.0) for r in trows]
            station = {
                "max_drift_m": max(drift) if drift else None,
                "rms_error_m": math.sqrt(sum(d * d for d in drift) / len(drift)) if drift else None,
                "altitude_drift_m": max(altd) if altd else None,
            }
        elif name.startswith("pt_"):
            home = [r for r in trows if r.get("phase") == "home"]
            if not home:
                continue
            d_nom, v_nom = _parse_pt_name(name)
            err_fwds = [_num(r.get("err_fwd")) or 0.0 for r in home]
            ef0 = err_fwds[0]
            # err_fwd = target - fwd_pos, so forward travel from loop closure to the furthest
            # forward point = ef0 - min(err_fwd); overshoot past the target = max(0, -min(err_fwd)).
            brake_distance = ef0 - min(err_fwds)
            peak_overshoot = max(0.0, -min(err_fwds))
            entry_v = _num(home[0].get("vh"))
            t0 = _num(home[0].get("t")) or 0.0
            settling = final_err = None
            for r in home:
                ef = _num(r.get("err_fwd")) or 0.0
                el = _num(r.get("err_lat")) or 0.0
                ea = _num(r.get("err_alt")) or 0.0
                vh = _num(r.get("vh")) or 0.0
                final_err = math.sqrt(ef * ef + el * el + ea * ea)
                if settling is None and final_err < 0.5 and vh < 0.5:
                    settling = (_num(r.get("t")) or 0.0) - t0
            approaches.append({
                "trial": name, "d_nom": d_nom, "v_nom": v_nom,
                "entry_speed_actual": entry_v, "brake_distance_m": brake_distance,
                "peak_overshoot_m": peak_overshoot, "settling_time_s": settling,
                "final_error_m": final_err,
            })
    approaches.sort(key=lambda a: (a["d_nom"] or 0.0, a["v_nom"] or 0.0))
    return {"station": station, "approaches": approaches}


# ---- rendering -----------------------------------------------------------------------------
def _att_mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def _att_steady(rows, frac=0.4):
    """Last `frac` of a hold = its steady state (after the attitude has settled)."""
    if not rows:
        return []
    return rows[max(0, int(len(rows) * (1.0 - frac))):]


def analyze_attitude(rows):
    """Tab 6: the send_attitude_setpoint sign/gain/convention map the rate tabs never covered.

    Per held attitude we read the ground-truth steady state: which way it TRANSLATES (sign), how
    much attitude it actually reached vs commanded (gain), terminal speed, the yaw convention, and
    the attitude-mode hover point."""
    if not rows:
        return None
    # One continuous flight: each hold has a unique `seg` whose PREFIX names the axis
    # (pitch.../roll.../yaw.../thr...); "level" settles between holds are skipped. We key off the seg
    # prefix rather than the `group` column so this is robust regardless of how group was written.
    def _grp(seg):
        for g in ("pitch", "roll", "yaw", "thr"):
            if (seg or "").startswith(g):
                return "thrust" if g == "thr" else g
        return "level"

    by_seg = {}
    for r in rows:
        seg = r.get("seg")
        g = _grp(seg)
        if g == "level":
            continue
        by_seg.setdefault((g, seg), []).append(r)

    def fwd_right(r):
        yaw = _num(r.get("yaw")) or 0.0
        vx, vy = _num(r.get("vx_w")) or 0.0, _num(r.get("vy_w")) or 0.0
        return (vx * math.cos(yaw) + vy * math.sin(yaw),      # body-forward speed (world vel on heading)
                -vx * math.sin(yaw) + vy * math.cos(yaw))     # body-right speed

    out = {"pitch": None, "roll": None, "yaw": None, "hover": None, "thrust_curve": []}

    for axis in ("pitch", "roll"):
        pts = []
        for (grp, _seg), rs in by_seg.items():
            if grp != axis:
                continue
            steady = _att_steady(rs)
            cmd = _att_mean([_num(r.get(f"cmd_{axis}_deg")) for r in steady])
            act = _att_mean([_num(r.get(axis)) for r in steady])          # actual angle, rad
            fr = [fwd_right(r) for r in steady]
            motion = _att_mean([(v[0] if axis == "pitch" else v[1]) for v in fr])
            speed = _att_mean([_num(r.get("vh")) for r in steady])
            if cmd is not None:
                pts.append({"cmd_deg": cmd, "act_deg": math.degrees(act) if act is not None else None,
                            "motion": motion, "speed": speed})
        if pts:
            pos = [p for p in pts if p["cmd_deg"] > 0 and p["motion"] is not None]
            direction = None
            if pos:
                big = max(pos, key=lambda p: p["cmd_deg"])
                if axis == "pitch":
                    direction = "forward (+x body)" if big["motion"] > 0 else "backward (-x body)"
                else:
                    direction = "right (+y body)" if big["motion"] > 0 else "left (-y body)"
            gains = [p["act_deg"] / p["cmd_deg"] for p in pts if p["act_deg"] and p["cmd_deg"]]
            out[axis] = {"direction": direction, "gain": _att_mean(gains),
                         "term_speed": max((p["speed"] or 0.0) for p in pts),
                         "pts": sorted(pts, key=lambda p: p["cmd_deg"])}

    steps = []
    for (grp, seg), rs in by_seg.items():
        if grp != "yaw":
            continue
        steady = _att_steady(rs)
        steps.append({"seg": seg,
                      "cmd_deg": _att_mean([_num(r.get("cmd_yaw_deg")) for r in steady]),
                      "act_deg": (lambda a: math.degrees(a) if a is not None else None)(
                          _att_mean([_num(r.get("yaw")) for r in steady]))})
    steps = [s for s in steps if s["cmd_deg"] is not None]
    if steps:
        out["yaw"] = sorted(steps, key=lambda s: (s["cmd_deg"], s["seg"]))

    curve = []
    for (grp, _seg), rs in by_seg.items():
        if grp != "thrust":
            continue
        steady = _att_steady(rs)
        thr = _att_mean([_num(r.get("cmd_thr")) for r in steady])
        climb = _att_mean([_num(r.get("climb_up")) for r in steady])
        if thr is not None and climb is not None:
            curve.append((thr, climb))
    curve.sort()
    out["thrust_curve"] = curve
    for (t0, c0), (t1, c1) in zip(curve, curve[1:]):
        if c0 <= 0.0 <= c1 and c1 != c0:
            out["hover"] = t0 + (t1 - t0) * (0.0 - c0) / (c1 - c0)
            break
    return out


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


def _render(rot_rows, drag_rows, rec_rows, feas_rows, pt_rows, source_label, out_path,
            provenance=None, att_rows=None):
    rot = analyze_rotational(rot_rows)
    drag = analyze_drag(drag_rows)
    rec = analyze_recovery(rec_rows)
    feas = analyze_feasibility(feas_rows, drag.get("drag_k"))
    pt = analyze_point_tracking(pt_rows)
    att = analyze_attitude(att_rows or [])

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

    # ---- closed-loop point tracking & station hold ----
    L.append("## 6. Closed-loop point tracking & station hold")
    L.append("")
    L.append("Full outer position loop on world-frame ODOMETRY (Round-1 ground truth), no vision "
             "or gate involved: accelerate to an entry speed, then close the loop and stop at a "
             "known point. These three numbers set the constraints for any forward-speed control "
             "law — at speed `v` the **braking distance** is the lead room the controller must "
             "reserve before a gate, the **settling time** is how long after braking the position "
             "locks, and the **station-hold drift** is the closed-loop hover floor.")
    L.append("")
    st = pt["station"]
    if st and st.get("max_drift_m") is not None:
        L.append(f"**Station-hold baseline (10 s hover):** max drift **{_fmt(st['max_drift_m'])} m**, "
                 f"RMS error {_fmt(st['rms_error_m'])} m, altitude drift {_fmt(st['altitude_drift_m'])} m.")
    else:
        L.append("_No station-hold data._")
    L.append("")
    if pt["approaches"]:
        dists = sorted({a["d_nom"] for a in pt["approaches"] if a["d_nom"] is not None})
        speeds = sorted({a["v_nom"] for a in pt["approaches"] if a["v_nom"] is not None})
        idx = {(a["d_nom"], a["v_nom"]): a for a in pt["approaches"]}

        L.append("**Braking distance (m) — forward travel from loop closure to the furthest point:**")
        L.append("")
        L.append("| entry speed \\ target dist | " + " | ".join(f"{d:g} m" for d in dists) + " |")
        L.append("|---|" + "---|" * len(dists))
        for v in speeds:
            cells = [(_fmt(idx[(d, v)]["brake_distance_m"]) if (d, v) in idx else "—") for d in dists]
            L.append(f"| {v:g} m/s | " + " | ".join(cells) + " |")
        L.append("")

        L.append("**Settling time (s) — loop closure to |err|<0.5 m & |v|<0.5 m/s** "
                 "(— = did not converge within the timeout):")
        L.append("")
        L.append("| entry speed \\ target dist | " + " | ".join(f"{d:g} m" for d in dists) + " |")
        L.append("|---|" + "---|" * len(dists))
        for v in speeds:
            cells = []
            for d in dists:
                a = idx.get((d, v))
                cells.append(_fmt(a["settling_time_s"], 1) if (a and a["settling_time_s"] is not None)
                             else "—")
            L.append(f"| {v:g} m/s | " + " | ".join(cells) + " |")
        L.append("")

        L.append("Per-trial detail:")
        L.append("")
        L.append("| trial | entry v (m/s) | brake dist (m) | overshoot (m) | settling (s) | "
                 "final err (m) |")
        L.append("|---|---|---|---|---|---|")
        for a in pt["approaches"]:
            settle = _fmt(a["settling_time_s"], 1) if a["settling_time_s"] is not None else "n/a"
            L.append(f"| {a['trial']} | {_fmt(a['entry_speed_actual'])} | "
                     f"{_fmt(a['brake_distance_m'])} | {_fmt(a['peak_overshoot_m'])} | {settle} | "
                     f"{_fmt(a['final_error_m'])} |")
        L.append("")
        L.append("> Conservative starting gains (kp_fwd=0.3, kd_fwd=0.8, kp_lat=0.3, kp_alt=0.5) "
                 "chosen to respect the 96 ms rotational lag and the weak passive drag; blank "
                 "settling cells mark cells that need gain tuning, not a hard airframe limit.")
        L.append("")
    else:
        L.append("_No point-approach data._")
        L.append("")

    # ---- 6. Attitude-setpoint interface (only if the battery was run) ----
    if att:
        L.append("## 6. Attitude-setpoint interface (`send_attitude_setpoint`)")
        L.append("")
        L.append("Sign conventions, attitude gain and lag for the ABSOLUTE-attitude command path the "
                 "vision / hover pilots fly. The rate tabs above do NOT cover this path, which is why "
                 "its signs kept surprising the pilots. Measured against ground-truth odometry/attitude.")
        L.append("")
        L.append("| Axis | +command drives the drone | Attitude gain (actual/cmd) | Terminal speed (m/s) |")
        L.append("|---|---|---|---|")
        for axis in ("pitch", "roll"):
            a = att.get(axis)
            if a:
                L.append(f"| {axis} | **{a.get('direction') or '?'}** | {_fmt(a.get('gain'), 2)} | "
                         f"{_fmt(a.get('term_speed'))} |")
        L.append("")
        if att.get("yaw"):
            L.append("**Yaw convention** — commanded absolute yaw vs the heading actually reached "
                     "(does it track = absolute? which sign?):")
            L.append("")
            L.append("| commanded yaw (°) | actual yaw reached (°) |")
            L.append("|---|---|")
            for s in att["yaw"]:
                L.append(f"| {_fmt(s['cmd_deg'], 0)} | {_fmt(s['act_deg'], 0)} |")
            L.append("")
        if att.get("hover") is not None:
            L.append(f"**Hover thrust (attitude mode):** {att['hover']:.3f}  "
                     f"(dynamics.py `HOVER_THRUST` = {dynamics.HOVER_THRUST:.3f})")
            L.append("")
        if att.get("thrust_curve"):
            L.append("Thrust → climb (level attitude hold):")
            L.append("")
            L.append("| thrust | climb (m/s, up+) |")
            L.append("|---|---|")
            for thr, climb in att["thrust_curve"]:
                L.append(f"| {thr:.3f} | {_fmt(climb)} |")
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
                   load(os.path.join(out_dir, "tab5_point_tracking.csv")),
                   f"`{os.path.basename(out_dir)}`",
                   os.path.join(out_dir, "SYSID_REPORT.md"),
                   att_rows=load(os.path.join(out_dir, "tab6_attitude.csv")))


TABS = ["tab1_rotational", "tab2_drag", "tab3_recovery", "tab4_feasibility", "tab5_point_tracking",
        "tab6_attitude"]


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
                   picked["tab5_point_tracking"][1],
                   f"`{os.path.basename(out_dir)}` (merged)",
                   os.path.join(out_dir, "SYSID_REPORT.md"), provenance=provenance,
                   att_rows=picked["tab6_attitude"][1])


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
