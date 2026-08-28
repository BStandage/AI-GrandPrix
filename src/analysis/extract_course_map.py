"""
Reconstruct the VQ2 course map from a steady_pilot survey (VQ2 = vision + IMU only).

    python -m analysis.extract_course_map datasets/steady_dbg_YYYYMMDD_HHMMSS.csv
    python -m analysis.extract_course_map <csv> --session datasets/session_...

Method: dead-reckon the flown path from the ~90 Hz debug CSV with plant_fit.json,
then place each gate at the DR pose at the *exact* race tick
(`race_status.last_gate_race_time`), not at the blob-shrink `gates_passed`
increment (MEGA_AUDIT §1.1 — that heuristic caused the 17-vs-18 topology fork).

Session auto-discovery: steady_dbg_YYYYMMDD_HHMMSS.csv → session_YYYYMMDD_HHMMSS/.
July-27 CSVs have no telemetry → blob fallback with a loud warning.

Models (ONE plant — pilots/giga_pilot/plant_fit.json):
  attitude   first-order lag tau_rp from plant_fit (0.27 s)
  thrust     A = g*thr/h0 along lagged body-z; drag = d1*v + d2*v^2
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys

from analysis.fit_plant import load_plant_fit, body_thrust_accel

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
DATASETS = os.path.join(SRC, "datasets")


def _plant_constants():
    fit = load_plant_fit()
    return (
        float(fit.get("h0", 0.299)),
        float(fit.get("tau_rp", 0.27)),
        float(fit.get("d1", 0.0)),
        float(fit.get("d2", 0.0115)),
    )


def resolve_session_for_csv(csv_path, session=None):
    """Explicit --session, else exact stamp match, else nearest session within 120 s."""
    if session:
        return session
    base = os.path.basename(csv_path)
    m = re.match(r"steady_dbg_(\d{8})_(\d{6})", base)
    if not m:
        return None
    day, hms = m.group(1), m.group(2)
    exact = os.path.join(DATASETS, f"session_{day}_{hms}")
    tel = os.path.join(exact, "telemetry.jsonl")
    if os.path.isfile(tel):
        return exact
    # dbg CSV stamp can lag session start by a few seconds
    try:
        from datetime import datetime, timedelta
        t0 = datetime.strptime(day + hms, "%Y%m%d%H%M%S")
    except ValueError:
        return None
    best = None
    for name in os.listdir(DATASETS):
        mm = re.match(r"session_(\d{8})_(\d{6})$", name)
        if not mm or mm.group(1) != day:
            continue
        cand = os.path.join(DATASETS, name)
        if not os.path.isfile(os.path.join(cand, "telemetry.jsonl")):
            continue
        try:
            ts = datetime.strptime(mm.group(1) + mm.group(2), "%Y%m%d%H%M%S")
        except ValueError:
            continue
        dt = abs((ts - t0).total_seconds())
        if dt <= 120.0 and (best is None or dt < best[0]):
            best = (dt, cand)
    return best[1] if best else None


def load_survey_ticks(session_or_tel, min_gates: int = 3):
    """Exact (gate_id, t_race_s) from last_gate_race_time."""
    from analysis.race_ticks import best_lap_ticks, ticks_from_last_gate

    ticks = best_lap_ticks(session_or_tel, min_gates=min_gates)
    if not ticks:
        ticks = ticks_from_last_gate(session_or_tel)
    return ticks


def _interp_path(path, t):
    """path rows: (t, x, y, z, vx, vy, yaw_rad[, alt_est]). Linear interp."""
    if not path:
        return None
    if t <= path[0][0]:
        return path[0]
    if t >= path[-1][0]:
        return path[-1]
    lo, hi = 0, len(path) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if path[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = path[lo], path[hi]
    span = max(b[0] - a[0], 1e-9)
    u = (t - a[0]) / span
    n = len(a)
    return tuple(a[i] + (b[i] - a[i]) * u for i in range(n))


def _gate_from_state(gid, t_pass, state, anchor, z_src="plant"):
    # state: (t, x, y, z_plant, vx, vy, yaw_rad[, alt_est])
    _t, x, y, z_plant, vx, vy, yaw = state[:7]
    alt = state[7] if len(state) > 7 else None
    if alt is not None and z_src == "alt_est":
        z = float(alt)
    else:
        z = float(z_plant)
    spd = math.hypot(vx, vy)
    fwd = (math.cos(yaw), math.sin(yaw))
    g = {
        "gate_id": int(gid),
        "t_pass": round(float(t_pass), 3),
        "x": round(x, 2), "y": round(y, 2), "z": round(z, 2),
        "heading_deg": round(math.degrees(yaw), 1),
        "cross_dir": [
            round(vx / spd, 3) if spd > 0.2 else round(fwd[0], 3),
            round(vy / spd, 3) if spd > 0.2 else round(fwd[1], 3),
        ],
        "v_pass": round(spd, 2),
        "anchor": anchor,
        "z_src": z_src if alt is not None else "plant",
    }
    if alt is not None:
        g["z_plant"] = round(float(z_plant), 2)
        g["alt_est"] = round(float(alt), 2)
    return g


def _csv_index_edges(rows, key):
    """(gate_id, t_csv) when integer column advances N→N+1 (gate N passed)."""
    events = []
    prev = None
    for r in rows:
        raw = r.get(key, "")
        if raw is None or raw == "":
            continue
        try:
            gi = int(float(raw))
        except ValueError:
            continue
        if prev is not None and gi == prev + 1:
            events.append((gi - 1, float(r["t"])))
        prev = gi
    return events


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return 0.0
    return xs[len(xs) // 2]


def _align_csv_to_race(rows, ticks):
    """Median (t_csv - t_race) using race_gate edges, else gates_passed edges."""
    by_race = {g: t for g, t in ticks}
    for key in ("race_gate", "gates_passed"):
        edges = _csv_index_edges(rows, key)
        offs = [tc - by_race[g] for g, tc in edges if g in by_race]
        if len(offs) >= max(2, len(ticks) // 3):
            return _median(offs), key, len(offs)
    # CSV t already ~ race clock on some surveys (logging from GO)
    return 0.0, "identity", 0


def _dead_reckon(rows, h0, att_tau, d1, d2):
    """Full path + blob gate list (legacy). path includes yaw_rad and alt_est."""
    th_lag = ph_lag = 0.0
    vx = vy = vz = 0.0
    x = y = z = 0.0
    t_prev = None
    path = []
    blob_gates = []
    prev_g = 0
    has_alt = bool(rows) and "alt_est" in rows[0]
    z_src = "alt_est" if has_alt else "plant"
    for r in rows:
        t = float(r["t"])
        dt = 0.0 if t_prev is None else t - t_prev
        t_prev = t
        if not (0.0 < dt <= 0.1):
            dt = 1.0 / 90.0
        th_cmd = math.radians(float(r["pitch_deg"]))
        ph_cmd = math.radians(float(r["roll_cmd_deg"]))
        a = 1.0 - math.exp(-dt / att_tau)
        th_lag += (th_cmd - th_lag) * a
        ph_lag += (ph_cmd - ph_lag) * a
        psi = math.radians(float(r["yaw_deg"]))
        thr = max(float(r["thr"]), 0.0)
        A = body_thrust_accel(thr, d1, d2)
        tx = math.cos(psi) * math.sin(th_lag) + math.sin(psi) * math.sin(ph_lag)
        ty = math.sin(psi) * math.sin(th_lag) - math.cos(psi) * math.sin(ph_lag)
        tz = math.cos(th_lag) * math.cos(ph_lag)
        spd = math.sqrt(vx * vx + vy * vy + vz * vz)
        ax, ay, az = A * tx, A * ty, A * tz - G
        if spd > 0.05:
            dr = d1 * spd + d2 * spd * spd
            ax -= dr * vx / spd
            ay -= dr * vy / spd
            az -= dr * vz / spd
        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        x += vx * dt
        y += vy * dt
        z += vz * dt
        if z < 0.0:
            z = 0.0
            vz = max(0.0, vz)
        alt = float(r["alt_est"]) if has_alt and r.get("alt_est", "") != "" else z
        path.append((t, x, y, z, vx, vy, psi, alt))
        g = int(float(r["gates_passed"])) if r.get("gates_passed", "") != "" else prev_g
        if g != prev_g and g > prev_g:
            for gid in range(prev_g, g):
                blob_gates.append(
                    _gate_from_state(gid, t, path[-1], "gates_passed_blob", z_src))
            prev_g = g
    path_out = [
        (round(t, 3), round(x, 3), round(y, 3), round(z, 3),
         round(vx, 3), round(vy, 3))
        for t, x, y, z, vx, vy, _yaw, _alt in path
    ]
    return path, path_out, blob_gates, z_src


def reconstruct(csv_path, t0v=None, session=None, prefer_ticks=True):
    """Return (gates, path_xy, meta).

    gates are tick-anchored when a session with last_gate_race_time is available.
    """
    h0, att_tau, d1, d2 = _plant_constants()
    if t0v is not None:
        h0 = float(t0v)
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8", errors="replace")))
    path_full, path_out, blob_gates, z_src = _dead_reckon(rows, h0, att_tau, d1, d2)

    meta = {
        "anchor": "gates_passed_blob",
        "session": None,
        "n_ticks": 0,
        "clock_offset_s": None,
        "align_via": None,
        "blob_gate_count": len(blob_gates),
        "z_src": z_src,
    }

    sess = resolve_session_for_csv(csv_path, session) if prefer_ticks else None
    ticks = load_survey_ticks(sess) if sess else []
    if not prefer_ticks:
        meta["warn"] = "forced --blob (gates_passed); ticks ignored"
        return blob_gates, path_out, meta
    if not ticks:
        meta["warn"] = (
            "no last_gate_race_time ticks — using blob gates_passed "
            "(17-vs-18 risk; re-fly with session logging)"
        )
        return blob_gates, path_out, meta

    offset, align_via, n_align = _align_csv_to_race(rows, ticks)
    gates = []
    for gid, t_race in ticks:
        state = _interp_path(path_full, t_race + offset)
        if state is None:
            continue
        gates.append(_gate_from_state(
            gid, t_race, state, "last_gate_race_time", z_src))

    meta.update({
        "anchor": "last_gate_race_time",
        "session": os.path.basename(sess.rstrip("\\/")),
        "n_ticks": len(ticks),
        "clock_offset_s": round(offset, 4),
        "align_via": align_via,
        "n_align": n_align,
        "blob_gate_count": len(blob_gates),
    })
    if abs(len(gates) - len(blob_gates)) >= 1:
        meta["count_delta_vs_blob"] = len(gates) - len(blob_gates)
    return gates, path_out, meta


def main():
    ap = argparse.ArgumentParser(
        description="DR course map; tick-anchored when session telemetry exists")
    ap.add_argument("csvs", nargs="+", help="steady_dbg_*.csv survey logs")
    ap.add_argument("--session", default=None,
                    help="telemetry session dir (default: auto-match stamp)")
    ap.add_argument("--blob", action="store_true",
                    help="force gates_passed blob anchors (ignore ticks)")
    ap.add_argument("--write", default=None,
                    help="output map JSON (default: pilots/ace_pilot/course_map.json)")
    args = ap.parse_args()

    all_maps = []
    for f in args.csvs:
        gates, path, meta = reconstruct(
            f, session=args.session, prefer_ticks=not args.blob)
        all_maps.append((f, gates, path, meta))
        if not gates:
            print(f"\n{os.path.basename(f)}: 0 gates ({meta})")
            continue
        span_x = (min(g["x"] for g in gates), max(g["x"] for g in gates))
        span_y = (min(g["y"] for g in gates), max(g["y"] for g in gates))
        print(f"\n{os.path.basename(f)}: {len(gates)} gates  "
              f"anchor={meta['anchor']}"
              + (f"  session={meta['session']}" if meta.get("session") else "")
              + (f"  off={meta['clock_offset_s']:+.3f}s via {meta['align_via']}"
                 if meta.get("clock_offset_s") is not None else ""))
        if meta.get("warn"):
            print(f"  WARN {meta['warn']}")
        if meta.get("count_delta_vs_blob") is not None:
            print(f"  blob counted {meta['blob_gate_count']}  "
                  f"ticks={meta['n_ticks']}  "
                  f"delta={meta['count_delta_vs_blob']:+d}")
        print(f"  x span {span_x[0]:.0f}..{span_x[1]:.0f}, "
              f"y span {span_y[0]:.0f}..{span_y[1]:.0f}")
        for g in gates:
            print(f"  g{g['gate_id']:2d} t={g['t_pass']:7.3f}  "
                  f"({g['x']:+8.2f}, {g['y']:+8.2f}, {g['z']:+6.2f})  "
                  f"hdg={g['heading_deg']:+6.1f}  v={g['v_pass']:.1f}")

    if len(all_maps) >= 2:
        print("\ncross-run gate disagreement (leg-relative, run A vs run B):")
        ga, gb = all_maps[0][1], all_maps[1][1]
        n = min(len(ga), len(gb))
        fork_at = None
        FORK_LEG_M = 3.0
        FORK_HDG_DEG = 35.0
        for i in range(1, n):
            la = (ga[i]["x"] - ga[i - 1]["x"], ga[i]["y"] - ga[i - 1]["y"],
                  ga[i]["z"] - ga[i - 1]["z"])
            lb = (gb[i]["x"] - gb[i - 1]["x"], gb[i]["y"] - gb[i - 1]["y"],
                  gb[i]["z"] - gb[i - 1]["z"])
            d = math.sqrt(sum((p - q) ** 2 for p, q in zip(la, lb)))
            dxy = math.hypot(la[0] - lb[0], la[1] - lb[1])
            dhdg = abs((gb[i]["heading_deg"] - ga[i]["heading_deg"] + 180.0)
                       % 360.0 - 180.0)
            flag = ""
            if dxy >= FORK_LEG_M or dhdg >= FORK_HDG_DEG:
                flag = "  FORK"
                if fork_at is None:
                    fork_at = i
            print(f"  leg {i - 1}->{i}: |A| {math.hypot(la[0], la[1]):5.1f} m  "
                  f"|B| {math.hypot(lb[0], lb[1]):5.1f} m  "
                  f"disagreement {d:5.2f} m{flag}")
        if fork_at is not None:
            print(f"\nTOPOLOGY FORK at g{fork_at} — refusing donor tail-merge "
                  f"(trusted prefix g0..g{fork_at - 1}). "
                  f"Use analysis.map_correspondence + freeze_map_v1.")

    f, gates, path, meta = all_maps[0]
    if len(all_maps) >= 2:
        print("tail-merge DISABLED — pass a single survey to write, or freeze_map_v1")
    h0, tau, d1, d2 = _plant_constants()
    out = {
        "source": os.path.basename(f),
        "frame": "spawn origin, yaw-frame world axes (heading CCW+), z up (plant thrust)",
        "plant": {"h0": h0, "tau_rp": tau, "d1": d1, "d2": d2},
        "anchor": meta,
        "gates": gates,
        "path": path[::9],
        "note": (
            "tick-anchored DR when session telemetry present; "
            "July blob maps are diagnostic only — re-survey with session logging"
        ),
    }
    out_path = args.write or os.path.join(
        SRC, "pilots", "ace_pilot", "course_map.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    json.dump(out, open(out_path, "w"), indent=1)
    print(f"\nwrote {out_path} ({len(gates)} gates, anchor={meta['anchor']})")


if __name__ == "__main__":
    main()
