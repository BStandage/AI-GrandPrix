"""
Reconstruct the VQ2 course map from a steady_pilot completion run (VQ2 = vision + IMU only, no
odometry, no track broadcast - the map must come from our own flight).

    python -m analysis.extract_course_map datasets/steady_dbg_20260727_180905.csv

Method: dead-reckon the flown path from the ~90 Hz debug CSV using the flight-validated models,
then anchor gates at the pilot's pass events (at a `gates_passed` increment the drone is inside
that gate, +-0.4 m).

Models (all measured in flight, see COORDINATE_CONVENTIONS.md + sysid):
  attitude   the sim holds attitude setpoints at gain 1.0 through a first-order ~96 ms lag, so
             the COMMANDED pitch/roll/yaw columns are the true attitude (lagged)
  fwd/lat    v' = g*tan(angle_lag) - k*v*|v|, k = 0.0343 (drag model validated at cruise)
  signs      logged +pitch_deg = forward accel, +roll_cmd_deg = rightward accel (confirmed:
             run 220336 banked +roll toward a right-side aim and physically moved right),
             yaw_deg = CCW-positive world heading (spawn faces +97 deg)
  vertical   alt_est column (leaky IMU integrator, tau 4 s) - weakest axis; per-leg deltas are
             decent, absolute alt sags on long climbs. Marked in the output for the solver to
             treat with margin.

Frame: spawn at origin, axes = the yaw-frame world (heading CCW+ about +Z up). Consistent frame
is all the solver needs. Per-leg RELATIVE accuracy is what matters: the racer re-anchors on
vision at every gate, so global drift never compounds at runtime.

Validation: run on two independent completions and compare - reconstruction error shows up as
gate-position disagreement.
"""

import csv
import json
import math
import os
import sys

G = 9.81
# Two-term drag from the Tab 7 ace-envelope battery (sysid_20260728_095038, VQ1 range, odometry
# ground truth): a_drag = C1*v + C2*v^2. The old quadratic-only k=0.0343 crossed this curve at
# cruise (which is why it validated there) but under-dragged the creep regime ~1.8x - the exact
# stretch this map showed before the fit. Relative-error weighted, max +-14% over 2-20 m/s;
# predicted creep equilibria 1.93/2.92/3.70 m/s vs measured 1.9/2.9/3.7.
DRAG_C1 = 0.1141
DRAG_C2 = 0.0192
ATT_TAU = 0.096
# Vertical channel: the VALIDATED thrust model (see ace config ACE_T0V block) run over the
# logged collective - replaces the alt_est column, whose leaky integrator compressed the
# course's climbs (the "flying above the gate" map errors). c1v/c2v from the Tab 7 fit;
# T0 is per-sim (VQ2 ~0.27) and can be trimmed via cross-run consistency.
T0V = 0.265   # NOTE: the map RECONSTRUCTION keeps the legacy linear model - it was
THR_EXP = 1.0  # cross-run validated on steady logs (steady flew ~0.30 LEVEL near the
               # floor vs probe hover 0.265 at altitude - unexplained, maybe ground
               # effect; p=1.45 turns steady logs into a phantom 50 m climb ramp)
C1V = 0.50
C2V = 0.010


def reconstruct(csv_path, t0v=T0V):
    rows = list(csv.DictReader(open(csv_path)))
    th_lag = 0.0    # lagged pitch (rad, +fwd)
    ph_lag = 0.0    # lagged roll (rad, +right)
    vx = vy = 0.0   # world-frame velocity (m/s)
    x = y = 0.0
    vz = z = 0.0
    t_prev = None
    path = []       # (t, x, y, z, vx, vy)
    gates = []
    prev_g = 0
    for r in rows:
        t = float(r["t"])
        dt = 0.0 if t_prev is None else t - t_prev
        t_prev = t
        if not (0.0 < dt <= 0.1):
            dt = 1.0 / 90.0
        th_cmd = math.radians(float(r["pitch_deg"]))
        ph_cmd = math.radians(float(r["roll_cmd_deg"]))
        a = 1.0 - math.exp(-dt / ATT_TAU)
        th_lag += (th_cmd - th_lag) * a
        ph_lag += (ph_cmd - ph_lag) * a
        psi = math.radians(float(r["yaw_deg"]))
        fwd = (math.cos(psi), math.sin(psi))
        right = (math.sin(psi), -math.cos(psi))
        # thrust-scaled horizontal accel through the MEASURED lift curve
        tw = G * (max(float(r["thr"]), 0.0) / t0v) ** THR_EXP
        af = tw * math.cos(ph_lag) * math.sin(th_lag)
        ar = tw * math.cos(th_lag) * math.sin(ph_lag)
        sp = math.hypot(vx, vy)
        drag = DRAG_C1 + DRAG_C2 * sp
        ax = af * fwd[0] + ar * right[0] - drag * vx
        ay = af * fwd[1] + ar * right[1] - drag * vy
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        # thrust-model vertical (validated 1.09 m/s; replaces the leak-compressed alt_est)
        a_up = (G * (max(float(r["thr"]), 0.0) / t0v) ** THR_EXP
                * math.cos(th_lag) * math.cos(ph_lag) - G
                - (C1V + C2V * abs(vz)) * vz)
        vz += a_up * dt
        z += vz * dt
        path.append((round(t, 3), round(x, 3), round(y, 3), round(z, 3),
                     round(vx, 3), round(vy, 3)))
        g = int(r["gates_passed"])
        if g != prev_g:
            spd = math.hypot(vx, vy)
            gates.append({
                "gate_id": g - 1,           # 0-based: first increment = gate 0 passed
                "t_pass": round(t, 2),
                "x": round(x, 2), "y": round(y, 2), "z": round(z, 2),
                "heading_deg": round(math.degrees(psi), 1),
                "cross_dir": [round(vx / spd, 3) if spd > 0.2 else fwd[0],
                              round(vy / spd, 3) if spd > 0.2 else fwd[1]],
                "v_pass": round(spd, 2),
            })
            prev_g = g
    return gates, path


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: python -m analysis.extract_course_map <steady_dbg.csv> [more.csv ...]")
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    all_maps = []
    for f in sys.argv[1:]:
        gates, path = reconstruct(f)
        all_maps.append((f, gates, path))
        span_x = (min(g["x"] for g in gates), max(g["x"] for g in gates))
        span_y = (min(g["y"] for g in gates), max(g["y"] for g in gates))
        print(f"\n{os.path.basename(f)}: {len(gates)} gates, "
              f"x span {span_x[0]:.0f}..{span_x[1]:.0f}, y span {span_y[0]:.0f}..{span_y[1]:.0f}")
        for g in gates:
            print(f"  g{g['gate_id']:2d} t={g['t_pass']:6.1f}  ({g['x']:+8.2f}, {g['y']:+8.2f}, "
                  f"{g['z']:+6.2f})  hdg={g['heading_deg']:+6.1f}  v={g['v_pass']:.1f}")

    if len(all_maps) >= 2:
        print("\ncross-run gate disagreement (leg-relative, run A vs run B):")
        ga, gb = all_maps[0][1], all_maps[1][1]
        n = min(len(ga), len(gb))
        for i in range(1, n):
            la = (ga[i]["x"] - ga[i - 1]["x"], ga[i]["y"] - ga[i - 1]["y"], ga[i]["z"] - ga[i - 1]["z"])
            lb = (gb[i]["x"] - gb[i - 1]["x"], gb[i]["y"] - gb[i - 1]["y"], gb[i]["z"] - gb[i - 1]["z"])
            d = math.sqrt(sum((p - q) ** 2 for p, q in zip(la, lb)))
            print(f"  leg {i - 1}->{i}: |A| {math.hypot(la[0], la[1]):5.1f} m  "
                  f"|B| {math.hypot(lb[0], lb[1]):5.1f} m  disagreement {d:5.2f} m")

    # write the map from the FIRST file given (pass the best/official run first)
    f, gates, path = all_maps[0]
    # TAIL MERGE: if a later run passed MORE gates (run A's log ends at its last increment, so
    # the finish leg is missing), chain the extra gates on by their leg-relative vectors -
    # leg-relative is the frame-safe currency (headings at the junction gate agreed 52.7 vs
    # 54.3 deg across runs A/B). z comes along but is only as good as the donor run's alt_est.
    for _, gb, _ in all_maps[1:]:
        while len(gb) > len(gates):
            i = len(gates)
            leg = (gb[i]["x"] - gb[i - 1]["x"], gb[i]["y"] - gb[i - 1]["y"],
                   gb[i]["z"] - gb[i - 1]["z"])
            tail = dict(gb[i])
            tail.update({"gate_id": i - 1 + 1,
                         "x": round(gates[-1]["x"] + leg[0], 2),
                         "y": round(gates[-1]["y"] + leg[1], 2),
                         "z": round(gates[-1]["z"] + leg[2], 2),
                         "merged_from_donor": True})
            gates.append(tail)
            print(f"  appended gate {tail['gate_id']} from donor run "
                  f"(leg {math.hypot(leg[0], leg[1]):.1f} m)")
    out = {"source": os.path.basename(f),
           "frame": "spawn origin, yaw-frame world axes (heading CCW+), z up (alt_est)",
           "gates": gates,
           "path": path[::9]}   # ~10 Hz decimated reference path
    out_path = os.path.join(here, "pilots", "ace_pilot", "course_map.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    json.dump(out, open(out_path, "w"))
    print(f"\nwrote {out_path} (map from {os.path.basename(f)})")


if __name__ == "__main__":
    main()
