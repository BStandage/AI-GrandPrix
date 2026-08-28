"""
BATCH per-gate innovation measurement from one tape flight - corrects every gate the camera
saw, not just the one that was hit.

    python -m analysis.gate_innovations <dbg.csv> <session_dir> [--apply]

For each race_gate window: take located openings of the chased gate (biggest, sane range) in
the FINAL approach (range < 9 m), compute the opening's lateral and vertical offset from our
camera axis-projected track, and report the median innovation. With --apply, write damped
(x0.8) corrections into race_offsets.json (z only trusted below 8 m range; lateral to 12 m).
"""

import csv
import json
import math
import os
import sys

from common.camera import HALF_TAN_X, HALF_TAN_Y

UPTILT = math.radians(20.0)
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    dbg_path, sess = sys.argv[1], sys.argv[2]
    apply = "--apply" in sys.argv
    rows = list(csv.DictReader(open(dbg_path)))
    frames = [json.loads(l) for l in open(os.path.join(sess, "vision_frames.jsonl"))]
    go = None
    for line in open(os.path.join(sess, "telemetry.jsonl")):
        r = json.loads(line)
        if r.get("kind") == "race_status":
            st = r.get("race_start_boot_time_ms", -1)
            now = r.get("sim_boot_time_ms", 0)
            if st is not None and st >= 0 and now >= st:
                go = r["recv_time_ns"]
                break
    fn = min(frames, key=lambda f: abs((f.get("recv_ns") or f.get("recv_time_ns")) - go))
    t0 = fn["sim_time_ns"] - ((fn.get("recv_ns") or fn.get("recv_time_ns")) - go)

    def dbg_at(t):
        lo = rows[0]
        for r in rows:
            if float(r["t"]) <= t:
                lo = r
            else:
                break
        return lo

    meas = {}
    for f in frames:
        t = (f["sim_time_ns"] - t0) * 1e-9
        if t < 0 or not f.get("dets"):
            continue
        r = dbg_at(t)
        rg = r.get("race_gate", "")
        if rg in ("", None) or not str(rg).isdigit():
            continue                    # ghost/idle log rows - not a race
        gi = int(rg)
        best = None
        for d in f["dets"]:
            if d.get("has_opening") and d.get("pnp_dist") and 2.2 < d["pnp_dist"] < 12.0:
                if best is None or d["area_frac"] > best["area_frac"]:
                    best = d
        if best is None:
            continue
        pitch = math.radians(float(r["pitch_deg"]))
        bx, by, bw, bh = best["bbox"]
        ox = ((bx + bw / 2.0) - 320.0) / 320.0
        oy = ((by + bh / 2.0) - 180.0) / 180.0
        el = (UPTILT - pitch) - math.atan(oy * HALF_TAN_Y)
        lat = best["pnp_dist"] * math.tan(math.atan(ox * HALF_TAN_X))  # gate right of track +
        above = -best["pnp_dist"] * math.sin(el)                       # drone above gate line +
        meas.setdefault(gi, []).append((t, best["pnp_dist"], lat, above))

    print("gate  n(final)  lateral(gate right of track)   drone-above-gate")
    sugg = {}
    for gi in sorted(meas):
        # CLOSEST-RANGE ONLY: on turning approaches the camera aims at the gate while the
        # track drifts, so mid-range bearings read "centered" even when the crossing misses;
        # the true miss appears as bearing blow-up in the last metres (dist*sin(bearing) ->
        # the actual miss as range -> 0).
        fin = [m for m in meas[gi] if m[1] < 4.5]
        if len(fin) < 4:
            fin = [m for m in meas[gi] if m[1] < 6.5]
        if len(fin) < 5:
            continue
        latm = sorted(m[2] for m in fin)
        lat = latm[len(latm) // 2]
        zm = sorted(m[3] for m in fin)
        z = zm[len(zm) // 2]
        # dead zone 0.55: only correct REAL misses. Centering a gate that already passes
        # moves the spline at its neighbours and breaks them (gate 1, twice).
        if abs(lat) < 0.55:
            lat = 0.0
        if abs(z) < 0.55:
            z = None
        print(f"{gi:4d} {len(fin):4d}      {lat:+6.2f}"
              + (f"                              {z:+6.2f}" if z is not None else "        (z: too few close obs)"))
        sugg[gi] = (lat, z)

    # FRONTIER-ONLY: the race counter is ground truth. A gate the counter passed was PASSED -
    # its camera innovation is turn-approach bias (same passed gate read -0.65/+0.70/-1.14 on
    # three consecutive threading flights). Only the gate the flight died chasing gets corrected.
    frontier = max((int(r["race_gate"]) for r in rows
                    if str(r.get("race_gate", "")).isdigit()), default=None)
    lock_p = os.path.join(HERE, "pilots", "ace_pilot", "tape_lock.json")
    if apply and os.path.exists(lock_p):
        upto = json.load(open(lock_p)).get("upto", -1)
        if frontier is not None and frontier <= upto:
            # a banked gate was hit: launch-to-launch scatter, not a bias - correcting it
            # would chase noise on a gate the same bytes already passed
            print(f"\nfrontier {frontier} is inside the banked prefix (0..{upto}) - "
                  "variance hit, nothing applied")
            sugg = {}
    if apply and sugg:
        sugg = {gi: v for gi, v in sugg.items() if gi == frontier}
        if not sugg:
            print(f"\nfrontier gate {frontier}: no usable close-range measurement - nothing applied")
    if apply and sugg:
        m = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "course_map.json")))
        cds = {g["gate_id"]: g["cross_dir"] for g in m["gates"]}
        ro_p = os.path.join(HERE, "pilots", "ace_pilot", "race_offsets.json")
        ro = json.load(open(ro_p))
        protected = set(ro.get("protect_z", []))
        for gi, (lat, z) in sugg.items():
            cd = cds.get(gi)
            if cd is None:
                continue
            if str(gi) in protected:
                z = None            # intentional off-center crossing (leg lift) - hands off
            rx, ry = cd[1], -cd[0]
            cur = ro["gates"].get(str(gi), [0.0, 0.0, 0.0])
            # gate sits `lat` right of our track -> move the plan target right by 0.8*lat,
            # CAPPED at 1.0 m per flight per gate (one uncapped 3.7 m first-reading wrecked
            # the solve; big innovations need a confirming flight)
            step_l = max(-1.0, min(1.0, 0.8 * lat))
            step_z = max(-1.0, min(1.0, 0.8 * z)) if z is not None else 0.0
            nx = cur[0] + step_l * rx
            ny = cur[1] + step_l * ry
            nz = cur[2] - step_z
            ro["gates"][str(gi)] = [round(nx, 2), round(ny, 2), round(nz, 2)]
        ro.setdefault("_log", []).append(f"gate_innovations --apply from {os.path.basename(dbg_path)}")
        json.dump(ro, open(ro_p, "w"), indent=1)
        print("\napplied damped corrections:", {k: ro["gates"][str(k)] for k in sugg if str(k) in ro["gates"]})


if __name__ == "__main__":
    main()
