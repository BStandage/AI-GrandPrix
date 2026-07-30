"""
TAPE-FLIGHT ALTIMETRY: how high did the real drone fly relative to each gate it approached?

    python -m analysis.tape_altimetry datasets/ace_dbg_<tape run>.csv datasets/session_<same>

Uses the flight's own recorded detections: for frames where the upcoming gate's OPENING is
located at moderate range, elevation (bias-calibrated, +3.70 deg from the steady-session
gate-line calibration) x range = drone height ABOVE(-)/BELOW(+) the gate line. This is the
per-gate feedback channel tape flights otherwise lack.
Outputs suggested race_offsets z entries (negative = plan should go lower).
"""

import csv
import json
import math
import os
import sys

from common.camera import HALF_TAN_X, HALF_TAN_Y

UPTILT = math.radians(20.0)
EL_BIAS = math.radians(3.70)


def main():
    dbg_path, sess = sys.argv[1], sys.argv[2]
    rows = [r for r in csv.DictReader(open(dbg_path))]
    # tape dbg: t, race_gate, yaw_deg, pitch_deg per tick (no frame column in tape mode) -
    # align by TIME: session frames carry sim_time_ns; dbg t is race-relative. Use race_gate
    # transitions + frame ordering: simpler - per frame, find nearest dbg row by fraction of
    # total duration (both streams cover the same flight span).
    t_dbg = [float(r["t"]) for r in rows]
    dur_dbg = t_dbg[-1] - t_dbg[0]

    frames = []
    for line in open(os.path.join(sess, "vision_frames.jsonl")):
        d = json.loads(line)
        frames.append(d)
    t0_ns = frames[0]["sim_time_ns"]
    dur_ns = frames[-1]["sim_time_ns"] - t0_ns

    def dbg_at(frac):
        i = min(int(frac * len(rows)), len(rows) - 1)
        return rows[i]

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    gates = json.load(open(os.path.join(here, "pilots", "ace_pilot", "course_map.json")))["gates"]

    # per gate: collect drone-height-relative-to-gate measurements while approaching it
    meas = {g["gate_id"]: [] for g in gates}
    for f in frames:
        if not f.get("dets"):
            continue
        frac = (f["sim_time_ns"] - t0_ns) / max(dur_ns, 1)
        r = dbg_at(frac)
        try:
            pitch = math.radians(float(r["pitch_deg"]))
            yaw = math.radians(float(r["yaw_deg"]))
        except (ValueError, KeyError):
            continue
        # attribute by PLAN schedule (race_gate stalls when gate 0 is missed and then every
        # detection pollutes gate 0's statistics)
        gi = min(int(frac * len(gates) * 1.15), len(gates) - 1)
        for det in f["dets"]:
            dist = det.get("pnp_dist") or det.get("pinhole_dist")
            if not dist or not (3.0 < dist < 14.0):
                continue
            if not (det.get("has_opening") and det.get("bbox")):
                continue
            bx, by, bw, bh = det["bbox"]
            ox = ((bx + bw / 2.0) - 320.0) / 320.0
            oy = ((by + bh / 2.0) - 180.0) / 180.0
            el = (UPTILT - pitch) - math.atan(oy * HALF_TAN_Y) - EL_BIAS
            # crude gate association: bearing within 25 deg of dead ahead = the gate we chase
            if abs(math.atan(ox * HALF_TAN_X)) > math.radians(25):
                continue
            dz_gate_above_drone = dist * math.sin(el)
            meas[gi].append(-dz_gate_above_drone)   # + = drone ABOVE the gate line
    print("gate  n    drone-above-gate (median, p25..p75)")
    sugg = {}
    for gid in sorted(meas):
        m = sorted(meas[gid])
        if len(m) < 5:
            continue
        med = m[len(m) // 2]
        print(f"{gid:4d} {len(m):4d}   {med:+6.2f}  ({m[len(m)//4]:+.2f}..{m[3*len(m)//4]:+.2f})")
        sugg[gid] = round(-med, 2)
    print("\nsuggested race_offsets z (subtract the excess):", sugg)


if __name__ == "__main__":
    main()
