"""
GEOMETRIC gate-altitude map from vision PnP - zero dynamics model.

    python -m analysis.geo_z_map datasets/steady_dbg_20260727_180905.csv \
                                 datasets/session_20260727_180903

Principle: at the instant steady passes gate k, its altitude IS gate k's altitude (it flew
through the opening). In that frame the NEXT gate's PnP solution (pnp_fwd/right/down, metric,
from the known 1.5 m gate geometry) gives z(k+1) - z(k) as a single geometric measurement:

    dz = pnp_fwd * sin(el) - pnp_down * cos(el),   el = camera-axis elevation = UPTILT - pitch

Chained over the course: a z-profile with INDEPENDENT per-leg errors and no integration drift -
unlike every dynamics-based z we have had. Gate 0's absolute height comes from the pre-launch
pad frames (drone at rest at known height).
"""

import csv
import json
import math
import os
import sys

UPTILT = math.radians(20.0)


def main():
    dbg_path, sess = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(dbg_path)))
    # frame -> (pitch_cmd_rad, roll_cmd_rad, yaw_deg, gates_passed, t)
    by_frame = {}
    passes = []          # (gate_id_passed, frame_id, t)
    prev_g = 0
    for r in rows:
        fr = r["frame"]
        if fr not in ("", None):
            by_frame[int(fr)] = r
        g = int(r["gates_passed"])
        if g != prev_g:
            passes.append((g - 1, int(fr) if fr not in ("", None) else None, float(r["t"])))
            prev_g = g

    # dbg logs the session-local SEQ; vision_frames keys on the sim frame_id - frames.jsonl
    # carries the seq -> frame_id mapping
    seq2fid = {}
    for line in open(os.path.join(sess, "frames.jsonl")):
        d = json.loads(line)
        seq2fid[d["seq"]] = d["frame_id"]
    by_fid = {}
    for line in open(os.path.join(sess, "vision_frames.jsonl")):
        d = json.loads(line)
        by_fid[d["frame_id"]] = d
    frames = {seq: by_fid.get(fid) for seq, fid in seq2fid.items()}

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cmap = json.load(open(os.path.join(here, "pilots", "ace_pilot", "course_map.json")))
    gates = cmap["gates"]

    def det_dz_dxy(det, pitch_rad):
        el = UPTILT - pitch_rad
        dz = det["pnp_fwd"] * math.sin(el) - det["pnp_down"] * math.cos(el)
        fwd_h = det["pnp_fwd"] * math.cos(el) + det["pnp_down"] * math.sin(el)
        return dz, fwd_h, det["pnp_right"]

    # ---- absolute z of gate 0 from pre-launch frames (t < 0.5, drone at rest on the pad) ----
    z0_meas = []
    for r in rows:
        if float(r["t"]) > 0.5:
            break
        fr = r["frame"]
        if fr in ("", None) or int(fr) not in frames:
            continue
        f = frames[int(fr)]
        best = max(f["dets"], key=lambda d: d["area_frac"], default=None) if f["dets"] else None
        if best and best.get("pnp_fwd"):
            dz, _, _ = det_dz_dxy(best, math.radians(float(r["pitch_deg"])))
            z0_meas.append(dz)
    z0 = sorted(z0_meas)[len(z0_meas) // 2] if z0_meas else 0.0
    print(f"gate 0 absolute z above pad-camera: {z0:+.2f} m  ({len(z0_meas)} frames)")

    # ---- per-leg dz at each pass instant ----
    z = {0: z0}
    print("\nleg   dz_med  n   z_next   (spread p25..p75)")
    for gid, fr_pass, t_pass in passes:
        nxt = gid + 1
        if nxt >= len(gates) or fr_pass is None:
            continue
        # expected horizontal direction to the next gate (map xy is trusted)
        g_now, g_nxt = gates[gid], gates[nxt]
        exp_dx, exp_dy = g_nxt["x"] - g_now["x"], g_nxt["y"] - g_now["y"]
        exp_dist = math.hypot(exp_dx, exp_dy)
        meas = []
        for foff in range(0, 12):
            f = frames.get(fr_pass + foff)
            if f is None or not f["dets"]:
                continue
            # tick state nearest this frame
            r = by_frame.get(fr_pass + foff) or by_frame.get(fr_pass)
            pitch = math.radians(float(r["pitch_deg"]))
            for det in f["dets"]:
                if not det.get("pnp_fwd"):
                    continue
                dz, fwd_h, right = det_dz_dxy(det, pitch)
                horiz = math.hypot(fwd_h, right)
                if abs(horiz - exp_dist) < 0.35 * exp_dist:   # range-plausible = the next gate
                    meas.append(dz)
        if len(meas) >= 3:
            meas.sort()
            med = meas[len(meas) // 2]
            z[nxt] = z[gid] + med
            print(f"{gid:2d}->{nxt:2d} {med:+6.2f} {len(meas):3d}  {z[nxt]:+7.2f}  "
                  f"({meas[len(meas)//4]:+.2f}..{meas[3*len(meas)//4]:+.2f})")
        else:
            z[nxt] = z[gid]
            print(f"{gid:2d}->{nxt:2d}   ----  {len(meas):3d}  {z[nxt]:+7.2f}  (carry: no plausible dets)")

    print("\ngeometric z-profile vs current map z:")
    for g in gates:
        gid = g["gate_id"]
        if gid in z:
            print(f"  g{gid:2d}  geo {z[gid]:+6.2f}   map {g['z']:+6.2f}   diff {z[gid]-g['z']:+6.2f}")
    out = os.path.join(here, "pilots", "ace_pilot", "geo_z.json")
    json.dump({str(k): round(v, 3) for k, v in z.items()}, open(out, "w"), indent=1)
    print("\nwrote", out)


if __name__ == "__main__":
    main()
