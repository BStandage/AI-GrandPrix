"""
VISION-METRIC course map: gate positions chained from PnP measurements at pass anchors.
NO dynamics model anywhere - every distance is metric from the known 1.5 m gate geometry.

    python -m analysis.pnp_map datasets/steady_dbg_20260727_180905.csv \
                               datasets/session_20260727_180903

Principle: when steady passes gate k its position IS gate k's position. In the frames just
after, every detection gives a metric 3D vector (PnP dist + image angles + logged commanded
attitude/heading) to a neighbouring gate. gate_{k+1} = gate_k + median(vector). The old
dynamics-integrated map had the right SHAPE but was ~1.4x too small (PnP: spawn->g0 = 10.2 m
horizontal vs map 6.8) - the depth squish seen in every tape attempt. Angles survive uniform
scaling, so the old map is still used to MATCH detections to gates (by direction only).
"""

import csv
import json
import math
import os
import sys

from common.camera import HALF_TAN_X, HALF_TAN_Y

UPTILT = math.radians(20.0)


def main():
    dbg_path, sess = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(dbg_path)))
    by_seq = {}
    passes = []
    prev_g = 0
    for r in rows:
        fr = r["frame"]
        if fr not in ("", None):
            by_seq[int(fr)] = r
        g = int(r["gates_passed"])
        if g != prev_g:
            passes.append((g - 1, int(fr) if fr not in ("", None) else None))
            prev_g = g

    seq2fid = {}
    for line in open(os.path.join(sess, "frames.jsonl")):
        d = json.loads(line)
        seq2fid[d["seq"]] = d["frame_id"]
    by_fid = {}
    for line in open(os.path.join(sess, "vision_frames.jsonl")):
        d = json.loads(line)
        by_fid[d["frame_id"]] = d

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    old = json.load(open(os.path.join(here, "pilots", "ace_pilot", "course_map.json")))
    gates_old = old["gates"]

    def det_vec(det, r):
        """World-frame metric vector camera->OPENING from PnP dist + image angles + commands.
        Angles come from the OPENING bbox when located - det offx/offy is the STRUCTURE centre,
        which sits ~1 m above the hole on the tall gates: building the map from it put every
        gate half a gate too high (attempts 4-5, Brian's consistent observation)."""
        dist = det.get("pnp_dist") or det.get("pinhole_dist")
        if not dist:
            return None
        if det.get("has_opening") and det.get("bbox"):
            bx, by, bw, bh = det["bbox"]
            ox = ((bx + bw / 2.0) - 320.0) / 320.0
            oy = ((by + bh / 2.0) - 180.0) / 180.0
        else:
            return None                      # structure-only detections do not locate the hole
        psi = math.radians(float(r["yaw_deg"]))
        pitch = math.radians(float(r["pitch_deg"]))
        bear = psi - math.atan(ox * HALF_TAN_X)            # +CCW; +offx = right
        el = (UPTILT - pitch) - math.atan(oy * HALF_TAN_Y) - det_vec.el_bias
        return (dist * math.cos(el) * math.cos(bear),
                dist * math.cos(el) * math.sin(bear),
                dist * math.sin(el))

    def collect(seq0, n_frames, exp_dir, ang_tol=0.35, max_dist=None):
        """Median metric vector over frames whose direction matches exp_dir (angle-only match)."""
        vx, vy, vz = [], [], []
        for off in range(n_frames):
            fid = seq2fid.get(seq0 + off)
            f = by_fid.get(fid) if fid else None
            r = by_seq.get(seq0 + off) or by_seq.get(seq0)
            if not f or not f["dets"] or r is None:
                continue
            for det in f["dets"]:
                v = det_vec(det, r)
                if v is None:
                    continue
                h = math.hypot(v[0], v[1])
                if h < 2.0:
                    continue
                ang = math.atan2(v[1], v[0])
                d_ang = (ang - exp_dir + math.pi) % (2 * math.pi) - math.pi
                if abs(d_ang) < ang_tol and (max_dist is None or h < max_dist):
                    vx.append(v[0]); vy.append(v[1]); vz.append(v[2])
        if len(vx) < 3:
            return None, 0
        vx.sort(); vy.sort(); vz.sort()
        m = len(vx) // 2
        return (vx[m], vy[m], vz[m]), len(vx)

    det_vec.el_bias = 0.0
    # ---- SELF-CALIBRATION of the elevation bias: just before each pass steady IS at the
    # gate line (its row servo converged), so the tracked opening's elevation must read ZERO.
    # The median of what it reads instead is the systematic bias (uptilt/pitch reference) that
    # was ramping the chained z by ~+0.5 m per leg.
    el_meas = []
    for gid, seq_pass in passes:
        if seq_pass is None:
            continue
        for off in range(-25, -6):
            fid = seq2fid.get(seq_pass + off)
            f = by_fid.get(fid) if fid else None
            r = by_seq.get(seq_pass + off)
            if not f or not f["dets"] or r is None:
                continue
            best = max(f["dets"], key=lambda d: d["area_frac"])
            if not (0.06 <= best["area_frac"] <= 0.35) or not best.get("has_opening"):
                continue
            bx, by, bw, bh = best["bbox"]
            oy = ((by + bh / 2.0) - 180.0) / 180.0
            pitch = math.radians(float(r["pitch_deg"]))
            el_meas.append((UPTILT - pitch) - math.atan(oy * HALF_TAN_Y))
    el_meas.sort()
    EL_BIAS = el_meas[len(el_meas) // 2] if el_meas else 0.0
    print(f"elevation bias (should be 0 at the gate line): {math.degrees(EL_BIAS):+.2f} deg "
          f"({len(el_meas)} near-pass frames) - subtracted from all measurements")

    det_vec.el_bias = EL_BIAS
    # spawn -> gate 0 from the pad frames (drone at rest; direction = spawn facing)
    psi0 = math.radians(97.1)
    v0, n0 = collect(1, 60, psi0, ang_tol=0.5, max_dist=14.0)
    pos = {0: v0 if v0 else (gates_old[0]["x"], gates_old[0]["y"], gates_old[0]["z"])}
    print(f"spawn->g0: {v0} ({n0} dets)")

    # chain at pass anchors; direction prior from the OLD map (shape/angles are trusted)
    for gid, seq_pass in passes:
        nxt = gid + 1
        if nxt >= len(gates_old) or seq_pass is None or gid not in pos:
            continue
        go, gn = gates_old[gid], gates_old[nxt]
        exp_dir = math.atan2(gn["y"] - go["y"], gn["x"] - go["x"])
        old_leg_d = math.hypot(gn["x"] - go["x"], gn["y"] - go["y"])
        v, n = collect(seq_pass, 12, exp_dir, max_dist=2.0 * old_leg_d)
        if v is None:
            # fall back: old-map leg scaled by the MEDIAN measured scale (the old map is
            # uniformly ~1.3x small; carrying it unscaled kinks the chain)
            sx = 1.31
            pos[nxt] = (pos[gid][0] + (gn["x"] - go["x"]) * sx,
                        pos[gid][1] + (gn["y"] - go["y"]) * sx,
                        pos[gid][2] + (gn["z"] - go["z"]))
            print(f"g{gid}->g{nxt}: NO MEASUREMENT - carried old leg x{sx}")
        else:
            pos[nxt] = (pos[gid][0] + v[0], pos[gid][1] + v[1], pos[gid][2] + v[2])
            old_leg = math.hypot(gn["x"] - go["x"], gn["y"] - go["y"])
            new_leg = math.hypot(v[0], v[1])
            print(f"g{gid}->g{nxt}: leg {new_leg:5.1f} m (old {old_leg:5.1f}, x{new_leg/max(old_leg,0.1):.2f}) "
                  f"dz {v[2]:+5.2f}  ({n} dets)")

    # write the metric map (cross_dir recomputed from the new positions)
    out_gates = []
    for g in gates_old:
        gid = g["gate_id"]
        if gid not in pos:
            continue
        p = pos[gid]
        p_prev = pos.get(gid - 1)
        p_next = pos.get(gid + 1)
        a = p_prev if p_prev else p
        b = p_next if p_next else p
        dx, dy = b[0] - a[0], b[1] - a[1]
        h = math.hypot(dx, dy) or 1.0
        out_gates.append({"gate_id": gid, "x": round(p[0], 2), "y": round(p[1], 2),
                          "z": round(p[2], 2), "cross_dir": [round(dx / h, 3), round(dy / h, 3)],
                          "heading_deg": round(math.degrees(math.atan2(dy, dx)), 1),
                          "v_pass": g.get("v_pass", 2.0), "t_pass": g.get("t_pass", 0)})
    out = {"source": "PNP-METRIC (analysis/pnp_map.py) - vision-chained, no dynamics",
           "frame": old["frame"], "gates": out_gates, "path": old.get("path", [])[:2]}
    out_path = os.path.join(here, "pilots", "ace_pilot", "course_map.json")
    json.dump(out, open(out_path, "w"))
    print(f"\nwrote {out_path}: {len(out_gates)} gates")
    for g in out_gates:
        print(f"  g{g['gate_id']:2d} ({g['x']:+8.2f}, {g['y']:+8.2f}, {g['z']:+6.2f}) hdg {g['heading_deg']:+6.1f}")


if __name__ == "__main__":
    main()
