"""
Diagnose a vision flight from vision_frames.jsonl: WHY does the pilot miss gates?

Reads the rich per-frame dataset (live collector or analysis.vision_replay) and answers, per gate:
  * MISS:      true 3D distance at closest approach + the body-frame miss vector (passed how far
               left/right and high/low of the gate centre). This is "did we thread it".
  * SEEN:      fraction of approach frames (gate ahead, in FOV, < SEE_RANGE m) the detector
               actually found it - a low number means we were flying blind into it.
  * PnP error: median azimuth / elevation / range error while approaching (10-30 m band), i.e. how
               wrong the camera pose was when it still mattered for steering.
  * LOCK:      frames where the pilot's tracked target sat on the WRONG gate (locked onto a
               neighbour), which whips the strafe off the gate it should be threading.

Usage:
  python -m analysis.vision_diag                 # newest session
  python -m analysis.vision_diag <session_dir>
"""

import glob
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict

from common.paths import DATASETS_DIR

SEE_RANGE = 40.0      # consider a gate "on approach" within this distance, in front, in FOV
HALF_OPENING = 0.75   # gate inner opening is 1.5 m square -> half-width per axis


def _newest_with_frames():
    for sd in sorted(glob.glob(os.path.join(DATASETS_DIR, "session_*")), key=os.path.getmtime, reverse=True):
        if os.path.exists(os.path.join(sd, "vision_frames.jsonl")):
            return sd
    return None


def _med(xs):
    xs = [x for x in (xs or []) if x is not None]
    return st.median(xs) if xs else None


def _plane_crossing(series):
    """Given a per-frame body-frame series of (right, down, fwd, dist) for one gate, find where the
    drone crosses the gate plane (fwd: + ahead -> - behind) at its nearest pass, and linearly
    interpolate the (right, down) offset there. That offset vs the opening half-width is the true
    'did we go through the hole' test - frame-rate-robust and independent of the forward residual.
    Returns (right, down) at the crossing, or (None, None) if it never crossed in front."""
    s = [x for x in series if x is not None]
    if len(s) < 2:
        return None, None
    best = None                       # (dist_at_crossing, right, down)
    for a, b in zip(s, s[1:]):
        if a[2] > 0 >= b[2]:          # fwd goes + -> <=0: crossed the plane this step
            span = a[2] - b[2]
            t = a[2] / span if span > 1e-9 else 0.0
            rt = a[0] + (b[0] - a[0]) * t
            dn = a[1] + (b[1] - a[1]) * t
            dist = min(a[3], b[3])
            if best is None or dist < best[0]:
                best = (dist, rt, dn)
    if best is None:
        # never crossed in front (e.g. gate passed to the side / above): fall back to the min-|fwd|
        k = min(range(len(s)), key=lambda i: abs(s[i][2]))
        return s[k][0], s[k][1]
    return best[1], best[2]


def load(session_dir):
    recs = []
    for line in open(os.path.join(session_dir, "vision_frames.jsonl")):
        try:
            recs.append(json.loads(line))
        except Exception:
            pass
    return recs


def diagnose(session_dir):
    recs = load(session_dir)
    if not recs:
        print(f"no records in {session_dir}")
        return
    mode = recs[0].get("mode")
    n = len(recs)
    poses = [r for r in recs if r.get("pose")]
    print(f"\n=== vision_diag: {os.path.basename(session_dir)} ===")
    print(f"mode={mode}  frames={n}  with_pose={len(poses)}")
    if poses:
        speeds = [r["speed_h"] for r in recs if r.get("speed_h") is not None]
        alts = [r["pose"]["alt"] for r in poses]
        print(f"speed_h: median {_med(speeds):.1f}  max {max(speeds):.1f} m/s   "
              f"alt range {min(alts):.1f}..{max(alts):.1f} m")
    dets_per = [r["n_dets"] for r in recs]
    print(f"detections/frame: median {int(_med(dets_per) or 0)}  max {max(dets_per)}  "
          f"(0 dets on {sum(1 for d in dets_per if d == 0)}/{n} frames)")

    gate_ids = sorted({g["gate_id"] for r in recs for g in r.get("gt_gates", [])})

    # Authoritative pass/fail from the sim: it advances active_gate_index as gates are cleared.
    actives = [r["race"]["active_gate_index"] for r in recs
               if r.get("race") and r["race"].get("active_gate_index") is not None]
    max_active = max(actives) if actives else -1
    finished = any(r.get("race") and (r["race"].get("race_finish_time_ns") or -1) >= 0 for r in recs)
    passed_gids = {g for g in gate_ids if g < max_active}    # gid < current target = cleared
    if finished and max_active in gate_ids:
        passed_gids.add(max_active)                          # final gate if the index parks on it

    # gather per-gate info across the run
    # Per-gate time series over frames-with-pose: drone world pos + the gate's body-frame offset.
    # The true closest approach is the min distance from the gate to the flown PATH (segments
    # between samples), NOT the nearest sampled frame - at 11 m/s a 30 Hz sample sits up to ~0.2 m
    # off the real crossing, which alone reads as a 0.7 m "miss" on a dead-centre pass.
    dpath = []                          # world (x,y,z) per posed frame
    body = defaultdict(list)            # gid -> (right, down) per posed frame (parallel to dpath)
    gworld = {}                         # gid -> gate world pos
    approach_frames = defaultdict(int)
    seen_frames = defaultdict(int)
    pnp_az = defaultdict(list); pnp_el = defaultdict(list); pnp_rng = defaultdict(list)
    for r in recs:
        for g in r.get("gt_gates", []):
            gid = g["gate_id"]
            if g["fwd"] > 0 and g["in_fov"] and g["distance"] < SEE_RANGE:
                approach_frames[gid] += 1
                if g.get("detected"):
                    seen_frames[gid] += 1
        for d in r.get("dets", []):
            gid = d.get("match_gid")
            if gid is None:
                continue
            gt = next((g for g in r["gt_gates"] if g["gate_id"] == gid), None)
            if gt and 10.0 <= gt["distance"] <= 30.0:
                if d.get("az_err_deg") is not None: pnp_az[gid].append(d["az_err_deg"])
                if d.get("el_err_deg") is not None: pnp_el[gid].append(d["el_err_deg"])
                if d.get("range_err") is not None: pnp_rng[gid].append(d["range_err"])
        p = r.get("pose")
        gg = r.get("gt_gates") or []
        if p is None or not gg:           # need pose AND gates so dpath / body stay index-aligned
            continue                       # (pre-race frames have a pose but no track yet)
        dpath.append((p["x"], p["y"], p["z"]))
        by_gid = {g["gate_id"]: g for g in gg}
        for gid in gate_ids:
            g = by_gid.get(gid)
            body[gid].append((g["right"], g["down"], g["fwd"], g["distance"]) if g else None)
            if g:
                gworld.setdefault(gid, g["world_ned"])

    print(f"\n{'gate':>4} {'pass':>5} {'offset_m':>8} {'offset_dir':>22} {'seen%':>6} {'pnp_az°':>7} {'pnp_el°':>7} {'pnp_rng_m':>9}")
    for gid in gate_ids:
        rt, dn = _plane_crossing(body.get(gid, []))      # in-plane offset where body fwd -> 0
        if rt is None:
            side, off = "  -", float("nan")
        else:
            side = f"{'R' if rt >= 0 else 'L'}{abs(rt):.1f} {'DN' if dn >= 0 else 'UP'}{abs(dn):.1f}"
            off = math.hypot(rt, dn)
        ap = approach_frames.get(gid, 0)
        seenpct = (100.0 * seen_frames.get(gid, 0) / ap) if ap else 0.0
        az = _med(pnp_az.get(gid)); el = _med(pnp_el.get(gid)); rg = _med(pnp_rng.get(gid))
        ok = gid in passed_gids
        print(f"{gid:>4} {('YES' if ok else 'NO'):>5} {off:>8.2f} {side:>22} {seenpct:>5.0f}% "
              f"{('%+.1f'%az) if az is not None else '   -':>7} "
              f"{('%+.1f'%el) if el is not None else '   -':>7} "
              f"{('%+.1f'%rg) if rg is not None else '   -':>9}"
              f"{'' if ok else '   <== MISSED'}")
    print(f"\ngates passed (per sim active_gate_index): {len(passed_gids)}/{len(gate_ids)}"
          f"{'  - race FINISHED' if finished else ''}")
    print("offset = how far off centre the path crossed the gate plane (opening is 1.5 m square, "
          "half-width 0.75 m).")
    print("offset_dir = R/L = right/left, DN/UP = below/above gate centre.")
    print(f"seen% = approach frames (in FOV, <{SEE_RANGE:.0f}m) where the detector found the gate.")

    # wrong-gate lock: pilot's tracked image target far from the ACTIVE gate's projection
    wrong = 0; total = 0
    for r in recs:
        tg = r.get("ctrl", {}).get("vis_tracked_img")
        act = next((g for g in r.get("gt_gates", []) if g.get("is_active")), None)
        if tg is None or act is None or act["proj_offx"] is None:
            continue
        total += 1
        if math.hypot(tg[0] - act["proj_offx"], tg[1] - act["proj_offy"]) > 0.5:
            wrong += 1
    if total:
        print(f"\nwrong-gate lock: tracked target was >0.5 off the active gate on {wrong}/{total} "
              f"frames ({100.0*wrong/total:.0f}%).")


if __name__ == "__main__":
    sd = sys.argv[1] if len(sys.argv) > 1 else _newest_with_frames()
    if sd is None:
        print("no session with vision_frames.jsonl found (run analysis.vision_replay first, or fly "
              "with COLLECT_VISION_DATA on)")
    else:
        diagnose(sd)
