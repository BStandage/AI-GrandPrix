"""
Validate the perception -> racing-line estimate against ground truth, from a logged session's
vision_frames.jsonl (which carries per-detection PnP AND truth). Answers the only question that
matters before we fly: can we estimate the gate positions well enough to spline a line through them?

It runs the REAL GateMapper (perception/gate_map.py) two ways so we can separate the error sources:
  TRUE-pose   : place gates with the true drone pose + PnP  -> isolates PERCEPTION/PnP error.
  DEADRECKON  : place gates with the integrated local pose  -> adds the dead-reckoning (VIO) drift,
                i.e. the realistic live estimate.

For each, every estimated gate is matched to the nearest true gate and the position error reported.

Usage:
  python -m analysis.vision_map                 # newest session with vision_frames.jsonl
  python -m analysis.vision_map <session_dir>
"""

import glob
import json
import math
import os
import sys

from common.paths import DATASETS_DIR
from common.gate_geometry import quat_to_rotmat
from perception.gate_map import GateMapper, LocalFrame


def _newest():
    for sd in sorted(glob.glob(os.path.join(DATASETS_DIR, "session_*")), key=os.path.getmtime, reverse=True):
        if os.path.exists(os.path.join(sd, "vision_frames.jsonl")):
            return sd
    return None


def _true_gates(recs):
    """gate_id -> true world NED, from the logged ground truth (any frame that has it)."""
    out = {}
    for r in recs:
        for g in r.get("gt_gates", []):
            if g.get("world_ned") and g["gate_id"] not in out:
                out[g["gate_id"]] = g["world_ned"]
    return out


def _run(recs, use_true_pose):
    mapper = GateMapper()
    lf = LocalFrame()
    anchored = False
    prev_t = None
    start_pos = None
    for r in recs:
        pose = r.get("pose")
        if pose is None:
            continue
        quat = (pose["qw"], pose["qx"], pose["qy"], pose["qz"])
        # integrate the local frame from body velocity (dead reckoning)
        t = r.get("sim_time_ns")
        if prev_t is not None and t is not None:
            dt = max(0.0, (t - prev_t) * 1e-9)
            if dt < 0.5:
                lf.update(quat, (pose["vx_body"], pose["vy_body"], pose["vz_body"]), dt)
        prev_t = t
        if not anchored:                       # anchor both frames at the first posed frame
            start_pos = [pose["x"], pose["y"], pose["z"]]
            lf.pos = list(start_pos)
            anchored = True
        pos = [pose["x"], pose["y"], pose["z"]] if use_true_pose else lf.pos
        for d in r.get("dets", []):
            if "pnp_fwd" not in d:
                continue
            mapper.observe(pos, quat, (d["pnp_fwd"], d["pnp_right"], d["pnp_down"]), d.get("pnp_dist"))
    return mapper.estimate()


def _report(label, est, truth):
    print(f"\n--- {label}: {len(est)} gates estimated (truth has {len(truth)}) ---")
    print(f"{'est#':>4} {'n_obs':>5} {'matched':>7} {'pos_err_m':>9} {'lat_err':>7} {'vert_err':>8}")
    used = set()
    errs = []
    for i, g in enumerate(est):
        # nearest true gate
        gid, gt = min(truth.items(), key=lambda kv: math.dist(kv[1], g["pos"]))
        err = math.dist(gt, g["pos"])
        lat = math.hypot(g["pos"][0] - gt[0], g["pos"][1] - gt[1])
        vert = abs(g["pos"][2] - gt[2])
        errs.append(err)
        dup = " (dup)" if gid in used else ""
        used.add(gid)
        print(f"{i:>4} {g['n']:>5} {('g%d' % gid):>7} {err:>9.2f} {lat:>7.2f} {vert:>8.2f}{dup}")
    miss = sorted(set(truth) - used)
    if errs:
        print(f"median pos err {sorted(errs)[len(errs)//2]:.2f} m   max {max(errs):.2f} m   "
              f"matched {len(used)}/{len(truth)}" + (f"   MISSED {miss}" if miss else ""))


def main(session_dir):
    recs = [json.loads(l) for l in open(os.path.join(session_dir, "vision_frames.jsonl"))]
    truth = _true_gates(recs)
    print(f"=== vision_map: {os.path.basename(session_dir)}  ({len(recs)} frames, {len(truth)} true gates) ===")
    if not truth:
        print("no ground truth in this session - can't validate (was the track broadcast received?)")
    _report("TRUE-pose + PnP  (perception error only)", _run(recs, True), truth)
    _report("DEADRECKON + PnP (realistic live estimate)", _run(recs, False), truth)


if __name__ == "__main__":
    sd = sys.argv[1] if len(sys.argv) > 1 else _newest()
    if sd is None:
        print("no session with vision_frames.jsonl found")
    else:
        main(sd)
