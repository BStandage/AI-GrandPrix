"""
Perception accuracy vs ground truth - the "is the camera-computed gate good enough?" check.

Reads a vision_frames.jsonl (written by VisionDataCollector on a TELEMETRY run - VQ1/training - where
ground truth is available) and reports the two numbers that decide whether a vision pilot can ever
complete the course:

  1. DETECTION RATE  - of the gates that are actually in front of the drone AND inside the camera FOV,
     what fraction did the detector find? (a low rate = the pilot goes blind = can't chain gates)
  2. LOCALISATION ERROR - for the gates it DID detect, how far off is the computed bearing / elevation
     / range / 3D position from the true relative gate position?

Ground truth is logged for analysis only; the live pilot never reads it. This is pure post-processing
- no sim needed:

    python analysis/perception_accuracy.py [vision_frames.jsonl | session_dir]

With no argument it picks the most recent session whose log actually contains ground truth (a VQ2 run
has no pose, so no truth, and is skipped).
"""

import json
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.paths import DATASETS_DIR


def _load(path):
    return [json.loads(l) for l in open(path) if l.strip()]


def _has_truth(path):
    """True if the log has at least one GT-matched detection (i.e. it was a telemetry run)."""
    try:
        for l in open(path):
            if '"az_err_deg"' in l:
                return True
    except OSError:
        pass
    return False


def _find_latest_with_truth():
    if not os.path.isdir(DATASETS_DIR):
        return None
    sess = sorted((d for d in os.listdir(DATASETS_DIR) if d.startswith("session_")), reverse=True)
    for d in sess:
        p = os.path.join(DATASETS_DIR, d, "vision_frames.jsonl")
        if os.path.exists(p) and _has_truth(p):
            return p
    return None


def _stat_line(name, xs, unit):
    if not xs:
        print(f"  {name:12s} no data")
        return
    xs = sorted(xs)
    n = len(xs)
    print(f"  {name:12s} n={n:5d}  median={st.median(xs):7.2f}  mean={st.mean(xs):7.2f}  "
          f"p90={xs[int(0.9 * n)]:7.2f}  max={xs[-1]:7.2f} {unit}")


def analyze(path):
    rows = _load(path)
    print(f"Perception accuracy report: {path}")
    print(f"frames: {len(rows)}\n")

    az, el, rng, pos = [], [], [], []
    for r in rows:
        for d in r.get("dets", []):
            if d.get("az_err_deg") is None:
                continue
            az.append(abs(d["az_err_deg"]))
            el.append(abs(d["el_err_deg"]))
            rng.append(abs(d["range_err"]))
            pos.append(d["pos_err"])

    print("=== LOCALISATION ERROR vs ground truth (matched detections) ===")
    _stat_line("azimuth", az, "deg")
    _stat_line("elevation", el, "deg")
    _stat_line("range", rng, "m")
    _stat_line("pos (3D)", pos, "m")
    print()

    # DETECTION RATE. The flat "all in-FOV gates" rate is misleading - it counts gates 100m+ away that
    # are a few pixels and can't reasonably be detected. What matters is (a) the ACTIVE gate we're flying
    # to, and (b) how the rate falls off with range. Report both.
    act_fov = act_det = 0
    by_range = {}                       # 10m bucket -> [in_fov, detected]
    for r in rows:
        for g in r.get("gt_gates", []):
            if not g.get("in_fov"):
                continue
            b = int(g.get("distance", 0.0) // 10) * 10
            slot = by_range.setdefault(b, [0, 0])
            slot[0] += 1
            hit = 1 if g.get("detected") else 0
            slot[1] += hit
            if g.get("is_active"):
                act_fov += 1
                act_det += hit
    act_rate = 100.0 * act_det / act_fov if act_fov else 0.0
    print("=== DETECTION RATE ===")
    print(f"  ACTIVE gate (the one being flown): in-FOV={act_fov}  detected={act_det}  "
          f"rate = {act_rate:.1f}%")
    print("  by range (in-FOV GT gates):")
    for b in sorted(by_range):
        i, d = by_range[b]
        print(f"    {b:3d}-{b + 10:3d}m: {d:4d}/{i:4d}  = {100.0 * d / max(i, 1):5.1f}%")
    print()

    # localisation error by true range (does it fall apart with distance?)
    buckets = {}
    for r in rows:
        for d in r.get("dets", []):
            if d.get("pos_err") is None or d.get("pnp_dist") is None:
                continue
            b = int(d["pnp_dist"] // 5) * 5
            buckets.setdefault(b, []).append(d["pos_err"])
    print("=== pos error by (estimated) range ===")
    for b in sorted(buckets):
        xs = buckets[b]
        print(f"  {b:3d}-{b + 5:3d}m: n={len(xs):4d}  median_pos_err={st.median(xs):6.2f} m")
    print()

    # verdict
    print("=== VERDICT ===")
    problems = []
    if act_rate < 80:
        problems.append(f"active-gate detection {act_rate:.0f}% (need ~90%+ to chain gates without going blind)")
    if rng and st.median(rng) > 3.0:
        problems.append(f"range error median {st.median(rng):.1f} m (unusable for range-based control)")
    if az and st.median(az) > 5.0:
        problems.append(f"bearing error median {st.median(az):.1f} deg")
    if problems:
        print("  PERCEPTION IS THE BLOCKER:")
        for p in problems:
            print(f"   - {p}")
    else:
        print("  Perception looks good enough - the blocker is elsewhere (control/guidance).")
    return {"rate": rate, "az": az, "el": el, "rng": rng, "pos": pos}


def main():
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        path = arg if arg.endswith(".jsonl") else os.path.join(arg, "vision_frames.jsonl")
    else:
        path = _find_latest_with_truth()
        if not path:
            print("No vision_frames.jsonl with ground truth found. Run a VQ1/training lap "
                  "(telemetry on) with COLLECT_VISION_DATA=True.", flush=True)
            return 1
    if not os.path.exists(path):
        print(f"not found: {path}", flush=True)
        return 1
    analyze(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
