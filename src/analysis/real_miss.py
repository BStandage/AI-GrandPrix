"""
Measure REAL per-gate miss distance from a flight, WITHOUT needing the in-session track broadcast.

The course is deterministic (identical layout, fixed spawn origin), so we use the known gate layout
from any session's gates.json as ground truth and overlay this session's real telemetry trajectory
(odometry, race frame) on it. Reports the true closest approach of the flown path to each gate plane
and the in-plane miss direction. This is ground truth from real telemetry - no sim, no guessing.

Usage:
  python -m analysis.real_miss                 # newest session
  python -m analysis.real_miss <session_dir>
"""

import glob
import json
import math
import os
import sys

import numpy as np

from common.paths import DATASETS_DIR

HALF_OPENING = 0.75


def _known_gates():
    for f in sorted(glob.glob(os.path.join(DATASETS_DIR, "*", "gates.json"))):
        g = json.load(open(f)).get("gates")
        if g:
            return sorted(g, key=lambda x: x.get("gate_id", 0))
    raise SystemExit("no gates.json found anywhere in datasets/")


def _newest_session():
    s = sorted(glob.glob(os.path.join(DATASETS_DIR, "session_*")), key=os.path.getmtime)
    return s[-1] if s else None


def _race_path(session_dir):
    """Real trajectory in the race frame (the dominant/last odometry reset_counter)."""
    odo = []
    for line in open(os.path.join(session_dir, "telemetry.jsonl")):
        try:
            r = json.loads(line)
        except Exception:
            continue
        if r.get("kind") == "odometry" and r.get("x") is not None:
            odo.append(r)
    if not odo:
        return []
    # Pick the odometry frame the flight actually happened in: the reset_counter with the MOST
    # samples (a sim-reset at the end can leave a tiny stub at a higher counter - don't pick that).
    from collections import Counter
    counts = Counter(o.get("reset_counter", 0) for o in odo)
    rc = counts.most_common(1)[0][0]
    return [(o["x"], o["y"], o["z"]) for o in odo if o.get("reset_counter", 0) == rc]


def _seg_closest(a, b, g):
    a, b, g = np.array(a), np.array(b), np.array(g)
    d = b - a
    s = float(d.dot(d))
    t = 0.0 if s < 1e-9 else max(0.0, min(1.0, float((g - a).dot(d)) / s))
    c = a + t * d
    return float(np.linalg.norm(c - g)), c


def main(session_dir):
    gates = _known_gates()
    path = _race_path(session_dir)
    print(f"=== real_miss: {os.path.basename(session_dir)} ===")
    if len(path) < 2:
        print("no race-frame trajectory in telemetry")
        return
    alt = [-p[2] for p in path]
    print(f"trajectory: {len(path)} pts  N {min(p[0] for p in path):.0f}..{max(p[0] for p in path):.0f}  "
          f"alt {min(alt):.1f}..{max(alt):.1f}")
    along = np.array(gates[-1]["position_ned"]) - np.array(gates[0]["position_ned"])
    along[2] = 0
    along /= (np.linalg.norm(along) or 1.0)
    cross = np.array([-along[1], along[0], 0.0])
    print(f"\n{'gate':>4} {'pass':>5} {'miss_m':>7} {'dir':>16} {'gate_alt':>8}")
    passed = 0
    for ga in gates:
        gp = ga["position_ned"]
        best, bc = 1e9, None
        for i in range(len(path) - 1):
            d, c = _seg_closest(path[i], path[i + 1], gp)
            if d < best:
                best, bc = d, c
        e = np.array(bc) - np.array(gp)
        cr = float(e.dot(cross))
        ve = -e[2]
        ok = best <= HALF_OPENING * 1.4 and abs(cr) <= HALF_OPENING and abs(ve) <= HALF_OPENING
        passed += ok
        side = f"{'R' if cr >= 0 else 'L'}{abs(cr):.1f} {'UP' if ve >= 0 else 'DN'}{abs(ve):.1f}"
        print(f"{ga['gate_id']:>4} {'YES' if ok else 'NO ':>5} {best:>7.2f} {side:>16} {-gp[2]:>8.1f}")
    print(f"\nthreaded ~{passed}/{len(gates)}  (miss = closest approach of flown path to gate centre; "
          f"opening half-width {HALF_OPENING} m)")


if __name__ == "__main__":
    sd = sys.argv[1] if len(sys.argv) > 1 else _newest_session()
    main(sd)
