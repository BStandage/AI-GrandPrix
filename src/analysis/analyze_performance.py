#
# Analyze a characterization CSV (from CONTROL_MODE="characterize") into a
# performance envelope: hover thrust, climb/descent rate vs thrust, vertical
# acceleration, forward speed vs lean angle, and max body rates + response lag.
#
# Usage:
#   python -m analysis.analyze_performance [path_to_characterize_*.csv]
# (defaults to the newest characterize_*.csv in datasets/)
#

import csv
import glob
import math
import os
import sys

# Allow running this file directly (python analysis/analyze_performance.py) by putting
# the src/ package root on sys.path, not just this subpackage's folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.paths import DATASETS_DIR


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            for k, v in r.items():
                if k != "phase":
                    r[k] = float(v)
            rows.append(r)
    return rows


def phase_rows(rows, label):
    return [r for r in rows if r["phase"] == label]


def steady(rows, label, key, tail_frac=0.5):
    """Mean of `key` over the last tail_frac of a phase (steady state)."""
    rs = phase_rows(rows, label)
    if not rs:
        return None
    rs = rs[int(len(rs) * (1 - tail_frac)):]
    return sum(r[key] for r in rs) / len(rs)


def peak(rows, label, key, fn=max):
    rs = phase_rows(rows, label)
    return fn((r[key] for r in rs), default=None) if rs else None


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        cands = sorted(glob.glob(os.path.join(DATASETS_DIR, "characterize_*.csv")))
        if not cands:
            print("No characterize_*.csv found in datasets/. Run CONTROL_MODE='characterize' first.")
            return
        path = cands[-1]

    rows = load(path)
    print(f"Analyzing {os.path.basename(path)}  ({len(rows)} samples)\n")

    # --- vertical: climb rate (up = -vz) vs commanded thrust ---
    print("THRUST -> STEADY CLIMB RATE (up, m/s)")
    thr_pts = []
    for label in ["thr_0.00", "thr_0.20", "thr_0.35", "thr_0.50", "thr_0.75", "thr_1.00"]:
        thr = steady(rows, label, "cmd_thrust")
        vz = steady(rows, label, "vz")
        if thr is None:
            continue
        climb = -vz
        thr_pts.append((thr, climb))
        print(f"  thrust={thr:.2f}  climb={climb:+.2f} m/s")

    # hover thrust = where climb crosses 0 (linear interp between bracketing points)
    hover = None
    s = sorted(thr_pts)
    for (t0, c0), (t1, c1) in zip(s, s[1:]):
        if (c0 <= 0 <= c1) or (c1 <= 0 <= c0):
            hover = t0 + (t1 - t0) * (0 - c0) / (c1 - c0) if c1 != c0 else t0
            break
    print(f"\n  hover thrust  ~ {hover:.3f}" if hover else "\n  hover thrust  : (not bracketed)")
    mc = max((c for _, c in thr_pts), default=None)
    md = min((c for _, c in thr_pts), default=None)
    print(f"  max climb     ~ {mc:+.2f} m/s (at full thrust)")
    print(f"  max descent   ~ {md:+.2f} m/s (at zero thrust)")

    # --- vertical acceleration: peak dvz/dt right after the full-thrust step ---
    full = phase_rows(rows, "thr_1.00")
    if len(full) > 3:
        accs = []
        for a, b in zip(full, full[1:]):
            dt = b["t_s"] - a["t_s"]
            if dt > 0:
                accs.append(-(b["vz"] - a["vz"]) / dt)
        if accs:
            print(f"  peak vert accel ~ {max(accs):+.1f} m/s^2 (full thrust)")

    # --- horizontal: forward speed vs lean angle ---
    print("\nLEAN ANGLE -> STEADY FORWARD SPEED")
    for deg, label in [(10, "pitch_10"), (20, "pitch_20"), (30, "pitch_30")]:
        sp = steady(rows, label, "speed_h")
        if sp is not None:
            print(f"  pitch={deg:2d}deg  speed={sp:.2f} m/s")
    top = peak(rows, "pitch_30", "speed_h")
    if top is not None:
        print(f"  top speed seen ~ {top:.2f} m/s (at 30deg)")

    # --- rotational: max achieved body rates vs the 6.0 rad/s command ---
    print("\nMAX BODY RATES (commanded 6.0 rad/s)")
    rr = peak(rows, "rollrate", "rollspeed", fn=lambda xs, default=None: max(xs, key=abs, default=default))
    yr = peak(rows, "yawrate", "yawspeed", fn=lambda xs, default=None: max(xs, key=abs, default=default))
    if rr is not None:
        print(f"  max roll rate ~ {abs(rr):.2f} rad/s ({math.degrees(abs(rr)):.0f} deg/s)")
    if yr is not None:
        print(f"  max yaw rate  ~ {abs(yr):.2f} rad/s ({math.degrees(abs(yr)):.0f} deg/s)")

    print("\nUse these to set an aggressive controller: real hover thrust, climb/descent")
    print("authority, cruise speed per lean angle, and the rate limits the airframe can hit.")


if __name__ == "__main__":
    main()
