"""
ONE-SHOT post-flight processing: run after every tape attempt.

    python -m analysis.attempt            # newest run: measure -> apply -> rebuild tape
    python -m analysis.attempt --dry      # measure and report only, apply nothing

Does, in order:
  1. finds the newest ace_dbg_*.csv + session_* pair
  2. prints the race's gate progression (how far the attempt got, with timestamps)
  3. gate_innovations --apply  (measured per-gate corrections, dead-zoned, damped)
  4. clears the model-iteration offsets and rebuilds the tape (iterate_tape)
Ends with "tape is flight-ready" when the rebuild converged - then fly again.
"""

import csv
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    dry = "--dry" in sys.argv
    dbg = max(glob.glob(os.path.join(HERE, "datasets", "ace_dbg_*.csv")), key=os.path.getmtime)
    sess = max(glob.glob(os.path.join(HERE, "datasets", "session_2026*")), key=os.path.getmtime)
    print(f"run:     {os.path.basename(dbg)}")
    print(f"session: {os.path.basename(sess)}")

    rows = list(csv.DictReader(open(dbg)))
    prev = None
    print("progress:")
    for r in rows:
        if r.get("race_gate") != prev:
            prev = r.get("race_gate")
            print(f"  race_gate -> {prev} at t={r['t']}")
    print(f"  flight span {rows[0]['t']} .. {rows[-1]['t']} s")

    args = [sys.executable, "-m", "analysis.gate_innovations", dbg, sess]
    if not dry:
        args.append("--apply")
    print("\n-- gate innovations --")
    subprocess.run(args, cwd=HERE)

    if dry:
        print("\n(dry run: nothing applied, tape unchanged)")
        return
    off = os.path.join(HERE, "pilots", "ace_pilot", "gate_offsets.json")
    if os.path.exists(off):
        os.remove(off)
    print("\n-- rebuilding tape --")
    subprocess.run([sys.executable, "-m", "analysis.iterate_tape", "8"], cwd=HERE)


if __name__ == "__main__":
    main()
