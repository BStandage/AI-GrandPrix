"""
ONE-COMMAND post-flight instrument readout - run after every attempt:

    python -m analysis.postflight

Prints, for the newest real flight:
  1. gate progression (counter ticks = crossing TIMES; not pass truth)
  2. camera innovations table (final-approach, the measuring tape for corrections)
  3. first-contact fix (IMU spike -> time + planned-position world fix)
No modifications are made. Corrections are sized from section 2, one damped step
per flight, and Brian's eyes veto. Adjectives never size corrections.
"""
import csv
import glob
import json
import math
import os
import subprocess
import sys

csv.field_size_limit(10 ** 8)
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def newest_flight():
    for d in sorted(glob.glob(os.path.join(HERE, "datasets", "ace_dbg_*.csv")),
                    key=os.path.getmtime, reverse=True):
        try:
            rows = list(csv.DictReader(open(d)))
        except Exception:
            continue
        if len(rows) > 400 and any(str(r.get("race_gate", "")).isdigit() for r in rows):
            return d, rows
    return None, None


def main():
    dbg, rows = newest_flight()
    ts = os.path.basename(dbg)[8:-4]
    sess = max((s for s in glob.glob(os.path.join(HERE, "datasets", "session_2026*"))
                if os.path.basename(s)[8:] <= ts), key=os.path.getmtime)
    print(f"flight {os.path.basename(dbg)}  session {os.path.basename(sess)}")

    print("\n-- 1. progression (tick = crossing time; NOT pass truth) --")
    prev = None
    for r in rows:
        if r.get("race_gate") != prev:
            prev = r.get("race_gate")
            if prev and prev.isdigit():
                print(f"   race_gate -> {prev} at t={float(r['t']):.2f}")
    print(f"   log ends t={float(rows[-1]['t']):.2f}")

    print("\n-- 2. camera innovations (the measuring tape) --")
    subprocess.run([sys.executable, "-m", "analysis.gate_innovations", dbg, sess], cwd=HERE)

    print("\n-- 3. first contact --")
    go = None
    imu = []
    for line in open(os.path.join(sess, "telemetry.jsonl")):
        r = json.loads(line)
        if r.get("kind") == "race_status" and go is None:
            st = r.get("race_start_boot_time_ms", -1)
            if st is not None and st >= 0 and r.get("sim_boot_time_ms", 0) >= st:
                go = r["recv_time_ns"]
        if r.get("kind") == "highres_imu":
            imu.append(r)
    hits = []
    for r in imu:
        a = math.sqrt(r["xacc"] ** 2 + r["yacc"] ** 2 + r["zacc"] ** 2)
        if a > 120:
            t = (r["recv_time_ns"] - go) * 1e-9
            if not hits or t - hits[-1][0] > 1.0:
                hits.append((t, a))
    if not hits:
        print("   no contact detected")
    else:
        subprocess.run([sys.executable, "-m", "analysis.sim_ace", "--truthstate", "--launchlag",
                        "0.4", "--tmax", str(hits[0][0] + 3), "--prefix",
                        "pilots/ace_pilot/tape.json", "--splicet", "99",
                        "--dump-trail", "pf_trail.json", "--tape-out", "NUL"],
                       cwd=HERE, capture_output=True)
        trail = json.load(open(os.path.join(HERE, "pf_trail.json")))
        for t, a in hits[:3]:
            p = min(trail, key=lambda r: abs(r[0] - t))
            print(f"   t={t:.2f}  |a|={a:.0f}  planned pos ({p[1]:+.1f},{p[2]:+.1f}) z={p[3]:.1f}")


if __name__ == "__main__":
    main()
