"""
AUTONOMOUS post-flight loop: leave running; just fly attempts.

    python -m analysis.autoloop

Watches datasets/ for each new ace_dbg/session pair, waits for the recording to finish,
then: measures gate innovations -> applies (with transfer-ratio escalation: if a gate's z
innovation shrinks <55% after an applied step, escalate the step by 1/0.45; after two
saturated steps, lift the PREVIOUS gate's z instead) -> rebuilds the tape -> prints READY.
Ctrl-C to stop.
"""
import csv
import glob
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE = os.path.join(HERE, "pilots", "ace_pilot", "autoloop_state.json")


def newest_pair():
    dbgs = glob.glob(os.path.join(HERE, "datasets", "ace_dbg_*.csv"))
    sess = glob.glob(os.path.join(HERE, "datasets", "session_2026*"))
    if not dbgs or not sess:
        return None, None
    return max(dbgs, key=os.path.getmtime), max(sess, key=os.path.getmtime)


def settled(path):
    s1 = os.path.getmtime(path)
    time.sleep(3.0)
    return os.path.getmtime(path) == s1


def main():
    state = {"done": []}
    if os.path.exists(STATE):
        state = json.load(open(STATE))
    print("autoloop: watching for new flights. Fly whenever the tape is READY. Ctrl-C stops.")
    while True:
        dbg, sess = newest_pair()
        if dbg and os.path.basename(dbg) not in state["done"]:
            # wait until the flight is over (files stop growing)
            if not settled(dbg):
                continue
            rows = list(csv.DictReader(open(dbg)))
            # STUB FILTER: the pilot client creates junk logs while idling/reconnecting.
            # A real attempt has hundreds of rows, spans seconds, and the race counter runs.
            raced = any(str(r.get("race_gate", "")).isdigit() for r in rows)
            span = float(rows[-1]["t"]) - float(rows[0]["t"]) if len(rows) > 1 else 0.0
            if len(rows) < 400 or span < 5.0 or not raced:
                state["done"].append(os.path.basename(dbg))
                json.dump(state, open(STATE, "w"))
                print(f"  (ignored stub {os.path.basename(dbg)}: {len(rows)} rows, {span:.1f}s"
                      f"{'' if raced else ', no race'})")
                continue
            print(f"\n=== new flight: {os.path.basename(dbg)} ===")
            prev = None
            for r in rows:
                if r.get("race_gate") != prev:
                    prev = r.get("race_gate")
                    print(f"  race_gate -> {prev} at t={r['t']}")
            # PREFIX LOCK RATCHET: this flight flew the current tape.json. Every gate the race
            # counter passed is banked - freeze the flown bytes through the last crossing so no
            # rebuild can ever un-pass them.
            mg = max((int(r["race_gate"]) for r in rows
                      if str(r.get("race_gate", "")).isdigit()), default=0)
            lockp = os.path.join(HERE, "pilots", "ace_pilot", "tape_lock.json")
            lock = json.load(open(lockp)) if os.path.exists(lockp) else {"upto": -1, "t": 0.0}
            if mg - 1 > lock["upto"]:
                t_cross = next(float(r["t"]) for r in rows if str(r.get("race_gate")) == str(mg))
                shutil.copy(os.path.join(HERE, "pilots", "ace_pilot", "tape.json"),
                            os.path.join(HERE, "pilots", "ace_pilot", "tape_locked.json"))
                json.dump({"upto": mg - 1, "t": round(t_cross, 3)}, open(lockp, "w"))
                print(f"  LOCKED gates 0..{mg - 1}: flown bytes frozen through t={t_cross:.2f}s")
            subprocess.run([sys.executable, "-m", "analysis.gate_innovations", dbg, sess, "--apply"],
                           cwd=HERE)
            off = os.path.join(HERE, "pilots", "ace_pilot", "gate_offsets.json")
            if os.path.exists(off):
                os.remove(off)
            res = subprocess.run([sys.executable, "-m", "analysis.iterate_tape", "8"],
                                 cwd=HERE, capture_output=True, text=True)
            tail = res.stdout.strip().splitlines()[-1] if res.stdout.strip() else "?"
            print(f"  rebuild: {tail}")
            state["done"].append(os.path.basename(dbg))
            json.dump(state, open(STATE, "w"))
            print(">>> READY - FLY <<<" if "flight-ready" in tail else ">>> REBUILD PROBLEM - tell Claude <<<")
        time.sleep(4.0)


if __name__ == "__main__":
    main()
