"""Open-loop tape iteration: fly the offline sim, nudge each gate's waypoint by its measured
miss, re-solve, repeat. The identical loop runs against the REAL sim using race feedback.

    python -m analysis.iterate_tape [rounds]
"""
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OFF = os.path.join(HERE, "pilots", "ace_pilot", "gate_offsets.json")
LOCK = os.path.join(HERE, "pilots", "ace_pilot", "tape_lock.json")


def run_sim(lock=None):
    # FITTED DYNAMICS (flight_sysid): without these the sim truth flies at 100%% while reality
    # is ~0.90 - the model then runs AHEAD, and since the tape is time-indexed every turn,
    # climb and descent fires before the real drone reaches the point (Brian: "turns, climbs,
    # descents all being early"). Measured lag on identical banked bytes: 0.00 s at gate 5,
    # +0.29 at gate 6, +0.96 at gate 7 - it compounds.
    cmd = [sys.executable, "-m", "analysis.sim_ace", "--truthstate", "--launchlag", "0.4",
           "--tmax", "90", "--tape-out", "pilots/ace_pilot/tape.json"]
    if lock:
        cmd += ["--prefix", "pilots/ace_pilot/tape_locked.json", "--splicet", str(lock["t"])]
    out = subprocess.run(cmd, capture_output=True, text=True, cwd=HERE).stdout
    misses = {}
    n_ok = 0
    planes = 0
    for line in out.splitlines():
        m = re.match(r"\s+g\s*(\d+) t=\s*[\d.]+\s+lateral ([+-][\d.]+)\s+vertical ([+-][\d.]+)\s+(PASS|MISS)", line)
        if m and m.group(4) == "MISS":
            misses[int(m.group(1))] = (float(m.group(2)), float(m.group(3)))
        s = re.search(r"sim result: (\d+)/18", line)
        if s:
            n_ok = int(s.group(1))
        p = re.search(r"(\d+) planes crossed", line)
        if p:
            planes = int(p.group(1))
    return n_ok, misses, planes, out


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    gates = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "course_map.json")))["gates"]
    cd = {g["gate_id"]: g["cross_dir"] for g in gates}
    offsets = {}
    if os.path.exists(OFF):
        offsets = {int(k): v for k, v in json.load(open(OFF)).items()}
    lock = json.load(open(LOCK)) if os.path.exists(LOCK) else None
    locked = set(range(lock["upto"] + 1)) if lock else set()
    if lock:
        print(f"prefix lock: gates 0..{lock['upto']} frozen bytes through t={lock['t']}s")
    for r in range(rounds):
        subprocess.run([sys.executable, "-m", "pilots.ace_pilot.trajectory"],
                       capture_output=True, cwd=HERE)
        n_ok, misses, planes, out = run_sim(lock)
        # locked gates fly reality-corrected bytes: their in-model crossings read off-center
        # by design. Only unlocked misses count. An uncrossed gate produces no MISS line,
        # so also demand every gate plane was actually crossed (tape covers the full course).
        misses = {k: v for k, v in misses.items() if k not in locked}
        print(f"round {r}: {n_ok}/18 clean in-model, {planes}/18 planes, "
              f"unlocked misses at {sorted(misses)}")
        if not misses and planes == 18 and n_ok >= 18 - len(locked):
            # STANDING LAW (Brian): climb with thrust at level pitch - the follower's nose-up
            # ticks are parasitic speed-shedding, and stripping them has never hurt a crossing.
            # Clamp suffix pitch >= 0 (banked bytes untouched).
            tp = os.path.join(HERE, "pilots", "ace_pilot", "tape.json")
            tape = json.load(open(tp))
            lt = lock["t"] if lock else 0.0
            nclamp = 0
            from pilots.ace_pilot.config import ACE_PITCH_UP_MAX
            for row in tape["commands"]:
                # SAME limit the generation follower flew under - a clamp tighter than
                # generation makes the flown tape differ from the verified one (the roll
                # timing was computed at the stalled speed and then flies fast)
                if row[0] > lt and row[2] < -ACE_PITCH_UP_MAX:
                    row[2] = -ACE_PITCH_UP_MAX
                    nclamp += 1
            json.dump(tape, open(tp, "w"))
            print(f"18/18 - tape is flight-ready ({nclamp} nose-up ticks clamped in the suffix)")
            return
        for gid, (lat, vert) in misses.items():
            # lateral miss is measured along the gate's left-perp; convert back to world dx,dy
            px, py = -cd[gid][1], cd[gid][0]
            o = offsets.get(gid, [0.0, 0.0, 0.0])
            offsets[gid] = [o[0] - 0.5 * lat * px, o[1] - 0.5 * lat * py, o[2] - 0.5 * vert]
        json.dump({str(k): v for k, v in offsets.items()}, open(OFF, "w"))
    print("did not converge in", rounds, "rounds")


if __name__ == "__main__":
    main()
