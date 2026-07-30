"""
Build pilots/ace_pilot/vz_ref.json - the tape's median vertical-velocity profile - from
flown VQ2 sessions. The damper (ACE_DAMPER_K) trims thrust toward this profile at replay.

    python -m analysis.make_vz_ref            # newest 4 sessions
    python -m analysis.make_vz_ref 6          # newest N
    python -m analysis.make_vz_ref sess_dir1 sess_dir2 ...

Uses the SAME estimator math as the pilot (leaky world-up specific-force integral with the
tape's commanded attitude) so estimator biases are common-mode between reference and replay.
Only use sessions flown on the CURRENT tape in VQ2; re-run after any tape change that
alters the vertical profile.
"""
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TAU = 4.0                                     # keep equal to ACE_DAMPER_TAU
BIN = 0.05


def tape_cmd(cmds, t):
    lo, hi = 0, len(cmds) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if cmds[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return cmds[lo]


def run_profile(sess, cmds):
    go, g0, vz, prev, out = None, None, 0.0, None, {}
    pre = []
    for line in open(os.path.join(sess, "telemetry.jsonl")):
        r = json.loads(line)
        if r.get("kind") == "race_status" and go is None:
            st = r.get("race_start_boot_time_ms", -1)
            if st is not None and st >= 0 and r.get("sim_boot_time_ms", 0) >= st:
                go = r["recv_time_ns"]
        if r.get("kind") != "highres_imu":
            continue
        if go is None:
            pre.append(-r.get("zacc", 0.0))   # on the ground, level: world-up force == g
            continue
        t = (r["recv_time_ns"] - go) * 1e-9
        if t < 0:
            continue
        if g0 is None:
            tail = pre[-100:] or [9.81]
            g0 = sum(tail) / len(tail)
        _, roll, pitch, _, _ = tape_cmd(cmds, t)
        fx, fy, fz = r.get("xacc", 0.0), r.get("yacc", 0.0), r.get("zacc", 0.0)
        up = -(-math.sin(pitch) * fx + math.cos(pitch) * math.sin(roll) * fy
               + math.cos(pitch) * math.cos(roll) * fz)
        if prev is not None:
            dt = t - prev
            if 0.0 < dt <= 0.1:
                vz = (vz + (up - g0) * dt) * (1.0 - dt / TAU)
        prev = t
        out[round(t / BIN)] = vz
    return out


def main():
    args = sys.argv[1:]
    if args and os.path.isdir(args[0]):
        sessions = args
    else:
        n = int(args[0]) if args else 4
        sessions = sorted(glob.glob(os.path.join(HERE, "datasets", "session_2026*")),
                          key=os.path.getmtime)[-n:]
    cmds = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "tape.json")))["commands"]
    profiles = []
    for s in sessions:
        p = run_profile(s, cmds)
        if len(p) > 200:
            profiles.append(p)
            print(f"  {os.path.basename(s)}: {len(p)} bins, to t={max(p) * BIN:.1f}s")
        else:
            print(f"  {os.path.basename(s)}: skipped ({len(p)} bins)")
    if len(profiles) < 2:
        print("need >= 2 usable sessions - nothing written")
        return
    keys = sorted(set.union(*(set(p.keys()) for p in profiles)))
    samples = []
    for k in keys:
        vs = sorted(p[k] for p in profiles if k in p)
        if len(vs) >= 2:                       # median wherever >=2 runs still alive
            samples.append([round(k * BIN, 3), round(vs[len(vs) // 2], 4)])
    out = os.path.join(HERE, "pilots", "ace_pilot", "vz_ref.json")
    json.dump({"n_runs": len(profiles), "samples": samples}, open(out, "w"))
    print(f"wrote {out}: {len(samples)} samples over {len(profiles)} runs, "
          f"t 0..{samples[-1][0]:.1f}s")


if __name__ == "__main__":
    main()
