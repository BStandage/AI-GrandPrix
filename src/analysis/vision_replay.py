"""
Rebuild the rich per-frame vision dataset (vision_frames.jsonl) from a SAVED session - the same
records the live collector (perception.vision_data_collector) writes, but reconstructed offline from the
recorded frames + telemetry + gates. Lets us extract the full perception-vs-truth dataset from any
past flight without re-flying, and re-run it after improving the detector/PnP.

Each frame's pose is the telemetry sample nearest its timestamp (the same alignment the live
collector gets for free by reading shared_data at frame time).

Usage:
  python -m analysis.vision_replay                 # newest session
  python -m analysis.vision_replay <session_dir>   # a specific session
Writes <session_dir>/vision_frames.jsonl.
"""

import glob
import json
import os
import sys

import cv2

from common.paths import DATASETS_DIR
from perception.vision_data_collector import VisionDataCollector


def _newest_session():
    sess = sorted(glob.glob(os.path.join(DATASETS_DIR, "session_*")), key=os.path.getmtime)
    return sess[-1] if sess else None


def _load_telemetry(path):
    """kind -> list of (recv_time_ns, fields) sorted by time, for nearest-time lookup."""
    streams = {}
    for line in open(path):
        try:
            r = json.loads(line)
        except Exception:
            continue
        k = r.get("kind")
        t = r.get("recv_time_ns")
        if k is None or t is None:
            continue
        streams.setdefault(k, []).append((t, r))
    for k in streams:
        streams[k].sort(key=lambda x: x[0])
    return streams


def _nearest(stream, t):
    """Last sample at or before t (else the first sample). Linear scan with a cursor is overkill
    for a few thousand frames; a simple bisect-free walk is fine here."""
    if not stream:
        return None
    import bisect
    times = [x[0] for x in stream]
    i = bisect.bisect_right(times, t) - 1
    if i < 0:
        i = 0
    return stream[i][1]


def replay(session_dir):
    frames_jsonl = os.path.join(session_dir, "frames.jsonl")
    gates_json = os.path.join(session_dir, "gates.json")
    if not os.path.exists(frames_jsonl):
        print(f"no frames.jsonl in {session_dir}", flush=True)
        return
    gates = json.load(open(gates_json))["gates"] if os.path.exists(gates_json) else None
    telem = _load_telemetry(os.path.join(session_dir, "telemetry.jsonl"))

    # The odometry frame RESETS at race start (reset_counter bumps), but the gates are broadcast in
    # the post-reset race frame. Mixing the two makes pre-reset poses look metres off the gates. Keep
    # only the dominant (latest) odometry frame so the race-time geometry aligns with the gates -
    # the live collector never hits this because it reads the current pose at frame time.
    odo = telem.get("odometry", [])
    if odo:
        rc = max((r.get("reset_counter", 0) for _, r in odo), default=0)
        telem["odometry"] = [(t, r) for (t, r) in odo if r.get("reset_counter", 0) == rc]
        print(f"  odometry: keeping reset_counter={rc} frame ({len(telem['odometry'])}/{len(odo)} samples)",
              flush=True)

    out_path = os.path.join(session_dir, "vision_frames.jsonl")
    vl = VisionDataCollector.__new__(VisionDataCollector)   # bypass the "open w" in __init__
    vl.path = out_path
    vl._f = open(out_path, "w")
    vl._n = 0
    vl._warned = False
    print(f"replay -> {out_path}", flush=True)

    n = 0
    for line in open(frames_jsonl):
        try:
            fr = json.loads(line)
        except Exception:
            continue
        t = fr.get("recv_time_ns") or fr.get("sim_time_ns")
        img = cv2.imread(os.path.join(session_dir, fr["file"]))
        if img is None:
            continue
        data = {
            "gates": gates,
            "odometry": _nearest(telem.get("odometry", []), t),
            "attitude": _nearest(telem.get("attitude", []), t),
            "race_status": _nearest(telem.get("race_status", []), t) or {},
            "_control_mode": "replay",
        }
        vl.log(fr["frame_id"], img, fr.get("sim_time_ns"), data)
        n += 1
        if n % 300 == 0:
            print(f"  {n} frames", flush=True)
    vl.close()


if __name__ == "__main__":
    sd = sys.argv[1] if len(sys.argv) > 1 else _newest_session()
    if sd is None:
        print("no session found")
    else:
        replay(sd)
