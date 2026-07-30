"""Ground truth per session: did race_status.active_gate_index EVER advance?"""
import glob
import json
import os

for sd in sorted(glob.glob(r"src\datasets\session_20260727_1*")):
    tj = os.path.join(sd, "telemetry.jsonl")
    if not os.path.exists(tj):
        continue
    idx_changes = []
    prev = None
    t0 = None
    for line in open(tj):
        r = json.loads(line)
        if r.get("kind") != "race_status":
            continue
        if t0 is None:
            t0 = r["recv_time_ns"]
        gi = r.get("active_gate_index")
        if gi != prev:
            idx_changes.append((round((r["recv_time_ns"] - t0) * 1e-9, 1), gi))
            prev = gi
    print(f"{os.path.basename(sd)}: active_gate_index changes: {idx_changes}")
1