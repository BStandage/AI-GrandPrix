"""
Race state helpers shared by the pilots: whether we're cleared to fly (countdown gating)
and how the gate list gets populated.

The live ground-truth track is broadcast once at race start (see mavlink_rx.py) and is always
in this run's frame. We rely on that broadcast. A cached gates.json from a previous run is in
a different frame and would miss every gate, so the cache fallback is disabled by default.
"""

import glob
import json
import os

from common.gate_geometry import relative_gate
from common.paths import DATASETS_DIR

# Wait out the "3..2..1..GO" countdown: arm but hold zero thrust until the race goes live,
# then launch instantly. Start the client during the countdown so it also catches the live
# track broadcast in this run's frame.
REQUIRE_RACE = True

# Cached tracks are in a stale frame and would miss every gate. Keep False; idling until the
# live track arrives is strictly better than flying a mis-framed line.
ALLOW_CACHED_GATES = False


def load_cached_gates(data):
    """Populate data["gates"] from a previous run's gates.json if (and only if) caching is
    enabled and a cache aligns with this run's pose. Disabled by default."""
    if data.get("gates"):
        return
    if not ALLOW_CACHED_GATES:
        if not data.get("_cache_warned") and data.get("odometry") is not None:
            data["_cache_warned"] = True
            print("[track] LIVE TRACK NOT RECEIVED YET - idling. The track is a one-shot broadcast; "
                  "if this persists, RESTART THE RACE in the sim with this client already running.",
                  flush=True)
        return
    odo = data.get("odometry")
    if odo is None:
        return   # need our pose to test alignment; retry next tick
    pose = ((odo["x"], odo["y"], odo["z"]), (odo["qw"], odo["qx"], odo["qy"], odo["qz"]))
    ds = DATASETS_DIR
    files = sorted(glob.glob(os.path.join(ds, "*", "gates.json")), key=os.path.getmtime, reverse=True)
    for f in files:
        try:
            gates = json.load(open(f)).get("gates")
        except Exception:
            continue
        if not gates:
            continue
        g0 = next((g for g in gates if g.get("gate_id") == 0), gates[0])
        rel = relative_gate(pose[0], pose[1], g0)
        if rel["forward"] > 2.0 and abs(rel["azimuth_deg"]) < 40.0:   # roughly matches our spawn
            data["gates"] = gates
            data["_gates_from_cache"] = True
            data["_cached_gates_obj"] = gates
            print(f"[track] using cached layout {os.path.relpath(f, ds)} "
                  f"(gate0 {rel['forward']:.0f} m ahead). Overridden when the sim sends one.", flush=True)
            return
    if not data.get("_cache_warned"):
        data["_cache_warned"] = True
        print("[track] no cached track matches this run's spawn frame. Restart the race with this "
              "client running to load the live track.", flush=True)


def should_fly(data):
    """True once we have gates + pose and the race is live (and not finished)."""
    if not (data.get("gates") and data.get("odometry") is not None):
        return False
    rs = data.get("race_status") or {}
    finish = rs.get("race_finish_time_ns", -1)
    if finish is not None and finish >= 0:
        return False   # race over
    if not REQUIRE_RACE:
        return True
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    return start is not None and start >= 0 and now >= start


def seconds_to_go(data):
    """Seconds until the race goes live. >0 waiting, <=0 live, None if there's no race yet."""
    rs = data.get("race_status") or {}
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    if start is None or start < 0:
        return None
    return (start - now) / 1000.0
