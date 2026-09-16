"""
ANGLE-mode probe: does the SITL level itself, and which way do the sticks tilt it?

    cd src && python -m raceline.batch_fly --solver solvers.angle_probe --angle ../out/plans/plan_RACE.json

Sequence (ground truth logged every tick to out/flightlogs/angle_probe_NNN.csv):
  0.0-0.5  disarmed
  0.5-1.0  armed, throttle min
  1.0-3.0  throttle at the toml takeoff value, sticks centred, aux2 high (ANGLE)
  3.0-4.5  pitch stick +150 (ANGLE: +24 deg nose-down target at angle_limit 80)
  4.5-6.0  sticks centred
  6.0-7.5  roll stick +150
  7.5-9.0  sticks centred, then disarm

Verdict printed at the end: in ANGLE mode the tilt during a step settles
near 24 deg and returns to ~0 when the stick centres; in ACRO the tilt
runs away past 90 within half a second. The signs tell whether +pitch
is nose-down (forward = the direction the nose points) and +roll is
right-wing-down.
"""

from __future__ import annotations

import csv
import math
import os

import numpy as np

from raceline.config import load_config, AIGP_REPO
from raceline.planner import next_numbered
from raceline.rc_backend import rot_from_quat
from solver.api import RCCommand, SensorUpdate

CFG = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
STEP = 150
_log = {"w": None, "f": None, "rows": []}


def _phase(t):
    if t < 0.5: return "disarmed", 1000, 1000, 1500, 1500
    if t < 1.0: return "arm-idle", 1800, 1000, 1500, 1500
    thr = int(CFG.follower.takeoff_pwm)
    if t < 3.0: return "hover", 1800, thr, 1500, 1500
    if t < 4.5: return "pitch+", 1800, thr, 1500, 1500 + STEP
    if t < 6.0: return "centre", 1800, thr, 1500, 1500
    if t < 7.5: return "roll+", 1800, thr, 1500 + STEP, 1500
    if t < 9.0: return "centre2", 1800, thr, 1500, 1500
    return "done", 1000, 1000, 1500, 1500


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    name, arm, thr, roll, pitch = _phase(t)
    R = rot_from_quat(update.world_pos[0:4])
    zb = R[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(zb[2])))))
    yaw = math.atan2(R[1, 0], R[0, 0])
    # tilt direction in the yaw frame: forward component of body z (nose-down > 0), left component
    fwd = math.cos(yaw) * zb[0] + math.sin(yaw) * zb[1]
    left = -math.sin(yaw) * zb[0] + math.cos(yaw) * zb[1]
    if _log["w"] is None:
        p = next_numbered(str(AIGP_REPO / "out" / "flightlogs" / "angle_probe_XXX.csv"))
        _log["f"] = open(p, "w", newline="", encoding="utf-8"); _log["w"] = csv.writer(_log["f"])
        _log["w"].writerow(["t", "phase", "tilt_deg", "fwd", "left", "x", "y", "z", "roll_stk", "pitch_stk", "thr"])
        print(f"[PROBE] log -> {p}")
    _log["w"].writerow([f"{t:.3f}", name, f"{tilt:.1f}", f"{fwd:.3f}", f"{left:.3f}",
                        f"{update.world_pos[4]:.2f}", f"{update.world_pos[5]:.2f}", f"{update.world_pos[6]:.2f}", roll, pitch, thr])
    _log["rows"].append((t, name, tilt, fwd, left, float(update.world_pos[6])))
    if int(t * 100) % 50 == 0:
        print(f"[PROBE] t={t:4.1f} {name:8s} tilt={tilt:5.1f} fwd={fwd:+.2f} left={left:+.2f} z={update.world_pos[6]:.2f}")
    if name == "done" and not _log.get("verdict"):
        _log["verdict"] = True
        rows = _log["rows"]
        def stat(ph):
            r = [x for x in rows if x[1] == ph]
            if not r: return None
            tail = r[len(r) // 2:]
            return max(x[2] for x in r), sum(x[2] for x in tail) / len(tail), sum(x[3] for x in tail) / len(tail), sum(x[4] for x in tail) / len(tail)
        h, p, c, rl = stat("hover"), stat("pitch+"), stat("centre"), stat("roll+")
        print("[PROBE] VERDICT")
        print(f"[PROBE]   hover: max tilt {h[0]:.0f}, settled {h[1]:.0f} deg")
        print(f"[PROBE]   pitch+{STEP}: max tilt {p[0]:.0f}, settled {p[1]:.0f} deg, body-z forward {p[2]:+.2f} (nose-down > 0)")
        print(f"[PROBE]   centre: settled {c[1]:.0f} deg")
        print(f"[PROBE]   roll+{STEP}:  max tilt {rl[0]:.0f}, settled {rl[1]:.0f} deg, body-z left {rl[3]:+.2f} (right-wing-down < 0)")
        mode = "ANGLE (self-levelling)" if p[0] < 60 and c[1] < 10 else "NOT angle mode: tilt ran away (acro) or the FC attitude is wrong"
        print(f"[PROBE]   => {mode}")
        _log["f"].flush()
    return RCCommand(throttle=thr, roll=roll, pitch=pitch, yaw=1500, arm=arm, aux2=1800)
