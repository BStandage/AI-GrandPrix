"""Altitude-loop sysid: closed-loop vertical steps, then tilt pulses.

Measures what the altitude loop (rc_backend.AltitudeLoop, gains from the
toml) actually does on this plant, in the two situations that matter in a
race: a change of target altitude, and a hard bank at constant target.
Every tick is logged so the response can be compared with the vertical
model the gains were designed from (a model prediction is a hypothesis;
this flight is the check).

Protocol (all at one spot, fits the 5 x 5 m training cage):
    SETTLE          hover to HOVER_Z
    STEP_UP/DOWN    z target +-STEP_M for STEP_S each, hold between
    TILT_<deg>_<s>  lateral accel pulse = g*tan(deg) for PULSE_S, both
                    directions, altitude target unchanged (position hold
                    brings it back between pulses)

Fly (in the sim):
    RACE_SOLVER=solvers.sysid_alt AIGP_SIM_TIME=60 \\
        uv run elodin run sim/main.py
Analyze:
    python -m solvers.sysid_alt --analyze out/sysid/alt_000.csv
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

from raceline import course as course_bridge  # noqa: F401  (sim repo on path)
from raceline.config import AIGP_REPO, G, load_config

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
HOVER = np.array([0.0, 0.0, 1.35])   # race altitude
STEP_M = 0.5
STEP_S = 2.5
TILT_DEG = (45.0, 64.0)              # plan corners sit at 53-64 deg
PULSE_S = 1.0
RECOVER_S = 6.0

_rows: list = []
_state: dict = {}


def _out_path() -> Path:
    p = os.environ.get("AIGP_SYSID_OUT")
    if p:
        return Path(p)
    from raceline.planner import next_numbered
    return next_numbered(str(AIGP_REPO / "out" / "sysid" / "alt_XXX.csv"))


def _write():
    if "path" not in _state:
        _state["path"] = _out_path()
    path = _state["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t,phase,z_target,a_cmd_y,x,y,z,vx,vy,vz,throttle,cos_tilt,"
                "a_z_cmd,thrust_cmd,m_mean,m_min,m_max\n")
        f.writelines(_rows)
    print(f"[SYSID] {len(_rows)} rows -> {path}")


def _motor_stats(update) -> str:
    m = getattr(update, "motors", None)
    if m is None or len(m) == 0:
        return "nan,nan,nan"
    return f"{float(np.mean(m)):.3f},{float(np.min(m)):.3f},{float(np.max(m)):.3f}"


def _settled(est) -> bool:
    return (float(np.linalg.norm(est.p - HOVER)) < 0.3
            and float(np.linalg.norm(est.v)) < 0.4)


def _queue():
    q = [("STEP_UP", "step", +STEP_M), ("STEP_DOWN", "step", -STEP_M)]
    for deg in TILT_DEG:
        for sgn in (+1, -1):
            q.append((f"TILT_{int(deg)}_{sgn:+d}", "tilt",
                      sgn * G * math.tan(math.radians(deg))))
    return q


def autopilot(update):
    from solver.api import RCCommand
    from raceline.rc_backend import (AltitudeLoop, GroundTruthSource,
                                     YawLoop, attitude_sticks)
    if not _state:
        cfg = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
        _state.update(cfg=cfg, src=GroundTruthSource(), alt=AltitudeLoop(cfg),
                      yaw=YawLoop(cfg), yaw0=None, phase="SETTLE", t0=0.0,
                      queue=_queue(), kind="", amp=0.0, written=False)
        print(f"[SYSID] altitude protocol: {len(_state['queue'])} events, "
              f"kp_z {cfg.follower.kp_z} kd_z {cfg.follower.kd_z} "
              f"ki_z {cfg.follower.ki_z}, cfg {cfg.sha1[:8]}")

    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)

    cfg = _state["cfg"]
    est = _state["src"].estimate(update)
    phase = _state["phase"]
    if _state["yaw0"] is None:
        _state["yaw0"] = est.yaw

    hold = (cfg.follower.kp_pos * (HOVER[:2] - est.p[:2])
            - cfg.follower.kd_pos * est.v[:2])
    a_cmd = hold
    z_target = float(HOVER[2])
    if phase == "SETTLE":
        if (_settled(est) and t > 6.0) or t - T_ARM_IDLE_END > 20.0:
            _advance(t)
    elif _state["kind"] == "step" and phase.startswith("STEP"):
        z_target = float(HOVER[2] + _state["amp"])
        if t - _state["t0"] > STEP_S:
            _advance(t)
    elif _state["kind"] == "tilt" and phase.startswith("TILT"):
        a_cmd = np.array([0.0, _state["amp"]])
        if t - _state["t0"] > PULSE_S:
            _advance(t)
    elif phase == "RECOVER":
        if ((t - _state["t0"] > 2.0 and _settled(est))
                or t - _state["t0"] > RECOVER_S):
            _advance(t)
    else:  # DONE
        if not _state["written"]:
            _state["written"] = True
            _write()
        return RCCommand(arm=1000, throttle=1000)

    airborne = est.p[2] >= cfg.follower.min_alt_translation_m
    throttle = _state["alt"].throttle(t, est, z_target, 0.0, airborne,
                                      update.baro_fresh)
    roll = pitch = yaw_stick = 1500
    if airborne:
        roll, pitch, _ = attitude_sticks(cfg, est, a_cmd)
        yaw_stick = _state["yaw"].stick(est, _state["yaw0"])

    alt = _state["alt"]
    _rows.append(f"{t:.4f},{phase},{z_target:.3f},{a_cmd[1]:.3f},"
                 f"{est.p[0]:.4f},{est.p[1]:.4f},{est.p[2]:.4f},"
                 f"{est.v[0]:.4f},{est.v[1]:.4f},{est.v[2]:.4f},"
                 f"{throttle},{est.R[2, 2]:.4f},"
                 f"{alt.a_cmd:.3f},{alt.thrust:.3f},{_motor_stats(update)}\n")
    return RCCommand(arm=1800, throttle=int(throttle), roll=roll,
                     pitch=pitch, yaw=yaw_stick)


def _advance(t):
    prev = _state["phase"]
    if prev in ("SETTLE", "RECOVER"):
        if _state["queue"]:
            name, kind, amp = _state["queue"].pop(0)
            _state.update(phase=name, kind=kind, amp=amp)
        else:
            _state["phase"] = "DONE"
    else:
        _state["phase"] = "RECOVER"
    _state["t0"] = t
    print(f"[SYSID] t={t:5.1f} {prev} -> {_state['phase']}")


def reset_state():
    _rows.clear()
    _state.clear()


# ---------------------------------------------------------------------------

def analyze(csv_path):
    """Per event: altitude excursion, overshoot (steps), 5 cm settling time,
    throttle range. Each event's window runs through the RECOVER after it."""
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    t, z, zt, ph = d["t"], d["z"], d["z_target"], d["phase"].astype(str)
    print(f"{'phase':14s} {'dur':>5s} {'z_start':>7s} {'z_min':>6s} "
          f"{'z_max':>6s} {'overshoot':>9s} {'settle5cm':>9s} "
          f"{'thr_min':>7s} {'thr_max':>7s}")
    n = len(t)
    i = 0
    while i < n:
        lab = ph[i]
        j = i
        while j < n and ph[j] == lab:
            j += 1
        if lab in ("SETTLE", "DONE", "RECOVER"):
            i = j
            continue
        k = j
        while k < n and ph[k] == "RECOVER":
            k += 1
        seg = slice(i, k)
        z0 = float(z[i])
        if lab.startswith("STEP"):
            sgn = 1.0 if zt[i] > z0 else -1.0
            over = float(np.max(sgn * (z[i:j] - zt[i])))
            bad = np.flatnonzero(np.abs(z[i:j] - zt[i]) > 0.05)
        else:
            over = float("nan")
            bad = np.flatnonzero(np.abs(z[seg] - zt[i]) > 0.05)
        settle = float(t[i + bad[-1]] - t[i]) if len(bad) else 0.0
        print(f"{lab:14s} {t[j - 1] - t[i]:5.1f} {z0:7.2f} "
              f"{z[seg].min():6.2f} {z[seg].max():6.2f} {over:9.3f} "
              f"{settle:9.2f} {d['throttle'][seg].min():7.0f} "
              f"{d['throttle'][seg].max():7.0f}")
        i = k
    print("(measured on THIS plant/config; a race flight is still the only "
          "pass/fail oracle)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", required=True)
    analyze(ap.parse_args().analyze)
