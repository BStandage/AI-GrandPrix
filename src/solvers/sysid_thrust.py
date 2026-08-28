"""Throttle-sweep sysid: measure the REAL pwm -> climb authority curve.

Why: hover is 1240, yet at 1600 the drone climbed ~0.1 m/s during the
validation race - the thrust curve above hover has never been measured
(the toml's curve is a labeled placeholder). This flight steps RAW
throttle values across the full range with the attitude held level and
records the climb response. The analyzer turns it into measured
curve_pwm/curve_acc points and shows where the curve flattens.

This is also day-1 measurement #4 (PQ_PROCEDURE.md) rehearsed early.

Fly:
    RACE_SOLVER=solvers.sysid_thrust AIGP_SIM_TIME=60 \
        uv run elodin run sim/main.py
Analyze:
    python -m solvers.sysid_thrust --analyze out/sysid/thrust_000.csv
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np

from raceline import course as course_bridge
from raceline.config import AIGP_REPO, load_config

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
HOVER = np.array([0.0, 0.0, 2.0])   # start a bit high: descent room
STEP_S = 1.5
RECOVER_S = 6.0
STEP_PWMS = (1350, 1500, 1650, 1800, 2000)

_rows: list = []
_state: dict = {}


def _out_path() -> Path:
    p = os.environ.get("AIGP_SYSID_OUT")
    if p:
        return Path(p)
    from raceline.planner import next_numbered
    return next_numbered(str(AIGP_REPO / "out" / "sysid" / "thrust_XXX.csv"))


def _write():
    if "path" not in _state:
        _state["path"] = _out_path()
    path = _state["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t,phase,pwm_cmd,z,vz\n")
        f.writelines(_rows)
    print(f"[SYSID] {len(_rows)} rows -> {path}")


def _settled(est) -> bool:
    return (abs(est.p[2] - HOVER[2]) < 0.3
            and float(np.linalg.norm(est.v)) < 0.5)


def autopilot(update):
    from solver.api import RCCommand
    from raceline.rc_backend import (AltitudeLoop, GroundTruthSource,
                                     attitude_sticks)
    if not _state:
        cfg = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
        _state.update(cfg=cfg, src=GroundTruthSource(),
                      alt=AltitudeLoop(cfg), phase="SETTLE", t0=0.0,
                      queue=list(STEP_PWMS), pwm=0, written=False)
        print(f"[SYSID] thrust sweep: steps {STEP_PWMS}, hover ref "
              f"{cfg.thrust.hover_pwm}, pwm_max {cfg.thrust.pwm_max}")

    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)

    cfg = _state["cfg"]
    est = _state["src"].estimate(update)
    phase = _state["phase"]
    airborne = est.p[2] >= 1.0

    # position hold (XY) the whole time; attitude stays ~level
    a_cmd = (cfg.follower.kp_pos * (HOVER[:2] - est.p[:2])
             - cfg.follower.kd_pos * est.v[:2])

    if phase == "SETTLE":
        throttle = _state["alt"].throttle(t, est, float(HOVER[2]), 0.0,
                                          airborne, update.baro_fresh)
        if (_settled(est) and t > 6.0) or t - T_ARM_IDLE_END > 20.0:
            _advance(t)
    elif phase.startswith("STEP"):
        throttle = _state["pwm"]          # RAW - the measurement itself
        if t - _state["t0"] > STEP_S:
            _advance(t)
    elif phase == "RECOVER":
        throttle = _state["alt"].throttle(t, est, float(HOVER[2]), 0.0,
                                          airborne, update.baro_fresh)
        if (t - _state["t0"] > 2.0 and _settled(est)) \
                or t - _state["t0"] > RECOVER_S:
            _advance(t)
    else:  # DONE
        if not _state["written"]:
            _state["written"] = True
            _write()
        return RCCommand(arm=1000, throttle=1000)

    roll = pitch = 1500
    if airborne:
        roll, pitch, _ = attitude_sticks(cfg, est, a_cmd)
    _rows.append(f"{t:.4f},{phase},{_state['pwm'] if phase.startswith('STEP') else 0},"
                 f"{est.p[2]:.4f},{est.v[2]:.4f}\n")
    return RCCommand(arm=1800, throttle=int(throttle), roll=roll,
                     pitch=pitch, yaw=1500)


def _advance(t):
    prev = _state["phase"]
    if prev in ("SETTLE", "RECOVER"):
        if _state["queue"]:
            _state["pwm"] = _state["queue"].pop(0)
            _state["phase"] = f"STEP_{_state['pwm']}"
        else:
            _state["phase"] = "DONE"
    elif prev.startswith("STEP"):
        _state["phase"] = "RECOVER"
    _state["t0"] = t
    print(f"[SYSID] t={t:5.1f} {prev} -> {_state['phase']}")


def reset_state():
    _rows.clear()
    _state.clear()


# ---------------------------------------------------------------------------

def analyze(csv_path):
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    t, vz = d["t"], d["vz"]
    print(f"{'pwm':>5s} {'vz_peak':>8s} {'a_z_initial':>12s}")
    rows = []
    for pwm in sorted(set(int(p) for p in d["pwm_cmd"] if p > 0)):
        m = d["pwm_cmd"] == pwm
        i0, i1 = np.flatnonzero(m)[[0, -1]]
        vz_pk = float(vz[i0:i1 + 1].max())
        j = i0 + np.searchsorted(t[i0:i1 + 1], t[i0] + 0.4)
        az = float((vz[j] - vz[i0]) / max(t[j] - t[i0], 1e-3))
        rows.append((pwm, vz_pk, az))
        print(f"{pwm:5d} {vz_pk:8.2f} {az:12.2f}")
    print("\nsuggested toml (accel = initial dvz/dt + g):")
    pwms = [r[0] for r in rows]
    accs = [round(r[2] + 9.81, 2) for r in rows]
    print(f"curve_pwm = {pwms}")
    print(f"curve_acc = {accs}")
    print("(measured on THIS plant/config; look for where vz_peak stops "
          "growing - that is the real ceiling)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", required=True)
    args = ap.parse_args()
    analyze(args.analyze)
