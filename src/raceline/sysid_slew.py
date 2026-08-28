"""Attitude-slew sysid: measure a_lat_rate_max on the actual plant.

This is BOTH the sim measurement and the rehearsal of day-1 measurement #2
(PQ_PROCEDURE.md): hover-settle, then lateral-accel steps at three
amplitudes in both directions, logging state every tick. The analyzer
turns the log into a recommended `a_lat_rate_max` for vehicle.toml.

Fly it (in the sim):
    RACE_SOLVER=raceline.sysid_slew AIGP_SIM_TIME=80 \
        uv run elodin run sim/main.py

Analyze:
    python -m raceline.sysid_slew --analyze out/sysid/slew_000.csv

Three amplitudes matter: slew is amplitude-dependent (stick clamps, rate
limits, prop spin-up), and day 1 gives no second chances to discover that.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

from raceline import course as course_bridge
from raceline.config import AIGP_REPO, load_config

# ---------------------------------------------------------------------------
# Analyzer (no sim needed)
# ---------------------------------------------------------------------------

def _smooth(x: np.ndarray, t: np.ndarray, win_s: float) -> np.ndarray:
    n = max(1, int(round(win_s / max(np.median(np.diff(t)), 1e-4))) | 1)
    return np.convolve(x, np.ones(n) / n, mode="same")


def analyze(csv_path, margin: float = 0.8) -> dict:
    """Per-step peak lateral accel + peak accel RATE; recommend
    a_lat_rate_max = margin * min(full-amplitude peak rates)."""
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    t = d["t"]
    ay = np.gradient(_smooth(d["vy"], t, 0.04), t)      # world lateral accel
    ay_rate = np.gradient(_smooth(ay, t, 0.04), t)

    steps = []
    labels = d["phase"].astype(str)
    in_step = None
    for i, lab in enumerate(labels):
        if lab.startswith("STEP") and in_step is None:
            in_step = (lab, i)
        elif in_step is not None and not lab.startswith("STEP"):
            steps.append((in_step[0], in_step[1], i))
            in_step = None
    results = []
    for lab, i0, i1 in steps:
        j1 = min(i1, i0 + np.searchsorted(t[i0:i1], t[i0] + 0.8))
        results.append({
            "step": lab,
            "a_cmd": float(d["a_cmd_y"][i0]),
            "a_peak": float(np.max(np.abs(ay[i0:i1]))),
            "rate_peak": float(np.max(np.abs(ay_rate[i0:j1]))),
        })
    full = [r["rate_peak"] for r in results
            if abs(abs(r["a_cmd"]) - max(abs(x["a_cmd"]) for x in results))
            < 1e-6]
    rec = margin * min(full) if full else float("nan")
    return {"steps": results, "recommended_a_lat_rate_max": rec,
            "margin": margin}


def print_analysis(res: dict) -> None:
    print("step          a_cmd   a_peak  rate_peak (m/s^3)")
    for r in res["steps"]:
        print(f"  {r['step']:10s} {r['a_cmd']:+6.2f} {r['a_peak']:7.2f} "
              f"{r['rate_peak']:9.1f}")
    print(f"recommended a_lat_rate_max = {res['recommended_a_lat_rate_max']:.1f}"
          f"  ({res['margin']:.0%} of the slowest full-amplitude step)")
    print("(measured on THIS plant/config only — sim numbers do not "
          "transfer to hardware)")


# ---------------------------------------------------------------------------
# The in-sim solver (only imported paths below need the sim repo)
# ---------------------------------------------------------------------------

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
HOVER = np.array([0.0, 0.0, 1.5])
STEP_S = 1.2          # step hold
RECOVER_S = 5.0       # max recover time between steps
AMP_FRACS = (1 / 3, 2 / 3, 1.0)

_rows = []
_state = {"phase": "SETTLE", "t0": 0.0, "queue": None, "yaw0": None,
          "written": False}


def _out_path() -> Path:
    p = os.environ.get("AIGP_SYSID_OUT")
    if p:
        return Path(p)
    from raceline.planner import next_numbered
    return next_numbered(str(AIGP_REPO / "out" / "sysid" / "slew_XXX.csv"))


def _write():
    path = _out_path() if not _state.get("path") else _state["path"]
    _state["path"] = path
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t,phase,a_cmd_y,x,y,z,vx,vy,vz\n")
        f.writelines(_rows)
    print(f"[SYSID] {len(_rows)} rows -> {path}")


def _settled(est) -> bool:
    return (float(np.linalg.norm(est.p - HOVER)) < 0.4
            and float(np.linalg.norm(est.v)) < 0.5)


def autopilot(update):
    # Imports deferred so `--analyze` works without the sim repo.
    from solver.api import RCCommand
    from raceline.rc_backend import (AltitudeLoop, GroundTruthSource,
                                     YawLoop, attitude_sticks)
    if "cfg" not in _state:
        _state["cfg"] = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
        _state["src"] = GroundTruthSource()
        _state["alt"] = AltitudeLoop(_state["cfg"])
        _state["yaw"] = YawLoop(_state["cfg"])
        a_max = _state["cfg"].a_lat_full()
        _state["queue"] = [(f"STEP_{int(fr*100)}pct_{sgn:+d}", sgn * fr * a_max)
                           for fr in AMP_FRACS for sgn in (+1, -1)]
        print(f"[SYSID] slew protocol: {len(_state['queue'])} steps, "
              f"a_max={a_max:.2f} m/s^2, cfg {_state['cfg'].sha1[:8]}")

    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)

    cfg = _state["cfg"]
    est = _state["src"].estimate(update)
    phase = _state["phase"]

    a_cmd = np.zeros(2)
    if phase == "SETTLE":
        if _state["yaw0"] is None:
            _state["yaw0"] = est.yaw
        a_cmd = (cfg.follower.kp_pos * (HOVER[:2] - est.p[:2])
                 - cfg.follower.kd_pos * est.v[:2])
        if (_settled(est) and t > 6.0) or t - T_ARM_IDLE_END > 20.0:
            _advance_phase(t)
    elif phase.startswith("STEP"):
        a_cmd = np.array([0.0, _state["amp"]])
        if t - _state["t0"] > STEP_S:
            _advance_phase(t)
    elif phase == "RECOVER":
        a_cmd = (cfg.follower.kp_pos * (HOVER[:2] - est.p[:2])
                 - cfg.follower.kd_pos * est.v[:2])
        if _settled(est) or t - _state["t0"] > RECOVER_S:
            _advance_phase(t)
    elif phase == "DONE":
        if not _state["written"]:
            _state["written"] = True
            _write()
        return RCCommand(arm=1000, throttle=1000)

    airborne = est.p[2] >= 1.0
    throttle = _state["alt"].throttle(t, est, float(HOVER[2]), 0.0, airborne,
                                      update.baro_fresh)
    roll = pitch = 1500
    yaw_stick = 1500
    if airborne:
        roll, pitch, _ = attitude_sticks(cfg, est, a_cmd)
        yaw_stick = _state["yaw"].stick(est, _state["yaw0"])

    _rows.append(f"{t:.4f},{_state['phase']},{a_cmd[1]:.3f},"
                 f"{est.p[0]:.4f},{est.p[1]:.4f},{est.p[2]:.4f},"
                 f"{est.v[0]:.4f},{est.v[1]:.4f},{est.v[2]:.4f}\n")
    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch,
                     yaw=yaw_stick)


def _advance_phase(t):
    prev = _state["phase"]
    if prev == "SETTLE" or prev == "RECOVER":
        if _state["queue"]:
            name, amp = _state["queue"].pop(0)
            _state["phase"], _state["amp"] = name, amp
        else:
            _state["phase"] = "DONE"
    elif prev.startswith("STEP"):
        _state["phase"] = "RECOVER"
    _state["t0"] = t
    print(f"[SYSID] t={t:5.1f} {prev} -> {_state['phase']}")


def reset_state():
    _rows.clear()
    _state.clear()
    _state.update(phase="SETTLE", t0=0.0, queue=None, yaw0=None,
                  written=False)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", required=True, help="slew CSV to analyze")
    ap.add_argument("--margin", type=float, default=0.8)
    args = ap.parse_args()
    print_analysis(analyze(args.analyze, args.margin))
