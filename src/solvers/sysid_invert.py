"""Inverted-push sysid: can the drone roll past 90, push DOWN with thrust,
roll back and pull out, and what does that cost in height and time?

Protocol (at altitude, clear of every gate):
  SETTLE   hover at HOVER (9 m up)
  PUSH_x   desired body-z pointed DOWN (with a small roll kick to break the
           180 deg singularity), collective at T_PUSH, for x seconds
  RECOVER  desired body-z UP, collective at T_RECOVER until vz >= 0, then
           the altitude loop brings it back to HOVER
  ...      three push durations, then DONE -> CSV

Launch:   run_race_docker.cmd solvers.sysid_invert
Analyze:  python -m solvers.sysid_invert --analyze out/sysid/invert_000.csv

Reports per push: time to roll inverted, peak downward speed, height lost to
the lowest point, time from push start to the lowest point, time inverted.
The stack's drop (g10-top to g10-low) is 2.3 m and takes 1.1 s in free fall
with the follower today (race_056); this measures what an inverted push buys.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

from raceline.config import AIGP_REPO, load_config

G = 9.81


# ---------------------------------------------------------------------------
# Analyzer (no sim needed)
# ---------------------------------------------------------------------------

def analyze(csv_path) -> dict:
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    t = d["t"]; z = d["z"]; vz = d["vz"]; ct = d["cos_tilt"]
    labels = d["phase"].astype(str)
    out = {"pushes": []}
    names = [p for p in dict.fromkeys(labels) if p.startswith("PUSH")]
    for name in names:
        i0 = int(np.argmax(labels == name))
        t0 = t[i0]; z0 = z[i0]
        # window: from push start until the drone is back above z0 - 0.2 or 4 s
        w = (t >= t0) & (t <= t0 + 4.0)
        tw, zw, vzw, ctw = t[w], z[w], vz[w], ct[w]
        i_min = int(np.argmin(zw))
        inv = ctw < 0.0
        t_inv = float(tw[np.argmax(inv)] - t0) if inv.any() else float("nan")
        dur_inv = float(np.sum(inv) * np.median(np.diff(tw))) if inv.any() else 0.0
        out["pushes"].append({
            "name": name, "t_to_inverted_s": t_inv, "time_inverted_s": dur_inv,
            "peak_down_mps": float(-vzw.min()), "height_lost_m": float(z0 - zw[i_min]),
            "t_to_lowest_s": float(tw[i_min] - t0),
            "drop_2p3m_s": float((tw[np.argmax(zw <= z0 - 2.3)] - t0) if (zw <= z0 - 2.3).any() else float("nan")),
        })
    return out


def print_analysis(res: dict) -> None:
    print(f"{'push':10s} {'t_inv':>6s} {'inv_dur':>7s} {'peak_dn':>7s} {'lost_m':>6s} {'t_low':>6s} {'2.3m_in':>7s}")
    for p in res["pushes"]:
        print(f"{p['name']:10s} {p['t_to_inverted_s']:6.2f} {p['time_inverted_s']:7.2f} "
              f"{p['peak_down_mps']:7.1f} {p['height_lost_m']:6.2f} {p['t_to_lowest_s']:6.2f} {p['drop_2p3m_s']:7.2f}")
    print("(free fall: 2.3 m in 0.68 s; the follower's drop today: ~1.1 s)")


# ---------------------------------------------------------------------------
# Solver
# ---------------------------------------------------------------------------

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
HOVER = np.array([-4.0, -6.0, 9.0])   # high and clear of every gate
PUSH_DURATIONS_S = (0.15, 0.25, 0.35)
T_PUSH = 20.0          # m/s^2 of collective while inverted (pushes DOWN)
T_RECOVER = 30.0       # m/s^2 of collective while pulling out upright
ROLL_KICK = 0.35       # x-component of the desired body-z to pick a roll direction
RECOVER_S = 6.0        # max recover time
_rows = []
_state = {"phase": "SETTLE", "t0": 0.0, "queue": None, "yaw0": None,
          "written": False}


def _out_path() -> Path:
    p = os.environ.get("AIGP_SYSID_OUT")
    if p:
        return Path(p)
    from raceline.planner import next_numbered
    return next_numbered(str(AIGP_REPO / "out" / "sysid" / "invert_XXX.csv"))


def _write():
    path = _out_path() if not _state.get("path") else _state["path"]
    _state["path"] = path
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t,phase,x,y,z,vx,vy,vz,roll,pitch,throttle,wx,wy,cos_tilt,e_att\n")
        f.writelines(_rows)
    print(f"[SYSID] {len(_rows)} rows -> {path}")


def _settled(est) -> bool:
    return (float(np.linalg.norm(est.p - HOVER)) < 0.5
            and float(np.linalg.norm(est.v)) < 0.6)


def _sticks_for_zd(cfg, est, zd):
    """Roll/pitch sticks that rotate the body z-axis toward zd (world), the
    same tilt-vector error the follower uses, but with zd given directly so
    it can point DOWN."""
    f = cfg.follower
    zd = np.asarray(zd, dtype=float)
    zd = zd / max(float(np.linalg.norm(zd)), 1e-9)
    zb = est.R[:, 2]
    e = np.cross(zb, zd)
    eb = est.R.T @ e
    wx = float(est.omega[0]) if est.omega is not None else 0.0
    wy = float(est.omega[1]) if est.omega is not None else 0.0
    lim = f.stick_clamp
    roll = int(round(max(1500 - lim, min(1500 + lim, 1500.0 + f.ka_att * eb[0] - f.kw_att * wx))))
    pitch = int(round(max(1500 - lim, min(1500 + lim, 1500.0 + f.ka_att * eb[1] - f.kw_att * wy))))
    return roll, pitch, eb


def autopilot(update):
    from solver.api import RCCommand
    from raceline.rc_backend import (AltitudeLoop, GroundTruthSource,
                                     YawLoop, attitude_sticks)
    if "cfg" not in _state:
        _state["cfg"] = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
        _state["src"] = GroundTruthSource()
        _state["alt"] = AltitudeLoop(_state["cfg"])
        _state["yaw"] = YawLoop(_state["cfg"])
        _state["queue"] = [(f"PUSH_{int(d*1000)}ms", d) for d in PUSH_DURATIONS_S]
        print(f"[SYSID] inverted-push protocol: {len(_state['queue'])} pushes, "
              f"T_push {T_PUSH} T_recover {T_RECOVER}, cfg {_state['cfg'].sha1[:8]}")

    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)

    cfg = _state["cfg"]
    est = _state["src"].estimate(update)
    phase = _state["phase"]
    airborne = est.p[2] >= 1.0
    roll = pitch = 1500
    yaw_stick = 1500
    e_att = 0.0
    throttle = 1000

    if phase == "SETTLE" or phase == "RECOVER_HOVER":
        if _state["yaw0"] is None:
            _state["yaw0"] = est.yaw
        a_cmd = (cfg.follower.kp_pos * (HOVER[:2] - est.p[:2])
                 - cfg.follower.kd_pos * est.v[:2])
        throttle = _state["alt"].throttle(t, est, float(HOVER[2]), 0.0, airborne,
                                          update.baro_fresh)
        if airborne:
            roll, pitch, eb = attitude_sticks(cfg, est, a_cmd)
            e_att = float(np.hypot(eb[0], eb[1]))
            yaw_stick = _state["yaw"].stick(est, _state["yaw0"])
        if phase == "SETTLE":
            if (_settled(est) and t > 8.0) or t - T_ARM_IDLE_END > 30.0:
                _advance_phase(t)
        else:
            if (t - _state["t0"] > 2.0 and _settled(est)) or t - _state["t0"] > RECOVER_S:
                _advance_phase(t)
    elif phase.startswith("PUSH"):
        # body-z DOWN, with a roll kick so the tilt-vector error is not zero
        roll, pitch, eb = _sticks_for_zd(cfg, est, (ROLL_KICK, 0.0, -1.0))
        e_att = float(np.hypot(eb[0], eb[1]))
        throttle = cfg.pwm_for_thrust(T_PUSH)
        if t - _state["t0"] > _state["dur"]:
            _advance_phase(t)
    elif phase == "RECOVER":
        # body-z UP, collective high until the descent is arrested
        roll, pitch, eb = _sticks_for_zd(cfg, est, (0.0, 0.0, 1.0))
        e_att = float(np.hypot(eb[0], eb[1]))
        upright = float(est.R[2, 2]) > 0.5
        throttle = cfg.pwm_for_thrust(T_RECOVER) if upright else cfg.pwm_for_thrust(T_PUSH)
        if upright and est.v[2] >= 0.0:
            _state["alt"].reset()
            _advance_phase(t)
        elif t - _state["t0"] > RECOVER_S:
            _advance_phase(t)
    elif phase == "DONE":
        if not _state["written"]:
            _state["written"] = True
            _write()
        return RCCommand(arm=1000, throttle=1000)

    _rows.append(f"{t:.4f},{phase},"
                 f"{est.p[0]:.4f},{est.p[1]:.4f},{est.p[2]:.4f},"
                 f"{est.v[0]:.4f},{est.v[1]:.4f},{est.v[2]:.4f},"
                 f"{roll},{pitch},{int(throttle)},"
                 f"{float(est.omega[0]) if est.omega is not None else 0.0:.3f},"
                 f"{float(est.omega[1]) if est.omega is not None else 0.0:.3f},"
                 f"{est.R[2, 2]:.4f},{e_att:.3f}\n")
    return RCCommand(arm=1800, throttle=int(throttle), roll=roll, pitch=pitch,
                     yaw=yaw_stick)


def _advance_phase(t):
    prev = _state["phase"]
    _state["t0"] = t
    if prev in ("SETTLE", "RECOVER_HOVER"):
        if _state["queue"]:
            name, dur = _state["queue"].pop(0)
            _state["phase"], _state["dur"] = name, dur
        else:
            _state["phase"] = "DONE"
    elif prev.startswith("PUSH"):
        _state["phase"] = "RECOVER"
    elif prev == "RECOVER":
        _state["phase"] = "RECOVER_HOVER"
    print(f"[SYSID] t={t:5.1f} {prev} -> {_state['phase']}")


def reset_state():
    global _rows
    _rows = []
    _state.clear()
    _state.update(phase="SETTLE", t0=0.0, queue=None, yaw0=None, written=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", help="CSV from a sim run")
    args = ap.parse_args()
    if args.analyze:
        print_analysis(analyze(args.analyze))
        return 0
    print(__doc__)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
