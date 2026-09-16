"""Straight-line sprint sysid: measure the REAL speed ceiling and the
speed-dependent loss (drag / thrust loss with airspeed).

Why (2026-09-09, race_029): the plan asks for 15 m/s on the g1-g3 straight
and budgets 12 m/s^2 of forward accel. The flown run never exceeded 9.3
m/s anywhere: the follower commanded 19-24 m/s^2, the airframe held a
real 58 deg mean tilt with altitude held, and delivered at most 7.4 m/s^2.
Level-flight kinematics say g*tan(58) = 15.7 m/s^2, so roughly half the
thrust is going to something that grows with speed. vehicle.toml has
drag_lin = drag_quad = 0 and the planner does not model drag at all, so
every plan speed above ~9 m/s is fiction and the follower saturates on
every straight.

Protocol: hover, then for each tilt level command a CONSTANT horizontal
accel g*tan(tilt) along a fixed heading with the altitude loop holding
z, until the speed stops growing or a distance/time budget runs out;
brake with the same magnitude; fly back; repeat. The analyzer recovers
the equivalent drag accel at each speed from

    a_loss(v) = (a_z + g) * tan(tilt_actual) - a_horiz_actual

and fits a_loss = c1*v + c2*v^2, reporting the terminal speed per tilt
and the toml numbers (drag_lin = m*c1, drag_quad = m*c2).

Fly (WSL, from the sim repo):
    RACE_SOLVER=solvers.sysid_sprint AIGP_SIM_TIME=150 \
        uv run elodin run sim/main.py
Analyze:
    python -m solvers.sysid_sprint --analyze out/sysid/sprint_000.csv
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

from raceline.config import AIGP_REPO, load_config

G = 9.81
T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
HOVER = np.array([-3.0, 0.0, 2.0])   # start point; sprint runs toward -x
DIRECTION = np.array([-1.0, 0.0])    # unit sprint direction (world XY)
TILT_DEG = (30.0, 45.0, 58.0, 68.0)  # 58 = flown mean on the straights
SPRINT_MAX_S = 7.0                   # per-sprint time budget
SPRINT_MAX_M = 55.0                  # per-sprint distance budget
V_PLATEAU_MPS2 = 0.3                 # stop when dv/dt over 1 s falls below
RETURN_V_MPS = 6.0                   # cruise back toward HOVER at this
RETURN_A_MPS2 = 6.0                  # and this accel cap
RECOVER_S = 8.0

_rows: list = []
_state: dict = {}


def _out_path() -> Path:
    p = os.environ.get("AIGP_SYSID_OUT")
    if p:
        return Path(p)
    from raceline.planner import next_numbered
    return next_numbered(str(AIGP_REPO / "out" / "sysid" / "sprint_XXX.csv"))


def _write():
    if "path" not in _state:
        _state["path"] = _out_path()
    path = _state["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t,phase,tilt_deg,a_des,x,y,z,vx,vy,vz,cos_tilt,throttle,"
                "a_z_cmd,thrust_cmd,m_mean,m_min,m_max\n")
        f.writelines(_rows)
    print(f"[SYSID] {len(_rows)} rows -> {path}")


def _motor_stats(update) -> str:
    m = getattr(update, "motors", None)
    if m is None or len(m) == 0:
        return "nan,nan,nan"
    return f"{float(np.mean(m)):.3f},{float(np.min(m)):.3f},{float(np.max(m)):.3f}"


def _settled(est) -> bool:
    return (float(np.linalg.norm(est.p - HOVER)) < 0.5
            and float(np.linalg.norm(est.v)) < 0.5)


def _hold_accel(cfg, est, target_xy):
    return (cfg.follower.kp_pos * (target_xy - est.p[:2])
            - cfg.follower.kd_pos * est.v[:2])


def _return_accel(cfg, est):
    """Cruise back to HOVER: speed-capped position controller."""
    err = HOVER[:2] - est.p[:2]
    dist = float(np.linalg.norm(err))
    v_des = err / max(dist, 1e-6) * min(RETURN_V_MPS, 1.0 * dist)
    a = cfg.follower.kd_pos * (v_des - est.v[:2])
    mag = float(np.linalg.norm(a))
    if mag > RETURN_A_MPS2:
        a *= RETURN_A_MPS2 / mag
    return a


def autopilot(update):
    from solver.api import RCCommand
    from raceline.rc_backend import (AltitudeLoop, GroundTruthSource,
                                     YawLoop, attitude_sticks)
    if not _state:
        cfg = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
        _state.update(cfg=cfg, src=GroundTruthSource(), alt=AltitudeLoop(cfg),
                      yaw=YawLoop(cfg), yaw0=None, phase="SETTLE", t0=0.0,
                      queue=list(TILT_DEG), tilt=0.0, amp=0.0, written=False,
                      v_hist=[])
        print(f"[SYSID] sprint protocol: tilts {TILT_DEG} deg along "
              f"{DIRECTION.tolist()}, cfg {cfg.sha1[:8]}")

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

    a_cmd = _hold_accel(cfg, est, HOVER[:2])
    v_along = float(np.dot(est.v[:2], DIRECTION))
    s_along = float(np.dot(est.p[:2] - HOVER[:2], DIRECTION))

    if phase == "SETTLE":
        if (_settled(est) and t > 6.0) or t - T_ARM_IDLE_END > 20.0:
            _advance(t)
    elif phase.startswith("SPRINT"):
        a_cmd = DIRECTION * _state["amp"]
        hist = _state["v_hist"]
        hist.append((t, v_along))
        while hist and t - hist[0][0] > 1.0:
            hist.pop(0)
        plateau = (len(hist) > 5 and t - hist[0][0] > 0.9
                   and (v_along - hist[0][1]) < V_PLATEAU_MPS2
                   and t - _state["t0"] > 2.0)
        if (t - _state["t0"] > SPRINT_MAX_S or s_along > SPRINT_MAX_M
                or plateau):
            _advance(t)
    elif phase.startswith("BRAKE"):
        a_cmd = -DIRECTION * _state["amp"]
        if v_along < 0.5 or t - _state["t0"] > 6.0:
            _advance(t)
    elif phase == "RETURN":
        a_cmd = _return_accel(cfg, est)
        if float(np.linalg.norm(HOVER[:2] - est.p[:2])) < 1.5 \
                or t - _state["t0"] > 30.0:
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
    throttle = _state["alt"].throttle(t, est, float(HOVER[2]), 0.0, airborne,
                                      update.baro_fresh)
    roll = pitch = yaw_stick = 1500
    if airborne:
        roll, pitch, _ = attitude_sticks(cfg, est, a_cmd)
        yaw_stick = _state["yaw"].stick(est, _state["yaw0"])

    alt = _state["alt"]
    a_des = float(np.dot(a_cmd, DIRECTION))
    tilt = _state["tilt"] if phase.startswith(("SPRINT", "BRAKE")) else 0.0
    _rows.append(f"{t:.4f},{phase},{tilt:.0f},{a_des:.3f},"
                 f"{est.p[0]:.4f},{est.p[1]:.4f},{est.p[2]:.4f},"
                 f"{est.v[0]:.4f},{est.v[1]:.4f},{est.v[2]:.4f},"
                 f"{est.R[2, 2]:.4f},{throttle},"
                 f"{alt.a_cmd:.3f},{alt.thrust:.3f},{_motor_stats(update)}\n")
    return RCCommand(arm=1800, throttle=int(throttle), roll=roll,
                     pitch=pitch, yaw=yaw_stick)


def _advance(t):
    prev = _state["phase"]
    if prev in ("SETTLE", "RECOVER"):
        if _state["queue"]:
            deg = _state["queue"].pop(0)
            _state["tilt"] = deg
            _state["amp"] = G * math.tan(math.radians(deg))
            _state["v_hist"] = []
            _state["phase"] = f"SPRINT_{int(deg)}"
        else:
            _state["phase"] = "DONE"
    elif prev.startswith("SPRINT"):
        _state["phase"] = f"BRAKE_{int(_state['tilt'])}"
    elif prev.startswith("BRAKE"):
        _state["phase"] = "RETURN"
    elif prev == "RETURN":
        _state["phase"] = "RECOVER"
    _state["t0"] = t
    print(f"[SYSID] t={t:5.1f} {prev} -> {_state['phase']}")


def reset_state():
    _rows.clear()
    _state.clear()


# ---------------------------------------------------------------------------

def _smooth(x, n):
    n = max(1, int(n) | 1)
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def analyze(csv_path, mass_kg=None):
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    t = d["t"]
    dirx, diry = DIRECTION
    v = d["vx"] * dirx + d["vy"] * diry
    vz = d["vz"]
    cos_t = np.clip(d["cos_tilt"], 1e-3, 1.0)
    dt = float(np.median(np.diff(t)))
    win = int(round(0.25 / dt))
    a_h = np.gradient(_smooth(v, win), t)
    a_z = np.gradient(_smooth(vz, win), t)
    tan_t = np.sqrt(1.0 - cos_t ** 2) / cos_t
    # horizontal thrust component, assuming the tilt vector lies along the
    # sprint direction (yaw held, lateral hold small) - sign by phase
    a_thrust = (a_z + G) * tan_t
    vs, losses = [], []
    print(f"{'tilt':>5s} {'v_term':>7s} {'t_sprint':>9s} {'dist':>6s} "
          f"{'tilt_act':>9s} {'a_h@3':>6s} {'a_h@6':>6s} {'a_h@8':>6s}")
    for deg in sorted(set(int(x) for x in d["tilt_deg"] if x > 0)):
        m = (d["tilt_deg"] == deg) & np.char.startswith(
            d["phase"].astype(str), "SPRINT")
        if m.sum() < 10:
            continue
        i0, i1 = np.flatnonzero(m)[[0, -1]]
        sl = slice(i0 + 3, i1 - 2)          # drop edge samples
        vv, aa, ath = v[sl], a_h[sl], a_thrust[sl]
        tilt_act = np.degrees(np.arccos(cos_t[sl]))
        dist = float(np.hypot(d["x"][i1] - d["x"][i0], d["y"][i1] - d["y"][i0]))

        def a_at(vq):
            j = np.flatnonzero(vv >= vq)
            return float(aa[j[0]]) if len(j) else float("nan")
        print(f"{deg:5d} {vv.max():7.2f} {t[i1] - t[i0]:9.2f} {dist:6.1f} "
              f"{np.median(tilt_act):9.1f} {a_at(3):6.2f} {a_at(6):6.2f} "
              f"{a_at(8):6.2f}")
        # only the steady-tilt part of the sprint contributes to the fit:
        # skip the first 0.4 s (attitude slew) and keep v > 1
        keep = (t[sl] - t[i0] > 0.4) & (vv > 1.0)
        vs.extend(vv[keep].tolist())
        losses.extend((ath[keep] - aa[keep]).tolist())
    vs = np.asarray(vs)
    losses = np.asarray(losses)
    if len(vs) < 20:
        print("not enough sprint samples to fit")
        return
    A = np.column_stack([vs, vs * vs])
    c, *_ = np.linalg.lstsq(A, losses, rcond=None)
    c1, c2 = float(c[0]), float(c[1])
    resid = losses - A @ c
    A2 = np.column_stack([vs * vs])
    cq, *_ = np.linalg.lstsq(A2, losses, rcond=None)
    print(f"\nequivalent loss accel fit over {len(vs)} samples:")
    print(f"  a_loss = {c1:.3f}*v + {c2:.4f}*v^2   (m/s^2; rms resid "
          f"{float(np.sqrt(np.mean(resid ** 2))):.2f})")
    print(f"  pure quadratic: a_loss = {float(cq[0]):.4f}*v^2")
    for vq in (3, 6, 9, 12):
        print(f"  v={vq:2d}: loss {c1 * vq + c2 * vq * vq:5.1f} m/s^2")
    if mass_kg:
        print(f"\nsuggested toml [vehicle] (mass {mass_kg} kg):")
        print(f"drag_lin  = {mass_kg * c1:.3f}")
        print(f"drag_quad = {mass_kg * c2:.4f}")
    print("(model fit through ground-truth sim state; the planner must "
          "consume it before any plan speed above the terminal speeds "
          "above is believable)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", required=True)
    ap.add_argument("--mass", type=float, default=None)
    args = ap.parse_args()
    analyze(args.analyze, args.mass)
