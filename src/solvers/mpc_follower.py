"""Model-predictive tracker for a plan (CasADi/IPOPT, receding horizon).

Same plant model as raceline.traj_opt: point mass with the measured quadratic
drag, and the achieved thrust vector following the commanded one through a
first-order attitude lag. Every MPC_PERIOD_S the controller reads the true
state, takes the plan's reference over the next HORIZON_S (positions,
velocities and accelerations at the plan's own timing, anchored at the
nearest point on the path), and solves for the thrust commands that track it
subject to the thrust ceiling. The first command becomes the roll/pitch
sticks through the shared tilt mapping and the collective through the
thrust curve; yaw follows the plan's nose profile as in solvers.follower.

Because the sim is lockstep, solve time costs wall clock, not correctness.

    RACE_SOLVER=solvers.mpc_follower AIGP_TRAJ=/path/plan.json (entrypoint sets both)
"""
from __future__ import annotations

import math
import os
import time

import numpy as np

from raceline import course as course_bridge  # ensures sim repo on sys.path
from raceline import planner as plan_io
from raceline.config import G, load_config
course_bridge.pq_course()          # side effect: sim repo importable
from solver.api import RCCommand, SensorUpdate  # noqa: E402
from raceline.rc_backend import (   # noqa: E402
    GroundTruthSource, StateEstimate, YawLoop, attitude_sticks)
from solvers.follower import Tracker  # noqa: E402  (nearest-point index, fold flags, yaw target)

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75
_TRAJ_PATH = os.environ.get("AIGP_TRAJ")
if not _TRAJ_PATH:
    raise RuntimeError("solvers.mpc_follower needs AIGP_TRAJ=/path/to/plan.json")
CFG = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
PLAN = plan_io.load_plan(_TRAJ_PATH)

HORIZON_S = 0.6
N_NODES = 12
MPC_PERIOD_S = 0.02        # re-solve every 20 ms; hold the command in between
TAU_ATT = 0.10             # attitude lag (s), the same number the optimizer plans with
U_MAX = float(max(CFG.thrust.curve_acc)) * 0.72  # batches 14-16: at 0.85 (throttle 0.67) the mixer clipped the collective 52-67% of the flight (one motor at 1.0, mean 0.5); 0.72 (throttle ~0.55) leaves the attitude loop its share
TILT_MAX_DEG = 70.0        # commanded thrust-vector tilt cap (batch 14, g6: 83-90 deg commanded, sideways shove during the attitude transient)
U_ZMIN = 3.0               # thrust floor the plant really delivers at idle throttle: motor idle plus the mixer lift from the rate loop (race_126: -5 m/s^2 net fall, sticks centred); 1.0 planned a fall the drone could not do and the tracker pulled out early
LOW_TILT_DEG = 30.0        # tilt cone apex at the thrust floor (see _build_mpc)
FLOOR_HOLD_BAND = 1.0      # |u| below U_ZMIN + this -> attitude hold, no steering (see autopilot)
R_DU = 0.05                # penalty on the command change between nodes: no attitude flapping
# Yaw is FROZEN (stick 1500): the referee ignores the nose, the race camera
# looks down, and any yaw demand steals collective through the mixer (race_113:
# 1900 for 0.3 s in the g9 climb halved the mean motor; race_115: a clamped
# +-60 still pinned one motor at 1.0 for the whole climb and trimmed the
# throttle 0.577 -> mean 0.54, so the planned 27 m/s^2 arrived as 23)
YAW_FROZEN = True
# delivered/commanded thrust ratio, tracked from the mixer's mean motor output
# through the thrust curve (a real FC does not report motors; the IMU specific
# force is the equivalent signal there); the collective is commanded at |u|/eta
ETA_TAU_S = 0.3
ETA_MIN, ETA_MAX = 0.70, 1.0
Q_POS, Q_VEL, R_U, Q_TERM = 40.0, 4.0, 0.02, 80.0
K_D = np.asarray(CFG.drag_k_xyz(), dtype=float)   # per-axis quadratic drag, 1/m (vertical is ~2x horizontal on this plant)

_SOURCE = GroundTruthSource()
_TRACKER = Tracker(PLAN, CFG)
_YAW = YawLoop(CFG)
_state = {"done_t": None, "last_solve_t": -1.0, "u_cmd": np.array([0.0, 0.0, G]),
          "a_est": np.array([0.0, 0.0, G]), "trace": None, "trace_n": 0, "solves": 0, "solve_ms": 0.0,
          "mpc": None, "u_warm": None, "eta": 1.0, "thr_prev": None, "t_prev": None}
TRACE_EVERY = 10
T_ARR = PLAN["t_arr"]


def _build_mpc():
    """Parametric NLP: given the current state (p, v, a) and the reference
    (p_ref, v_ref, u_ref over the horizon), return the command sequence."""
    import casadi as ca
    K_D_CA = ca.DM(K_D)
    N = N_NODES; dt = HORIZON_S / N
    opti = ca.Opti()
    X = opti.variable(9, N + 1)
    U = opti.variable(3, N)
    x0 = opti.parameter(9)
    Pref = opti.parameter(3, N + 1)
    Vref = opti.parameter(3, N + 1)
    Uref = opti.parameter(3, N)
    opti.subject_to(X[:, 0] == x0)
    J = 0
    for i in range(N):
        u = U[:, i]
        def f(x_):
            v_ = x_[3:6]; a_ = x_[6:9]
            sp_ = ca.sqrt(ca.sumsqr(v_) + 1e-6)
            return ca.vertcat(v_, a_ + ca.vertcat(0, 0, -G) - K_D_CA * sp_ * v_, (u - a_) / TAU_ATT)
        x = X[:, i]
        k1 = f(x); k2 = f(x + dt / 2 * k1); k3 = f(x + dt / 2 * k2); k4 = f(x + dt * k3)
        opti.subject_to(X[:, i + 1] == x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
        opti.subject_to(ca.sumsqr(u) <= U_MAX ** 2)
        opti.subject_to(u[2] >= U_ZMIN)
        # tilt cone: TILT_MAX_DEG above the thrust floor, LOW_TILT_DEG at it
        # (race_122/123: at u_z = 1 the 70 deg cone let a 2.7 m/s^2 correction
        # command 70 deg of tilt; the sticks slammed, the mixer lifted idle
        # throttle to fit the mix and the drone floated at the top of the stack)
        opti.subject_to(math.tan(math.radians(TILT_MAX_DEG)) * (u[2] - U_ZMIN) + math.tan(math.radians(LOW_TILT_DEG)) * U_ZMIN
                        >= ca.sqrt(u[0] ** 2 + u[1] ** 2 + 1e-6))
        if i > 0:
            J += R_DU * ca.sumsqr(u - U[:, i - 1])
        J += Q_POS * ca.sumsqr(X[0:3, i + 1] - Pref[:, i + 1]) + Q_VEL * ca.sumsqr(X[3:6, i + 1] - Vref[:, i + 1])
        J += R_U * ca.sumsqr(u - Uref[:, i])
    J += Q_TERM * ca.sumsqr(X[0:3, N] - Pref[:, N])
    opti.minimize(J)
    opti.solver("ipopt", {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": 60,
                          "ipopt.tol": 1e-3, "ipopt.acceptable_tol": 1e-2, "ipopt.warm_start_init_point": "yes",
                          "ipopt.mu_init": 1e-2})
    return {"opti": opti, "X": X, "U": U, "x0": x0, "Pref": Pref, "Vref": Vref, "Uref": Uref, "dt": dt}


def _reference(t_ref0: float, rate: float = 1.0):
    """Plan reference over the horizon starting at plan time t_ref0."""
    N = N_NODES; dt = HORIZON_S / N
    tq = t_ref0 + rate * dt * np.arange(N + 1)   # rate < 1: the plan clock runs slower than the horizon
    tq = np.minimum(tq, T_ARR[-1])
    P = np.column_stack([np.interp(tq, T_ARR, PLAN["pos"][:, j]) for j in range(3)])
    V = np.column_stack([np.interp(tq, T_ARR, PLAN["vel"][:, j]) for j in range(3)])
    A = np.column_stack([np.interp(tq, T_ARR, PLAN["acc"][:, j]) for j in range(3)])
    sp = np.linalg.norm(V, axis=1, keepdims=True)
    U = A + np.array([0.0, 0.0, G]) + K_D[None, :] * sp * V
    U[:, 2] = np.maximum(U[:, 2], U_ZMIN)
    return P, V, U[:N]


def _solve(est: StateEstimate, t_ref0: float, rate: float = 1.0):
    m = _state["mpc"]
    if m is None:
        m = _state["mpc"] = _build_mpc()
    P, V, U = _reference(t_ref0, rate)
    x0 = np.concatenate([est.p, est.v, _state["a_est"]])
    m["opti"].set_value(m["x0"], x0); m["opti"].set_value(m["Pref"], P.T); m["opti"].set_value(m["Vref"], V.T); m["opti"].set_value(m["Uref"], U.T)
    if _state["u_warm"] is not None:
        m["opti"].set_initial(m["U"], _state["u_warm"])
    Xg = np.vstack([P.T, V.T, np.column_stack([U.T, U.T[:, -1]])])
    m["opti"].set_initial(m["X"], Xg)
    t0 = time.time()
    try:
        sol = m["opti"].solve()
        Uo = np.array(sol.value(m["U"]))
    except RuntimeError:
        Uo = np.array(m["opti"].debug.value(m["U"]))
    _state["solve_ms"] = (time.time() - t0) * 1000.0
    _state["solves"] += 1
    _state["u_warm"] = np.column_stack([Uo[:, 1:], Uo[:, -1]])
    return Uo[:, 0]


def _trace_open():
    try:
        from raceline.planner import next_numbered
        from raceline.config import AIGP_REPO
        path = next_numbered(str(AIGP_REPO / "out" / "flightlogs" / "race_XXX.csv"))
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = open(path, "w", encoding="utf-8")
        fh.write("t,s,gate,x,y,z,vx,vy,vz,ax_des,ay_des,z_target,vz_ff,cos_tilt,wx,wy,a_z_cmd,thrust_cmd,roll,pitch,throttle,yaw,m_max,m_mean,solve_ms,eta,m_br,m_fr,m_bl,m_fl,wz\n")
        print(f"[MPC] trace -> {path}")
        return fh
    except Exception as ex:
        print(f"[MPC] trace disabled: {ex}")
        return None


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)
    est = _SOURCE.estimate(update)
    _, z_target, vz_ff, yaw_des, done = _TRACKER.step(est, update.next_gate_index)
    if done:
        if _state["done_t"] is None:
            _state["done_t"] = t
        if t - _state["done_t"] > 1.0:
            return RCCommand(arm=1000, throttle=1000)
    airborne = est.p[2] >= CFG.follower.min_alt_translation_m
    # plan time at the nearest point: the reference horizon starts there
    t_ref0 = float(T_ARR[_TRACKER.idx])
    if airborne and (t - _state["last_solve_t"] >= MPC_PERIOD_S):
        # (a plan clock slowed to the drone's speed ratio at the thrust floor was
        # tried in race_129/130 and made the earlier low-thrust sections worse)
        _state["u_cmd"] = _solve(est, t_ref0)
        _state["last_solve_t"] = t
    u = _state["u_cmd"] if airborne else np.array([0.0, 0.0, G])
    a_h = np.array([u[0], u[1]]); az = float(u[2] - G)
    t_mag = float(np.linalg.norm(u))
    # ACHIEVED thrust vector from the MEASURED attitude: the body z-axis times
    # the DELIVERED collective (mean motor output through the thrust curve).
    # The MPC's initial state then carries the real attitude lag and the
    # mixer's trimming instead of a model guess (batch 15: the model-only
    # estimate let the tracker under-thrust through the g3 turn and take the
    # frame; batch 17: the commanded 27 m/s^2 arrived as 23 on the g9 climb)
    zb = np.asarray(est.R[:, 2], dtype=float)
    motors = getattr(update, "motors", None)
    t_del = t_mag
    if airborne and motors is not None and len(motors) > 0:
        pwm_eq = 1000.0 + 1000.0 * float(np.mean(motors))
        t_del = float(np.interp(pwm_eq, CFG.thrust.curve_pwm, CFG.thrust.curve_acc))
        thr_prev = _state["thr_prev"]
        if thr_prev is not None and thr_prev > CFG.thrust.hover_pwm:
            t_asked = float(np.interp(thr_prev, CFG.thrust.curve_pwm, CFG.thrust.curve_acc))
            r = min(max(t_del / max(t_asked, 1e-6), ETA_MIN), ETA_MAX)
            dt = max(t - _state["t_prev"], 1e-3) if _state["t_prev"] is not None else 0.01
            k = min(dt / ETA_TAU_S, 1.0)
            _state["eta"] += k * (r - _state["eta"])
    _state["a_est"] = zb * t_del if airborne else np.array([0.0, 0.0, G])
    t_ask = min(t_mag / _state["eta"], float(max(CFG.thrust.curve_acc)))
    throttle = int(round(CFG.pwm_for_thrust(t_ask))) if airborne else int(round(CFG.pwm_for_thrust(1.5 * G)))
    _state["thr_prev"], _state["t_prev"] = throttle, t
    roll = pitch = yaw_stick = 1500
    if airborne:
        roll, pitch, eb = attitude_sticks(CFG, est, a_h, az)
        if t_mag < U_ZMIN + FLOOR_HOLD_BAND:
            # THRUST AT THE FLOOR (the stack drop): hold the body still instead
            # of steering it. Betaflight (airmode off) fits any attitude mix
            # by RAISING idle throttle (mixer.c: throttle >= -minMotor), so
            # every correction at idle is lift: race_125 fell at 2.7 m/s^2
            # with a quiet 20 deg attitude and motor mean 0.1-0.3. Rates
            # held at zero cost ~nothing; the MPC steers again on the pull-out.
            roll = pitch = 1500
        yaw_stick = 1500 if YAW_FROZEN else _YAW.stick(est, yaw_des)
        if throttle < CFG.thrust.hover_pwm - 150:
            yaw_stick = 1500
    _state["trace_n"] += 1
    if _state["trace_n"] == 1:
        _state["trace"] = _trace_open()
    fh = _state["trace"]
    if fh is not None and _state["trace_n"] % TRACE_EVERY == 0:
        w = est.omega if est.omega is not None else (0.0, 0.0, 0.0)
        m_max = float(np.max(update.motors)) if getattr(update, "motors", None) is not None else 0.0
        m_mean = float(np.mean(update.motors)) if getattr(update, "motors", None) is not None else 0.0
        m4 = list(update.motors)[:4] if getattr(update, "motors", None) is not None and len(update.motors) >= 4 else [0.0] * 4
        fh.write(f"{t:.3f},{_TRACKER.s[_TRACKER.idx]:.2f},{update.next_gate_index},"
                 f"{est.p[0]:.3f},{est.p[1]:.3f},{est.p[2]:.3f},{est.v[0]:.3f},{est.v[1]:.3f},{est.v[2]:.3f},"
                 f"{a_h[0]:.2f},{a_h[1]:.2f},{z_target:.2f},{vz_ff:.2f},{est.R[2, 2]:.3f},{float(w[0]):.2f},{float(w[1]):.2f},"
                 f"{az:.2f},{t_mag:.2f},{roll},{pitch},{throttle},{yaw_stick},{m_max:.2f},{m_mean:.2f},{_state['solve_ms']:.1f},{_state['eta']:.3f},{m4[0]:.2f},{m4[1]:.2f},{m4[2]:.2f},{m4[3]:.2f},{float(w[2]) if len(w) > 2 else 0.0:.2f}\n")
    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw_stick)
