"""
ace_pilot - fly the solved trajectory (trajectory.json) at speed; vision + IMU only over the
wire (VQ2-legal). Architecture:

  FEEDFORWARD   the offline speed/heading profile is the plan (solver: trajectory.py)
  ESTIMATOR     position/velocity dead-reckoned from our OWN attitude commands through the
                measured 96 ms lag + validated drag model (v_est's proven recipe, in 2D), plus
                steady's leaky IMU vertical integrator. The sim holds attitude setpoints at
                gain 1.0 - the command IS the attitude.
  ANCHORING     every confident detection of the EXPECTED gate corrects the estimate: bearing
                innovation -> lateral, elevation innovation -> vertical, area-range -> along
                track. Anchoring is CHOOSY (unlike acquisition's fail-open rule): a wrong
                anchor corrupts the estimate, while flying the model blind for a leg is safe.
  FOLLOWER      pure pursuit on the sampled path: lookahead target + speed profile -> desired
                velocity vector -> bounded accel -> attitude setpoint + collective.

Conventions (flight-verified 2026-07, see COORDINATE_CONVENTIONS.md + runs 220336/223110):
  send_attitude_setpoint(roll, pitch, yaw, thrust): logged/sent +pitch = forward accel,
  +roll = rightward accel, yaw = CCW-positive world heading (spawn faces +97 deg).
  Map/trajectory frame: spawn at origin, axes = world yaw frame, z up.
"""

import csv
import json
import math
import os
import time

import keyboard

from common.camera import HALF_TAN_X, HALF_TAN_Y, HEIGHT, UPTILT_RAD, WIDTH
from common.dynamics import CONTROL_HZ, send_attitude_setpoint, send_rate_attitude
from common.paths import DATASETS_DIR
from pilots.ace_pilot.config import *   # ACE_* params

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ---- command tape (ACE_MODE = "tape": pure open-loop replay, Brian's directive - the top
# teams fly NO vision; on a deterministic sim the tape + per-gate offset iteration IS the
# correct architecture) --------------------------------------------------------------------
_TAPE = None


_VZREF = None
_VZREF_MTIME = None


def _vz_ref_at(t):
    # median vz profile of flown runs (analysis/make_vz_ref.py); mtime-reloaded like _tape.
    # None past the profile's end or when the file is absent -> damper contributes nothing.
    global _VZREF, _VZREF_MTIME
    p = os.path.join(HERE, "vz_ref.json")
    if not os.path.exists(p):
        return None
    mt = os.path.getmtime(p)
    if _VZREF is None or mt != _VZREF_MTIME:
        _VZREF = json.load(open(p))["samples"]     # [[t, vz], ...] ascending
        _VZREF_MTIME = mt
    if not _VZREF or t < _VZREF[0][0] or t > _VZREF[-1][0]:
        return None
    lo, hi = 0, len(_VZREF) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if _VZREF[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return _VZREF[lo][1]


def _tape():
    # reload whenever tape.json changes on disk: the client process persists across flights,
    # and a once-only cache silently flew STALE tapes for every rebuild (the lost hour of
    # "identical" jet impacts while the corridor never actually flew)
    global _TAPE, _TAPE_MTIME
    p = os.path.join(HERE, "tape.json")
    mt = os.path.getmtime(p)
    if _TAPE is None or mt != globals().get("_TAPE_MTIME"):
        d = json.load(open(p))
        _TAPE = d["commands"]           # [t, roll, pitch, yaw, thrust] at ~90 Hz sim time
        _TAPE_MTIME = mt
        print(f"[ace] tape loaded: {len(_TAPE)} commands, {_TAPE[-1][0]:.1f} s", flush=True)
    return _TAPE


# ---- trajectory ------------------------------------------------------------------------------
_TRAJ = None


def _traj():
    global _TRAJ
    if _TRAJ is None:
        d = json.load(open(os.path.join(HERE, "trajectory.json")))
        _TRAJ = {"s": [r[1] for r in d["samples"]],
                 "p": [(r[2], r[3], r[4]) for r in d["samples"]],
                 "v": [(r[5], r[6], r[7]) for r in d["samples"]],
                 "t": [r[0] for r in d["samples"]],
                 "gate_s": d["gate_s"],
                 "total": d["total_len_m"]}
        # acceleration feedforward per sample: central difference of the velocity samples -
        # pure velocity-error feedback turns LATE in corners (the offline sim's systematic
        # -1 m under-turn at every high-curvature gate); centripetal accel must be commanded
        # BEFORE the error develops
        n_ = len(_TRAJ["t"])
        acc = []
        for k in range(n_):
            k0, k1 = max(k - 1, 0), min(k + 1, n_ - 1)
            dtk = max(_TRAJ["t"][k1] - _TRAJ["t"][k0], 1e-3)
            acc.append(tuple((_TRAJ["v"][k1][m] - _TRAJ["v"][k0][m]) / dtk for m in range(3)))
        _TRAJ["a"] = acc
        print(f"[ace] trajectory loaded: {d['total_len_m']} m, plan {d['total_time_s']} s",
              flush=True)
    return _TRAJ


_MAP = None


def _cmap():
    global _MAP
    if _MAP is None:
        _MAP = json.load(open(os.path.join(HERE, "course_map.json")))["gates"]
    return _MAP


# ---- debug log -------------------------------------------------------------------------------
_DBG_W = _DBG_F = None
_DBG_N = 0
_DBG_HDR = ["t", "frame", "race_gate", "gate_i", "s_est", "x", "y", "z", "vx", "vy", "vz_est",
            "yaw_deg", "pitch_deg", "roll_deg", "thr", "v_prof", "v_est",
            "ax_cmd", "ay_cmd", "n_dets", "area", "ox", "oy", "ox_pred", "oy_pred",
            "anch_lat", "anch_z", "anch_rng"]


def _dbg_log(row):
    global _DBG_W, _DBG_F, _DBG_N
    if _DBG_W is None:
        path = os.path.join(DATASETS_DIR, time.strftime("ace_dbg_%Y%m%d_%H%M%S.csv"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _DBG_F = open(path, "w", newline="")
        _DBG_W = csv.writer(_DBG_F)
        _DBG_W.writerow(_DBG_HDR)
        print(f"ace debug -> {path}", flush=True)
    _DBG_W.writerow(row)
    _DBG_N += 1
    if _DBG_N % 30 == 0:
        _DBG_F.flush()


# ---- race gating (steady's proven pattern) ---------------------------------------------------
def _race_live(data):
    rs = data.get("race_status") or {}
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    finish = rs.get("race_finish_time_ns", -1)
    over = finish is not None and finish >= 0
    live = start is not None and start >= 0 and now >= start
    return live and not over


def _seconds_to_go(data):
    rs = data.get("race_status") or {}
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    if start is None or start < 0:
        return None
    return (start - now) / 1000.0


def _idle(mavlink_conn, system_boot_ms, data, regime):
    data["ace_regime"] = regime
    send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)


def _reset(data):
    data["_ace_x"] = 0.0
    data["_ace_y"] = 0.0
    data["_ace_vx"] = 0.0
    data["_ace_vy"] = 0.0
    data["_ace_th_lag"] = 0.0     # lagged pitch (rad, +fwd)
    data["_ace_ph_lag"] = 0.0     # lagged roll (rad, +right)
    data["_ace_vz"] = 0.0
    data["_ace_alt"] = 0.0
    data["_ace_vz_us"] = None
    data["_ace_us"] = None
    data["_ace_idx"] = 0          # path sample index (advance-only)
    data["_ace_gate_i"] = 0       # next gate to cross (index into gate_s / map)
    data["_ace_yaw"] = math.radians(97.1)   # spawn facing (world yaw frame, CCW+)
    data["_ace_psi_est"] = math.radians(97.1)  # LAGGED heading estimate - the sim tracks yaw
                                               # through ~150 ms too; treating heading as
                                               # instantly-commanded misdirected the velocity
                                               # integral through every fast turn (offline sim:
                                               # ~0.3 rad error at hairpin yaw rates)
    data["_ace_pitch"] = 0.0
    data["_ace_roll"] = 0.0
    data["_ace_fid"] = None
    data["_ace_t0_us"] = None
    data["_ace_anch_us"] = None   # last accepted anchor (starvation widens the accept tolerance)
    data["_ace_vscale"] = 1.0     # ONLINE velocity-scale adaptation: the dynamics constants are
                                  # VQ1 fits; run 120743's estimate ran ~40% fast on VQ2. The
                                  # range innovation stream measures the mismatch directly and
                                  # this factor absorbs it, slowly, bounded.
    data["_ace_thr_lag"] = 0.0        # ACTUAL thrust lags the command (motor response) - without
                                      # this the vertical model led reality by the motor lag and
                                      # the altitude servo limit-cycled +-1 m (real run 115245)
    data["_ace_hist"] = []        # (t_us, x, y, alt, psi, th) ring - anchors judge each frame
                                  # against the estimate AT FRAME TIME (offline-sim ablation:
                                  # with latency uncompensated the anchors INJECTED ~5 m of
                                  # error - worse than no vision; with it, 0.1-0.3 m all lap)


def _aim(d):
    """Aim-point offsets: the OPENING's bbox centre when the detector located the hole, else the
    structure centre (steady's oldest scar, never ported until real run 115245: on the tall
    start gate the structure centre sits ~1 m off the opening; the z-anchor consumed that
    offset as truth and the altitude estimate oscillated +-1 m at 2 Hz - physically impossible
    for the drone, entirely possible for an estimate fed alternating aim points)."""
    if getattr(d, "has_opening", False) and getattr(d, "bbox", None):
        bx, by, bw, bh = d.bbox
        return ((bx + bw / 2.0) - WIDTH / 2.0) / (WIDTH / 2.0),                ((by + bh / 2.0) - HEIGHT / 2.0) / (HEIGHT / 2.0)
    return d.offset_x, d.offset_y


def _vz_alt(data):
    """THRUST-MODEL vertical channel (validated 1.09 m/s mean vz error over the full envelope
    vs 13-26 for every IMU-integration variant - see config ACE_T0V block). Our own commanded
    collective through the tilt geometry and fitted vertical drag; no IMU accels involved, so
    no impact-rejection gate to eat real race accelerations and no leak to eat real climbs.
    The vision z-anchor owns the low-frequency truth."""
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")   # IMU timestamps still clock the integration
    if t_us is None:
        return data.get("_ace_vz", 0.0)
    prev = data.get("_ace_vz_us")
    if prev is not None and t_us > prev:
        dt = (t_us - prev) * 1e-6
        if dt <= 0.1:
            th = data.get("_ace_th_lag", 0.0)
            ph = data.get("_ace_ph_lag", 0.0)
            tl = data.get("_ace_thr_lag", 0.0)
            cmd_t = data.get("_ace_thr", ACE_HOVER)
            t0u = data.get("_ace_t0_us")
            if t0u is not None and (t_us - t0u) * 1e-6 < ACE_SPOOL_S:
                cmd_t = 0.0            # motors spooling: reality produces no thrust yet, and
                                       # the model must know it (floor start, V2)
            tl += (cmd_t - tl) * (1.0 - math.exp(-dt / ACE_THR_TAU))
            data["_ace_thr_lag"] = tl
            thr = tl
            vz = data.get("_ace_vz", 0.0)
            ge = 1.0 + ACE_GE_GAIN * max(0.0, 1.0 - data.get("_ace_alt", 0.0) / ACE_GE_H)
            lift = ge * G * (max(thr, 0.0) / ACE_T0V) ** ACE_THR_EXP
            a_up = (lift * math.cos(th) * math.cos(ph) - G
                    - (ACE_C1V + ACE_C2V * abs(vz)) * vz)
            vz_new = clamp(vz + a_up * dt, -35.0, 35.0)
            alt_new = data.get("_ace_alt", 0.0) + vz_new * dt
            if alt_new <= 0.0:
                # GROUND CONTACT in the model (floor start): the pad holds the drone up while
                # thrust < weight - without this the model free-fell through the spool and the
                # whole flight ran 10 m of phantom vertical offset
                alt_new = 0.0
                vz_new = max(vz_new, 0.0)
            data["_ace_vz"] = vz_new
            data["_ace_alt"] = alt_new
    data["_ace_vz_us"] = t_us
    return data.get("_ace_vz", 0.0)


def update_ace_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed("esc"):
        data["running"] = False

    if not _race_live(data):
        t_go = _seconds_to_go(data)
        _idle(mavlink_conn, system_boot_ms, data,
              f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0) else "IDLE")
        data["_ace_live"] = False
        # damper gravity baseline: on the ground, level, world-up specific force == g exactly
        imu = data.get("highres_imu") or {}
        if imu:
            up = -imu.get("zacc", 0.0)
            data["_dmp_g"] = 0.98 * data.get("_dmp_g", up) + 0.02 * up
        return

    if not data.get("_ace_live"):
        _reset(data)
        data["_ace_live"] = True

    if ACE_MODE == "tape":
        imu = data.get("highres_imu") or {}
        t_us = imu.get("time_usec")
        if t_us is None:
            return
        if data.get("_ace_t0_us") is None:
            data["_ace_t0_us"] = t_us
        t_race = (t_us - data["_ace_t0_us"]) * 1e-6
        tape = _tape()
        i = data.get("_ace_tape_i", 0)
        while i < len(tape) - 1 and tape[i + 1][0] <= t_race:
            i += 1
        data["_ace_tape_i"] = i
        _, roll, pitch, yaw, thr = tape[i]
        if t_race > tape[-1][0] + 0.5:
            roll = pitch = 0.0
            thr = ACE_HOVER

        # ---- IMU vertical damper: trim thrust toward the tape's own median vz profile ------
        # (leaky world-up specific-force integral; same math as make_vz_ref.py so biases are
        # common-mode. Zero correction on a median flight - the tuned path is untouched.)
        vz = data.get("_dmp_vz", 0.0)
        trim = 0.0
        prev_us = data.get("_dmp_us")
        data["_dmp_us"] = t_us
        if prev_us is not None and imu:
            dt = (t_us - prev_us) * 1e-6
            if 0.0 < dt <= 0.1:
                fx, fy, fz = imu.get("xacc", 0.0), imu.get("yacc", 0.0), imu.get("zacc", 0.0)
                up = -(-math.sin(pitch) * fx
                       + math.cos(pitch) * math.sin(roll) * fy
                       + math.cos(pitch) * math.cos(roll) * fz)
                g0 = data.get("_dmp_g", up)
                vz = (vz + (up - g0) * dt) * (1.0 - dt / ACE_DAMPER_TAU)
        data["_dmp_vz"] = vz
        if ACE_DAMPER_K > 0.0 and abs(roll) < ACE_DAMPER_ROLL_GATE:
            ref = _vz_ref_at(t_race)
            if ref is not None:
                trim = clamp(ACE_DAMPER_K * (ref - vz),
                             -ACE_DAMPER_CLAMP, ACE_DAMPER_CLAMP)
        thr_out = clamp(thr + trim, 0.0, 1.0)

        send_attitude_setpoint(mavlink_conn, system_boot_ms, roll, pitch, yaw, thr_out)
        data["ace_regime"] = f"TAPE {t_race:.1f}/{tape[-1][0]:.0f}s"
        data["_ace_thr"] = thr_out
        _dbg_log([f"{t_race:.3f}", "", (data.get("race_status") or {}).get("active_gate_index", ""),
                  "", "", "", "", "", "", "", f"{vz:+.3f}",
                  f"{math.degrees(yaw):+.1f}", f"{math.degrees(pitch):+.2f}",
                  f"{math.degrees(roll):+.2f}", f"{thr_out:.3f}",
                  f"{trim:+.4f}", "", "", "", "", "", "", "", "", "", "", "", ""])
        return

    traj = _traj()
    gates = _cmap()

    # ---- estimator propagate (IMU timestamps, never 1/CONTROL_HZ) ----------------------------
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")
    prev = data.get("_ace_us")
    dt = (t_us - prev) * 1e-6 if (t_us is not None and prev is not None) else 1.0 / CONTROL_HZ
    if not (0.0 < dt <= 0.1):
        dt = 1.0 / CONTROL_HZ
    if t_us is not None:
        data["_ace_us"] = t_us
        if data.get("_ace_t0_us") is None:
            data["_ace_t0_us"] = t_us

    t_race = ((t_us - data["_ace_t0_us"]) * 1e-6
              if (t_us is not None and data.get("_ace_t0_us") is not None) else 0.0)
    launching = t_race < ACE_LAUNCH_S

    a = 1.0 - math.exp(-dt / ACE_ATT_TAU)
    data["_ace_th_lag"] += (data["_ace_pitch"] - data["_ace_th_lag"]) * a
    data["_ace_ph_lag"] += (data["_ace_roll"] - data["_ace_ph_lag"]) * a
    ay_ = 1.0 - math.exp(-dt / ACE_YAW_TAU)
    data["_ace_psi_est"] += _wrap_pi(data["_ace_yaw"] - data["_ace_psi_est"]) * ay_
    psi = data["_ace_psi_est"]
    fwd = (math.cos(psi), math.sin(psi))
    right = (math.sin(psi), -math.cos(psi))
    # THRUST-SCALED horizontal accel (replay-validated: halves the error vs g*tan(tilt), which
    # silently assumes hover-equilibrium collective - wrong whenever the vertical servo or a
    # big tilt moves the collective, which in a race is always)
    _thr_prev = data.get("_ace_thr_lag", ACE_HOVER)
    # horizontal stays LINEAR in collective (the 1.45 exponent is a VERTICAL measurement;
    # applying it horizontally over-modeled accel ~15-20% and squished every leg - attempt 3)
    _tw = G * (max(_thr_prev, 0.0) / ACE_T0V) * data.get("_ace_vscale", 1.0)
    af = _tw * math.cos(data["_ace_ph_lag"]) * math.sin(data["_ace_th_lag"])
    ar = _tw * math.cos(data["_ace_th_lag"]) * math.sin(data["_ace_ph_lag"])
    sp2d = math.hypot(data["_ace_vx"], data["_ace_vy"])
    drag = ACE_DRAG_C1 + ACE_DRAG_C2 * sp2d
    # no horizontal integration while on the pad - ground contact is not in the model, and
    # flight 3 integrated 20 deg of commanded pitch into phantom motion before liftoff
    if not launching:
        data["_ace_vx"] += (af * fwd[0] + ar * right[0] - drag * data["_ace_vx"]) * dt
        data["_ace_vy"] += (af * fwd[1] + ar * right[1] - drag * data["_ace_vy"]) * dt
        data["_ace_x"] += data["_ace_vx"] * dt
        data["_ace_y"] += data["_ace_vy"] * dt
    vz_est = _vz_alt(data)
    x, y, z = data["_ace_x"], data["_ace_y"], data["_ace_alt"]
    if t_us is not None:
        h = data["_ace_hist"]
        h.append((t_us, x, y, z, psi, data["_ace_th_lag"]))
        if len(h) > 45:
            del h[0]

    if launching:
        # LEVEL VERTICAL HOP: clean liftoff before the follower gets authority. Attitude level,
        # yaw held at spawn facing, collective SERVOED to a gentle climb rate - the first
        # open-loop version (0.40 for 0.9 s = ~4.7 m/s^2 for a full second) rocketed the drone
        # far above gate height and the follower spent the first leg diving back down.
        data["_ace_pitch"] = 0.0
        data["_ace_roll"] = 0.0
        _lr = max(G + ACE_K_VZ * (ACE_LAUNCH_VZ - vz_est), 0.1)
        thr = clamp(ACE_T0V * (_lr / G) ** (1.0 / ACE_THR_EXP), 0.10, 0.45)
        send_attitude_setpoint(mavlink_conn, system_boot_ms, 0.0, 0.0, psi, thr)
        data["ace_regime"] = "LAUNCH"
        data["_ace_thr"] = thr
        return

    # ---- gate progression: GEOMETRIC, never via the corruptible arc estimate ----------------
    # Offline-sim finding (run 112708): after passing gate 0, the anchor still targeted gate 0
    # - behind us, out of view - matched OTHER gates' detections to its prediction, dragged the
    # estimate backward, which froze s_est, which was what advanced the gate index: a circular
    # death loop. The target gate now advances the moment its plane is at/behind the estimated
    # position, independent of the path-index machinery.
    gi = data["_ace_gate_i"]
    while gi < len(gates):
        _g = gates[gi]
        _along = (_g["x"] - x) * math.cos(psi) + (_g["y"] - y) * math.sin(psi)
        if _along < 1.0:
            gi += 1
            data["_ace_gate_i"] = gi
        else:
            break

    # ---- vision anchoring --------------------------------------------------------------------
    n_dets, area, ox, oy = 0, 0.0, 0.0, 0.0
    ox_pred = oy_pred = 0.0
    anch_lat = anch_z = anch_rng = 0.0
    fid = data.get("latest_frame_id")
    dets = data.get("vision_gates") or []
    if fid is not None and fid != data.get("_ace_fid") and dets and gi < len(gates):
        data["_ace_fid"] = fid
        n_dets = len(dets)
        g = gates[gi]
        # LATENCY COMPENSATION: the frame shows the world as of ~ACE_VIS_LAT_S ago - judge it
        # against the estimate from THEN, apply the correction NOW (the error is persistent)
        hx, hy, hz, hpsi, hth = x, y, z, psi, data["_ace_th_lag"]
        if t_us is not None:
            t_frame = t_us - int(ACE_VIS_LAT_S * 1e6)
            for hrec in data.get("_ace_hist", []):
                if hrec[0] <= t_frame:
                    hx, hy, hz, hpsi, hth = hrec[1], hrec[2], hrec[3], hrec[4], hrec[5]
                else:
                    break
        dx, dy, dz = g["x"] - hx, g["y"] - hy, g["z"] - hz
        rng_pred = math.hypot(dx, dy)
        if rng_pred > 1.5:
            bear = _wrap_pi(math.atan2(dy, dx) - hpsi)         # +left of nose (CCW)
            el = math.atan2(dz, rng_pred)                       # +above drone
            ox_pred = -math.tan(bear) / HALF_TAN_X              # image x: +right
            oy_pred = math.tan(UPTILT_RAD - el - hth) / HALF_TAN_Y
            # candidate = detection CLOSEST to the predicted bearing (the biggest blob is often
            # a different gate - n_dets ran 3-4 all of flight 2), tolerance WIDENING with time
            # since the last accepted anchor so a drifted estimate can always re-acquire.
            la = data.get("_ace_anch_us")
            starve_s = ((t_us - la) * 1e-6 if (t_us is not None and la is not None) else 0.0)
            tol = min(ACE_ANCHOR_OX_TOL + ACE_ANCHOR_TOL_GROW * starve_s, ACE_ANCHOR_TOL_MAX)
            # candidates must be plausible in bearing AND range (a far gate's small blob must
            # not be matched to a near prediction - the wrong-gate anchor is the death loop)
            cands = [d for d in dets
                     if ACE_ANCHOR_MIN_AREA <= d.area_frac <= ACE_ANCHOR_MAX_AREA
                     and 0.5 <= (ACE_C_RNG / math.sqrt(d.area_frac)) / rng_pred <= 2.0]
            best = min(cands, key=lambda d: abs(_aim(d)[0] - ox_pred)) if cands else None
            if best is not None and abs(_aim(best)[0] - ox_pred) <= tol:
                data["_ace_anch_us"] = t_us
                area = best.area_frac
                ox, oy = _aim(best)
                data["_ace_home"] = (t_us, ox, rng_pred)   # terminal-homing memory
                # SIGN-SAFE-BY-CONSTRUCTION anchor (flight 1's lesson, run 103152: a hand-
                # derived vertical sign was INVERTED - positive feedback ran z_est to +174 m in
                # ten frames, thrust pinned at the floor clamp, drone never left the ground).
                # Identity instead of hand-derived signs: project the MEASURED line of sight
                # from the ESTIMATED position; the apparent gate lands at p + rng*LoS, the real
                # gate is at the map position, so the position error IS
                #     (apparent gate - map gate)
                # in one vector expression, all three axes. Every correction is also clamped
                # per frame, so no future geometry bug can run away in ten frames again.
                bear_m = -math.atan(ox * HALF_TAN_X)            # measured bearing, +CCW of nose
                el_m = UPTILT_RAD - hth - math.atan(oy * HALF_TAN_Y)
                cb, sb = math.cos(hpsi + bear_m), math.sin(hpsi + bear_m)
                ce, se = math.cos(el_m), math.sin(el_m)
                gax = hx + rng_pred * cb * ce                   # apparent gate (frame-time frame)
                gay = hy + rng_pred * sb * ce
                gaz = hz + rng_pred * se
                # absurd-innovation rejection: an error this large means the match is a WRONG
                # gate, and applying it (even step-capped) feeds the death loop
                if math.hypot(gax - g["x"], gay - g["y"]) <= 6.0:
                    stp = ACE_ANCHOR_STEP_MAX
                    ex = clamp(ACE_ANCHOR_GAIN_LAT * (gax - g["x"]), -stp, stp)
                    ey = clamp(ACE_ANCHOR_GAIN_LAT * (gay - g["y"]), -stp, stp)
                    ez = clamp(ACE_ANCHOR_GAIN_Z * (gaz - g["z"]), -stp, stp)
                    data["_ace_x"] -= ex
                    data["_ace_y"] -= ey
                    data["_ace_alt"] -= ez
                    anch_lat = -math.hypot(ex, ey)
                    anch_z = -ez
                    # along-track: area-range innovation (weakest gain - noisiest channel).
                    # rng_pred > rng_meas = we are closer than estimated -> slide estimate
                    # toward the gate along the line of sight.
                    rng_meas = ACE_C_RNG / math.sqrt(best.area_frac)
                    drng = clamp(ACE_ANCHOR_GAIN_RNG * (rng_pred - rng_meas), -stp, stp)
                    data["_ace_x"] += drng * cb * ce
                    data["_ace_y"] += drng * sb * ce
                    anch_rng = drng
                    # velocity-scale adaptation: persistently measuring the gate FARTHER than
                    # predicted means the model accelerates faster than this sim's reality
                    data["_ace_vscale"] = clamp(
                        data.get("_ace_vscale", 1.0)
                        + ACE_VSCALE_K * (rng_meas - rng_pred), 0.6, 1.4)
                    x, y, z = data["_ace_x"], data["_ace_y"], data["_ace_alt"]

    # ---- follower ----------------------------------------------------------------------------
    s_arr, p_arr, v_arr = traj["s"], traj["p"], traj["v"]
    n = len(s_arr)
    i = data["_ace_idx"]
    # advance-only nearest sample within a forward window
    best_i, best_d = i, float("inf")
    for j in range(i, min(i + 80, n)):
        d2 = (p_arr[j][0] - x) ** 2 + (p_arr[j][1] - y) ** 2 + (p_arr[j][2] - z) ** 2
        if d2 < best_d:
            best_d, best_i = d2, j
    i = best_i
    data["_ace_idx"] = i
    s_est = s_arr[i]
    # gate progression by arc position
    while gi < len(traj["gate_s"]) and s_est > traj["gate_s"][gi] + 1.0:
        gi += 1
        data["_ace_gate_i"] = gi

    v_est_vec = (data["_ace_vx"], data["_ace_vy"], vz_est)
    v_here = math.hypot(*v_arr[i])
    L = clamp(ACE_LOOKAHEAD_K * max(sp2d, v_here), ACE_LOOKAHEAD_MIN, ACE_LOOKAHEAD_MAX)
    j = i
    while j < n - 1 and s_arr[j] - s_est < L:
        j += 1
    tgt = p_arr[j]
    v_prof = math.hypot(*v_arr[j])
    # desired velocity: profile speed toward the lookahead point + explicit cross-track pull
    # toward the NEAREST path point (pure pursuit alone cuts corners by ~1 m at speed - the
    # offline sim's five remaining misses were all sub-1.6 m laterals from exactly this)
    to_tgt = (tgt[0] - x, tgt[1] - y, tgt[2] - z)
    dist = math.sqrt(sum(c * c for c in to_tgt)) or 1.0
    dirv = tuple(c / dist for c in to_tgt)
    near = p_arr[i]
    v_des = tuple(v_prof * dirv[k] + ACE_K_POS * (near[k] - (x, y, z)[k]) for k in range(3))
    # TERMINAL VISUAL HOMING: inside ACE_HOME_RNG of the target gate, steer at the MEASURED
    # opening - nulls map + estimator + tracking error at the crossing plane itself (steady's
    # doctrine at speed; the follower's ~0.8 m tracking floor lives exactly here otherwise)
    hm = data.get("_ace_home")
    if hm is not None and t_us is not None and (t_us - hm[0]) * 1e-6 < 0.25 and hm[2] < ACE_HOME_RNG:
        v_corr = ACE_K_HOME * hm[1] * max(sp2d, 2.0)
        v_des = (v_des[0] + v_corr * math.sin(psi),
                 v_des[1] - v_corr * math.cos(psi),
                 v_des[2])
    a_ff = traj["a"][i]   # at the NEAREST sample: at the lookahead it fires corners early
    ax_cmd = clamp(ACE_K_V * (v_des[0] - v_est_vec[0]) + ACE_A_FF_GAIN * a_ff[0],
                   -ACE_A_CMD_MAX, ACE_A_CMD_MAX)
    ay_cmd = clamp(ACE_K_V * (v_des[1] - v_est_vec[1]) + ACE_A_FF_GAIN * a_ff[1],
                   -ACE_A_CMD_MAX, ACE_A_CMD_MAX)
    # drag compensation so the profile speed is actually held
    ax_cmd += drag * data["_ace_vx"]
    ay_cmd += drag * data["_ace_vy"]

    # yaw follows the path tangent (camera leads into the next gate)
    yaw_cmd_prev = data["_ace_yaw"]
    yaw_des = math.atan2(v_arr[j][1], v_arr[j][0]) if v_prof > 0.3 else yaw_cmd_prev
    dpsi = _wrap_pi(yaw_des - yaw_cmd_prev)
    psi = yaw_cmd_prev + clamp(dpsi, -ACE_YAW_SLEW * dt, ACE_YAW_SLEW * dt)
    data["_ace_yaw"] = psi

    # accel -> attitude (body frame of the NEW heading)
    fwd = (math.cos(psi), math.sin(psi))
    right = (math.sin(psi), -math.cos(psi))
    a_fwd = ax_cmd * fwd[0] + ay_cmd * fwd[1]
    a_rgt = ax_cmd * right[0] + ay_cmd * right[1]
    # SPEED-SCALED lean cap (real run 114324: the pursuit demanded ~5 m/s right after the
    # launch hop -> +38 deg nose-down at 0.7 m altitude and walking pace -> floor strike; the
    # offline sim had been printing "min alt 0.00" - dragging the floor - all along and only a
    # crash condition made it count). Full authority arrives with airspeed.
    lean_cap = clamp(0.20 + 0.055 * sp2d, 0.20, ACE_LEAN_MAX)
    # NO NOSE-UP IN GENERATION (Brian standing law; and a correctness bug): iterate_tape
    # clamps nose-up out of the finished tape, so a follower that climbs by pitching back
    # produced a tape whose roll timing was computed at 0.6-3 m/s while the FLOWN tape (with
    # the nose-up stripped) crosses the same ground at 4-8 m/s - the same rolls then turned
    # the drone metres further. Generation now flies under the same constraint as the tape,
    # so the sim verifies what actually flies. Altitude is thrust's job, not pitch's.
    pitch_des = clamp(math.atan(a_fwd / G), -ACE_PITCH_UP_MAX, lean_cap)
    roll_des = clamp(math.atan(a_rgt / G), -lean_cap, lean_cap)
    data["_ace_pitch"] += clamp(pitch_des - data["_ace_pitch"], -ACE_ATT_SLEW * dt, ACE_ATT_SLEW * dt)
    data["_ace_roll"] += clamp(roll_des - data["_ace_roll"], -ACE_ATT_SLEW * dt, ACE_ATT_SLEW * dt)

    # thrust: hover + climb-rate servo toward the profile altitude, tilt-compensated
    # altitude servos on the NEAREST path point - the lookahead z leads climbs by up to a
    # metre (offline sim: gate 5 crossed +1.1 high); the climb-rate feedforward anticipates
    vz_des = clamp(ACE_K_Z * (near[2] + ACE_Z_EXEC_TRIM - z) + v_arr[i][2],
                   -ACE_VZ_MAX, ACE_VZ_MAX)
    a_up = ACE_K_VZ * (vz_des - vz_est)          # m/s^2 of vertical accel demand
    tilt = max(math.cos(data["_ace_pitch"]) * math.cos(data["_ace_roll"]), 0.5)
    # INVERT the measured lift curve: required lift -> collective. The old linear inversion
    # overthrusted every tilt by (1/cos)^(p-1) - the measured brake balloon.
    ge = 1.0 + ACE_GE_GAIN * max(0.0, 1.0 - z / ACE_GE_H)
    lift_req = max((G + a_up) / tilt / ge, 0.1)
    thrust = clamp(ACE_T0V * (lift_req / G) ** (1.0 / ACE_THR_EXP), 0.05, 0.95)

    send_attitude_setpoint(mavlink_conn, system_boot_ms, data["_ace_roll"],
                           data["_ace_pitch"], psi, thrust)

    data["ace_regime"] = f"RACE g{gi} s={s_est:.0f}"
    data["_ace_thr"] = thrust
    t_s = ((t_us - data["_ace_t0_us"]) * 1e-6
           if (t_us is not None and data.get("_ace_t0_us") is not None) else 0.0)
    _dbg_log([f"{t_s:.3f}", ("" if fid is None else fid),
              (data.get("race_status") or {}).get("active_gate_index", ""), gi, f"{s_est:.1f}",
              f"{x:+.2f}", f"{y:+.2f}", f"{z:+.2f}",
              f"{data['_ace_vx']:+.2f}", f"{data['_ace_vy']:+.2f}", f"{vz_est:+.2f}",
              f"{math.degrees(psi):+.1f}", f"{math.degrees(data['_ace_pitch']):+.2f}",
              f"{math.degrees(data['_ace_roll']):+.2f}", f"{thrust:.3f}",
              f"{v_prof:.2f}", f"{sp2d:.2f}", f"{ax_cmd:+.2f}", f"{ay_cmd:+.2f}",
              n_dets, f"{area:.4f}", f"{ox:+.3f}", f"{oy:+.3f}",
              f"{ox_pred:+.3f}", f"{oy_pred:+.3f}",
              f"{anch_lat:+.3f}", f"{anch_z:+.3f}", f"{anch_rng:+.3f}"])
