"""
Steady pilot v2 - an attitude-setpoint visual servo. The sim balances; we steer.

Post-mortem of v1 (rate loop + IMU attitude filter + world-frame bearing servo): four flights, four
failures, and every root cause lived in the layer UNDER the steering - the estimated attitude biased
the bearings (flew 1 m under the opening reading the gate "below the horizon"), the unobservable
velocity left every vertical command open-loop (launch overshot 2x, then the ceiling), and the aim
point was the ring-bbox centre, ~1 m off the hole on the start structure. None of that layer is
required: the sim's stabilised controller holds a commanded attitude with gain 1.0 and tracks
ABSOLUTE yaw within 1 deg (sysid tab 6, measured). v2 commands attitude setpoints like hover_pilot
(flight-proven interface and signs) and keeps only what v1's data showed working: the gate tracker,
pass counting, the IMU vz integrator, and the per-tick debug log.

The law (roll parked at 0 forever):
  YAW     integrated heading, nudged toward the aim point once per camera frame (the ONLY steering)
  THRUST  holds the aim point on the fly-at-gate-height image row; the row is COMPUTED from the
          camera uptilt and our own commanded pitch (both known exactly - no attitude estimate)
  PITCH   a small forward creep, faded while off-aim, constant through the COMMIT window

Aim point: the gate OPENING (detector bbox) when located, else the ring centre.
Takeoff needs no special phase: on the pad the opening sits far above the target row, so the row
servo climbs off the floor and stops AT the gate line by construction.
"""

import csv
import math
import os
import time

import keyboard

from common.camera import HALF_TAN_X, HALF_TAN_Y, HEIGHT, UPTILT_RAD, WIDTH
from common.dynamics import (CONTROL_HZ, MAX_THRUST, MIN_THRUST, clamp,
                      send_attitude_setpoint, send_rate_attitude)
from common.paths import DATASETS_DIR
from pilots.steady_pilot.config import *   # STEADY_* params

# ---- SURVEY MODE (mapping flights only; STEADY_SURVEY=1) -------------------
# Hold + pulses for the mapper. Tracking thrust = race SERVO with cruise_up=0
# and oy_bias=0. SEEK after a pass uses vision memory (next_doy / seen2) to
# descend when the next gate is below the FoV — that is NOT a thrust bias.
STEADY_SURVEY = os.environ.get("STEADY_SURVEY", "0") == "1"
STEADY_SURVEY_HOLD_TICKS = int(5.0 * CONTROL_HZ)        # 5 s spawn hold
STEADY_SURVEY_PULSE_TICKS = int(6.0 * CONTROL_HZ)       # doublet every 6 s
_SURVEY_BRAKE_TICKS = int(0.5 * CONTROL_HZ)
_SURVEY_SURGE_TICKS = int(1.0 * CONTROL_HZ)
STEADY_SURVEY_SEEK_WINDOW_X = 3.0
# Post-pass altitude from remembered next_doy. Enough to put the next gate in
# the FoV — NOT a floor dump (run 174950: DOY_M=10 → −8.5 then climbed into
# the top of g2). Blind drop must not dig past the bake.
STEADY_SURVEY_DOY_M = 7.0
STEADY_SURVEY_MAX_DROP = 5.5
STEADY_SURVEY_DIVE = 1.0             # m/s when memory says LOWER
STEADY_SURVEY_SEEK_SINK = 0.7        # m/s blind progressive drop
STEADY_SURVEY_SEEK_GRACE_S = 1.0
STEADY_SURVEY_SEEK_DROP_RATE = 1.0   # m of search-alt per blind second
STEADY_SURVEY_SEEK_DROP_MAX = 8.0
print(f"[steady] SURVEY MODE {'ON — zero-oy track + memory SEEK + pulses' if STEADY_SURVEY else 'off (race pilot)'}",
      flush=True)


_DBG_W = _DBG_F = None
_DBG_N = 0
# EVERY control tick (~90 Hz): what was measured, what the law decided, what was sent.
_DBG_HDR = ["t", "frame", "fresh", "gates_passed", "n_dets", "area", "aim_ox", "aim_oy", "oy_tgt",
            "yaw_deg", "pitch_deg", "roll_cmd_deg", "ox_rate", "commit", "stale", "vz_est", "alt_est",
            "roll_meas_deg", "pitch_meas_deg", "trim", "thr", "next_head_deg", "seek_dir", "next_doy",
            # mapper columns (Stage 0, restored 2026-08-14 after a session
            # collision reverted them — gate_graph REQUIRES these):
            "sim_time_ns", "has_opening", "raw_ox", "raw_oy", "distance_m",
            "yaw_meas_deg", "droll_deg", "dpitch_deg", "race_gate", "lookaround"]


def _dbg_log(row):
    global _DBG_W, _DBG_F, _DBG_N
    if _DBG_W is None:
        path = os.path.join(DATASETS_DIR, time.strftime("steady_dbg_%Y%m%d_%H%M%S.csv"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _DBG_F = open(path, "w", newline="")
        _DBG_W = csv.writer(_DBG_F)
        _DBG_W.writerow(_DBG_HDR)
        print(f"steady debug -> {path}", flush=True)
    _DBG_W.writerow(row)
    _DBG_N += 1
    if _DBG_N % 30 == 0:
        _DBG_F.flush()


def _imu_attitude(data):
    """Roll/pitch complementary filter - kept for LOGGING and the replay horizon only. v2 control
    NEVER steers on estimated attitude (that bias is what flew v1 a metre under the openings)."""
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")
    if t_us is None:
        return data.get("_sp_est_roll", 0.0), data.get("_sp_est_pitch", 0.0)
    ax, ay, az = imu.get("xacc", 0.0), imu.get("yacc", 0.0), imu.get("zacc", 0.0)
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    valid = abs(mag - 9.81) <= 2.5
    roll_a = math.atan2(ay, -az)
    pitch_a = math.atan2(-ax, math.hypot(ay, az))
    prev = data.get("_sp_att_us")
    roll = data.get("_sp_est_roll", 0.0)
    pitch = data.get("_sp_est_pitch", 0.0)
    if prev is None or t_us == prev:
        if valid:
            roll, pitch = roll_a, pitch_a
    else:
        dt = (t_us - prev) * 1e-6
        if 0.0 < dt <= 0.1:
            roll_g = roll - imu.get("xgyro", 0.0) * dt   # roll gyro NEGATED (measured convention)
            pitch_g = pitch + imu.get("ygyro", 0.0) * dt
            if valid:
                a = 0.98
                roll = a * roll_g + (1 - a) * roll_a
                pitch = a * pitch_g + (1 - a) * pitch_a
            else:
                roll, pitch = roll_g, pitch_g
            # measured yaw: integrated zgyro, seeded at spawn facing.
            # SIGN IS EMPIRICAL, NOT THEORETICAL: run 175259 proved -zgyro
            # anti-correlates with commanded yaw (sim tracks cmd at gain 1),
            # which mirrored the whole map. +zgyro matches the command.
            # LOGGING ONLY — gate_graph/ImuTraverse read yaw_meas_deg (and
            # self-check the sign against yaw_deg as belt-and-braces).
            data["_sp_yaw_meas"] = data.get("_sp_yaw_meas", STEADY_YAW_SPAWN) \
                + imu.get("zgyro", 0.0) * dt
    data["_sp_est_roll"], data["_sp_est_pitch"], data["_sp_att_us"] = roll, pitch, t_us
    return roll, pitch


def _vz_alt_estimate(data, m_roll, m_pitch):
    """Leaky-integrated vertical velocity (m/s, up+) and altitude-above-start (m) from the IMU
    specific force rotated through the measured attitude. Validated against the recorded ceiling
    flight (commanded +1.5, actual +4.0 - it caught it). Used for mild thrust damping + logging."""
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")
    if t_us is None:
        return data.get("_sp_vz", 0.0)
    prev = data.get("_sp_vz_us")
    if prev is not None and t_us > prev:
        dt = (t_us - prev) * 1e-6
        if dt <= 0.1:
            sp, cp = math.sin(-m_pitch), math.cos(m_pitch)
            sr, cr = math.sin(m_roll), math.cos(m_roll)
            wd = (-sp * imu.get("xacc", 0.0) + cp * sr * imu.get("yacc", 0.0)
                  + cp * cr * imu.get("zacc", 0.0))
            a_up = -(wd + 9.81)
            # IMPACT REJECTION: crash/bounce accels integrated into vz_est read "+2.7 m/s climb"
            # while sitting ON THE FLOOR (run 094047, t~30) and the damping term stripped thrust,
            # pinning it down. Skip transient spikes; clamp the estimate to sane flight speeds.
            if abs(a_up) <= 12.0:
                vz = (data.get("_sp_vz", 0.0) + a_up * dt) * math.exp(-dt / STEADY_VZ_TAU)
                data["_sp_vz"] = clamp(vz, -3.0, 3.0)
                data["_sp_alt"] = data.get("_sp_alt", 0.0) + data["_sp_vz"] * dt
    data["_sp_vz_us"] = t_us
    return data.get("_sp_vz", 0.0)


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


def _ang_err(target, current):
    """Shortest signed angle error target-current in (-pi, pi]."""
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _reset(data):
    data["_sp_off"] = (0.0, 0.0)
    data["_sp_off_smooth"] = None
    data["_sp_area"] = 0.0
    data["_sp_track_off"] = None
    data["_sp_track_miss"] = 0
    data["_sp_fid"] = None
    data["_sp_gate_count"] = 0
    data["_sp_pass_armed"] = False
    data["_sp_pass_peak"] = 0.0
    data["_sp_pass_block"] = False
    data["_sp_commit_latch"] = False
    data["_sp_avoid_off"] = None
    data["_sp_stale"] = 0
    data["_sp_t0_us"] = None
    data["_sp_heading"] = STEADY_YAW_SPAWN   # integrated ABSOLUTE yaw command (rad). The setpoint yaw is
                                          # a WORLD direction: seeding 0 snapped the drone ~97 deg off
                                          # the course at race start (run 20260727_000330). Seed with
                                          # the measured spawn facing so tick one means "hold still".
    data["_sp_pitch_cmd"] = 0.0        # slewed pitch command (rad)
    data["_sp_roll_cmd"] = 0.0         # slewed roll command (rad)
    data["_sp_climb_rng"] = None       # range anchor for the near-gate climb station-hold
    data["_sp_thr_trim"] = 0.0         # learned hover-thrust correction (integral trim)
    data["_sp_ox_rate"] = 0.0          # aim-point lateral rate (units/s) - damps the roll strafe
    data["_sp_ox_blind"] = True        # no valid lateral-rate sample yet -> roll capped gentle
    data["_sp_prev_ox"] = None
    data["_sp_prev_oy"] = None
    data["_sp_prev_ox_us"] = None
    data["_sp_yawed"] = False          # a yaw nudge moved the aim last frame -> skip one rate sample
    data["_sp_next_head"] = None       # remembered ABSOLUTE heading of the next gate (rad)
    data["_sp_next_doy"] = None        # its elevation RELATIVE to the then-tracked gate (offset_y)
    data["_sp_next_area_mem"] = 0.0    # its size when sighted
    data["_sp_next_us"] = None
    data["_sp_next_area"] = 0.0        # decaying area bar - biggest credible sighting wins
    data["_sp_mem_armed"] = False      # after a pass until we lock the remembered next gate
    data["_sp_pass_us"] = None         # when the last pass counted (starts the SEEK window)
    data["_sp_pass_head"] = 0.0
    data["_sp_pass_alt"] = 0.0         # alt_est when the last pass counted (= the gate line)
    data["_sp_seek_alt"] = None        # baked at pass from next_doy — survives junk reacquire wipe
    data["_sp_seek_dir"] = 1.0         # which way SEEK rotates (signed; from the memory)
    data["_sp_lock_frames"] = 0        # consecutive frames on the SAME lock (maturity gate)
    data["_sp_slew_us"] = None
    data["_sp_est_roll"] = 0.0
    data["_sp_est_pitch"] = 0.0
    data["_sp_att_us"] = None
    data["_sp_vz"] = 0.0
    data["_sp_alt"] = 0.0
    data["_sp_vz_us"] = None
    data["_sp_live"] = False


def _idle(mavlink_conn, system_boot_ms, data, regime):
    data["steady_regime"] = regime
    data["oracle_thrust"] = 0.0
    _reset(data)
    send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)


def _aim_offsets(d):
    """Normalised image offsets of the AIM POINT: the opening's bbox centre when the detector located
    the hole, else the ring centre. d.offset_x/y are the ring-BBOX centre - on the tall start-gate
    structure that sits ~1 m off the opening, and v1 flew a metre under the hole trusting it."""
    if d.has_opening and d.bbox:
        x, y, bw, bh = d.bbox
        return ((x + bw / 2.0) - WIDTH / 2.0) / (WIDTH / 2.0), \
               ((y + bh / 2.0) - HEIGHT / 2.0) / (HEIGHT / 2.0)
    return d.offset_x, d.offset_y


def _acquire(cands):
    """Fresh acquisition: biggest blob, preferring detections with a LOCATED OPENING. Real gates in
    range have one; the station-sign decoys never do (one stole two whole flights)."""
    op = [d for d in cands if d.has_opening]
    return max(op or cands, key=lambda d: d.area)


def _bearing_err_to_mem(d, data):
    """Absolute heading error between a detection and remembered next_head."""
    nh = data.get("_sp_next_head")
    if nh is None:
        return None
    head = data.get("_sp_heading", 0.0)
    b = head - math.atan(d.offset_x * HALF_TAN_X)
    return abs(_ang_err(nh, b))


def _acquire_postpass(cands, data):
    """Post-pass acquire: prefer memory cone, FAIL-OPEN to biggest.

    Memory steers SEEK yaw; it must NOT veto locks. Cone-veto regressions
    (220147/135943): g1 always in view, acquire returned None → SEEK forever
    → land. Prefer cone when it has anyone; otherwise lock something.
    """
    if not cands:
        return None
    nh = data.get("_sp_next_head")
    if nh is not None:
        near = [d for d in cands
                if (_bearing_err_to_mem(d, data) or 99.0) <= STEADY_SEEK_CONE]
        if near:
            return _acquire(near)
    return _acquire(cands)


def _select_target(gates, data):
    """Follow ONE gate by spatial proximity; post-pass, avoid the flown-through gate (proven)."""
    avoid = data.get("_sp_avoid_off")
    # The avoid lock EXPIRES: its whole job is the 1-2 s post-pass handoff.
    _us = (data.get("highres_imu") or {}).get("time_usec")
    p_us = data.get("_sp_pass_us")
    if avoid is not None and _us is not None and p_us is not None \
            and (_us - p_us) * 1e-6 > STEADY_AVOID_S:
        avoid = None
        data["_sp_avoid_off"] = None
    if avoid is not None:
        ax, ay = avoid
        near = min(gates, key=lambda d: math.hypot(d.offset_x - ax, d.offset_y - ay))
        # Follow the avoided gate ONLY while the nearby blob is still BIG - the genuinely
        # just-passed gate filled the frame moments ago. A SMALL blob near the stale position is a
        # DIFFERENT gate that drifted in (run 110222: the next gate inherited the avoid lock and
        # the tracker flew at a 22 m speck instead) - drop the avoid, don't adopt it.
        if (math.hypot(near.offset_x - ax, near.offset_y - ay) <= STEADY_AVOID_RADIUS
                and near.area_frac >= STEADY_AVOID_MIN_AREA):
            data["_sp_avoid_off"] = (near.offset_x, near.offset_y)
        else:
            avoid = None
            data["_sp_avoid_off"] = None

    def usable():
        if avoid is None:
            return gates
        ax, ay = avoid
        return [d for d in gates if math.hypot(d.offset_x - ax, d.offset_y - ay) > STEADY_AVOID_RADIUS]

    in_pp = (p_us is not None and _us is not None and (_us - p_us) * 1e-6 < STEADY_SEEK_S)

    def _sane(ds):
        """Post-pass sky-blob exclusion: a candidate ~50deg+ above the horizon cannot be a course
        gate - only the just-passed gate's top bar / ceiling junk (one fired a full-climb spike at
        the gate plane, run 164646). Fail-open: yields if it would empty the list."""
        if not in_pp:
            return ds
        ok = [d for d in ds if d.offset_y > STEADY_JUNK_OY]
        return ok or ds

    prev = data.get("_sp_track_off")
    if prev is None:
        data["_sp_track_miss"] = 0
        cands = _sane(usable() or gates)
        if not cands:
            return None
        return _acquire_postpass(cands, data) if in_pp else _acquire(cands)
    px, py = prev
    cands = usable() or gates
    g = min(cands, key=lambda d: (d.offset_x - px) ** 2 + (d.offset_y - py) ** 2)
    if math.hypot(g.offset_x - px, g.offset_y - py) <= STEADY_TRACK_RADIUS:
        data["_sp_track_miss"] = 0
        if in_pp:
            # Dump only a true far speck (214731 area~0.003); never veto a real gate.
            if g.area_frac < STEADY_POSTPASS_MIN_LOCK:
                data["_sp_track_off"] = None
                data["_sp_off_smooth"] = None
                return _acquire_postpass(_sane(cands), data)
            best = _acquire(_sane(cands))
            if best is not g and best.area_frac >= STEADY_PREEMPT_RATIO * max(g.area_frac, 1e-6):
                return best
        return g
    miss = data.get("_sp_track_miss", 0) + 1
    data["_sp_track_miss"] = miss
    if miss >= STEADY_TRACK_MAX_MISS:
        data["_sp_track_miss"] = 0
        return _acquire_postpass(cands, data) if in_pp else _acquire(cands)
    return None


def update_steady_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    if not _race_live(data):
        t_go = _seconds_to_go(data)
        _idle(mavlink_conn, system_boot_ms, data,
              f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0) else "IDLE")
        return

    # SURVEY spawn hold: full bypass — the pilot's pipeline does not run at
    # all while held, so no state can wind up or stale-track. Setting
    # _sp_live False makes the pilot's OWN fresh-go init fire at release.
    if STEADY_SURVEY:
        hi = data.get("_sp_hold_i", 0) + 1
        data["_sp_hold_i"] = hi
        if hi <= STEADY_SURVEY_HOLD_TICKS:
            data["_sp_live"] = False
            data["steady_regime"] = "SURVEY-HOLD"
            send_attitude_setpoint(mavlink_conn, system_boot_ms,
                                   0.0, 0.0, STEADY_YAW_SPAWN, 0.05)
            return

    if not data.get("_sp_live"):
        _reset(data)
        data["_sp_live"] = True

    # Measured attitude + vertical estimate: LOGGING and mild thrust damping only.
    m_roll, m_pitch = _imu_attitude(data)
    vz_est = _vz_alt_estimate(data, m_roll, m_pitch)

    # 1) TRACK one gate; smooth the AIM offsets (the opening, when located).
    fresh = False
    fid = data.get("latest_frame_id")
    gates = [d for d in (data.get("vision_gates") or []) if d.area_frac >= STEADY_MIN_AREA]
    if fid is not None and fid != data.get("_sp_fid") and gates:
        data["_sp_fid"] = fid
        had_lock = data.get("_sp_track_off") is not None
        g = _select_target(gates, data)
        if g is None:
            data["_sp_stale"] = data.get("_sp_stale", 0) + 1
        else:
            # Clear SEEK yaw-memory only after a CONFIRMED lock near the remembered bearing.
            # Run 142254: first post-pass junk (bearing ≈ pass heading ≈ next_head on a straight
            # leg) wiped doy=+0.34 before SEEK could descend → blind, level, nothing in frame.
            # (clear runs after lock_frames update below)
            ax, ay = _aim_offsets(g)
            data["_sp_det_raw"] = (g.offset_x, g.offset_y,
                                   int(bool(getattr(g, "has_opening", False))),
                                   getattr(g, "distance_m", 0.0) or 0.0)
            if STEADY_SURVEY:
                # last-seen memory of the TRACKED gate (base pilot has none:
                # its memory only covers "next gate seen pre-pass"). Bearing,
                # frame row, altitude, timestamp at every confirmed sighting.
                _us_seen = (data.get("highres_imu") or {}).get("time_usec")
                _hd = data.get("_sp_heading", STEADY_YAW_SPAWN)
                data["_sp_seen"] = (_hd + math.atan(ax * HALF_TAN_X),
                                    ay, data.get("_sp_alt", 0.0), _us_seen)
                # AND the best NON-tracked detection = the next gate, visible
                # through most of the approach (the base bake misses it on
                # steep legs and SEEK spins blind after the pass)
                _others = [d for d in gates
                           if (abs(d.offset_x - g.offset_x)
                               + abs(d.offset_y - g.offset_y)) > 0.15
                           and d.area_frac >= 0.003]
                if _others:
                    _o = max(_others, key=lambda d: d.area_frac)
                    data["_sp_seen2"] = (
                        _hd + math.atan(_o.offset_x * HALF_TAN_X),
                        _o.offset_y, data.get("_sp_alt", 0.0), _us_seen)
            sm = data.get("_sp_off_smooth")
            hop = sm is None or math.hypot(ax - sm[0], ay - sm[1]) > STEADY_HOP_DIST
            if hop:
                sx, sy = ax, ay
            else:
                sx = STEADY_OFF_ALPHA * ax + (1 - STEADY_OFF_ALPHA) * sm[0]
                sy = STEADY_OFF_ALPHA * ay + (1 - STEADY_OFF_ALPHA) * sm[1]
            # AIM LATERAL RATE (units/s), frame-to-frame - damps the roll strafe. Discarded on a hop
            # (fake motion), on the frame after a YAW nudge (panning moves the aim with zero actual
            # translation), and during fast VERTICAL aim motion (climb/descent parallax pollutes the
            # x-rate: the takeoff climb read +0.30 and banked 5.7 deg with the gate dead ahead -
            # run 094047, t=0.1-0.5). Clamped to real strafe speeds (< 0.1 in flight).
            _us_f = (data.get("highres_imu") or {}).get("time_usec")
            p_ox, p_oy, p_us_f = data.get("_sp_prev_ox"), data.get("_sp_prev_oy"), data.get("_sp_prev_ox_us")
            if hop or data.get("_sp_yawed"):
                data["_sp_ox_rate"] = 0.0
                data["_sp_ox_blind"] = True     # no valid lateral rate -> roll flies capped-gentle
            elif p_ox is not None and p_us_f is not None and _us_f is not None:
                _dtf = (_us_f - p_us_f) * 1e-6
                if 0.0 < _dtf <= 0.2:
                    oy_rate = (sy - p_oy) / _dtf if p_oy is not None else 0.0
                    if abs(oy_rate) > STEADY_OYRATE_GATE:
                        data["_sp_ox_rate"] = 0.0
                        data["_sp_ox_blind"] = True
                    else:
                        # DE-PAN with the measured gyro: yaw motion sweeps the aim at zgyro*(1+offx^2)
                        # (offx is tan-normalized; sim zgyro is CCW-positive - calibration flights)
                        # with ZERO translation. The one-frame post-nudge skip can't cover the sim's
                        # ~150 ms yaw lag: run 114407 read ox_rate pinned +0.30 through an approach
                        # pan and the damping shoved the roll AWAY from the gate - left-edge hit.
                        zg = (data.get("highres_imu") or {}).get("zgyro", 0.0)
                        raw = (sx - p_ox) / _dtf - zg * (1.0 + sx * sx)
                        data["_sp_ox_rate"] = clamp(raw, -STEADY_OXRATE_MAX, STEADY_OXRATE_MAX)
                        data["_sp_ox_blind"] = False
            if _us_f is not None:
                data["_sp_prev_ox"], data["_sp_prev_oy"], data["_sp_prev_ox_us"] = sx, sy, _us_f
            data["_sp_yawed"] = False
            data["_sp_off_smooth"] = (sx, sy)
            data["_sp_off"] = (sx, sy)
            data["_sp_track_off"] = (g.offset_x, g.offset_y)   # continuity in RING space
            data["_sp_area"] = g.area_frac
            data["_sp_stale"] = 0
            fresh = True
            # Lock maturity: consecutive matched frames on the SAME target. A fresh acquisition (or
            # a hop) restarts it - and an unconfirmed lock gets no vertical authority (a junk blob
            # acquired right after the pitched-gate pass sprint-climbed the drone to 3 m in half a
            # second before dying, run 115646).
            data["_sp_lock_frames"] = (data.get("_sp_lock_frames", 0) + 1) if (had_lock and not hop) else 1
            if data.get("_sp_mem_armed") and data["_sp_lock_frames"] >= STEADY_LOCK_CONFIRM:
                nh = data.get("_sp_next_head")
                found = nh is None
                if not found:
                    b = (data.get("_sp_heading", 0.0)
                         - math.atan(g.offset_x * HALF_TAN_X))
                    found = abs(_ang_err(nh, b)) <= STEADY_SEEK_CONE
                # Receding just-passed gate is still huge and straight ahead — must NOT count as
                # "found next" (run 143848: area 0.42 wipe of doy/nh right after g1 → late g2 under).
                # Also require a real-sized lock before clearing memory — a cone speck must
                # not retire SEEK (215501/215420 sharp-turn deaths).
                if (found and g.area_frac < STEADY_COMMIT_AREA
                        and g.area_frac >= STEADY_POSTPASS_MIN_LOCK):
                    data["_sp_next_head"] = None
                    data["_sp_next_doy"] = None
                    data["_sp_next_area_mem"] = 0.0
                    data["_sp_next_area"] = 0.0
                    data["_sp_next_us"] = None
                    data["_sp_mem_armed"] = False
            # NEXT-GATE MEMORY: note the ABSOLUTE heading of the biggest OTHER detection (excluding
            # the just-passed avoid gate). At a sharp corner the next gate leaves the FoV before the
            # pass finishes - this remembered bearing tells SEEK which way to turn, signed, so left
            # and right corners are the same code. offset_x -> angle via atan(ox * HALF_TAN_X);
            # heading frame is CCW-positive, so a target to the RIGHT is a SMALLER heading.
            av = data.get("_sp_avoid_off")
            # Same visibility floor as tracking: a next gate often reads 0.002-0.004 while the
            # tracked one is bigger (run 141745: 96 fresh n=2 frames, others always empty at 0.004).
            others = [d for d in gates if d is not g and d.area_frac >= STEADY_MIN_AREA
                      and (av is None or math.hypot(d.offset_x - av[0], d.offset_y - av[1]) > STEADY_AVOID_RADIUS)]
            # BIGGEST credible sighting wins, not the LATEST: size bar blocks weak speck flicker
            # from overwriting a real bearing. Openings bypass the bar so a real next gate can
            # refresh an early junk lock (run 141745: t=0.3 doy=-0.358 froze until pass, then
            # STEADY_NEXT_MEM_S wiped it → SEEK flew straight with no descent).
            bar = data.get("_sp_next_area", 0.0) * 0.99
            data["_sp_next_area"] = bar
            # RECORDING HYGIENE - the memory only accepts sightings from CLEAN frames:
            #  * not within 1.5 s AFTER a pass (receding-gate fragments), and
            #  * not once COMMIT-near (pillar fragments). Keep noting through ALIGN so a
            #    lower/higher next gate still visible in the FoV can be remembered for SEEK.
            _pus = data.get("_sp_pass_us")
            _nus = (data.get("highres_imu") or {}).get("time_usec")
            settled = (_pus is None or _nus is None
                       or (_nus - _pus) * 1e-6 > STEADY_NEXT_HOLDOFF)
            settled = settled and g.area_frac < STEADY_COMMIT_AREA
            # Wait for a confirmed lock: launch frame-0 junk wrote doy=-0.358 (run 141745)
            # and poisoned SEEK for the whole first leg.
            settled = settled and data.get("_sp_lock_frames", 0) >= STEADY_LOCK_CONFIRM
            if others and settled:
                op = [d for d in others if getattr(d, "has_opening", False)]
                # Near ALIGN: openings only — pillars pose as "others" without a hole.
                if g.area_frac >= STEADY_ALIGN_AREA and not op:
                    nd = None
                else:
                    nd = max(op or others, key=lambda d: d.area)
                if nd is not None:
                    is_open = getattr(nd, "has_opening", False)
                    if is_open or nd.area_frac >= bar:
                        data["_sp_next_head"] = (data.get("_sp_heading", 0.0)
                                                 - math.atan(nd.offset_x * HALF_TAN_X))
                        # SIGHTING TRACK: bearing + elevation vs tracked gate (+doy = next LOWER).
                        data["_sp_next_doy"] = nd.offset_y - g.offset_y
                        data["_sp_next_area_mem"] = nd.area_frac
                        data["_sp_next_us"] = (data.get("highres_imu") or {}).get("time_usec")
                        data["_sp_next_area"] = nd.area_frac
    area = data.get("_sp_area", 0.0)
    offx, offy = data.get("_sp_off", (0.0, 0.0))

    # Regime predicates. COMMIT requires proximity AND alignment (run 095624 committed on area
    # alone, coasted wide right into the gate edge). ALIGN-mode (brake + strafe, no creep) engages
    # EARLY when misaligned - from STEADY_ALIGN_AREA, not commit range - because braking at 2 m with
    # 0.3 of offset just scrapes down the pillar (run 113145). Aligned approaches skip the band.
    stale = data.get("_sp_stale", 0) >= STEADY_MAX_STALE
    tracking = bool(gates) and not stale
    aligned_x = abs(offx) <= STEADY_COMMIT_ALIGN
    # Vertical readiness (run 122735: committed mid-climb at aim_oy -0.36 / vz +2.97 and sagged
    # into the bottom bar). Blocked-vertical routes through the SERVO branch, whose row servo +
    # climb-first pitch already do exactly the right thing: finish the climb, then commit.
    row_tgt = math.tan(UPTILT_RAD - data.get("_sp_pitch_cmd", 0.0)) / HALF_TAN_Y + (
        0.0 if STEADY_SURVEY else STEADY_OFFY_BIAS)
    aligned_y = abs(offy - row_tgt) <= STEADY_COMMIT_ROW
    # COMMIT LATCHES. Alignment (both axes) is an ENTRY condition only - at point-blank range the
    # aim balloons away from the far-field row target BY GEOMETRY, and re-checking it every tick
    # un-committed the pilot INSIDE the tilted gate (run 123331, t=67.7): the row servo woke mid-
    # pass, firewalled a +2.2 m/s climb in the gate throat, and the pass never counted. Enter on
    # readiness; release only on pass handoff, lost tracking, or the gate genuinely receding.
    vz_ok = abs(vz_est) <= STEADY_COMMIT_VZ
    if tracking and area >= STEADY_COMMIT_AREA and aligned_x and aligned_y and vz_ok:
        data["_sp_commit_latch"] = True
    if data.get("_sp_commit_latch") and (not tracking or area < STEADY_PASS_REARM):
        data["_sp_commit_latch"] = False
    commit = bool(data.get("_sp_commit_latch"))
    # ALIGN also engages when the VERTICAL SPEED is too hot to commit (descending off the cruise
    # glide, run 132724): the brake holds station while the arrest settles vz, then commit - level.
    align_mode = (tracking and not commit and area >= STEADY_ALIGN_AREA
                  and (not aligned_x or not vz_ok))

    # 2) PASS = area peaked then receded -> count it and hand off to the next gate. Arms ONLY while
    # COMMITTED: area receding from a lateral scrape is not a pass (run 095624 counted 6 "gates").
    if data.get("_sp_pass_block") and area < STEADY_PASS_REARM:
        data["_sp_pass_block"] = False
    if commit and area >= STEADY_PASS_AREA and not data.get("_sp_pass_block"):
        data["_sp_pass_armed"] = True
        data["_sp_pass_peak"] = max(data.get("_sp_pass_peak", 0.0), area)
    if data.get("_sp_pass_armed") and area < STEADY_PASS_DROP * data.get("_sp_pass_peak", 0.0):
        data["_sp_gate_count"] = data.get("_sp_gate_count", 0) + 1
        data["_sp_pass_armed"] = False
        data["_sp_pass_peak"] = 0.0
        data["_sp_pass_block"] = True
        data["_sp_commit_latch"] = False   # pass done - release the commit latch for the next gate
        data["_sp_avoid_off"] = data.get("_sp_track_off")
        data["_sp_track_off"] = None
        data["_sp_off_smooth"] = None
        # Start SEEK: keep fresh next-gate memory; wipe if stale. Bake altitude NOW from
        # next_doy so a junk reacquire cannot erase the anticipated descent (run 142254).
        _us_p = (data.get("highres_imu") or {}).get("time_usec")
        head_now = data.get("_sp_heading", 0.0)
        data["_sp_pass_us"] = _us_p
        data["_sp_pass_head"] = head_now
        pass_alt = data.get("_sp_alt", 0.0)
        data["_sp_pass_alt"] = pass_alt
        data["_sp_lock_frames"] = 0
        data["_sp_mem_armed"] = True
        # Only wipe memory when BOTH stamps exist and the sighting is stale.
        # Missing IMU time_usec used to null a live next_head at the pass tick
        # (215501: nh=+74° one frame before g1 pass → blank → no sharp-turn SEEK).
        nus = data.get("_sp_next_us")
        if (nus is not None and _us_p is not None
                and (_us_p - nus) * 1e-6 > STEADY_NEXT_MEM_S):
            data["_sp_next_head"] = None
            data["_sp_next_doy"] = None
            data["_sp_next_area_mem"] = 0.0
        doy = data.get("_sp_next_doy")
        if doy is not None and abs(float(doy)) > 0.15:
            _doy_m = STEADY_SURVEY_DOY_M if STEADY_SURVEY else STEADY_SEEK_DOY_M
            _max_drop = STEADY_SURVEY_MAX_DROP if STEADY_SURVEY else STEADY_SEEK_MAX_DROP
            signed = -_doy_m * clamp(
                float(doy), -STEADY_SEEK_DOY_CAP, STEADY_SEEK_DOY_CAP)
            data["_sp_seek_alt"] = pass_alt + clamp(
                signed, -_max_drop, STEADY_SEEK_MAX_CLIMB_M)
        else:
            data["_sp_seek_alt"] = pass_alt
        nh = data.get("_sp_next_head")
        if nh is not None and abs(_ang_err(nh, head_now)) > 0.05:
            data["_sp_seek_dir"] = 1.0 if _ang_err(nh, head_now) > 0.0 else -1.0
        data["_sp_next_area"] = 0.0

    # 3) CONTROL - three regimes: HOLD (blind), COMMIT (through the gate), SERVO (normal).
    t_us_now = (data.get("highres_imu") or {}).get("time_usec")
    p_us = data.get("_sp_slew_us")
    dt = (t_us_now - p_us) * 1e-6 if (t_us_now is not None and p_us is not None) else 1.0 / CONTROL_HZ
    if not (0.0 < dt <= 0.05):
        dt = 1.0 / CONTROL_HZ
    if t_us_now is not None:
        data["_sp_slew_us"] = t_us_now

    heading = data.get("_sp_heading", 0.0)
    pitch_prev = data.get("_sp_pitch_cmd", 0.0)
    trim = data.get("_sp_thr_trim", 0.0)
    oy_tgt = 0.0
    if not gates or stale:
        # SEEK (main behavior): rotate in seek_dir until vision acquires.
        # next_head only picks the SIGNED turn direction at pass — do NOT stop
        # at nh. Approach-vantage nh is short of the true post-pass bearing at
        # sharp corners (g3); stop-at-nh under-turns. FoV catches the gate
        # before the nose finishes; servo takes over on lock.
        des_pitch = 0.0
        des_roll = 0.0
        v_tgt = 0.0
        p_us_pass = data.get("_sp_pass_us")
        _seek_window = STEADY_SEEK_S * (STEADY_SURVEY_SEEK_WINDOW_X
                                        if STEADY_SURVEY else 1.0)
        if (p_us_pass is not None and t_us_now is not None
                and (t_us_now - p_us_pass) * 1e-6 < _seek_window):
            nh = data.get("_sp_next_head")
            _grace = STEADY_SURVEY_SEEK_GRACE_S if STEADY_SURVEY else 999.0
            _seen = data.get("_sp_seen") if STEADY_SURVEY else None
            _seen2 = data.get("_sp_seen2") if STEADY_SURVEY else None
            _mem = None
            if (_seen is not None and _seen[3] is not None
                    and _seen[3] > p_us_pass):
                _mem = _seen
            elif (_seen2 is not None and _seen2[3] is not None
                    and (t_us_now - _seen2[3]) * 1e-6 < 20.0):
                # next-gate sighting from the approach — use instead of blind spin
                _mem = _seen2
            if _mem is not None:
                nh = _mem[0]
                # Retarget seek_dir from survey sighting if it disagrees.
                err_m = _ang_err(nh, heading)
                if abs(err_m) > 0.05:
                    data["_sp_seek_dir"] = 1.0 if err_m > 0.0 else -1.0
                if STEADY_SURVEY and _mem[1] > 0.35:
                    _grace = 1.0
            heading += data.get("_sp_seek_dir", 1.0) * STEADY_SEEK_RATE * dt
            data["_sp_heading"] = heading
            if abs(_ang_err(heading, data.get("_sp_pass_head", heading))) > 0.5:
                data["_sp_avoid_off"] = None
            des_pitch = STEADY_SEEK_PITCH
            pass_alt = data.get("_sp_pass_alt", 0.0)
            alt_tgt = data.get("_sp_seek_alt")
            if alt_tgt is None:
                alt_tgt = pass_alt
            _doy_now = data.get("_sp_next_doy")
            # Memory says next is LOWER → descend now (holding pass alt keeps
            # g2 under the FoV). Race still looks first.
            _known_low = (STEADY_SURVEY and alt_tgt < pass_alt - 0.3) or (
                STEADY_SURVEY and _doy_now is not None and float(_doy_now) > 0.15)
            # Blind drop ONLY when there is no doy bake. With a bake, that is
            # the floor — digging past it (run 174950) puts us under g2 and the
            # zero-oy climb eats the top bar.
            _survey_drop = False
            if (STEADY_SURVEY and not _known_low
                    and p_us_pass is not None and t_us_now is not None):
                _t_ref = p_us_pass
                _base_alt = pass_alt
                if _mem is not None and _mem[3] is not None and _mem[3] > _t_ref:
                    _t_ref = _mem[3]
                    _base_alt = _mem[2]
                _blind_s = (t_us_now - _t_ref) * 1e-6
                if _blind_s > _grace:
                    _d = min(STEADY_SURVEY_SEEK_DROP_RATE * (_blind_s - _grace),
                             STEADY_SURVEY_SEEK_DROP_MAX)
                    if _base_alt - _d < alt_tgt:
                        alt_tgt = _base_alt - _d
                        _survey_drop = True
            alt_now = data.get("_sp_alt", 0.0)
            if alt_now > alt_tgt + STEADY_SEEK_ALT_TOL:
                if STEADY_SURVEY and (_known_low or _survey_drop):
                    v_tgt = -(STEADY_SURVEY_DIVE if _known_low
                              else STEADY_SURVEY_SEEK_SINK)
                else:
                    v_tgt = -STEADY_SEEK_SINK
            elif alt_now < alt_tgt - STEADY_SEEK_ALT_TOL:
                v_tgt = STEADY_SEEK_CLIMB
        thrust = STEADY_HOVER + trim - STEADY_KD_VZ_COMMIT * (vz_est - v_tgt)
    elif commit:
        # COMMIT: the aim point balloons this close - stop chasing it. Freeze the heading, wings
        # LEVEL, keep the creep on, let the pass logic call it. Thrust holds hover WITH a strong
        # vz-arrest: any residual climb/sink from the approach coasts into a gate bar otherwise
        # (gate 1's top bar, run 101632). ONE escape clause: if the hole climbs far above image
        # centre mid-coast (the TILTED gate - its passage line is higher than the level line, and
        # the frozen coast clipped its bottom bar, run 122217), follow it up. Climb-only, and a
        # straight gate's commit never breaches the deadband.
        des_pitch = STEADY_PITCH_FWD
        # Lateral escape clause (twin of the vertical one): wings-level UNLESS the aim escapes far
        # sideways mid-coast - run 130224 watched it walk to -0.41 and scraped the left pillar.
        # Gentle (blind-cap), toward the aim, zero inside the deadband normal commits never leave.
        if abs(offx) > STEADY_COMMIT_SIDE_OFFX:
            side = (abs(offx) - STEADY_COMMIT_SIDE_OFFX) * (1.0 if offx > 0 else -1.0)
            des_roll = clamp(STEADY_KP_COMMIT_SIDE * side, -STEADY_ROLL_BLIND, STEADY_ROLL_BLIND)
        else:
            des_roll = 0.0
        up = clamp(-offy - STEADY_COMMIT_UP_OFFY, 0.0, 0.5)
        thrust = STEADY_HOVER + trim + STEADY_KP_COMMIT_UP * up - STEADY_KD_VZ_COMMIT * vz_est
    else:
        # SERVO (far) or ALIGN (near but off-axis - stop the creep, keep sliding onto the axis).
        # YAW: COARSE FoV keeping only - nudge the heading ONCE PER FRAME when the aim point has
        # drifted well off centre; frozen otherwise AND frozen when NEAR (the ballooning aim would
        # whip it). At creep speed yaw doesn't change the direction of travel (a quad translates by
        # TILTING - the nose is a camera mount). ROLL owns the fine lateral work.
        if fresh and not align_mode and STEADY_YAW_ON < abs(offx) < STEADY_YAW_MAX_OFF:
            heading += clamp(STEADY_KP_YAW * offx, -STEADY_YAW_STEP_MAX, STEADY_YAW_STEP_MAX)
            data["_sp_heading"] = heading
            data["_sp_yawed"] = True    # panning moves the aim - skip the next lateral-rate sample
        # ROLL: the lateral translation, pitch-creep style - small bank toward the aim point,
        # DAMPED on the aim's measured lateral rate so it rolls back level BEFORE the centre is
        # crossed (offx -> roll is a double integrator; P-only is v1's teeter-totter). While rate
        # samples are being DISCARDED (climb transients) the cap drops to gentle: on a constant-
        # bearing pursuit the bearing doesn't move even at high lateral speed, and undamped-P at
        # full cap silently built ~3 m/s of sideways momentum during run 105441's big climb.
        roll_cap = STEADY_ROLL_BLIND if data.get("_sp_ox_blind") else STEADY_ROLL_MAX
        des_roll = clamp(STEADY_KP_LAT * offx + STEADY_KD_LAT * data.get("_sp_ox_rate", 0.0),
                         -roll_cap, roll_cap)
        # THRUST: geometric row. Survey = zero bias/cruise. Race gains. Nothing else.
        if STEADY_SURVEY:
            cruise_up = 0.0
            oy_bias = 0.0
        else:
            cruise_up = STEADY_CRUISE_UP * clamp(
                1.0 - area / STEADY_CRUISE_FADE_AREA, 0.0, 1.0)
            oy_bias = STEADY_OFFY_BIAS
        oy_tgt = (math.tan(UPTILT_RAD - pitch_prev) / HALF_TAN_Y
                  + oy_bias + cruise_up)
        err = oy_tgt - offy
        if data.get("_sp_lock_frames", 0) < STEADY_LOCK_CONFIRM:
            thrust = STEADY_HOVER + trim - STEADY_KD_VZ_COMMIT * vz_est
        else:
            if abs(vz_est) < STEADY_TRIM_VZ_GATE:
                trim = clamp(trim + STEADY_KI_VERT * err * dt, -STEADY_TRIM_MAX, STEADY_TRIM_MAX)
                data["_sp_thr_trim"] = trim
            dn, up = STEADY_THRUST_DN, STEADY_THRUST_UP
            _pp = data.get("_sp_pass_us")
            if (not STEADY_SURVEY and _pp is not None and t_us_now is not None
                    and (t_us_now - _pp) * 1e-6 < STEADY_POSTPASS_GENTLE_S):
                dn, up = STEADY_POSTPASS_DN, STEADY_POSTPASS_UP
            _cmd = STEADY_KP_VERT * err - STEADY_KD_VZ * vz_est
            thrust = clamp(STEADY_HOVER + trim + _cmd,
                           STEADY_HOVER + trim - dn, STEADY_HOVER + trim + up)
        # PITCH: creep forward, faded while off-aim (turn first, then close). In ALIGN (near but
        # off-axis) it BRAKES - a gentle backlean - because zeroing the command doesn't stop the
        # momentum already carried (run 111346 coasted into the gate throat mid-alignment and
        # clipped the top bar). Big ROW error (CLIMB-FIRST) still just zeroes: climb in place.
        aim = clamp(1.0 - abs(offx) / STEADY_AIM_REF, 0.0, 1.0)
        climb_hold = False
        if align_mode:
            des_pitch = -STEADY_ALIGN_BRAKE
        elif abs(err) > STEADY_CLIMB_GATE:
            # CLIMB-FIRST - and near a gate, HOLD STATION while climbing (hover_pilot's range-hold):
            # pitch-zero let residual drift carry the climb into the top bar (run 155544), and a
            # CONSTANT back-pitch kept accelerating backward over the multi-second climb (backed
            # away, clipped the top, no pass). A range servo brakes exactly as much as needed.
            if area >= STEADY_ALIGN_AREA:
                climb_hold = True
                rng = 1.0 / math.sqrt(max(area, 1e-9))
                hold = data.get("_sp_climb_rng")
                if hold is None:
                    hold = rng
                    data["_sp_climb_rng"] = rng
                des_pitch = clamp(STEADY_HOLD_KP * (rng - hold),
                                  -2.0 * STEADY_ALIGN_BRAKE, 0.6 * STEADY_PITCH_FWD)
            else:
                des_pitch = 0.0
        else:
            des_pitch = STEADY_PITCH_FWD * aim
        if not climb_hold:
            data["_sp_climb_rng"] = None
    thrust = clamp(thrust, MIN_THRUST, MAX_THRUST)

    # SURVEY excitation doublet: ADDITIVE perturbation on top of whatever the
    # pilot wants (never replaces it — overriding des_pitch defeated the
    # aim-fade and flew over gates). Only while actively tracking, far from
    # the gate. The preintegrator reads MEASURED attitude, so the pulse only
    # needs to exist, not to be exact.
    if (STEADY_SURVEY and gates and not stale and not commit
            and not align_mode
            and 0.004 <= area < STEADY_CRUISE_FADE_AREA):
        data["_sp_pulse_i"] = data.get("_sp_pulse_i", 0) + 1
        _ph = data["_sp_pulse_i"] % STEADY_SURVEY_PULSE_TICKS
        if _ph < _SURVEY_BRAKE_TICKS:
            des_pitch -= 0.5 * STEADY_PITCH_FWD
        elif _ph < _SURVEY_SURGE_TICKS:
            des_pitch += 0.8 * STEADY_PITCH_FWD

    # Slew pitch and roll so every correction eases in and eases back out (and the row target moves
    # smoothly with the pitch).
    pitch_cmd = pitch_prev + clamp(des_pitch - pitch_prev, -STEADY_PITCH_SLEW * dt, STEADY_PITCH_SLEW * dt)
    data["_sp_pitch_cmd"] = pitch_cmd
    roll_prev = data.get("_sp_roll_cmd", 0.0)
    roll_cmd = roll_prev + clamp(des_roll - roll_prev, -STEADY_ROLL_SLEW * dt, STEADY_ROLL_SLEW * dt)
    data["_sp_roll_cmd"] = roll_cmd

    # 4) SEND: attitude setpoint - the sim's stabilised controller does the balancing.
    send_attitude_setpoint(mavlink_conn, system_boot_ms, roll_cmd, pitch_cmd, heading, thrust)

    # 5) readout + log (every tick)
    n = len(gates)
    _blind = not gates or stale
    _seeking = (_blind and data.get("_sp_pass_us") is not None and t_us_now is not None
                and (t_us_now - data["_sp_pass_us"]) * 1e-6 < STEADY_SEEK_S)
    data["steady_regime"] = ("COMMIT" if commit else
                             "ALIGN" if align_mode else
                             "SEEK" if _seeking else
                             "HOLD" if _blind else "ATTACK")
    data["oracle_thrust"] = thrust
    data["_sp_des_roll"], data["_sp_des_pitch"], data["_sp_thrust"] = roll_cmd, pitch_cmd, thrust
    data["_sp_ndets"] = n
    data["_sp_az"], data["_sp_el"] = offx, offy
    data["_sp_climb"] = vz_est
    data["_sp_yaw"] = heading
    data["_sp_commit"] = 1.0 if commit else 0.0

    if t_us_now is not None and data.get("_sp_t0_us") is None:
        data["_sp_t0_us"] = t_us_now
    t_s = (t_us_now - data["_sp_t0_us"]) * 1e-6 if (t_us_now is not None and data.get("_sp_t0_us") is not None) else 0.0
    fseq = data.get("latest_frame_seq")
    _dbg_log([f"{t_s:.3f}", ("" if fseq is None else f"{fseq:06d}"), int(fresh),
              data.get("_sp_gate_count", 0), n, f"{area:.4f}",
              f"{offx:+.3f}", f"{offy:+.3f}", f"{oy_tgt:+.3f}",
              f"{math.degrees(heading):+.1f}", f"{math.degrees(pitch_cmd):+.2f}",
              f"{math.degrees(roll_cmd):+.2f}", f"{data.get('_sp_ox_rate', 0.0):+.3f}",
              int(commit), data.get("_sp_stale", 0),
              f"{vz_est:+.2f}", f"{data.get('_sp_alt', 0.0):+.2f}",
              f"{math.degrees(m_roll):+.1f}", f"{math.degrees(m_pitch):+.1f}",
              f"{data.get('_sp_thr_trim', 0.0):+.3f}", f"{thrust:.3f}",
              ("" if data.get("_sp_next_head") is None else f"{math.degrees(data['_sp_next_head']):+.1f}"),
              f"{data.get('_sp_seek_dir', 1.0):+.0f}",
              ("" if data.get("_sp_next_doy") is None else f"{data['_sp_next_doy']:+.3f}"),
              # mapper columns (gate_graph requires these)
              str((data.get("vision_frame") or {}).get("sim_time_ns") or ""),
              (data.get("_sp_det_raw") or (0, 0, 0, 0))[2],
              f"{(data.get('_sp_det_raw') or (0, 0, 0, 0))[0]:+.4f}",
              f"{(data.get('_sp_det_raw') or (0, 0, 0, 0))[1]:+.4f}",
              f"{(data.get('_sp_det_raw') or (0, 0, 0, 0))[3]:.2f}",
              f"{math.degrees(data.get('_sp_yaw_meas', STEADY_YAW_SPAWN)):+.1f}",
              f"{math.degrees(roll_cmd - m_roll):+.2f}",
              f"{math.degrees(pitch_cmd - m_pitch):+.2f}",
              (data.get("race_status") or {}).get("active_gate_index", 0),
              0])
