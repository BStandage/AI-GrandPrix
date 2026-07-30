"""
Sprint pilot v1 - steady_pilot's stop-and-align machinery plus a bang/counter-bang transit layer.

FORK of steady_pilot v2 (the 2:10 VQ2 qualifier - byte-preserved next door as the reliable
fallback). Everything steady proved in flight is kept verbatim: the gate tracker, the aim-point
smoothing and lateral-rate damping, the computed-row thrust servo with integral trim, the SEEK
sighting memory, ALIGN / COMMIT with their escape clauses, and pass counting. The mission changes
from COMPLETION to TIME: the F0 budget of the qualification lap put 54% of the 130 s in TRANSIT at
~1 m/s creep, so this pilot sprints the transits and hands the gate itself back to steady's code.

The transit law - a CONTINUOUS speed profile (v2 of the transit layer; the original discrete
bang/counter-bang brake was retired after five flights of nose-up slams blinding the roll damping):
  FLOW    while - and only while - the lock is confirmed and commit-grade aligned: target speed
          tapers linearly with range (v_tgt = V_PASS + K*(rng - R_LAND), capped at V_CRUISE) and
          pitch is a bounded servo around the drag-equilibrium lean. THE BRAKING HAPPENS AFTER
          THE GATE (Brian's doctrine): the taper lands at V_PASS ~3 m/s AT the plane, the commit
          holds it through the throat, and the blind post-pass backpressure sheds it on the far
          side while SEEK pans. Slowing before a gate survives only in the misaligned fallback.
  v_est   dead-reckoned forward speed: own commanded pitch through the measured 96 ms attitude lag
          (the sim holds setpoints at gain 1.0 - the command IS the attitude), sysid drag k=0.0343,
          integrated on IMU timestamps. Open-loop, but it only has to hold for the ~4 s of a
          transit and it re-anchors through every stop-and-align.

Fail-open by construction: the layer can only ADD pitch in the far field; any loss (stale frames,
lock change, aim escape) drops the target speed to zero and bleeds down gently into steady's
behaviour. It has no veto power over locks or commits - the commit latch just also waits for
v_est to cool (the forward twin of the vz gate), which the taper delivers by design.
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
from pilots.sprint_pilot.config import *   # SPRINT_* params


_DBG_W = _DBG_F = None
_DBG_N = 0
# EVERY control tick (~90 Hz): what was measured, what the law decided, what was sent.
# Columns 0..23 are steady's, byte-compatible (the analysis scripts parse them); sprint's transit
# telemetry is APPENDED after next_doy, never renamed or reordered.
_DBG_HDR = ["t", "frame", "fresh", "gates_passed", "n_dets", "area", "aim_ox", "aim_oy", "oy_tgt",
            "yaw_deg", "pitch_deg", "roll_cmd_deg", "ox_rate", "commit", "stale", "vz_est", "alt_est",
            "roll_meas_deg", "pitch_meas_deg", "trim", "thr", "next_head_deg", "seek_dir", "next_doy",
            "trans", "v_est", "v_meas", "rng_est", "d_stop", "thr_ff", "rng_trig", "v_land",
            "v_lat"]


def _dbg_log(row):
    global _DBG_W, _DBG_F, _DBG_N
    if _DBG_W is None:
        path = os.path.join(DATASETS_DIR, time.strftime("sprint_dbg_%Y%m%d_%H%M%S.csv"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _DBG_F = open(path, "w", newline="")
        _DBG_W = csv.writer(_DBG_F)
        _DBG_W.writerow(_DBG_HDR)
        print(f"sprint debug -> {path}", flush=True)
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
                vz = (data.get("_sp_vz", 0.0) + a_up * dt) * math.exp(-dt / SPRINT_VZ_TAU)
                data["_sp_vz"] = clamp(vz, -3.0, 3.0)
                data["_sp_alt"] = data.get("_sp_alt", 0.0) + data["_sp_vz"] * dt
    data["_sp_vz_us"] = t_us
    return data.get("_sp_vz", 0.0)


def _v_fwd_estimate(data, pitch_cmd):
    """Dead-reckoned forward speed (m/s, +ahead) - the transit layer's one new state variable.
    v' = g*tan(theta) - k*v*|v|, theta = our own commanded pitch through the measured 96 ms
    attitude lag (sysid tab 1; the sim holds setpoints at gain 1.0, so no attitude estimate is
    involved - v1's original sin). Integrated on highres_imu.time_usec deltas, never 1/CONTROL_HZ
    (the loop rate is fiction). LEAKS toward zero only while near-level in CREEP: the quadratic
    drag model has no low-speed term, so after a stop it would hold a phantom 0.5 m/s forever;
    mid-transit it NEVER leaks (an under-read v_est brakes late - the unsafe direction).
    Returns (v_est, v_meas): v_meas is the coast-only cross-check sqrt(|ax|/k) - the measured
    drag calibration - logged for the F1 comparison, never steered on."""
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")
    if t_us is None:
        return data.get("_sp_v_est", 0.0), None
    prev = data.get("_sp_v_us")
    if prev is not None and t_us > prev:
        dt = (t_us - prev) * 1e-6
        if dt <= 0.1:
            th = data.get("_sp_th_lag", 0.0)
            th += (pitch_cmd - th) * (1.0 - math.exp(-dt / SPRINT_ATT_TAU))
            data["_sp_th_lag"] = th
            v = data.get("_sp_v_est", 0.0)
            v += (9.81 * math.tan(th) - SPRINT_K_DRAG * v * abs(v)) * dt
            if data.get("_sp_trans", "CREEP") == "CREEP" and abs(th) < 0.03:
                v *= math.exp(-dt / SPRINT_V_LEAK_TAU)
            data["_sp_v_est"] = clamp(v, -3.0, 10.0)
    data["_sp_v_us"] = t_us
    v_meas = None
    if abs(data.get("_sp_th_lag", 0.0)) < 0.02:
        ax = imu.get("xacc")
        if ax is not None:
            v_meas = math.sqrt(abs(ax) / SPRINT_K_DRAG)
    return data.get("_sp_v_est", 0.0), v_meas


def _v_lat_estimate(data, roll_cmd):
    """Dead-reckoned LATERAL speed (m/s, +right) - the roll damper's velocity source. Mirrors
    _v_fwd_estimate: commanded roll through the measured 96 ms attitude lag, quadratic drag,
    IMU-timestamp integration, leak toward zero only while wings-near-level (no low-speed drag
    term). WHY: the vision lateral rate is structurally DEAD in FLOW - any fast vertical aim
    motion discards the sample (SPRINT_OYRATE_GATE), and at transit speed the aim row is always
    moving - so run 220336 flew undamped-P on a double integrator: 2.3 s of bank chasing the
    launch-parallax phantom built ~1.3 m/s rightward, the aim swung +0.13 -> -0.42, right post
    of gate 1. Physics is always available; vision rate samples are not."""
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")
    if t_us is None:
        return data.get("_sp_v_lat", 0.0)
    prev = data.get("_sp_vlat_us")
    if prev is not None and t_us > prev:
        dt = (t_us - prev) * 1e-6
        if dt <= 0.1:
            ph = data.get("_sp_ph_lag", 0.0)
            ph += (roll_cmd - ph) * (1.0 - math.exp(-dt / SPRINT_ATT_TAU))
            data["_sp_ph_lag"] = ph
            v = data.get("_sp_v_lat", 0.0)
            v += (9.81 * math.tan(ph) - SPRINT_K_DRAG * v * abs(v)) * dt
            # LEAK below ~3.4 deg of commanded roll (run 223110): unlike pitch (12-deg leans),
            # lateral rolls live at 1-3 deg, where command-vs-actual bias DOMINATES the integral
            # - a 1.7-deg command held 4 s read +0.81 m/s of phantom drift. Small-angle v_lat
            # bleeds fast; only decisive banks integrate as real motion.
            if abs(ph) < SPRINT_VLAT_LEVEL:
                v *= math.exp(-dt / SPRINT_VLAT_LEAK_TAU)
            data["_sp_v_lat"] = clamp(v, -2.5, 2.5)
    data["_sp_vlat_us"] = t_us
    return data.get("_sp_v_lat", 0.0)


def _flow_pitch(v_tgt, v_est, a_req=0.0):
    """Continuous transit pitch: drag-equilibrium feedforward for the target speed, MINUS the
    profile's demanded deceleration when braking (a_req, m/s^2 - drag already supplies part of it,
    only the remainder needs backlean), plus a proportional term on the speed error. Clamped
    [-TAPER_BACK, LEAN] - a slam stays structurally impossible. The decel feedforward is the run
    210832 fix: without it the KP term had to manufacture ALL the backlean from accumulated speed
    error, so v_est dragged above the taper line for the whole approach and the pitch chattered
    +-2 deg around zero (thrust pumping 0.25<->0.29 with it)."""
    ff = math.atan((SPRINT_K_DRAG * v_tgt * v_tgt - a_req) / 9.81)
    return clamp(ff + SPRINT_KP_V * (v_tgt - v_est), -SPRINT_TAPER_BACK, SPRINT_LEAN)


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _v_land(data, t_us):
    """LOOKAHEAD-PRICED landing speed (Brian's law): the pass speed THIS gate has earned, from the
    next-gate sighting made DURING THIS APPROACH. dpsi (turn demanded after the pass) is INFLATED -
    the mid-approach bearing systematically understates the true corner (SEEK-era scar) - and
    elevation change (|next_doy|) prices a climb/descent gate down. No sighting this approach, or a
    sharp/vertical next leg -> SPRINT_V_LAND_MIN = settled speed = steady-grade gate handling.
    Liveness is the GATE COUNT, not a wall clock: run 210832 proved sightings arrive in bursts
    early in the approach (2-11 s old at brake onset), so a 1 s freshness window silently priced
    every gate to the hairpin floor. What the wall clock was protecting against - pricing gate N+1
    with gate N's data - is exactly what the gate-count key forbids; NEXT_LIVE_S survives only as a
    loose backstop. The law can only ADD speed above the proven-safe floor; never subtract safety."""
    nh = data.get("_sp_next_head")
    nus = data.get("_sp_next_us")
    doy = data.get("_sp_next_doy")
    if (nh is None or nus is None or t_us is None
            or data.get("_sp_next_gate") != data.get("_sp_gate_count", 0)
            or (t_us - nus) * 1e-6 > SPRINT_NEXT_LIVE_S):
        return SPRINT_V_LAND_MIN
    dpsi = abs(_wrap_pi(nh - data.get("_sp_heading", 0.0))) * SPRINT_PSI_INFLATE
    dh = abs(doy) if doy is not None else SPRINT_DOY_REF
    score = 1.0 - dpsi / SPRINT_PSI_REF - dh / SPRINT_DOY_REF
    return clamp(SPRINT_V_LAND_MIN + (SPRINT_V_LAND_MAX - SPRINT_V_LAND_MIN) * score,
                 SPRINT_V_LAND_MIN, SPRINT_V_LAND_MAX)


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
    data["_sp_preturn_h0"] = None      # heading when the pre-turn engaged (caps the pre-pass sweep)
    data["_sp_pass_block"] = False
    data["_sp_commit_latch"] = False
    data["_sp_avoid_off"] = None
    data["_sp_stale"] = 0
    data["_sp_t0_us"] = None
    data["_sp_heading"] = SPRINT_YAW_SPAWN   # integrated ABSOLUTE yaw command (rad). The setpoint yaw is
                                          # a WORLD direction: seeding 0 snapped the drone ~97 deg off
                                          # the course at race start (run 20260727_000330). Seed with
                                          # the measured spawn facing so tick one means "hold still".
    data["_sp_pitch_cmd"] = 0.0        # slewed pitch command (rad)
    data["_sp_oy_pitch"] = 0.0         # lagged pitch for the row target (matches camera latency)
    data["_sp_roll_cmd"] = 0.0         # slewed roll command (rad)
    data["_sp_climb_rng"] = None       # range anchor for the near-gate climb station-hold
    data["_sp_thr_trim"] = 0.0         # learned hover-thrust correction (integral trim)
    data["_sp_ox_rate"] = 0.0          # aim-point lateral rate (units/s) - logging only (retired
                                       # as the roll damper; v_lat dead-reckoning owns damping)
    data["_sp_ox_blind"] = True        # no valid lateral-rate sample yet (logging only)
    data["_sp_v_lat"] = 0.0            # dead-reckoned lateral speed (m/s, +right) - roll damper
    data["_sp_ph_lag"] = 0.0           # commanded roll through the 96 ms attitude lag
    data["_sp_vlat_us"] = None
    data["_sp_prev_ox"] = None
    data["_sp_prev_oy"] = None
    data["_sp_prev_ox_us"] = None
    data["_sp_yawed"] = False          # a yaw nudge moved the aim last frame -> skip one rate sample
    data["_sp_next_head"] = None       # remembered ABSOLUTE heading of the next gate (rad)
    data["_sp_next_doy"] = None        # its elevation RELATIVE to the then-tracked gate (offset_y)
    data["_sp_next_area_mem"] = 0.0    # its size when sighted
    data["_sp_next_us"] = None
    data["_sp_next_gate"] = None       # gates_passed count when the sighting was written - pricing
                                       # only trusts a sighting made during THIS approach
    data["_sp_next_area"] = 0.0        # decaying area bar - biggest credible sighting wins
    data["_sp_pass_us"] = None         # when the last pass counted (starts the SEEK window)
    data["_sp_pass_head"] = 0.0
    data["_sp_pass_alt"] = 0.0         # alt_est when the last pass counted (= the gate line)
    data["_sp_seek_dir"] = 1.0         # which way SEEK rotates (signed; from the memory)
    data["_sp_lock_frames"] = 0        # consecutive frames on the SAME lock (maturity gate)
    data["_sp_slew_us"] = None
    data["_sp_est_roll"] = 0.0
    data["_sp_est_pitch"] = 0.0
    data["_sp_att_us"] = None
    data["_sp_vz"] = 0.0
    data["_sp_alt"] = 0.0
    data["_sp_vz_us"] = None
    data["_sp_trans"] = "CREEP"        # transit state: CREEP (= steady behaviour) / FLOW (profile)
    data["_sp_row_cap_latch"] = False  # row captured once this run -> lateral authority + FLOW allowed
    data["_sp_row_cap_n"] = 0          # consecutive small-row-error ticks while arming the latch
    data["_sp_trig_area"] = None       # growth-rate-limited area feeding the speed profile
    data["_sp_trig_us"] = None
    data["_sp_v_est"] = 0.0            # dead-reckoned forward speed (m/s)
    data["_sp_v_us"] = None
    data["_sp_th_lag"] = 0.0           # commanded pitch through the 96 ms attitude lag (rad)
    data["_sp_live"] = False


def _idle(mavlink_conn, system_boot_ms, data, regime):
    data["sprint_regime"] = regime
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


def _select_target(gates, data):
    """Follow ONE gate by spatial proximity; post-pass, avoid the flown-through gate (proven)."""
    avoid = data.get("_sp_avoid_off")
    # The avoid lock EXPIRES: its whole job is the 1-2 s post-pass handoff.
    _us = (data.get("highres_imu") or {}).get("time_usec")
    p_us = data.get("_sp_pass_us")
    if avoid is not None and _us is not None and p_us is not None \
            and (_us - p_us) * 1e-6 > SPRINT_AVOID_S:
        avoid = None
        data["_sp_avoid_off"] = None
    if avoid is not None:
        ax, ay = avoid
        near = min(gates, key=lambda d: math.hypot(d.offset_x - ax, d.offset_y - ay))
        # Follow the avoided gate ONLY while the nearby blob is still BIG - the genuinely
        # just-passed gate filled the frame moments ago. A SMALL blob near the stale position is a
        # DIFFERENT gate that drifted in (run 110222: the next gate inherited the avoid lock and
        # the tracker flew at a 22 m speck instead) - drop the avoid, don't adopt it.
        if (math.hypot(near.offset_x - ax, near.offset_y - ay) <= SPRINT_AVOID_RADIUS
                and near.area_frac >= SPRINT_AVOID_MIN_AREA):
            data["_sp_avoid_off"] = (near.offset_x, near.offset_y)
        else:
            avoid = None
            data["_sp_avoid_off"] = None

    def usable():
        if avoid is None:
            return gates
        ax, ay = avoid
        return [d for d in gates if math.hypot(d.offset_x - ax, d.offset_y - ay) > SPRINT_AVOID_RADIUS]

    in_pp = (p_us is not None and _us is not None and (_us - p_us) * 1e-6 < SPRINT_SEEK_S)

    def _sane(ds):
        """Post-pass sky-blob exclusion: a candidate ~50deg+ above the horizon cannot be a course
        gate - only the just-passed gate's top bar / ceiling junk. HARD refusal, no yield: run
        203302 killed itself when a LONE sky blob (n_dets=1, aim_oy -0.90) slipped through the old
        fail-open yield 0.96 s after the gate-2 pass and the row servo chased it upward for 3 s
        (vz pinned +3.0, alt 4.7 -> 6.2, every real gate below the FoV floor). A sky lock is WORSE
        than no lock: blind triggers SEEK, which pans safely at hover. Fail-open protects
        PLAUSIBLE gates from fragile checks; it does not protect the ceiling. The test covers the
        AIM point too (opening bbox) - a fragment's opening can sit far above its ring centre."""
        if not in_pp:
            return ds
        return [d for d in ds
                if d.offset_y > SPRINT_JUNK_OY and _aim_offsets(d)[1] > SPRINT_JUNK_OY]

    prev = data.get("_sp_track_off")
    if prev is None:
        data["_sp_track_miss"] = 0
        cands = _sane(usable() or gates)   # never go blind on a frozen bearing - fall back to ALL
        if not cands:
            return None
        # Acquisition is FAIL-OPEN: always lock the best available candidate (biggest, preferring a
        # located opening). The memory-consistent refusal layer (bearing cone + sighting checks) was
        # REMOVED after three rounds of regressions: when its remembered inputs were even slightly
        # off it refused to lock ANYTHING - flying blind past gate 2 - which is strictly worse than
        # occasionally locking the wrong gate. The seek memory still steers the SWEEP direction;
        # it no longer gets veto power over locks.
        return _acquire(cands)
    px, py = prev
    cands = usable() or gates
    g = min(cands, key=lambda d: (d.offset_x - px) ** 2 + (d.offset_y - py) ** 2)
    if math.hypot(g.offset_x - px, g.offset_y - py) <= SPRINT_TRACK_RADIUS:
        data["_sp_track_miss"] = 0
        # YOUNG-LOCK PREEMPTION (post-pass window only, fail-open): the corner sweep meets gates in
        # the wrong order - the far next-next gate enters frame first and gets locked; the TRUE next
        # gate is ~half its distance, so ~4x bigger the moment the sweep reaches it. A much-bigger
        # candidate steals the lock; nothing is ever refused.
        if in_pp:
            sane_cands = _sane(cands)
            if sane_cands:
                best = _acquire(sane_cands)
                if best is not g and best.area_frac >= SPRINT_PREEMPT_RATIO * max(g.area_frac, 1e-6):
                    return best
        return g
    miss = data.get("_sp_track_miss", 0) + 1
    data["_sp_track_miss"] = miss
    if miss >= SPRINT_TRACK_MAX_MISS:
        data["_sp_track_miss"] = 0
        return _acquire(cands)
    return None


def update_sprint_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    if not _race_live(data):
        t_go = _seconds_to_go(data)
        _idle(mavlink_conn, system_boot_ms, data,
              f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0) else "IDLE")
        return

    if not data.get("_sp_live"):
        _reset(data)
        data["_sp_live"] = True

    # Measured attitude + vertical estimate: LOGGING and mild thrust damping only.
    m_roll, m_pitch = _imu_attitude(data)
    vz_est = _vz_alt_estimate(data, m_roll, m_pitch)
    # Forward-speed dead-reckoning for the transit layer (from our own commanded pitch - the
    # measured attitude stays logging-only, exactly like steady).
    v_est, v_meas = _v_fwd_estimate(data, data.get("_sp_pitch_cmd", 0.0))
    v_lat = _v_lat_estimate(data, data.get("_sp_roll_cmd", 0.0))

    # 1) TRACK one gate; smooth the AIM offsets (the opening, when located).
    fresh = False
    fid = data.get("latest_frame_id")
    gates = [d for d in (data.get("vision_gates") or []) if d.area_frac >= SPRINT_MIN_AREA]
    if fid is not None and fid != data.get("_sp_fid") and gates:
        data["_sp_fid"] = fid
        had_lock = data.get("_sp_track_off") is not None
        g = _select_target(gates, data)
        if g is None:
            data["_sp_stale"] = data.get("_sp_stale", 0) + 1
        else:
            ax, ay = _aim_offsets(g)
            sm = data.get("_sp_off_smooth")
            hop = sm is None or math.hypot(ax - sm[0], ay - sm[1]) > SPRINT_HOP_DIST
            if hop:
                sx, sy = ax, ay
            else:
                sx = SPRINT_OFF_ALPHA * ax + (1 - SPRINT_OFF_ALPHA) * sm[0]
                sy = SPRINT_OFF_ALPHA * ay + (1 - SPRINT_OFF_ALPHA) * sm[1]
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
                    if abs(oy_rate) > SPRINT_OYRATE_GATE:
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
                        data["_sp_ox_rate"] = clamp(raw, -SPRINT_OXRATE_MAX, SPRINT_OXRATE_MAX)
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
            # NEXT-GATE MEMORY: note the ABSOLUTE heading of the biggest OTHER detection (excluding
            # the just-passed avoid gate). At a sharp corner the next gate leaves the FoV before the
            # pass finishes - this remembered bearing tells SEEK which way to turn, signed, so left
            # and right corners are the same code. offset_x -> angle via atan(ox * HALF_TAN_X);
            # heading frame is CCW-positive, so a target to the RIGHT is a SMALLER heading.
            av = data.get("_sp_avoid_off")
            others = [d for d in gates if d is not g and d.area_frac >= SPRINT_NEXT_MIN_AREA
                      and (av is None or math.hypot(d.offset_x - av[0], d.offset_y - av[1]) > SPRINT_AVOID_RADIUS)]
            # BIGGEST credible sighting wins, not the LATEST: the memory holds a decaying area bar
            # (~4 s window) a new candidate must beat. Run 123753: gate 4 was seen at the frame edge
            # (area 0.026, LEFT) during the corner handoff, then a 0.007 right-side speck was seen
            # one frame later and overwrote the memory - SEEK pirouetted ~180 deg the WRONG way.
            bar = data.get("_sp_next_area", 0.0) * 0.99
            data["_sp_next_area"] = bar
            # RECORDING HYGIENE - the memory only accepts sightings from CLEAN frames:
            #  * not within 1.5 s AFTER a pass (receding-gate fragments - run 124505's wrong-way
            #    orbit), and
            #  * not while the tracked gate is NEAR (area >= align range): at point-blank the
            #    current gate splits into huge pillar fragments that pose as "others" and poisoned
            #    the memory with a 0.3-area straight-ahead "next gate" - whose remembered size then
            #    vetoed every real candidate after the pass (run 130xxx: never tracked gate 2).
            _pus = data.get("_sp_pass_us")
            _nus = (data.get("highres_imu") or {}).get("time_usec")
            settled = (_pus is None or _nus is None
                       or (_nus - _pus) * 1e-6 > SPRINT_NEXT_HOLDOFF)
            settled = settled and g.area_frac < SPRINT_ALIGN_AREA
            if others and settled:
                nd = max(others, key=lambda d: d.area)
                if nd.area_frac >= bar:
                    data["_sp_next_head"] = (data.get("_sp_heading", 0.0)
                                             - math.atan(nd.offset_x * HALF_TAN_X))
                    # the SIGHTING TRACK: bearing + elevation RELATIVE to the tracked gate (survives
                    # whatever our attitude/altitude do during the pass) + size
                    data["_sp_next_doy"] = nd.offset_y - g.offset_y
                    data["_sp_next_area_mem"] = nd.area_frac
                    data["_sp_next_us"] = (data.get("highres_imu") or {}).get("time_usec")
                    data["_sp_next_gate"] = data.get("_sp_gate_count", 0)
                    data["_sp_next_area"] = nd.area_frac

    area = data.get("_sp_area", 0.0)
    offx, offy = data.get("_sp_off", (0.0, 0.0))

    # TRANSIT GEOMETRY: blob-area range proxy vs the stopping distance at the current speed.
    # d_stop charges the full kinematic arrest at the brake lean, PLUS the distance the vision
    # lag hides (the aim data is ~150 ms stale - metres, at speed), PLUS a flat margin. Drag
    # assist during the brake is deliberately ignored - free conservatism.
    rng_est = SPRINT_C_RNG / math.sqrt(max(area, 1e-6))
    # LOOKAHEAD-PRICED landing speed: what THIS gate has earned, from the live next-gate sighting.
    # Replaces the flat SPRINT_V_PASS that carried 3 m/s into hairpins and straights alike.
    v_land = _v_land(data, (data.get("highres_imu") or {}).get("time_usec"))
    # For the log's d_stop column: the range where the constant-decel profile starts demanding
    # deceleration for the current speed - "how much room the profile wants", the closest
    # analogue to the retired stopping distance.
    d_stop = SPRINT_R_LAND + max(v_est * v_est - v_land * v_land, 0.0) / (2.0 * SPRINT_A_BRAKE)
    # TRIGGER AREA: growth-rate-limited copy of the raw area for the transit layer's anticipatory
    # decisions ONLY (see SPRINT_AREA_GROW in config for the two-flight evidence). Reseeds on a
    # fresh lock (lock_frames <= 1); follows drops instantly; upward growth capped at
    # SPRINT_AREA_GROW x the kinematic rate for the current v_est and range.
    if fresh:
        ta = data.get("_sp_trig_area")
        t_us_f = (data.get("highres_imu") or {}).get("time_usec")
        p_us_f = data.get("_sp_trig_us")
        if ta is None or data.get("_sp_lock_frames", 0) <= 1 or t_us_f is None or p_us_f is None:
            ta = area
        else:
            dtf = (t_us_f - p_us_f) * 1e-6
            if 0.0 < dtf <= 0.2:
                r_now = SPRINT_C_RNG / math.sqrt(max(ta, 1e-6))
                grow = 1.0 + SPRINT_AREA_GROW * 2.0 * max(v_est, 0.5) * dtf / max(r_now, 0.5)
                ta = min(area, ta * grow)
            else:
                ta = area
        data["_sp_trig_area"] = ta
        data["_sp_trig_us"] = t_us_f
    trig_area = data.get("_sp_trig_area", area)
    rng_trig = SPRINT_C_RNG / math.sqrt(max(trig_area, 1e-6))

    # Regime predicates. COMMIT requires proximity AND alignment (run 095624 committed on area
    # alone, coasted wide right into the gate edge). ALIGN-mode (brake + strafe, no creep) engages
    # EARLY when misaligned - from SPRINT_ALIGN_AREA, not commit range - because braking at 2 m with
    # 0.3 of offset just scrapes down the pillar (run 113145). Aligned approaches skip the band.
    stale = data.get("_sp_stale", 0) >= SPRINT_MAX_STALE
    tracking = bool(gates) and not stale
    aligned_x = abs(offx) <= SPRINT_COMMIT_ALIGN
    # Vertical readiness (run 122735: committed mid-climb at aim_oy -0.36 / vz +2.97 and sagged
    # into the bottom bar). Blocked-vertical routes through the SERVO branch, whose row servo +
    # climb-first pitch already do exactly the right thing: finish the climb, then commit.
    # The READINESS row reference is the steady-state commit attitude (the creep lean), NOT the
    # instantaneous pitch command: right after a brake the still-recovering nose-up pitch inflates
    # the computed row to ~0.9+ (F1b latched against 0.95 while the true level row is ~0.49), so a
    # too-high approach can look vertically aligned. Steady never saw this - its pitch IS always
    # the creep - so this constant reference reproduces steady's proven geometry exactly.
    row_tgt = math.tan(UPTILT_RAD - SPRINT_PITCH_FWD) / HALF_TAN_Y + SPRINT_OFFY_BIAS
    aligned_y = abs(offy - row_tgt) <= SPRINT_COMMIT_ROW
    # COMMIT LATCHES. Alignment (both axes) is an ENTRY condition only - at point-blank range the
    # aim balloons away from the far-field row target BY GEOMETRY, and re-checking it every tick
    # un-committed the pilot INSIDE the tilted gate (run 123331, t=67.7): the row servo woke mid-
    # pass, firewalled a +2.2 m/s climb in the gate throat, and the pass never counted. Enter on
    # readiness; release only on pass handoff, lost tracking, or the gate genuinely receding.
    vz_ok = abs(vz_est) <= SPRINT_COMMIT_VZ
    # FORWARD twin of the vz gate (run 132724's lesson rotated 90 deg): never latch a commit while
    # still hot from a sprint. The BRAKE is already driving v_est down toward SPRINT_V_DONE, so
    # this only delays the latch a few ticks - it can never refuse a gate outright (fail-open).
    # The latch admits whatever speed THIS gate earned (plus slack for taper tracking error) - the
    # flat SPRINT_COMMIT_V let a hairpin gate latch at the same 3.5 m/s as a straight one.
    v_fwd_ok = v_est <= v_land + SPRINT_COMMIT_V_SLACK
    # NO v_lat GATE (reverted, run 223110): a "lateral velocity" commit gate blocked a PERFECTLY
    # positioned commit (ox 0.00, on-row, v 1.4) on a PHANTOM reading - v_lat integrates the
    # commanded roll, and at small sustained angles it is bias-dominated (+0.81 from a 1.7-deg
    # command held 4 s). The blocked pilot then loitered 2 s in the throat - the exact slow-
    # loiter failure the commit doctrine exists to prevent - drifted up 0.4 m, and hit the top
    # bar. v_lat may DAMP (bounded, recoverable) but never GATE (unbounded loiter on a phantom).
    if tracking and area >= SPRINT_COMMIT_AREA and aligned_x and aligned_y and vz_ok and v_fwd_ok:
        data["_sp_commit_latch"] = True
    if data.get("_sp_commit_latch") and (not tracking or area < SPRINT_PASS_REARM):
        data["_sp_commit_latch"] = False
    commit = bool(data.get("_sp_commit_latch"))
    # ALIGN also engages when the VERTICAL SPEED is too hot to commit (descending off the cruise
    # glide, run 132724): the brake holds station while the arrest settles vz, then commit - level.
    # Same for leftover FORWARD speed (BRAKE overrides ALIGN's gentle backlean until it's shed).
    align_mode = (tracking and not commit and area >= SPRINT_ALIGN_AREA
                  and (not aligned_x or not vz_ok or not v_fwd_ok))

    # 2) PASS = area peaked then receded -> count it and hand off to the next gate. Arms ONLY while
    # COMMITTED: area receding from a lateral scrape is not a pass (run 095624 counted 6 "gates").
    if data.get("_sp_pass_block") and area < SPRINT_PASS_REARM:
        data["_sp_pass_block"] = False
    # Velocity-aware arming: a fast pass may give the camera only 2-3 frames above the area
    # threshold, so a committed approach INSIDE arming range also arms (range proxy, same C_RNG).
    if commit and (area >= SPRINT_PASS_AREA or rng_trig <= SPRINT_R_ARM) \
            and not data.get("_sp_pass_block"):
        data["_sp_pass_armed"] = True
        data["_sp_pass_peak"] = max(data.get("_sp_pass_peak", 0.0), area)
    if data.get("_sp_pass_armed") and area < SPRINT_PASS_DROP * data.get("_sp_pass_peak", 0.0):
        data["_sp_gate_count"] = data.get("_sp_gate_count", 0) + 1
        data["_sp_pass_armed"] = False
        data["_sp_pass_peak"] = 0.0
        data["_sp_preturn_h0"] = None
        data["_sp_pass_block"] = True
        data["_sp_commit_latch"] = False   # pass done - release the commit latch for the next gate
        data["_sp_avoid_off"] = data.get("_sp_track_off")
        data["_sp_track_off"] = None
        data["_sp_off_smooth"] = None
        # Start the SEEK window: stamp the pass, and pick the rotation direction from the next-gate
        # memory (fresh -> signed bearing; stale -> its sign is still the best guess; none -> CCW).
        _us_p = (data.get("highres_imu") or {}).get("time_usec")
        head_now = data.get("_sp_heading", 0.0)
        data["_sp_pass_us"] = _us_p
        data["_sp_pass_head"] = head_now
        data["_sp_pass_alt"] = data.get("_sp_alt", 0.0)   # we just flew THROUGH a gate: this IS the
                                                          # gate line, drift and all - SEEK's reference
        nh = data.get("_sp_next_head")
        if nh is not None and abs(nh - head_now) > 0.05:
            data["_sp_seek_dir"] = 1.0 if nh > head_now else -1.0
        data["_sp_next_area"] = 0.0   # fresh memory window for the gate AFTER the one just acquired
        # RACING LINE: the sighting of the next gate made during the approach just finished IS
        # the lookahead for the leg we are entering - re-stamp it valid for the new gate count so
        # the post-pass carry and pricing read it instead of defaulting to the hairpin floor.
        if data.get("_sp_next_gate") == data["_sp_gate_count"] - 1:
            data["_sp_next_gate"] = data["_sp_gate_count"]

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
        # HOLD: hover (with the learned trim), level. Within the post-pass window this is SEEK:
        # rotate toward/past the remembered next-gate heading until vision acquires - the FoV
        # catches a gate up to 45 deg before the nose reaches it, and the servo branch takes over
        # the moment a detection lands. Clear the avoid zone once well into the turn (the passed
        # gate cannot still be in view, and its stale frame-position must not veto the new gate).
        des_pitch = 0.0
        des_roll = 0.0
        v_tgt = 0.0
        # BLIND AT SPEED = shed only down to the PRICED carry speed for this leg, not to zero
        # (the racing-line change - run 220921 dumped 2.5 -> 1.0 m/s after every pass no matter
        # what the turn needed, then crawled into the next gate). The pricing's dpsi term shrinks
        # AS the pre-turn/SEEK yaw closes on the remembered heading, so post-pass braking is
        # exactly proportional to how much turning remains: a straight-ahead next leg carries
        # speed, a hairpin still sheds to the floor, and NO sighting -> floor = old behavior.
        # min(v_land, v_est): hold what we have if already below the carry - never ADD energy
        # while blind.
        if v_est > SPRINT_V_SETTLED:
            data["_sp_trans"] = "FLOW"
            des_pitch = _flow_pitch(min(v_land, v_est), v_est)
        else:
            data["_sp_trans"] = "CREEP"
        p_us_pass = data.get("_sp_pass_us")
        if (p_us_pass is not None and t_us_now is not None
                and (t_us_now - p_us_pass) * 1e-6 < SPRINT_SEEK_S):
            heading += data.get("_sp_seek_dir", 1.0) \
                * (SPRINT_SEEK_RATE_FAST if SPRINT_PRETURN else SPRINT_SEEK_RATE) * dt
            data["_sp_heading"] = heading
            if abs(heading - data.get("_sp_pass_head", heading)) > 0.5:
                data["_sp_avoid_off"] = None
            # Sink back to the gate line while seeking: from above it, the NEAR next gate sits below
            # the camera's 9-deg down-limit and a FAR one steals the lock (the run-115646 miss).
            if data.get("_sp_alt", 0.0) > data.get("_sp_pass_alt", 0.0) + SPRINT_SEEK_ALT_TOL:
                v_tgt = -SPRINT_SEEK_SINK
        thrust = SPRINT_HOVER + trim - SPRINT_KD_VZ_COMMIT * (vz_est - v_tgt)
    elif commit:
        # COMMIT: the aim point balloons this close - stop chasing it. Freeze the heading, wings
        # LEVEL, keep the creep on, let the pass logic call it. Thrust holds hover WITH a strong
        # vz-arrest: any residual climb/sink from the approach coasts into a gate bar otherwise
        # (gate 1's top bar, run 101632). ONE escape clause: if the hole climbs far above image
        # centre mid-coast (the TILTED gate - its passage line is higher than the level line, and
        # the frozen coast clipped its bottom bar, run 122217), follow it up. Climb-only, and a
        # straight gate's commit never breaches the deadband.
        # COMMIT AT PASS SPEED (brake-after-the-gate doctrine): hold V_PASS through the throat
        # with the flow servo - the last 2 m take ~0.7 s, leaving no time for the drift/ascend
        # failures that killed every slow commit. Lean capped: never a full bang in a throat.
        des_pitch = min(_flow_pitch(v_land, v_est), SPRINT_COMMIT_LEAN)
        data["_sp_trans"] = "CREEP"   # transit bookkeeping resets; the pass handoff re-opens FLOW
        # CORNER PRE-TURN (config-gated - its own flight rung, OFF for F1): once the pass is ARMED
        # (area peaked >= PASS_AREA, i.e. inside the throat, where the ballooned aim is geometry
        # rather than signal), start the yaw toward the remembered next-gate heading. The frozen
        # commit heading is doctrine written for a pilot with no speed to lose; the FoV leads the
        # nose by up to 45 deg, so every degree turned here is SEEK dwell that never happens.
        # TIMING (run 221609): armed-by-range fired 3 m SHORT of the plane and the nose swung
        # 27-31 deg before crossing it - the lean is BODY-frame, so a nose 30 deg off the flight
        # path pushes the drone sideways through the plane (the edge hits). Pre-turn now needs
        # the area to have actually PEAKED in the throat, and the pre-pass sweep is capped: the
        # bulk of the turn belongs to the post-pass SEEK, whose fast rate is already paid for.
        if SPRINT_PRETURN and data.get("_sp_pass_peak", 0.0) >= SPRINT_PASS_AREA:
            nh = data.get("_sp_next_head")
            if nh is not None:
                h0 = data.get("_sp_preturn_h0")
                if h0 is None:
                    h0 = heading
                    data["_sp_preturn_h0"] = h0
                step = clamp(nh - heading, -SPRINT_PRETURN_RATE * dt, SPRINT_PRETURN_RATE * dt)
                if abs((heading + step) - h0) <= SPRINT_PRETURN_MAX_PRE:
                    heading += step
                    data["_sp_heading"] = heading
                    data["_sp_yawed"] = True   # panning - skip the next lateral-rate sample
        # Lateral escape clause (twin of the vertical one): near-wings-level UNLESS the aim
        # escapes far sideways mid-coast - run 130224 watched it walk to -0.41 and scraped the
        # left pillar. Gentle (blind-cap), toward the aim, zero inside the deadband normal
        # commits never leave. Baseline is DRIFT-KILL, not a hard zero (run 222247): residual
        # lateral velocity carried a centred entry sideways across the throat, and the aim at
        # point-blank is too chaotic to chase - but v_lat is pure dead reckoning, vision-free,
        # so damping it inside the throat is always safe.
        des_roll = clamp(-SPRINT_KD_VLAT * v_lat, -SPRINT_ROLL_BLIND, SPRINT_ROLL_BLIND)
        if abs(offx) > SPRINT_COMMIT_SIDE_OFFX:
            side = (abs(offx) - SPRINT_COMMIT_SIDE_OFFX) * (1.0 if offx > 0 else -1.0)
            des_roll = clamp(des_roll + SPRINT_KP_COMMIT_SIDE * side,
                             -SPRINT_ROLL_BLIND, SPRINT_ROLL_BLIND)
        up = clamp(-offy - SPRINT_COMMIT_UP_OFFY, 0.0, 0.5)
        thrust = SPRINT_HOVER + trim + SPRINT_KP_COMMIT_UP * up - SPRINT_KD_VZ_COMMIT * vz_est
    else:
        # SERVO (far) or ALIGN (near but off-axis - stop the creep, keep sliding onto the axis).
        # YAW: COARSE FoV keeping only - nudge the heading ONCE PER FRAME when the aim point has
        # drifted well off centre; frozen otherwise AND frozen when NEAR (the ballooning aim would
        # whip it). At creep speed yaw doesn't change the direction of travel (a quad translates by
        # TILTING - the nose is a camera mount). ROLL owns the fine lateral work.
        if (fresh and not align_mode and area < SPRINT_ALIGN_AREA
                and SPRINT_YAW_ON < abs(offx) < SPRINT_YAW_MAX_OFF):
            heading += clamp(SPRINT_KP_YAW * offx, -SPRINT_YAW_STEP_MAX, SPRINT_YAW_STEP_MAX)
            data["_sp_heading"] = heading
            data["_sp_yawed"] = True    # panning moves the aim - skip the next lateral-rate sample
        # ROLL: the lateral translation - bank toward the aim point, DAMPED on the DEAD-RECKONED
        # lateral velocity (offx -> roll is a double integrator; P-only is v1's teeter-totter).
        # The vision aim-rate damper is RETIRED (run 220336): its samples are discarded on any
        # fast vertical aim motion, which in FLOW is ALWAYS - so at transit speed the roll flew
        # undamped-P at the blind cap, built ~1.3 m/s sideways chasing the launch phantom, and
        # hit gate 1's right post. v_lat is physics from our own command - never blind, no cap
        # downgrade needed, and it also bounds a phantom chase: equilibrium drift is
        # KP_LAT*|offx|/KD_VLAT (~0.3 m/s for the 0.13 launch parallax) instead of runaway.
        des_roll = clamp(SPRINT_KP_LAT * offx - SPRINT_KD_VLAT * v_lat,
                         -SPRINT_ROLL_MAX, SPRINT_ROLL_MAX)
        # THRUST: hold the aim point on the fly-at-gate-height row. A gate at drone height sits
        # tan(uptilt - pitch) below the optical axis - uptilt is spec, pitch is OUR OWN command
        # (the sim holds it at gain 1.0), so the row needs no attitude estimate at all.
        # The INTEGRAL TRIM learns the true hover thrust (it drifts: 0.299 sysid, 0.26 in run
        # 092333) - without it the P-loop parks in equilibrium ABOVE the row by err = droop/KP.
        # CRUISE HIGH on long transits, descend late: hold a FAR gate lower in frame (= fly ~1 m
        # above the gate line), fading to the normal row by approach range. The detector cannot see
        # the floor-parked obstacles (fighters, stands) - run 121021 sagged onto a fighter's wing
        # mid-transit - but nothing on this course is above the transit line. Fly over the unseen.
        cruise_up = SPRINT_CRUISE_UP * clamp(1.0 - area / SPRINT_CRUISE_FADE_AREA, 0.0, 1.0)
        # ROW-TARGET PITCH IS DELAYED to match what the camera actually saw (attitude lag ~96 ms
        # + vision latency ~150 ms). Computing the target from the INSTANTANEOUS command made it
        # LEAD the stale measurement by ~0.3 s at every pitch transient: run 215329's brake
        # onsets show err spiking +0.29/-0.31 in phase with the pitch swing, whipping thrust
        # 0.22<->0.31 and vz to +1.1 m/s - the "balloon" that climbed the drone a full metre in
        # front of gate 2 was the row servo chasing a phantom row error, not physics.
        oyp = data.get("_sp_oy_pitch", 0.0)
        oyp += (pitch_prev - oyp) * clamp(dt / SPRINT_ROW_TGT_LAG_S, 0.0, 1.0)
        data["_sp_oy_pitch"] = oyp
        # BRAKE LOW (Brian's law, run 215329): do the shedding BELOW the gate line, so whatever
        # vertical disturbance braking still causes climbs INTO the line instead of through it
        # into the top bar. The standoff is proportional to the speed still to shed and fades to
        # zero as v_est reaches v_land - the final metres before commit fly the true line.
        brake_low = 0.0
        if data.get("_sp_trans") == "FLOW":
            brake_low = SPRINT_BRAKE_LOW_OY * clamp(
                (v_est - v_land) / max(SPRINT_V_CRUISE - SPRINT_V_LAND_MIN, 0.1), 0.0, 1.0)
        oy_tgt = (math.tan(UPTILT_RAD - oyp) / HALF_TAN_Y + SPRINT_OFFY_BIAS
                  + cruise_up - brake_low)
        err = oy_tgt - offy
        # ROW-CAPTURE LATCH: arm on SUSTAINED small row error (a transient crossing of the sweeping
        # launch aim doesn't count - that's how FLOW opened mid-climb in run 205713). Until latched,
        # the roll above is overridden to zero and FLOW stays closed: the drone spawns aligned, and
        # the aim_ox read during the ascent is viewpoint parallax, not displacement.
        if not data.get("_sp_row_cap_latch"):
            n = data.get("_sp_row_cap_n", 0) + 1 if abs(err) < SPRINT_ROW_CAP_ERR else 0
            data["_sp_row_cap_n"] = n
            if n >= SPRINT_ROW_CAP_TICKS:
                data["_sp_row_cap_latch"] = True
            des_roll = 0.0
        if data.get("_sp_lock_frames", 0) < SPRINT_LOCK_CONFIRM:
            # UNCONFIRMED lock: no vertical authority. A junk blob acquired right after a pass
            # demanded a huge row correction and sprint-climbed the drone to 3 m in the half-second
            # before it died (run 115646). Hold altitude until the target survives a few frames.
            thrust = SPRINT_HOVER + trim - SPRINT_KD_VZ_COMMIT * vz_est
        else:
            # Trim learns ONLY near vertical equilibrium: a persistent row error while climbing or
            # descending is approach GEOMETRY, not hover bias - the ungated trim wound down to
            # -0.048 during the gate-3 approach and dragged a sink through the commit (run 094047).
            # F1 REGRESSION LESSON (run 184505): v1 shipped with trim learning gated to CREEP
            # only, and the flight spent ~70% of its time in transit - trim sat at +0.001 all
            # flight while steady's qual lap had learned -0.034. Steady's |vz| equilibrium gate
            # was the proven mechanism all along - it applies at any lean the FLOW profile flies.
            if abs(vz_est) < SPRINT_TRIM_VZ_GATE:
                trim = clamp(trim + SPRINT_KI_VERT * err * dt, -SPRINT_TRIM_MAX, SPRINT_TRIM_MAX)
                data["_sp_thr_trim"] = trim
            # Post-pass GENTLE-DESCENT window: a wrong mid-corner lock must not spend altitude
            # before preemption corrects it (the descent-for-the-wrong-gate at the tilted corner).
            dn, up = SPRINT_THRUST_DN, SPRINT_THRUST_UP
            _pp = data.get("_sp_pass_us")
            if (_pp is not None and t_us_now is not None
                    and (t_us_now - _pp) * 1e-6 < SPRINT_POSTPASS_GENTLE_S):
                dn, up = SPRINT_POSTPASS_DN, SPRINT_POSTPASS_UP
            # CASCADED vertical: row error -> BOUNDED climb-rate target -> thrust servos the rate.
            # The old direct law (KP_VERT*err - KD_VZ*vz, KD 0.03) had an implied equilibrium
            # climb of ~5.9 m/s per unit of row error - structurally a runaway: run 221609 chased
            # an err of +1.11 to vz +2.9 m/s with thrust pinned at the band top, then slammed to
            # 0.076 to arrest. The cascade caps the climb at VZ_MAX no matter how big the row
            # step is, and the arrest is built in (vz_tgt -> 0 as the row closes).
            vz_tgt = clamp(SPRINT_VZ_PER_ERR * err, -SPRINT_VZ_MAX, SPRINT_VZ_MAX)
            thrust = clamp(SPRINT_HOVER + trim + SPRINT_KP_VZ * (vz_tgt - vz_est),
                           SPRINT_HOVER + trim - dn, SPRINT_HOVER + trim + up)
        # PITCH: creep forward, faded while off-aim (turn first, then close). In ALIGN (near but
        # off-axis) it BRAKES - a gentle backlean - because zeroing the command doesn't stop the
        # momentum already carried (run 111346 coasted into the gate throat mid-alignment and
        # clipped the top bar). Big ROW error (CLIMB-FIRST) still just zeroes: climb in place.
        aim = clamp(1.0 - abs(offx) / SPRINT_AIM_REF, 0.0, 1.0)
        climb_hold = False
        if align_mode:
            # Misaligned near the gate - the ONE remaining place that slows down before a gate.
            # Hot arrivals get the capped backpressure (steady's 2-deg brake was sized for creep
            # momentum, not 2-3 m/s); settled ones get steady's proven gentle brake.
            des_pitch = (-SPRINT_ALIGN_BRAKE if v_est <= SPRINT_V_SETTLED
                         else _flow_pitch(0.0, v_est))
        elif abs(err) > SPRINT_CLIMB_GATE:
            # CLIMB-FIRST - and near a gate, HOLD STATION while climbing (hover_pilot's range-hold):
            # pitch-zero let residual drift carry the climb into the top bar (run 155544), and a
            # CONSTANT back-pitch kept accelerating backward over the multi-second climb (backed
            # away, clipped the top, no pass). A range servo brakes exactly as much as needed.
            if area >= SPRINT_ALIGN_AREA:
                climb_hold = True
                rng = 1.0 / math.sqrt(max(area, 1e-9))
                hold = data.get("_sp_climb_rng")
                if hold is None:
                    hold = rng
                    data["_sp_climb_rng"] = rng
                des_pitch = clamp(SPRINT_HOLD_KP * (rng - hold),
                                  -2.0 * SPRINT_ALIGN_BRAKE, 0.6 * SPRINT_PITCH_FWD)
            else:
                des_pitch = 0.0
        else:
            des_pitch = SPRINT_PITCH_FWD * aim
        # ---- THE TRANSIT LAYER: a CONTINUOUS speed profile (the discrete brake is RETIRED -
        # five flights of counter-bang slams blinding the roll damping was the verdict). The
        # target speed tapers linearly with range on the fragmentation-proof trigger area; pitch
        # is a bounded servo around the drag equilibrium; the backlean cap (3.4 deg) cannot slam
        # and cannot trip the OYRATE gate, so roll damping stays alive through the whole approach.
        # Everything above this block is steady_pilot verbatim - in CREEP the block is a no-op,
        # and the normal row servo runs through the ENTIRE approach (the active_gate_index audit
        # showed a frozen vertical is what kept the drone off the gate line).
        trans = data.get("_sp_trans", "CREEP")
        if trans == "FLOW":
            # Losing the lock, the aim, or the frame is not an emergency any more - the target
            # speed drops to zero and the capped backlean bleeds it down; no state slams.
            soft_ok = (data.get("_sp_stale", 0) < SPRINT_STALE_ABORT
                       and data.get("_sp_lock_frames", 0) >= SPRINT_MIN_FRAMES
                       and abs(offx) <= SPRINT_ABORT_OFFX)
            # CONSTANT-DECEL profile: v_tgt = sqrt(v_land^2 + 2*A_BRAKE*room). The linear taper is
            # RETIRED (run 210832): a slope in m/s-per-m demands its HARDEST decel at the HIGHEST
            # speed and eases off as the gate nears - backwards - and its onset at 4.5 m/s was
            # 10.6 m out, swallowing whole transits. Physics braking starts later, sheds harder,
            # and lands at v_land exactly at R_LAND.
            v_tgt = clamp(math.sqrt(v_land * v_land + 2.0 * SPRINT_A_BRAKE
                                    * max(rng_trig - SPRINT_R_LAND, 0.0)),
                          0.0, SPRINT_V_CRUISE) if soft_ok else 0.0
            braking = soft_ok and v_tgt < SPRINT_V_CRUISE and v_est > v_tgt
            des_pitch = _flow_pitch(v_tgt, v_est, SPRINT_A_BRAKE if braking else 0.0)
            climb_hold = False       # the profile owns pitch while hot
            # FLOW runs all the way to the commit latch (the commit branch holds V_PASS from
            # there). It only hands back to CREEP after a LOSS has been bled down to settled.
            if not soft_ok and v_est <= SPRINT_V_SETTLED:
                trans = "CREEP"
        elif (not align_mode and not climb_hold
                and data.get("_sp_row_cap_latch")
                and area < SPRINT_ALIGN_AREA and abs(err) <= SPRINT_ENTRY_ROW
                and abs(offx) <= SPRINT_ENTRY_OFFX
                and data.get("_sp_stale", 0) == 0
                and data.get("_sp_lock_frames", 0) >= SPRINT_MIN_FRAMES):
            trans = "FLOW"           # confirmed lock, commit-grade aim, row CAPTURED - open up
        data["_sp_trans"] = trans
        if not climb_hold:
            data["_sp_climb_rng"] = None
    # Tilt feed-forward, transit-only: a lean scales the vertical thrust component by cos(pitch);
    # top the collective up so the transit line doesn't sag (the fighter-wing scar, run 121021,
    # lives exactly on the transit line). Tiny (~0.006 at the F1 lean) but free.
    thr_ff = 0.0
    if data.get("_sp_trans", "CREEP") == "FLOW":
        thr_ff = min((SPRINT_HOVER + trim) * (1.0 / max(math.cos(pitch_prev), 0.5) - 1.0),
                     SPRINT_THR_FF_MAX)
        thrust += thr_ff
    thrust = clamp(thrust, MIN_THRUST, MAX_THRUST)

    # Slew pitch and roll so every correction eases in and eases back out (and the row target moves
    # smoothly with the pitch). The transit layer gets its own faster pitch slew - the bang AND the
    # counter must develop in ~0.3 s, and steady's creep-rate slew would spend a full second (a 4 m
    # coast) just reaching the brake lean. The fast rate also holds until the command is back inside
    # the creep envelope, so a finished brake doesn't limp back to level at 0.15 rad/s while the
    # backlean keeps decelerating into reverse.
    p_slew = (SPRINT_PITCH_SLEW_T
              if (data.get("_sp_trans", "CREEP") == "FLOW"
                  or abs(pitch_prev) > 1.5 * SPRINT_PITCH_FWD)
              else SPRINT_PITCH_SLEW)
    pitch_cmd = pitch_prev + clamp(des_pitch - pitch_prev, -p_slew * dt, p_slew * dt)
    data["_sp_pitch_cmd"] = pitch_cmd
    roll_prev = data.get("_sp_roll_cmd", 0.0)
    roll_cmd = roll_prev + clamp(des_roll - roll_prev, -SPRINT_ROLL_SLEW * dt, SPRINT_ROLL_SLEW * dt)
    data["_sp_roll_cmd"] = roll_cmd

    # 4) SEND: attitude setpoint - the sim's stabilised controller does the balancing.
    send_attitude_setpoint(mavlink_conn, system_boot_ms, roll_cmd, pitch_cmd, heading, thrust)

    # 5) readout + log (every tick)
    n = len(gates)
    _blind = not gates or stale
    _seeking = (_blind and data.get("_sp_pass_us") is not None and t_us_now is not None
                and (t_us_now - data["_sp_pass_us"]) * 1e-6 < SPRINT_SEEK_S)
    _trans = data.get("_sp_trans", "CREEP")
    data["sprint_regime"] = ("COMMIT" if commit else
                             "ALIGN" if align_mode else
                             "SEEK" if _seeking else
                             "HOLD" if _blind else
                             _trans if _trans != "CREEP" else "CREEP")
    data["oracle_thrust"] = thrust
    data["_sp_des_roll"], data["_sp_des_pitch"], data["_sp_thrust"] = roll_cmd, pitch_cmd, thrust
    data["_sp_ndets"] = n
    data["_sp_az"], data["_sp_el"] = offx, offy
    data["_sp_climb"] = vz_est
    data["_sp_yaw"] = heading
    data["_sp_commit"] = 1.0 if commit else 0.0
    data["_sp_rng_est"], data["_sp_d_stop"] = rng_est, d_stop   # for the replay panel

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
              data.get("_sp_trans", "CREEP"), f"{v_est:+.2f}",
              ("" if v_meas is None else f"{v_meas:.2f}"),
              f"{rng_est:.1f}", f"{d_stop:.1f}", f"{thr_ff:.3f}", f"{rng_trig:.1f}",
              f"{v_land:.2f}", f"{v_lat:+.2f}"])
