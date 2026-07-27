"""
Attack pilot v2 - an attitude-setpoint visual servo. The sim balances; we steer.

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
from pilots.attack_pilot.config import *   # ATK_* params


_DBG_W = _DBG_F = None
_DBG_N = 0
# EVERY control tick (~90 Hz): what was measured, what the law decided, what was sent.
_DBG_HDR = ["t", "frame", "fresh", "gates_passed", "n_dets", "area", "aim_ox", "aim_oy", "oy_tgt",
            "yaw_deg", "pitch_deg", "roll_cmd_deg", "ox_rate", "commit", "stale", "vz_est", "alt_est",
            "roll_meas_deg", "pitch_meas_deg", "trim", "thr", "next_head_deg", "seek_dir", "next_doy"]


def _dbg_log(row):
    global _DBG_W, _DBG_F, _DBG_N
    if _DBG_W is None:
        path = os.path.join(DATASETS_DIR, time.strftime("attack_dbg_%Y%m%d_%H%M%S.csv"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _DBG_F = open(path, "w", newline="")
        _DBG_W = csv.writer(_DBG_F)
        _DBG_W.writerow(_DBG_HDR)
        print(f"attack debug -> {path}", flush=True)
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
        return data.get("_at_est_roll", 0.0), data.get("_at_est_pitch", 0.0)
    ax, ay, az = imu.get("xacc", 0.0), imu.get("yacc", 0.0), imu.get("zacc", 0.0)
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    valid = abs(mag - 9.81) <= 2.5
    roll_a = math.atan2(ay, -az)
    pitch_a = math.atan2(-ax, math.hypot(ay, az))
    prev = data.get("_at_att_us")
    roll = data.get("_at_est_roll", 0.0)
    pitch = data.get("_at_est_pitch", 0.0)
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
    data["_at_est_roll"], data["_at_est_pitch"], data["_at_att_us"] = roll, pitch, t_us
    return roll, pitch


def _vz_alt_estimate(data, m_roll, m_pitch):
    """Leaky-integrated vertical velocity (m/s, up+) and altitude-above-start (m) from the IMU
    specific force rotated through the measured attitude. Validated against the recorded ceiling
    flight (commanded +1.5, actual +4.0 - it caught it). Used for mild thrust damping + logging."""
    imu = data.get("highres_imu") or {}
    t_us = imu.get("time_usec")
    if t_us is None:
        return data.get("_at_vz", 0.0)
    prev = data.get("_at_vz_us")
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
                vz = (data.get("_at_vz", 0.0) + a_up * dt) * math.exp(-dt / ATK_VZ_TAU)
                data["_at_vz"] = clamp(vz, -3.0, 3.0)
                data["_at_alt"] = data.get("_at_alt", 0.0) + data["_at_vz"] * dt
    data["_at_vz_us"] = t_us
    return data.get("_at_vz", 0.0)


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
    data["_at_off"] = (0.0, 0.0)
    data["_at_off_smooth"] = None
    data["_at_area"] = 0.0
    data["_at_track_off"] = None
    data["_at_track_miss"] = 0
    data["_at_fid"] = None
    data["_at_gate_count"] = 0
    data["_at_pass_armed"] = False
    data["_at_pass_peak"] = 0.0
    data["_at_pass_block"] = False
    data["_at_commit_latch"] = False
    data["_at_avoid_off"] = None
    data["_at_stale"] = 0
    data["_at_t0_us"] = None
    data["_at_heading"] = ATK_YAW_SPAWN   # integrated ABSOLUTE yaw command (rad). The setpoint yaw is
                                          # a WORLD direction: seeding 0 snapped the drone ~97 deg off
                                          # the course at race start (run 20260727_000330). Seed with
                                          # the measured spawn facing so tick one means "hold still".
    data["_at_pitch_cmd"] = 0.0        # slewed pitch command (rad)
    data["_at_roll_cmd"] = 0.0         # slewed roll command (rad)
    data["_at_climb_rng"] = None       # range anchor for the near-gate climb station-hold
    data["_at_thr_trim"] = 0.0         # learned hover-thrust correction (integral trim)
    data["_at_ox_rate"] = 0.0          # aim-point lateral rate (units/s) - damps the roll strafe
    data["_at_ox_blind"] = True        # no valid lateral-rate sample yet -> roll capped gentle
    data["_at_prev_ox"] = None
    data["_at_prev_oy"] = None
    data["_at_prev_ox_us"] = None
    data["_at_yawed"] = False          # a yaw nudge moved the aim last frame -> skip one rate sample
    data["_at_next_head"] = None       # remembered ABSOLUTE heading of the next gate (rad)
    data["_at_next_doy"] = None        # its elevation RELATIVE to the then-tracked gate (offset_y)
    data["_at_next_area_mem"] = 0.0    # its size when sighted
    data["_at_next_us"] = None
    data["_at_next_area"] = 0.0        # decaying area bar - biggest credible sighting wins
    data["_at_pass_us"] = None         # when the last pass counted (starts the SEEK window)
    data["_at_pass_head"] = 0.0
    data["_at_pass_alt"] = 0.0         # alt_est when the last pass counted (= the gate line)
    data["_at_seek_dir"] = 1.0         # which way SEEK rotates (signed; from the memory)
    data["_at_lock_frames"] = 0        # consecutive frames on the SAME lock (maturity gate)
    data["_at_slew_us"] = None
    data["_at_est_roll"] = 0.0
    data["_at_est_pitch"] = 0.0
    data["_at_att_us"] = None
    data["_at_vz"] = 0.0
    data["_at_alt"] = 0.0
    data["_at_vz_us"] = None
    data["_at_live"] = False


def _idle(mavlink_conn, system_boot_ms, data, regime):
    data["attack_regime"] = regime
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
    avoid = data.get("_at_avoid_off")
    # The avoid lock EXPIRES: its whole job is the 1-2 s post-pass handoff.
    _us = (data.get("highres_imu") or {}).get("time_usec")
    p_us = data.get("_at_pass_us")
    if avoid is not None and _us is not None and p_us is not None \
            and (_us - p_us) * 1e-6 > ATK_AVOID_S:
        avoid = None
        data["_at_avoid_off"] = None
    if avoid is not None:
        ax, ay = avoid
        near = min(gates, key=lambda d: math.hypot(d.offset_x - ax, d.offset_y - ay))
        # Follow the avoided gate ONLY while the nearby blob is still BIG - the genuinely
        # just-passed gate filled the frame moments ago. A SMALL blob near the stale position is a
        # DIFFERENT gate that drifted in (run 110222: the next gate inherited the avoid lock and
        # the tracker flew at a 22 m speck instead) - drop the avoid, don't adopt it.
        if (math.hypot(near.offset_x - ax, near.offset_y - ay) <= ATK_AVOID_RADIUS
                and near.area_frac >= ATK_AVOID_MIN_AREA):
            data["_at_avoid_off"] = (near.offset_x, near.offset_y)
        else:
            avoid = None
            data["_at_avoid_off"] = None

    def usable():
        if avoid is None:
            return gates
        ax, ay = avoid
        return [d for d in gates if math.hypot(d.offset_x - ax, d.offset_y - ay) > ATK_AVOID_RADIUS]

    in_pp = (p_us is not None and _us is not None and (_us - p_us) * 1e-6 < ATK_SEEK_S)

    def _sane(ds):
        """Post-pass sky-blob exclusion: a candidate ~50deg+ above the horizon cannot be a course
        gate - only the just-passed gate's top bar / ceiling junk (one fired a full-climb spike at
        the gate plane, run 164646). Fail-open: yields if it would empty the list."""
        if not in_pp:
            return ds
        ok = [d for d in ds if d.offset_y > ATK_JUNK_OY]
        return ok or ds

    prev = data.get("_at_track_off")
    if prev is None:
        data["_at_track_miss"] = 0
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
    if math.hypot(g.offset_x - px, g.offset_y - py) <= ATK_TRACK_RADIUS:
        data["_at_track_miss"] = 0
        # YOUNG-LOCK PREEMPTION (post-pass window only, fail-open): the corner sweep meets gates in
        # the wrong order - the far next-next gate enters frame first and gets locked; the TRUE next
        # gate is ~half its distance, so ~4x bigger the moment the sweep reaches it. A much-bigger
        # candidate steals the lock; nothing is ever refused.
        if in_pp:
            best = _acquire(_sane(cands))
            if best is not g and best.area_frac >= ATK_PREEMPT_RATIO * max(g.area_frac, 1e-6):
                return best
        return g
    miss = data.get("_at_track_miss", 0) + 1
    data["_at_track_miss"] = miss
    if miss >= ATK_TRACK_MAX_MISS:
        data["_at_track_miss"] = 0
        return _acquire(cands)
    return None


def update_attack_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    if not _race_live(data):
        t_go = _seconds_to_go(data)
        _idle(mavlink_conn, system_boot_ms, data,
              f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0) else "IDLE")
        return

    if not data.get("_at_live"):
        _reset(data)
        data["_at_live"] = True

    # Measured attitude + vertical estimate: LOGGING and mild thrust damping only.
    m_roll, m_pitch = _imu_attitude(data)
    vz_est = _vz_alt_estimate(data, m_roll, m_pitch)

    # 1) TRACK one gate; smooth the AIM offsets (the opening, when located).
    fresh = False
    fid = data.get("latest_frame_id")
    gates = [d for d in (data.get("vision_gates") or []) if d.area_frac >= ATK_MIN_AREA]
    if fid is not None and fid != data.get("_at_fid") and gates:
        data["_at_fid"] = fid
        had_lock = data.get("_at_track_off") is not None
        g = _select_target(gates, data)
        if g is None:
            data["_at_stale"] = data.get("_at_stale", 0) + 1
        else:
            ax, ay = _aim_offsets(g)
            sm = data.get("_at_off_smooth")
            hop = sm is None or math.hypot(ax - sm[0], ay - sm[1]) > ATK_HOP_DIST
            if hop:
                sx, sy = ax, ay
            else:
                sx = ATK_OFF_ALPHA * ax + (1 - ATK_OFF_ALPHA) * sm[0]
                sy = ATK_OFF_ALPHA * ay + (1 - ATK_OFF_ALPHA) * sm[1]
            # AIM LATERAL RATE (units/s), frame-to-frame - damps the roll strafe. Discarded on a hop
            # (fake motion), on the frame after a YAW nudge (panning moves the aim with zero actual
            # translation), and during fast VERTICAL aim motion (climb/descent parallax pollutes the
            # x-rate: the takeoff climb read +0.30 and banked 5.7 deg with the gate dead ahead -
            # run 094047, t=0.1-0.5). Clamped to real strafe speeds (< 0.1 in flight).
            _us_f = (data.get("highres_imu") or {}).get("time_usec")
            p_ox, p_oy, p_us_f = data.get("_at_prev_ox"), data.get("_at_prev_oy"), data.get("_at_prev_ox_us")
            if hop or data.get("_at_yawed"):
                data["_at_ox_rate"] = 0.0
                data["_at_ox_blind"] = True     # no valid lateral rate -> roll flies capped-gentle
            elif p_ox is not None and p_us_f is not None and _us_f is not None:
                _dtf = (_us_f - p_us_f) * 1e-6
                if 0.0 < _dtf <= 0.2:
                    oy_rate = (sy - p_oy) / _dtf if p_oy is not None else 0.0
                    if abs(oy_rate) > ATK_OYRATE_GATE:
                        data["_at_ox_rate"] = 0.0
                        data["_at_ox_blind"] = True
                    else:
                        # DE-PAN with the measured gyro: yaw motion sweeps the aim at zgyro*(1+offx^2)
                        # (offx is tan-normalized; sim zgyro is CCW-positive - calibration flights)
                        # with ZERO translation. The one-frame post-nudge skip can't cover the sim's
                        # ~150 ms yaw lag: run 114407 read ox_rate pinned +0.30 through an approach
                        # pan and the damping shoved the roll AWAY from the gate - left-edge hit.
                        zg = (data.get("highres_imu") or {}).get("zgyro", 0.0)
                        raw = (sx - p_ox) / _dtf - zg * (1.0 + sx * sx)
                        data["_at_ox_rate"] = clamp(raw, -ATK_OXRATE_MAX, ATK_OXRATE_MAX)
                        data["_at_ox_blind"] = False
            if _us_f is not None:
                data["_at_prev_ox"], data["_at_prev_oy"], data["_at_prev_ox_us"] = sx, sy, _us_f
            data["_at_yawed"] = False
            data["_at_off_smooth"] = (sx, sy)
            data["_at_off"] = (sx, sy)
            data["_at_track_off"] = (g.offset_x, g.offset_y)   # continuity in RING space
            data["_at_area"] = g.area_frac
            data["_at_stale"] = 0
            fresh = True
            # Lock maturity: consecutive matched frames on the SAME target. A fresh acquisition (or
            # a hop) restarts it - and an unconfirmed lock gets no vertical authority (a junk blob
            # acquired right after the pitched-gate pass sprint-climbed the drone to 3 m in half a
            # second before dying, run 115646).
            data["_at_lock_frames"] = (data.get("_at_lock_frames", 0) + 1) if (had_lock and not hop) else 1
            # NEXT-GATE MEMORY: note the ABSOLUTE heading of the biggest OTHER detection (excluding
            # the just-passed avoid gate). At a sharp corner the next gate leaves the FoV before the
            # pass finishes - this remembered bearing tells SEEK which way to turn, signed, so left
            # and right corners are the same code. offset_x -> angle via atan(ox * HALF_TAN_X);
            # heading frame is CCW-positive, so a target to the RIGHT is a SMALLER heading.
            av = data.get("_at_avoid_off")
            others = [d for d in gates if d is not g and d.area_frac >= ATK_NEXT_MIN_AREA
                      and (av is None or math.hypot(d.offset_x - av[0], d.offset_y - av[1]) > ATK_AVOID_RADIUS)]
            # BIGGEST credible sighting wins, not the LATEST: the memory holds a decaying area bar
            # (~4 s window) a new candidate must beat. Run 123753: gate 4 was seen at the frame edge
            # (area 0.026, LEFT) during the corner handoff, then a 0.007 right-side speck was seen
            # one frame later and overwrote the memory - SEEK pirouetted ~180 deg the WRONG way.
            bar = data.get("_at_next_area", 0.0) * 0.99
            data["_at_next_area"] = bar
            # RECORDING HYGIENE - the memory only accepts sightings from CLEAN frames:
            #  * not within 1.5 s AFTER a pass (receding-gate fragments - run 124505's wrong-way
            #    orbit), and
            #  * not while the tracked gate is NEAR (area >= align range): at point-blank the
            #    current gate splits into huge pillar fragments that pose as "others" and poisoned
            #    the memory with a 0.3-area straight-ahead "next gate" - whose remembered size then
            #    vetoed every real candidate after the pass (run 130xxx: never tracked gate 2).
            _pus = data.get("_at_pass_us")
            _nus = (data.get("highres_imu") or {}).get("time_usec")
            settled = (_pus is None or _nus is None
                       or (_nus - _pus) * 1e-6 > ATK_NEXT_HOLDOFF)
            settled = settled and g.area_frac < ATK_ALIGN_AREA
            if others and settled:
                nd = max(others, key=lambda d: d.area)
                if nd.area_frac >= bar:
                    data["_at_next_head"] = (data.get("_at_heading", 0.0)
                                             - math.atan(nd.offset_x * HALF_TAN_X))
                    # the SIGHTING TRACK: bearing + elevation RELATIVE to the tracked gate (survives
                    # whatever our attitude/altitude do during the pass) + size
                    data["_at_next_doy"] = nd.offset_y - g.offset_y
                    data["_at_next_area_mem"] = nd.area_frac
                    data["_at_next_us"] = (data.get("highres_imu") or {}).get("time_usec")
                    data["_at_next_area"] = nd.area_frac

    area = data.get("_at_area", 0.0)
    offx, offy = data.get("_at_off", (0.0, 0.0))

    # Regime predicates. COMMIT requires proximity AND alignment (run 095624 committed on area
    # alone, coasted wide right into the gate edge). ALIGN-mode (brake + strafe, no creep) engages
    # EARLY when misaligned - from ATK_ALIGN_AREA, not commit range - because braking at 2 m with
    # 0.3 of offset just scrapes down the pillar (run 113145). Aligned approaches skip the band.
    stale = data.get("_at_stale", 0) >= ATK_MAX_STALE
    tracking = bool(gates) and not stale
    aligned_x = abs(offx) <= ATK_COMMIT_ALIGN
    # Vertical readiness (run 122735: committed mid-climb at aim_oy -0.36 / vz +2.97 and sagged
    # into the bottom bar). Blocked-vertical routes through the SERVO branch, whose row servo +
    # climb-first pitch already do exactly the right thing: finish the climb, then commit.
    row_tgt = math.tan(UPTILT_RAD - data.get("_at_pitch_cmd", 0.0)) / HALF_TAN_Y + ATK_OFFY_BIAS
    aligned_y = abs(offy - row_tgt) <= ATK_COMMIT_ROW
    # COMMIT LATCHES. Alignment (both axes) is an ENTRY condition only - at point-blank range the
    # aim balloons away from the far-field row target BY GEOMETRY, and re-checking it every tick
    # un-committed the pilot INSIDE the tilted gate (run 123331, t=67.7): the row servo woke mid-
    # pass, firewalled a +2.2 m/s climb in the gate throat, and the pass never counted. Enter on
    # readiness; release only on pass handoff, lost tracking, or the gate genuinely receding.
    vz_ok = abs(vz_est) <= ATK_COMMIT_VZ
    if tracking and area >= ATK_COMMIT_AREA and aligned_x and aligned_y and vz_ok:
        data["_at_commit_latch"] = True
    if data.get("_at_commit_latch") and (not tracking or area < ATK_PASS_REARM):
        data["_at_commit_latch"] = False
    commit = bool(data.get("_at_commit_latch"))
    # ALIGN also engages when the VERTICAL SPEED is too hot to commit (descending off the cruise
    # glide, run 132724): the brake holds station while the arrest settles vz, then commit - level.
    align_mode = (tracking and not commit and area >= ATK_ALIGN_AREA
                  and (not aligned_x or not vz_ok))

    # 2) PASS = area peaked then receded -> count it and hand off to the next gate. Arms ONLY while
    # COMMITTED: area receding from a lateral scrape is not a pass (run 095624 counted 6 "gates").
    if data.get("_at_pass_block") and area < ATK_PASS_REARM:
        data["_at_pass_block"] = False
    if commit and area >= ATK_PASS_AREA and not data.get("_at_pass_block"):
        data["_at_pass_armed"] = True
        data["_at_pass_peak"] = max(data.get("_at_pass_peak", 0.0), area)
    if data.get("_at_pass_armed") and area < ATK_PASS_DROP * data.get("_at_pass_peak", 0.0):
        data["_at_gate_count"] = data.get("_at_gate_count", 0) + 1
        data["_at_pass_armed"] = False
        data["_at_pass_peak"] = 0.0
        data["_at_pass_block"] = True
        data["_at_commit_latch"] = False   # pass done - release the commit latch for the next gate
        data["_at_avoid_off"] = data.get("_at_track_off")
        data["_at_track_off"] = None
        data["_at_off_smooth"] = None
        # Start the SEEK window: stamp the pass, and pick the rotation direction from the next-gate
        # memory (fresh -> signed bearing; stale -> its sign is still the best guess; none -> CCW).
        _us_p = (data.get("highres_imu") or {}).get("time_usec")
        head_now = data.get("_at_heading", 0.0)
        data["_at_pass_us"] = _us_p
        data["_at_pass_head"] = head_now
        data["_at_pass_alt"] = data.get("_at_alt", 0.0)   # we just flew THROUGH a gate: this IS the
                                                          # gate line, drift and all - SEEK's reference
        nh = data.get("_at_next_head")
        if nh is not None and abs(nh - head_now) > 0.05:
            data["_at_seek_dir"] = 1.0 if nh > head_now else -1.0
        data["_at_next_area"] = 0.0   # fresh memory window for the gate AFTER the one just acquired

    # 3) CONTROL - three regimes: HOLD (blind), COMMIT (through the gate), SERVO (normal).
    t_us_now = (data.get("highres_imu") or {}).get("time_usec")
    p_us = data.get("_at_slew_us")
    dt = (t_us_now - p_us) * 1e-6 if (t_us_now is not None and p_us is not None) else 1.0 / CONTROL_HZ
    if not (0.0 < dt <= 0.05):
        dt = 1.0 / CONTROL_HZ
    if t_us_now is not None:
        data["_at_slew_us"] = t_us_now

    heading = data.get("_at_heading", 0.0)
    pitch_prev = data.get("_at_pitch_cmd", 0.0)
    trim = data.get("_at_thr_trim", 0.0)
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
        p_us_pass = data.get("_at_pass_us")
        if (p_us_pass is not None and t_us_now is not None
                and (t_us_now - p_us_pass) * 1e-6 < ATK_SEEK_S):
            heading += data.get("_at_seek_dir", 1.0) * ATK_SEEK_RATE * dt
            data["_at_heading"] = heading
            if abs(heading - data.get("_at_pass_head", heading)) > 0.5:
                data["_at_avoid_off"] = None
            # Sink back to the gate line while seeking: from above it, the NEAR next gate sits below
            # the camera's 9-deg down-limit and a FAR one steals the lock (the run-115646 miss).
            if data.get("_at_alt", 0.0) > data.get("_at_pass_alt", 0.0) + ATK_SEEK_ALT_TOL:
                v_tgt = -ATK_SEEK_SINK
        thrust = ATK_HOVER + trim - ATK_KD_VZ_COMMIT * (vz_est - v_tgt)
    elif commit:
        # COMMIT: the aim point balloons this close - stop chasing it. Freeze the heading, wings
        # LEVEL, keep the creep on, let the pass logic call it. Thrust holds hover WITH a strong
        # vz-arrest: any residual climb/sink from the approach coasts into a gate bar otherwise
        # (gate 1's top bar, run 101632). ONE escape clause: if the hole climbs far above image
        # centre mid-coast (the TILTED gate - its passage line is higher than the level line, and
        # the frozen coast clipped its bottom bar, run 122217), follow it up. Climb-only, and a
        # straight gate's commit never breaches the deadband.
        des_pitch = ATK_PITCH_FWD
        # Lateral escape clause (twin of the vertical one): wings-level UNLESS the aim escapes far
        # sideways mid-coast - run 130224 watched it walk to -0.41 and scraped the left pillar.
        # Gentle (blind-cap), toward the aim, zero inside the deadband normal commits never leave.
        if abs(offx) > ATK_COMMIT_SIDE_OFFX:
            side = (abs(offx) - ATK_COMMIT_SIDE_OFFX) * (1.0 if offx > 0 else -1.0)
            des_roll = clamp(ATK_KP_COMMIT_SIDE * side, -ATK_ROLL_BLIND, ATK_ROLL_BLIND)
        else:
            des_roll = 0.0
        up = clamp(-offy - ATK_COMMIT_UP_OFFY, 0.0, 0.5)
        thrust = ATK_HOVER + trim + ATK_KP_COMMIT_UP * up - ATK_KD_VZ_COMMIT * vz_est
    else:
        # SERVO (far) or ALIGN (near but off-axis - stop the creep, keep sliding onto the axis).
        # YAW: COARSE FoV keeping only - nudge the heading ONCE PER FRAME when the aim point has
        # drifted well off centre; frozen otherwise AND frozen when NEAR (the ballooning aim would
        # whip it). At creep speed yaw doesn't change the direction of travel (a quad translates by
        # TILTING - the nose is a camera mount). ROLL owns the fine lateral work.
        if fresh and not align_mode and ATK_YAW_ON < abs(offx) < ATK_YAW_MAX_OFF:
            heading += clamp(ATK_KP_YAW * offx, -ATK_YAW_STEP_MAX, ATK_YAW_STEP_MAX)
            data["_at_heading"] = heading
            data["_at_yawed"] = True    # panning moves the aim - skip the next lateral-rate sample
        # ROLL: the lateral translation, pitch-creep style - small bank toward the aim point,
        # DAMPED on the aim's measured lateral rate so it rolls back level BEFORE the centre is
        # crossed (offx -> roll is a double integrator; P-only is v1's teeter-totter). While rate
        # samples are being DISCARDED (climb transients) the cap drops to gentle: on a constant-
        # bearing pursuit the bearing doesn't move even at high lateral speed, and undamped-P at
        # full cap silently built ~3 m/s of sideways momentum during run 105441's big climb.
        roll_cap = ATK_ROLL_BLIND if data.get("_at_ox_blind") else ATK_ROLL_MAX
        des_roll = clamp(ATK_KP_LAT * offx + ATK_KD_LAT * data.get("_at_ox_rate", 0.0),
                         -roll_cap, roll_cap)
        # THRUST: hold the aim point on the fly-at-gate-height row. A gate at drone height sits
        # tan(uptilt - pitch) below the optical axis - uptilt is spec, pitch is OUR OWN command
        # (the sim holds it at gain 1.0), so the row needs no attitude estimate at all.
        # The INTEGRAL TRIM learns the true hover thrust (it drifts: 0.299 sysid, 0.26 in run
        # 092333) - without it the P-loop parks in equilibrium ABOVE the row by err = droop/KP.
        # CRUISE HIGH on long transits, descend late: hold a FAR gate lower in frame (= fly ~1 m
        # above the gate line), fading to the normal row by approach range. The detector cannot see
        # the floor-parked obstacles (fighters, stands) - run 121021 sagged onto a fighter's wing
        # mid-transit - but nothing on this course is above the transit line. Fly over the unseen.
        cruise_up = ATK_CRUISE_UP * clamp(1.0 - area / ATK_CRUISE_FADE_AREA, 0.0, 1.0)
        oy_tgt = math.tan(UPTILT_RAD - pitch_prev) / HALF_TAN_Y + ATK_OFFY_BIAS + cruise_up
        err = oy_tgt - offy
        if data.get("_at_lock_frames", 0) < ATK_LOCK_CONFIRM:
            # UNCONFIRMED lock: no vertical authority. A junk blob acquired right after a pass
            # demanded a huge row correction and sprint-climbed the drone to 3 m in the half-second
            # before it died (run 115646). Hold altitude until the target survives a few frames.
            thrust = ATK_HOVER + trim - ATK_KD_VZ_COMMIT * vz_est
        else:
            # Trim learns ONLY near vertical equilibrium: a persistent row error while climbing or
            # descending is approach GEOMETRY, not hover bias - the ungated trim wound down to
            # -0.048 during the gate-3 approach and dragged a sink through the commit (run 094047).
            if abs(vz_est) < ATK_TRIM_VZ_GATE:
                trim = clamp(trim + ATK_KI_VERT * err * dt, -ATK_TRIM_MAX, ATK_TRIM_MAX)
                data["_at_thr_trim"] = trim
            # Post-pass GENTLE-DESCENT window: a wrong mid-corner lock must not spend altitude
            # before preemption corrects it (the descent-for-the-wrong-gate at the tilted corner).
            dn, up = ATK_THRUST_DN, ATK_THRUST_UP
            _pp = data.get("_at_pass_us")
            if (_pp is not None and t_us_now is not None
                    and (t_us_now - _pp) * 1e-6 < ATK_POSTPASS_GENTLE_S):
                dn, up = ATK_POSTPASS_DN, ATK_POSTPASS_UP
            thrust = clamp(ATK_HOVER + trim + ATK_KP_VERT * err - ATK_KD_VZ * vz_est,
                           ATK_HOVER + trim - dn, ATK_HOVER + trim + up)
        # PITCH: creep forward, faded while off-aim (turn first, then close). In ALIGN (near but
        # off-axis) it BRAKES - a gentle backlean - because zeroing the command doesn't stop the
        # momentum already carried (run 111346 coasted into the gate throat mid-alignment and
        # clipped the top bar). Big ROW error (CLIMB-FIRST) still just zeroes: climb in place.
        aim = clamp(1.0 - abs(offx) / ATK_AIM_REF, 0.0, 1.0)
        climb_hold = False
        if align_mode:
            des_pitch = -ATK_ALIGN_BRAKE
        elif abs(err) > ATK_CLIMB_GATE:
            # CLIMB-FIRST - and near a gate, HOLD STATION while climbing (hover_pilot's range-hold):
            # pitch-zero let residual drift carry the climb into the top bar (run 155544), and a
            # CONSTANT back-pitch kept accelerating backward over the multi-second climb (backed
            # away, clipped the top, no pass). A range servo brakes exactly as much as needed.
            if area >= ATK_ALIGN_AREA:
                climb_hold = True
                rng = 1.0 / math.sqrt(max(area, 1e-9))
                hold = data.get("_at_climb_rng")
                if hold is None:
                    hold = rng
                    data["_at_climb_rng"] = rng
                des_pitch = clamp(ATK_HOLD_KP * (rng - hold),
                                  -2.0 * ATK_ALIGN_BRAKE, 0.6 * ATK_PITCH_FWD)
            else:
                des_pitch = 0.0
        else:
            des_pitch = ATK_PITCH_FWD * aim
        if not climb_hold:
            data["_at_climb_rng"] = None
    thrust = clamp(thrust, MIN_THRUST, MAX_THRUST)

    # Slew pitch and roll so every correction eases in and eases back out (and the row target moves
    # smoothly with the pitch).
    pitch_cmd = pitch_prev + clamp(des_pitch - pitch_prev, -ATK_PITCH_SLEW * dt, ATK_PITCH_SLEW * dt)
    data["_at_pitch_cmd"] = pitch_cmd
    roll_prev = data.get("_at_roll_cmd", 0.0)
    roll_cmd = roll_prev + clamp(des_roll - roll_prev, -ATK_ROLL_SLEW * dt, ATK_ROLL_SLEW * dt)
    data["_at_roll_cmd"] = roll_cmd

    # 4) SEND: attitude setpoint - the sim's stabilised controller does the balancing.
    send_attitude_setpoint(mavlink_conn, system_boot_ms, roll_cmd, pitch_cmd, heading, thrust)

    # 5) readout + log (every tick)
    n = len(gates)
    _blind = not gates or stale
    _seeking = (_blind and data.get("_at_pass_us") is not None and t_us_now is not None
                and (t_us_now - data["_at_pass_us"]) * 1e-6 < ATK_SEEK_S)
    data["attack_regime"] = ("COMMIT" if commit else
                             "ALIGN" if align_mode else
                             "SEEK" if _seeking else
                             "HOLD" if _blind else "ATTACK")
    data["oracle_thrust"] = thrust
    data["_at_des_roll"], data["_at_des_pitch"], data["_at_thrust"] = roll_cmd, pitch_cmd, thrust
    data["_at_ndets"] = n
    data["_at_az"], data["_at_el"] = offx, offy
    data["_at_climb"] = vz_est
    data["_at_yaw"] = heading
    data["_at_commit"] = 1.0 if commit else 0.0

    if t_us_now is not None and data.get("_at_t0_us") is None:
        data["_at_t0_us"] = t_us_now
    t_s = (t_us_now - data["_at_t0_us"]) * 1e-6 if (t_us_now is not None and data.get("_at_t0_us") is not None) else 0.0
    fseq = data.get("latest_frame_seq")
    _dbg_log([f"{t_s:.3f}", ("" if fseq is None else f"{fseq:06d}"), int(fresh),
              data.get("_at_gate_count", 0), n, f"{area:.4f}",
              f"{offx:+.3f}", f"{offy:+.3f}", f"{oy_tgt:+.3f}",
              f"{math.degrees(heading):+.1f}", f"{math.degrees(pitch_cmd):+.2f}",
              f"{math.degrees(roll_cmd):+.2f}", f"{data.get('_at_ox_rate', 0.0):+.3f}",
              int(commit), data.get("_at_stale", 0),
              f"{vz_est:+.2f}", f"{data.get('_at_alt', 0.0):+.2f}",
              f"{math.degrees(m_roll):+.1f}", f"{math.degrees(m_pitch):+.1f}",
              f"{data.get('_at_thr_trim', 0.0):+.3f}", f"{thrust:.3f}",
              ("" if data.get("_at_next_head") is None else f"{math.degrees(data['_at_next_head']):+.1f}"),
              f"{data.get('_at_seek_dir', 1.0):+.0f}",
              ("" if data.get("_at_next_doy") is None else f"{data['_at_next_doy']:+.3f}")])
