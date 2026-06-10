"""
Vision pilot: fly the course from the CAMERA alone - the Round-1 deliverable.

It does NOT tune its own control. It ESTIMATES a racing line from perception and hands it to the
SAME measured-dynamics follower the oracle uses (common.line_follower). Only the line source differs.

SPEC (VADR-TS-002 sec 3.3): absolute position is not exposed. So we build a LOCAL frame by
integrating body-frame velocity through the orientation quat (dead reckoning - validated near-zero
drift), and locate gates in that frame from the camera.

What the data says perception can and can't do (see analysis.vision_map):
  * CROSS-TRACK (lateral + vertical bearing) is reliable to ~1 m - the axis that decides whether we
    fly through the hole. We take it straight from the image bearing.
  * DEPTH (range) is unreliable (tens of m) and weakly observable on a straight approach. We do NOT
    trust PnP range; gate DEPTH rides on dead reckoning (seeded once from the in-band pinhole range).

So each gate is tracked TEMPORALLY as a point fixed in the local frame: its cross-track is pulled
onto the camera bearing every detection; its depth is carried by our own motion. The current gate +
the next gate form a short rolling line; the follower flies it. Detection dropouts don't matter -
the line lives in the persistent local frame, so we keep tracking it between glimpses.
"""

import csv
import math
import os

import keyboard

from common.dynamics import (CONTROL_HZ, G_ACC, KP_ATT, KP_THRUST_V, MAX_RATE, MAX_THRUST, MIN_THRUST,
                      PITCH_SIGN, ROLL_SIGN, clamp, send_rate_attitude, thrust_for_climb)
from common.gate_geometry import quat_to_rotmat
from common.line_follower import build_line, follow_line
from common.paths import DATASETS_DIR
from pilots.vision_pilot.config import *   # perception / estimation params (the config "header")

_C_TILT, _S_TILT = math.cos(VIS_CAM_UPTILT), math.sin(VIS_CAM_UPTILT)


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


def _det_dir_body(det):
    """Unit body-frame direction (fwd, right, down) to a detection, from its image offset. The
    image bearing is the pilot's most reliable signal; inverts the 20 deg camera up-tilt + pinhole."""
    ox, oy = det["offset_x"], det["offset_y"]
    fc, rc, dc = 1.0, ox * VIS_HALF_TAN_X, oy * VIS_HALF_TAN_Y
    fwd = fc * _C_TILT + dc * _S_TILT
    right = rc
    down = -fc * _S_TILT + dc * _C_TILT
    n = math.sqrt(fwd * fwd + right * right + down * down) or 1.0
    fwd, right, down = fwd / n, right / n, down / n
    if VIS_EL_BIAS_DEG:                        # optional measured elevation-bias correction
        horiz = math.hypot(fwd, right)
        az = math.atan2(right, fwd)
        el = math.atan2(-down, horiz) + math.radians(VIS_EL_BIAS_DEG)
        ce = math.cos(el)
        fwd, right, down = ce * math.cos(az), ce * math.sin(az), -math.sin(el)
    return fwd, right, down


def _mat_vec(R, v):
    return [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]]


def _unit(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / n for c in v]


def _ang(a, b):
    return math.acos(clamp(a[0] * b[0] + a[1] * b[1] + a[2] * b[2], -1.0, 1.0))


# --- per-tick debug log (overwritten each run): the local-frame estimate + follower output.
_DBG_W = _DBG_F = None
_DBG_HDR = ["t", "regime", "lf_x", "lf_y", "lf_z", "cur_x", "cur_y", "cur_z",
            "next_x", "next_y", "next_z", "rng", "v_cur", "v_tgt", "xtrack", "az", "elev", "dclimb", "thr"]


def _dbg_log(row):
    global _DBG_W, _DBG_F
    if _DBG_W is None:
        path = os.path.join(DATASETS_DIR, "vision_pilot_dbg.csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _DBG_F = open(path, "w", newline="")
        _DBG_W = csv.writer(_DBG_F)
        _DBG_W.writerow(_DBG_HDR)
        print(f"vision-pilot debug -> {path}", flush=True)
    _DBG_W.writerow(row)
    _DBG_F.flush()


def _world_elev(world_dir):
    """Elevation (rad, +up) of a world-frame unit direction. Range-independent and, because
    world_dir = R @ body_bearing, already pitch-compensated."""
    return math.atan2(-world_dir[2], math.hypot(world_dir[0], world_dir[1]))


def _track_elev(data, world_dir):
    """EMA the current gate's WORLD elevation - the range-independent, pitch-compensated vertical
    signal, held through detection dropouts so the altitude command doesn't drop out with the camera."""
    elev = _world_elev(world_dir)
    prev = data.get("_vg_elev")
    data["_vg_elev"] = elev if prev is None else (1 - VIS_VERT_ALPHA) * prev + VIS_VERT_ALPHA * elev


def _track_az(data, body_dir):
    """EMA the current gate's BODY azimuth (atan2(right, fwd)) - the range-independent lateral signal.
    Body frame (nose fixed, yaw off), so this is the gate's left/right bearing off the nose."""
    az = math.atan2(body_dir[1], body_dir[0])
    prev = data.get("_vg_az")
    data["_vg_az"] = az if prev is None else (1 - VIS_AZ_ALPHA) * prev + VIS_AZ_ALPHA * az


def _reset(data):
    data["_lf_pos"] = [0.0, 0.0, 0.0]    # local dead-reckoned position (anchored at race start)
    data["_vg_cur"] = None               # current target gate, local-frame position
    data["_vg_next"] = None              # next gate, local-frame position (for look-ahead)
    data["_vg_traj"] = None              # cached racing line (rebuilt on a throttle / gate change)
    data["_vg_elev"] = None              # current gate's world elevation (range-indep, pitch-comp)
    data["_vg_az"] = None                # current gate's body azimuth (range-indep lateral signal)
    data["_vis_state"] = {}              # follower slew/integral state
    data["_vis_live"] = False


def _idle(mavlink_conn, system_boot_ms, data, regime):
    data["vision_regime"] = regime
    data["vis_lean"] = 0.0
    data["oracle_thrust"] = 0.0
    _reset(data)
    send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)


def _hover(mavlink_conn, system_boot_ms, data, regime, roll, pitch, climb_up):
    rr = clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
    pr = clamp(PITCH_SIGN * KP_ATT * (0.0 - pitch), -MAX_RATE, MAX_RATE)
    thrust = clamp(thrust_for_climb(0.0) + KP_THRUST_V * (0.0 - climb_up), MIN_THRUST, MAX_THRUST)
    data["vision_regime"] = regime
    data["oracle_thrust"] = thrust
    send_rate_attitude(mavlink_conn, system_boot_ms, rr, pr, 0.0, thrust)


def _update_gate(local, lf_pos, world_dir, pinhole, alpha):
    """Pull a tracked gate's local position onto the reliable camera bearing (cross-track), keeping
    its depth (carried by dead reckoning). `pinhole` gently corrects range when it's in the trusted
    band. Returns the new local position."""
    if pinhole:
        pinhole *= VIS_RANGE_BIAS           # undo the measured ~38%-short pinhole bias before fusing
    if local is None:
        rng = clamp(pinhole if pinhole else 12.0, VIS_ACQ_MIN_RANGE, VIS_ACQ_MAX_RANGE)
        return [lf_pos[i] + rng * world_dir[i] for i in range(3)]
    rng = math.dist(local, lf_pos) or 1.0
    if pinhole and abs(pinhole - rng) < VIS_RANGE_OUTLIER and VIS_ACQ_MIN_RANGE <= pinhole <= VIS_ACQ_MAX_RANGE:
        rng += VIS_RANGE_GAIN * (pinhole - rng)
    meas = [lf_pos[i] + rng * world_dir[i] for i in range(3)]
    new = [(1 - alpha) * local[i] + alpha * meas[i] for i in range(3)]
    # static-gate prior: cap how far the estimate may jump per update (rejects the noise/latency
    # swings that fed the roll PIO). A real fixed gate's local-frame estimate barely moves.
    step = math.dist(new, local)
    if step > VIS_EST_MAX_STEP:
        s = VIS_EST_MAX_STEP / step
        new = [local[i] + (new[i] - local[i]) * s for i in range(3)]
    return new


def update_vision_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    if not _race_live(data):
        t_go = _seconds_to_go(data)
        _idle(mavlink_conn, system_boot_ms, data,
              f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0) else "IDLE")
        return

    dt = 1.0 / CONTROL_HZ
    att = data.get("attitude", {})
    odo = data.get("odometry") or {}
    roll, pitch = att.get("roll", 0.0), att.get("pitch", 0.0)
    vb = (odo.get("vx", 0.0), odo.get("vy", 0.0), odo.get("vz", 0.0))
    quat = (odo.get("qw", 1.0), odo.get("qx", 0.0), odo.get("qy", 0.0), odo.get("qz", 0.0))
    R = quat_to_rotmat(quat)
    vw = _mat_vec(R, vb)
    climb_up = -vw[2]

    if not data.get("_vis_live"):            # first live tick: anchor the local frame at the origin
        _reset(data)
        data["_vis_live"] = True

    # 1) DEAD-RECKON the local position (integrate world velocity). No absolute position used.
    lf = data["_lf_pos"]
    lf = [lf[0] + vw[0] * dt, lf[1] + vw[1] * dt, lf[2] + vw[2] * dt]
    data["_lf_pos"] = lf

    cur = data.get("_vg_cur")
    nxt = data.get("_vg_next")

    # 2) MEASUREMENT UPDATE on a new camera frame: refine the tracked gates from the image bearing.
    fid = data.get("latest_frame_id")
    gates = data.get("vision_gates") or []
    if fid is not None and fid != data.get("_vis_fid") and gates:
        data["_vis_fid"] = fid
        # per detection: body direction (its [0] = forward), world direction, and pinhole range
        bdirs = [_det_dir_body(g) for g in gates]
        dirs = [(_mat_vec(R, bdirs[k]), gates[k].get("distance_m")) for k in range(len(gates))]
        used = [False] * len(dirs)

        # SELECT THE TARGET GATE = the NEAREST gate ahead (largest blob). Keep the currently-tracked
        # gate only while a detection still matches its bearing AND that detection is still
        # near-biggest (SIZE HYSTERESIS). Otherwise a clearly NEARER gate has come into view - e.g.
        # the gate that jogs sideways (gate 2) - and we switch to it instead of staying locked on a
        # smaller, farther gate and flying right past the near one. (Bearing-only continuity, without
        # this size check, was locking onto the far gate and skipping gate 2.)
        ahead = [k for k in range(len(dirs)) if bdirs[k][0] > 0.0]
        if ahead:
            biggest = max(ahead, key=lambda k: gates[k]["area"])
            j, keep = biggest, False
            if cur is not None:
                pred = _unit([cur[i] - lf[i] for i in range(3)])
                cand = min(ahead, key=lambda k: _ang(pred, dirs[k][0]))
                if (_ang(pred, dirs[cand][0]) < VIS_ASSOC_ANG
                        and gates[cand]["area"] >= VIS_KEEP_FRAC * gates[biggest]["area"]):
                    j, keep = cand, True              # same gate still near-biggest: keep tracking it
            if keep:
                cur = _update_gate(cur, lf, dirs[j][0], dirs[j][1], VIS_FUSE_ALPHA)
                _track_elev(data, dirs[j][0])
                _track_az(data, bdirs[j])
            else:                                     # acquire the nearest gate (new target)
                cur = _update_gate(None, lf, dirs[j][0], dirs[j][1], 1.0)
                data["_vg_elev"] = _world_elev(dirs[j][0])
                data["_vg_az"] = math.atan2(bdirs[j][1], bdirs[j][0])
            used[j] = True

        # associate + update the NEXT gate (the largest remaining blob roughly ahead)
        if cur is not None:
            cand = [k for k in range(len(dirs)) if not used[k] and bdirs[k][0] > 0.3]
            if nxt is not None and cand:
                predn = _unit([nxt[i] - lf[i] for i in range(3)])
                j = min(cand, key=lambda k: _ang(predn, dirs[k][0]))
                if _ang(predn, dirs[j][0]) < VIS_ASSOC_ANG:
                    nxt = _update_gate(nxt, lf, dirs[j][0], dirs[j][1], VIS_FUSE_ALPHA)
                # else: keep the dead-reckoned next-gate estimate (no good match this frame)
            elif nxt is None and cand:
                j = max(cand, key=lambda k: gates[k]["area"])
                nxt = _update_gate(None, lf, dirs[j][0], dirs[j][1], 1.0)

    # 3) GATE PASSED? release the current gate when it's close or behind; promote the next gate.
    if cur is not None:
        to_gate = [cur[i] - lf[i] for i in range(3)]
        fwd_axis = _mat_vec(R, (1.0, 0.0, 0.0))
        behind = (to_gate[0] * fwd_axis[0] + to_gate[1] * fwd_axis[1] + to_gate[2] * fwd_axis[2]) < 0
        if math.sqrt(sum(c * c for c in to_gate)) < VIS_PASS_DIST or behind:
            cur, nxt = nxt, None              # promote the next gate; keep follower slew continuous
            data["_vg_traj"] = None           # force a line rebuild for the new segment

    data["_vg_cur"], data["_vg_next"] = cur, nxt

    if cur is None:                           # nothing to chase yet (or finished): hold level
        _hover(mavlink_conn, system_boot_ms, data, "NO-GATE", roll, pitch, climb_up)
        return

    # 4) BUILD the racing line in the local frame and FOLLOW it with the shared measured-dynamics
    #    follower (same control as the oracle - no vision-specific gains). The line is anchored at the
    #    FIXED local origin (race-start), NOT the moving drone: that way the follower's nearest point
    #    advances along the line as the drone flies (so the speed profile gives cruise, not its
    #    standstill cap, and the altitude reference comes from the gate line, not the drone's own z).
    #    Fold in the next gate ONLY if it's genuinely beyond the current one (a far blob's bad range
    #    can collapse onto the current gate's depth and make a degenerate spline).
    line_gates = [{"gate_id": 0, "position_ned": list(cur)}]
    if nxt is not None and math.dist(nxt, cur) > VIS_MIN_GATE_GAP and \
            math.dist(nxt, (0.0, 0.0, 0.0)) > math.dist(cur, (0.0, 0.0, 0.0)):
        line_gates.append({"gate_id": 1, "position_ned": list(nxt)})

    # Rebuild the spline only every VIS_REBUILD_EVERY ticks (or when forced) - re-splining at 250 Hz
    # as the estimate micro-drifts just injects jitter into the carrot/tangent and makes it jerky.
    tick = data.get("_vis_t", 0)
    if data.get("_vg_traj") is None or tick % VIS_REBUILD_EVERY == 0:
        data["_vg_traj"] = build_line(line_gates, start=(0.0, 0.0, 0.0), v_max=VIS_V_MAX)
    traj = data["_vg_traj"]

    # VERTICAL: range-independent, pitch-compensated WORLD-elevation servo (NOT the line altitude,
    # which inherits the bad gate depth). Servo the gate's world elevation to VIS_TARGET_ELEV (~0 =
    # fly at the gate's altitude). No floor guard (no reliable absolute altitude).
    elev = data.get("_vg_elev")
    climb_cmd = VIS_KP_VE * (elev - VIS_TARGET_ELEV) if elev is not None else 0.0

    # LATERAL: the follower's line cross-track (position-based). A pure azimuth/bearing servo was
    # tried and reverted - it over-strafes as the gate sweeps to the side on the pass. The line is
    # range-corrupted on the one sideways-jog gate (gate 2); that is handled by the range-bias
    # correction in the gate estimate instead.
    state = data.setdefault("_vis_state", {})
    roll_rate, pitch_rate, yaw_rate, thrust, telem = follow_line(
        traj, tuple(lf), quat, vb, (roll, pitch), state, floor_alt=None, use_yaw=False,
        climb_override=climb_cmd)

    rng = math.dist(cur, lf)
    regime = "PUNCH" if rng < VIS_PASS_DIST + 2 else "PURSUE"
    data["vision_regime"] = regime
    data["pursuit_desired_climb"] = telem["desired_climb"]
    data["oracle_thrust"] = thrust
    data["vis_dbg"] = (telem["xtrack"], rng, telem["v_cur"], telem["desired_climb"])
    data["_vis_t"] = data.get("_vis_t", 0) + 1
    _dbg_log([f"{data['_vis_t'] * dt:.3f}", regime,
              f"{lf[0]:.1f}", f"{lf[1]:.1f}", f"{lf[2]:.1f}",
              f"{cur[0]:.1f}", f"{cur[1]:.1f}", f"{cur[2]:.1f}",
              f"{nxt[0]:.1f}" if nxt else "", f"{nxt[1]:.1f}" if nxt else "", f"{nxt[2]:.1f}" if nxt else "",
              f"{rng:.1f}", f"{telem['v_cur']:.1f}", f"{telem['v_target']:.1f}",
              f"{telem['xtrack']:.2f}", f"{data.get('_vg_az') or 0.0:.3f}",
              f"{elev:.3f}" if elev is not None else "", f"{climb_cmd:.2f}", f"{thrust:.3f}"])

    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)
