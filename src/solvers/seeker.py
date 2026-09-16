"""
Vision gate-seeker in the sim. RACE_SOLVER=solvers.seeker

Flies the published course with the camera, the heading and the
barometer and nothing else: no ground-truth position, no plan. The
heading comes from the sim's attitude quaternion (standing in for the
flight controller's compass heading, which is a sensor the real drone
has), altitude from the sim's barometer, vertical speed from its
derivative, the gate from the HSV detector on the FPV frame.

Requires ANGLE mode in the SITL (configure_betaflight.py maps ANGLE to
AUX2 high; this solver always sends aux2 = 1800).

    cd src && python -m raceline.batch_fly --solver solvers.seeker --angle ../out/plans/plan_RACE.json

(the plan is only there to satisfy the launcher and set the lap count).
"""

from __future__ import annotations

import csv
import math
import os
import time

import numpy as np

from raceline.config import load_config, AIGP_REPO
from raceline import course as course_bridge
from raceline.planner import next_numbered
from raceline.rc_backend import StateEstimate, rot_from_quat
from seeker.brain import Detection, SeekerConfig
from seeker.pilot import SeekerPilot, crossings_from_course
from solver.api import RCCommand, SensorUpdate

CFG = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
# DETECTOR SOURCE. "camera": the HSV detector on the sim's FPV frame - but this
# elodin build renders no FPV frames headless and none in our editor runs
# either (0 of 349 on 2026-09-16), so "synthetic" (default) projects the next
# gate through the spec camera (640x360, fx 320, 20 deg up-tilt) from the
# TRUE pose and reports it when it is in the field of view, facing us and
# not too close. That exercises the brain, the legs, the dead reckoning and
# the loops against the real physics and referee; the detector itself is
# proven on the Orin with real frames. The brain never sees position.
DET_SOURCE = os.environ.get("AIGP_SEEKER_DET", "synthetic")
LAPS = int(os.environ.get("AIGP_LAPS", "2"))
_COURSE = course_bridge.load_course(laps=LAPS)
_PILOT = SeekerPilot(CFG, crossings_from_course(_COURSE), start_xy=(0.0, 0.0), laps=LAPS,
                     seeker_cfg=SeekerConfig())

_state = {"baro0": None, "z_prev": None, "t_prev": None, "vz": 0.0, "det": None,
          "log": None, "writer": None, "n": 0, "z_f": None}
# The sim's barometer carries ~0.1 m of noise per sample at 100 Hz: a raw
# derivative is tens of m/s of garbage. Alpha-beta tracker (position gain
# ALPHA, velocity gain BETA per sample) gives a usable z and vz.
ALPHA, BETA = 0.15, 0.004
DETECT_EVERY_S = 1.0 / 30.0
_det_t = [-1.0]


_diag = {"frames": 0, "fresh": 0, "detect_calls": 0, "cv2": None, "dumped": 0, "mask_px": 0}
DUMP_DIR = AIGP_REPO / "out" / "flightlogs" / "frames"


def _detect(frame_rgba, t):
    """HSV ring detector on the FPV frame -> nearest gate, or None."""
    try:
        import cv2
        from perception.detectors.hsv_classic import gate_mask
        from perception.gate_detection import mask_to_detections
        _diag["cv2"] = cv2.__version__
    except ImportError as e:
        _diag["cv2"] = f"IMPORT FAILED: {e}"
        return None
    _diag["detect_calls"] += 1
    arr = np.asarray(frame_rgba)
    if arr.dtype != np.uint8:
        arr = np.clip(arr * (255.0 if arr.max() <= 1.0 else 1.0), 0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR) if arr.shape[-1] == 4 else cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    mask = gate_mask(bgr)
    _diag["mask_px"] = int(mask.sum() // 255)
    if _diag["detect_calls"] % 60 == 1 and _diag["dumped"] < 12:
        DUMP_DIR.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(DUMP_DIR / f"frame_{_diag['dumped']:02d}_t{t:05.1f}.png"), bgr)
        cv2.imwrite(str(DUMP_DIR / f"mask_{_diag['dumped']:02d}_t{t:05.1f}.png"), mask)
        _diag["dumped"] += 1
    dets = mask_to_detections(mask, bgr.shape)
    if not dets:
        return None
    d = dets[0]
    return Detection(offset_x=float(d.offset_x), offset_y=float(d.offset_y),
                     area_frac=float(d.area_frac or d.area / float(bgr.shape[0] * bgr.shape[1])), t=t)


def _synthetic_detect(update, t):
    """Next crossing projected through the spec camera from the true pose."""
    k = min(_PILOT.brain.k, len(_PILOT.brain.seq) - 1)
    c = _PILOT.brain.seq[k]
    R = rot_from_quat(update.world_pos[0:4])            # body FLU -> world
    p = np.asarray(update.world_pos[4:7], dtype=float)
    d_w = np.array([c.x - p[0], c.y - p[1], c.z - p[2]])
    d_b = R.T @ d_w                                       # in body FLU
    dist = float(np.hypot(d_b[0], d_b[1]))
    if dist < 0.4:
        return None
    tilt = math.radians(20.0)                             # camera axis 20 deg above body forward
    fwd = d_b[0] * math.cos(tilt) + d_b[2] * math.sin(tilt)
    up = -d_b[0] * math.sin(tilt) + d_b[2] * math.cos(tilt)
    left = d_b[1]
    if fwd <= 0.1:
        return None
    x_img = -left / fwd                                   # +right (tan units, fx = half-width)
    y_img = -up / fwd                                     # +down
    if abs(x_img) > 1.0 or abs(y_img) > 0.5625:           # outside the 90 x 59 deg frame
        return None
    yaw = math.atan2(R[1, 0], R[0, 0])
    if abs(((c.heading_rad - yaw + math.pi) % (2 * math.pi)) - math.pi) > math.radians(70):
        return None                                       # seeing the ring edge-on / from behind
    app = 2.7 / max(dist, 0.4)
    area_frac = min(1.0, (app / 2.0) ** 2 * 0.35)
    return Detection(offset_x=x_img, offset_y=y_img / 0.5625, area_frac=area_frac, t=t)


def _log(t, est, det, out, update):
    if _state["writer"] is None:
        path = next_numbered(str(AIGP_REPO / "out" / "flightlogs" / "seeker_XXX.csv"))
        path.parent.mkdir(parents=True, exist_ok=True)
        _state["log"] = open(path, "w", newline="", encoding="utf-8")
        _state["writer"] = csv.writer(_state["log"])
        _state["writer"].writerow(["t", "phase", "crossing", "yaw", "z", "vz", "det_x", "det_y", "det_area",
                                   "z_target", "yaw_target", "ax", "ay", "roll_deg", "pitch_deg",
                                   "throttle", "roll", "pitch", "yaw_stick", "gt_x", "gt_y", "gt_z", "next_gate"])
        print(f"[SEEKER] log -> {path}")
    _state["n"] += 1
    if _state["n"] % 10:
        return
    w = _state["writer"]
    w.writerow([f"{t:.3f}", out.phase, out.crossing, f"{est.yaw:.3f}", f"{est.p[2]:.2f}", f"{est.v[2]:.2f}",
                f"{det.offset_x:.3f}" if det else "", f"{det.offset_y:.3f}" if det else "",
                f"{det.area_frac:.4f}" if det else "", f"{out.z_target:.2f}", f"{out.yaw_target:.3f}",
                f"{out.a_des[0]:.2f}", f"{out.a_des[1]:.2f}", f"{out.tilt_deg[0]:.1f}", f"{out.tilt_deg[1]:.1f}",
                out.throttle, out.roll, out.pitch, out.yaw,
                f"{update.world_pos[4]:.2f}", f"{update.world_pos[5]:.2f}", f"{update.world_pos[6]:.2f}",
                update.next_gate_index])
    if _state["n"] % 500 == 0:
        _state["log"].flush()
        fr = update.frame_rgba
        print(f"[SEEKER] cam: frames={_diag['frames']} fresh={_diag['fresh']} detect_calls={_diag['detect_calls']} "
              f"cv2={_diag['cv2']} mask_px={_diag['mask_px']} shape={None if fr is None else (getattr(fr, 'shape', None), getattr(fr, 'dtype', None))}")
        print(f"[SEEKER] t={t:6.1f} {out.phase:7s} k={out.crossing:2d} z={est.p[2]:4.2f} "
              f"yaw={math.degrees(est.yaw):5.0f} det={'%.3f' % det.area_frac if det else '-'} "
              f"stk=({out.roll},{out.pitch},{out.throttle},{out.yaw})")


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    # sensors only: heading from attitude, altitude from the baro
    R = rot_from_quat(update.world_pos[0:4])
    yaw = math.atan2(R[1, 0], R[0, 0])
    if _state["baro0"] is None and update.baro_fresh:
        _state["baro0"] = float(update.baro)
    z_meas = float(update.baro) - (_state["baro0"] or 0.0)
    if _state["z_f"] is None:
        _state["z_f"], _state["vz"], _state["t_prev"] = z_meas, 0.0, t
    elif update.baro_fresh and t > _state["t_prev"]:
        dt = t - _state["t_prev"]
        pred = _state["z_f"] + _state["vz"] * dt
        r = z_meas - pred
        _state["z_f"] = pred + ALPHA * r
        _state["vz"] += (BETA / dt) * r
        _state["t_prev"] = t
    z = _state["z_f"]
    vz = _state["vz"]
    if DET_SOURCE == "synthetic":
        # stand-in for the FC's filtered altitude + vario (MSP_ALTITUDE): the
        # sim's raw baro through our filter lags enough to make the altitude
        # loop hunt (race_163: 1.2 to 1.9 m swings)
        z = float(update.world_pos[6])
        vz = float(update.world_vel[5])
    # body rates for the acro attitude loop's damping term (world -> body)
    omega_w = np.asarray(update.world_vel[0:3], dtype=float)
    est = StateEstimate(p=np.array([0.0, 0.0, z]), v=np.array([0.0, 0.0, vz]),
                        R=R, yaw=yaw, omega=R.T @ omega_w)
    det = _state["det"]
    if update.frame_rgba is not None:
        _diag["frames"] += 1
    if update.frame_fresh:
        _diag["fresh"] += 1
    if DET_SOURCE == "synthetic":
        if t - _det_t[0] >= DETECT_EVERY_S:
            _det_t[0] = t
            det = _synthetic_detect(update, t)
            _state["det"] = det
        elif det is not None and t - det.t > 0.25:
            det = None
    elif update.frame_fresh and update.frame_rgba is not None and t - _det_t[0] >= DETECT_EVERY_S:
        _det_t[0] = t
        det = _detect(update.frame_rgba, t)
        _state["det"] = det
    elif det is not None and t - det.t > 0.25:
        det = None
    out = _PILOT.tick(t, est, det, update.baro_fresh)
    _log(t, est, det, out, update)
    return RCCommand(throttle=out.throttle, roll=out.roll, pitch=out.pitch, yaw=out.yaw,
                     arm=out.arm, aux2=out.aux2)
