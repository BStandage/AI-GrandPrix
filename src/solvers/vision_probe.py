"""Vision probe: collect FPV frames of the (now orange) gates in-sim, then
run the classic HSV gate detector over them offline.

Two halves:

KNOWN LIMIT (measured 2026-08-28): headless `elodin run` renders ZERO FPV
frames on this build - the camera only produces frames with the editor
viewport attached. To capture frames, launch via the sim repo's
`run_race.cmd solvers.vision_probe` (which opens the editor), not race.py.

1. IN-SIM SOLVER (WSL, no cv2 needed) - arms, climbs to gate height at the
   spawn (which faces g0, 3 m away), holds, then creeps toward the gate so
   frames cover several ranges. Saves raw RGBA frames as .npy every 0.5 s:

       RACE_SOLVER=solvers.vision_probe AIGP_SIM_TIME=30 \
           uv run elodin run sim/main.py
       # frames -> AI-GrandPrix/out/vision/probe_XXX/frame_*.npy

2. ANALYZER (Windows, needs cv2) - runs perception.detectors.hsv_classic
   (the VQ-era detector, tuned for hue ~5; the PQ orange #ff3200 is hue
   ~6, organizers say "95% the same") over every saved frame, reports mask
   coverage + largest-blob box, writes frame|mask side-by-side PNGs:

       cd src && python -m solvers.vision_probe --analyze ../out/vision/probe_000
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import numpy as np

from raceline.config import AIGP_REPO, load_config

FRAME_EVERY_S = 0.5
HOLD_S = 8.0          # hold at spawn view first (fixed-range frames)
CREEP_MPS = 0.4       # then creep toward the gate
CREEP_STOP_M = 1.2    # stop this far before the gate plane

_state: dict = {}


def _probe_dir() -> Path:
    p = os.environ.get("AIGP_VISION_OUT")
    if p:
        return Path(p)
    from raceline.planner import next_numbered
    return next_numbered(str(AIGP_REPO / "out" / "vision" / "probe_XXX"))


def autopilot(update):
    from solver.api import RCCommand
    from raceline import course as course_bridge
    from raceline.rc_backend import (AltitudeLoop, GroundTruthSource,
                                     YawLoop, attitude_sticks)

    if not _state:
        cfg = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
        course = course_bridge.load_course()
        g0 = course.crossings[0]
        _state.update(
            cfg=cfg, src=GroundTruthSource(), alt=AltitudeLoop(cfg),
            yaw=YawLoop(cfg), g0=g0,
            hold=np.array([0.0, 0.0, g0.z]),
            n_hat=np.array([math.cos(g0.heading_rad),
                            math.sin(g0.heading_rad)]),
            t_air=None, last_save=-1e9, n_saved=0,
            outdir=_probe_dir(),
        )
        _state["outdir"].mkdir(parents=True, exist_ok=True)
        print(f"[VISION] probing g0 ({g0.label}) at z={g0.z}; "
              f"frames -> {_state['outdir']}")

    t = update.t
    if t < 0.50:
        return RCCommand(arm=1000, throttle=1000)
    if t < 0.75:
        return RCCommand(arm=1800, throttle=1000)

    cfg = _state["cfg"]
    est = _state["src"].estimate(update)
    g0 = _state["g0"]

    airborne = est.p[2] >= 1.0
    if airborne and _state["t_air"] is None:
        _state["t_air"] = t

    # target: hold at spawn view, then creep along g0's entry normal
    tgt = _state["hold"].copy()
    if _state["t_air"] is not None and t - _state["t_air"] > HOLD_S:
        creep = CREEP_MPS * (t - _state["t_air"] - HOLD_S)
        gate_dist = float(np.dot(
            np.array([g0.x, g0.y]) - _state["hold"][:2], _state["n_hat"]))
        creep = min(creep, max(0.0, gate_dist - CREEP_STOP_M))
        tgt[:2] = _state["hold"][:2] + creep * _state["n_hat"]

    a_des = (cfg.follower.kp_pos * (tgt[:2] - est.p[:2])
             - cfg.follower.kd_pos * est.v[:2])
    throttle = _state["alt"].throttle(t, est, float(tgt[2]), 0.0, airborne,
                                      update.baro_fresh)
    roll = pitch = yaw_stick = 1500
    if airborne:
        roll, pitch, _ = attitude_sticks(cfg, est, a_des)
        yaw_des = math.atan2(g0.y - est.p[1], g0.x - est.p[0])
        yaw_stick = _state["yaw"].stick(est, yaw_des)

        if (update.frame_fresh and update.frame_rgba is not None
                and t - _state["last_save"] >= FRAME_EVERY_S):
            _state["last_save"] = t
            i = _state["n_saved"]
            _state["n_saved"] = i + 1
            d = float(np.hypot(g0.x - est.p[0], g0.y - est.p[1]))
            np.save(_state["outdir"] / f"frame_{i:03d}_d{d:04.1f}m.npy",
                    update.frame_rgba)
            if i % 10 == 0:
                print(f"[VISION] saved {i + 1} frames (gate {d:.1f} m)")

    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch,
                     yaw=yaw_stick)


def reset_state():
    _state.clear()


# ---------------------------------------------------------------------------
# Analyzer (Windows side; needs cv2 for the real detector)
# ---------------------------------------------------------------------------

def analyze(probe_dir, out_subdir="masks"):
    import cv2
    from perception.detectors.hsv_classic import gate_mask

    probe_dir = Path(probe_dir)
    outdir = probe_dir / out_subdir
    outdir.mkdir(exist_ok=True)
    rows = []
    for f in sorted(probe_dir.glob("frame_*.npy")):
        rgba = np.load(f)
        bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        mask = gate_mask(bgr)
        cov = 100.0 * float(np.count_nonzero(mask)) / mask.size
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        box = None
        if cnts:
            box = cv2.boundingRect(max(cnts, key=cv2.contourArea))
        side = np.hstack([bgr, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        if box:
            x, y, w, h = box
            cv2.rectangle(side, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imwrite(str(outdir / (f.stem + ".png")), side)
        rows.append((f.stem, cov, len(cnts), box))
    print(f"{'frame':34s} {'mask%':>6s} {'blobs':>5s}  bbox(x,y,w,h)")
    for name, cov, nb, box in rows:
        print(f"{name:34s} {cov:6.2f} {nb:5d}  {box}")
    hit = sum(1 for _, cov, _, _ in rows if cov > 0.05)
    print(f"\n{hit}/{len(rows)} frames with gate pixels; "
          f"side-by-sides -> {outdir}")
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", required=True, help="probe_XXX directory")
    args = ap.parse_args()
    analyze(args.analyze)
