"""
Per-frame vision data collector.

For every camera frame it writes one rich JSON record to vision_frames.jsonl in the session folder.
Each record fuses, all at the same instant:

  * the drone's true pose, velocity, and attitude (odometry), sampled at frame time with the same
    "no replay mismatch" alignment the vision-shadow uses
  * the full perception output: every red-ring detection with its image offset, area, opening, and
    pinhole distance, plus a per-detection solvePnP body pose (not just the nearest gate)
  * the ground-truth geometry of every gate (body frame, world, and where it projects in the image),
    so each detection is auto-labelled against truth (which gate it is, how far off in bearing,
    elevation, and range), and we can see whether the gate we should be flying through was detected
  * the pilot's control intent that tick (regime, commanded roll/climb/lean/thrust, target)

This is the "collect everything, then diagnose from data" dataset. Run it on an oracle lap to see what
vision sees along the perfect racing line, and on a vision lap to see why it misses.

Ground truth is logged for analysis only. The live vision pilot never reads it. Toggle collection with
COLLECT_VISION_DATA below. Every operation is wrapped so a logging error can never crash the vision
thread.
"""

import json
import math
import os
import time

import numpy as np

from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections
from perception.vision_pose import pnp_pose_body_ex, project_body_to_offset
from common.gate_geometry import get_drone_pose, relative_gate, quat_to_rotmat

# Master switch. True = write vision_frames.jsonl every run (the data-collection deliverable).
COLLECT_VISION_DATA = True

# A detection is matched to the ground-truth gate whose projected image position is nearest, but
# only if within this image-offset radius (rejects matches to a gate that isn't really there).
_MATCH_OFFSET_RADIUS = 0.35


def _control_intent(data):
    """Whatever the active pilot decided this tick. Works for both vision and oracle modes.
    Only keys that are present are emitted (best-effort, never raises)."""
    keys = ("vision_regime", "traj_regime", "oracle_thrust", "vis_lean",
            "pursuit_desired_climb", "traj_xtrack", "traj_vtgt", "traj_vcur", "traj_yawrate")
    out = {k: data.get(k) for k in keys if data.get(k) is not None}
    dbg = data.get("vis_dbg")
    if dbg is not None and len(dbg) == 4:
        out["vis_offx"], out["vis_offy"], out["vis_dist"], out["vis_desired_climb"] = dbg
    tgt = data.get("_vis_tgt_img")
    if tgt is not None:
        out["vis_tracked_img"] = list(tgt)
    return out


def _gt_gates(data, pose):
    """Ground-truth geometry of every gate in this run's frame, relative to the drone now."""
    gates = data.get("gates")
    if pose is None or not gates:
        return []
    rs = data.get("race_status") or {}
    active = rs.get("active_gate_index", -1)
    out = []
    for g in gates:
        rel = relative_gate(pose[0], pose[1], g)
        proj = project_body_to_offset(rel["forward"], rel["right"], rel["down"])
        gid = g.get("gate_id")
        out.append({
            "gate_id": gid,
            "fwd": round(rel["forward"], 3),
            "right": round(rel["right"], 3),
            "down": round(rel["down"], 3),
            "distance": round(rel["distance"], 3),
            "az_deg": round(rel["azimuth_deg"], 2),
            "el_deg": round(rel["elevation_deg"], 2),
            "world_ned": [round(c, 3) for c in g["position_ned"]],
            "orientation_ned": [round(c, 4) for c in g.get("orientation_ned", [])],
            "proj_offx": round(proj[0], 4) if proj else None,
            "proj_offy": round(proj[1], 4) if proj else None,
            "in_fov": bool(proj and abs(proj[0]) <= 1.0 and abs(proj[1]) <= 1.0 and rel["forward"] > 0),
            "is_active": gid == active,
            "passed": isinstance(active, int) and active >= 0 and isinstance(gid, int) and gid < active,
        })
    return out


def _match_to_gt(det_offx, det_offy, gt_gates):
    """Nearest GT gate to a detection by image-offset distance (auto-label). Returns
    (gate_id, offset_dist), or (None, None) if nothing is within the match radius."""
    best, bd = None, _MATCH_OFFSET_RADIUS
    for g in gt_gates:
        if g["proj_offx"] is None:
            continue
        d = math.hypot(g["proj_offx"] - det_offx, g["proj_offy"] - det_offy)
        if d < bd:
            bd, best = d, g
    if best is None:
        return None, None
    return best["gate_id"], round(bd, 4)


class VisionDataCollector:

    def __init__(self, session_dir):
        self.path = os.path.join(session_dir, "vision_frames.jsonl")
        self._f = open(self.path, "w")
        self._n = 0
        self._warned = False
        print(f"Vision data collection writing to {self.path}  (per-frame perception, truth, control)",
              flush=True)

    def log(self, frame_id, img, sim_time_ns, data):
        try:
            self._log(frame_id, img, sim_time_ns, data)
        except Exception as e:           # never take down the vision thread for a log error
            if not self._warned:
                self._warned = True
                print(f"[vision-collect] logging error (suppressed further): {e!r}", flush=True)

    def _log(self, frame_id, img, sim_time_ns, data):
        h, w = img.shape[:2]
        pose = get_drone_pose(data)
        odo = data.get("odometry") or {}
        att = data.get("attitude") or {}
        rs = data.get("race_status") or {}

        # world-frame velocity + yaw (odometry vx/vy/vz are in the body frame)
        vw = yaw = None
        if pose is not None:
            R = quat_to_rotmat(pose[1])
            vb = (odo.get("vx", 0.0), odo.get("vy", 0.0), odo.get("vz", 0.0))
            vw = [R[0][0] * vb[0] + R[0][1] * vb[1] + R[0][2] * vb[2],
                  R[1][0] * vb[0] + R[1][1] * vb[1] + R[1][2] * vb[2],
                  R[2][0] * vb[0] + R[2][1] * vb[1] + R[2][2] * vb[2]]
            q = pose[1]
            yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))

        gt = _gt_gates(data, pose)

        # FULL perception (classic CV baseline), with a solvePnP body pose for every detection
        dets_raw = mask_to_detections(gate_mask(img), img.shape)
        dets = []
        for d in dets_raw:
            corners = d.corners
            pose_ex = pnp_pose_body_ex(corners, w, h) if corners is not None else None
            pose_b = pose_ex[0] if pose_ex is not None else None
            rvec = pose_ex[1] if pose_ex is not None else None
            rec = {
                "offx": round(d.offset_x, 4),
                "offy": round(d.offset_y, 4),
                "area_frac": round(d.area_frac, 6),
                "has_opening": d.has_opening,
                "pinhole_dist": round(d.distance_m, 2),
                "bbox": [int(v) for v in d.bbox],
                "ring_bbox": [int(v) for v in d.ring_bbox] if d.ring_bbox else None,
                "opening_bbox": ([int(v) for v in d.opening_bbox]
                                 if d.opening_bbox else None),
            }
            if corners is not None:
                # ordered TL,TR,BR,BL pixel corners for offline re-PnP / BA
                rec["corners"] = [[round(float(p[0]), 2), round(float(p[1]), 2)]
                                  for p in np.asarray(corners).reshape(-1, 2)]
            if pose_b is not None:
                pf, pr, pd = pose_b
                rec["pnp_fwd"] = round(pf, 3)
                rec["pnp_right"] = round(pr, 3)
                rec["pnp_down"] = round(pd, 3)
                rec["pnp_dist"] = round(math.hypot(pf, pr, pd), 3)
            if rvec is not None:
                rec["rvec"] = [round(float(v), 5) for v in rvec]
            gid, mdist = _match_to_gt(d.offset_x, d.offset_y, gt)
            rec["match_gid"] = gid
            rec["match_offdist"] = mdist
            if gid is not None and pose_b is not None:
                g = next((x for x in gt if x["gate_id"] == gid), None)
                if g is not None:
                    rec["az_err_deg"] = round(math.degrees(math.atan2(pose_b[1], pose_b[0])) - g["az_deg"], 2)
                    rec["el_err_deg"] = round(math.degrees(math.atan2(-pose_b[2], math.hypot(pose_b[0], pose_b[1]))) - g["el_deg"], 2)
                    rec["range_err"] = round(math.hypot(*pose_b) - g["distance"], 2)
                    rec["pos_err"] = round(math.dist(pose_b, (g["fwd"], g["right"], g["down"])), 2)
            dets.append(rec)

        # which GT gates (in front + in FOV) were detected this frame?
        matched = {d["match_gid"] for d in dets if d["match_gid"] is not None}
        for g in gt:
            g["detected"] = g["gate_id"] in matched

        record = {
            "frame_id": frame_id,
            "sim_time_ns": sim_time_ns,
            "recv_ns": time.time_ns(),
            "mode": data.get("_control_mode"),
            "pose": None if pose is None else {
                "x": round(odo.get("x", 0.0), 3), "y": round(odo.get("y", 0.0), 3),
                "z": round(odo.get("z", 0.0), 3), "alt": round(-odo.get("z", 0.0), 3),
                "qw": odo.get("qw"), "qx": odo.get("qx"), "qy": odo.get("qy"), "qz": odo.get("qz"),
                "vx_body": odo.get("vx"), "vy_body": odo.get("vy"), "vz_body": odo.get("vz"),
                "time_usec": odo.get("time_usec"), "reset_counter": odo.get("reset_counter"),
            },
            "vw": None if vw is None else [round(c, 3) for c in vw],
            "speed_h": None if vw is None else round(math.hypot(vw[0], vw[1]), 3),
            "climb_up": None if vw is None else round(-vw[2], 3),
            "att": {"roll": round(att.get("roll", 0.0), 4), "pitch": round(att.get("pitch", 0.0), 4),
                    "yaw": round(att.get("yaw", 0.0), 4)} if att else None,
            "yaw_odo": None if yaw is None else round(yaw, 4),
            "race": {"active_gate_index": rs.get("active_gate_index"),
                     "race_start_boot_time_ms": rs.get("race_start_boot_time_ms"),
                     "sim_boot_time_ms": rs.get("sim_boot_time_ms"),
                     "race_finish_time_ns": rs.get("race_finish_time_ns")},
            "n_dets": len(dets),
            "dets": dets,
            "gt_gates": gt,
            "ctrl": _control_intent(data),
        }
        self._f.write(json.dumps(record, default=_json_default) + "\n")
        self._f.flush()
        self._n += 1

    def close(self):
        try:
            self._f.close()
            print(f"Vision data collection closed: {self._n} frames written to {self.path}", flush=True)
        except Exception:
            pass


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)
