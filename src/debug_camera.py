#
# Live camera feed debug viewer
# Shows the raw simulator frame alongside the HSV-processed gate mask in real time.
#
# Run from main.py: set DEBUG_CAMERA = True

import math
import threading
import time

import cv2

from common.camera import CX, CY, FX, FY, WIDTH, HEIGHT
from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections, annotate

GATE_HALF = 1.35
CAM_UPTILT = math.radians(20.0)


def _project(px, py, pz, pose):
    """World point (z-up map frame) -> pixel using the pilot's own pose belief.

    Same camera model as estimate.expected_visible (yaw+pitch, roll ignored:
    debug square, not a measurement)."""
    x, y, z, yaw, pitch = pose
    dx, dy, dz = px - x, py - y, pz - z
    be = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    if abs(be) > 1.35:
        return None
    r_h = math.hypot(dx, dy)
    el = math.atan2(dz, max(r_h, 1e-6))
    el_cam = CAM_UPTILT - pitch - el   # angle below camera axis
    if abs(el_cam) > 1.2:
        return None
    u = CX - FX * math.tan(be)
    v = CY + FY * math.tan(el_cam)
    return int(round(u)), int(round(v))


_OVERLAY_ERR = [False]


def draw_map_gates(img, shared_data):
    """Blue square where the MAP (through the pilot's pose estimate) says each
    gate is. Green squares (vision) vs blue (belief) = live map error.

    Always writes a status line so a blank overlay is self-explaining."""
    try:
        return _draw_map_gates(img, shared_data)
    except Exception as e:                       # viewer must never die
        if not _OVERLAY_ERR[0]:
            _OVERLAY_ERR[0] = True
            print(f"[debug] map overlay error: {e!r}", flush=True)
        return img


class _ShadowDR:
    """Viewer-side pose belief for pilots without an estimator (steady/ace):
    IMU world-accel integration on the pilot's own heading, hard-snapped to
    the map gate at every race tick. Drifts meters mid-leg (no bias cal, no
    vision) — honest about what a map-flying pilot would believe."""

    def __init__(self):
        self.gates = None
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.att = [0.0, 0.0]        # lagged roll, pitch
        self.imu_us = None
        self.prev_gate = 0
        self.ticked = False

    def _load(self):
        if self.gates is None:
            import json
            import os
            from analysis.map_resolve import resolve_map_path
            path = (os.environ.get("AIGP_FAIR_MAP")
                    or os.environ.get("AIGP_CL_MAP") or resolve_map_path())
            blob = json.load(open(path))
            self.gates = {g["gate_id"]: dict(g) for g in blob["gates"]
                          if g.get("gate_id", -1) >= 0}
            # Overlay projects z-up. Maps disagree on z convention (chain =
            # z-up, extract/NED builds = z-down): if most gates sit "below
            # spawn", assume z-down and flip for display.
            zs = sorted(g["z"] for g in self.gates.values())
            flipped = zs[len(zs) // 2] < 0
            if flipped:
                for g in self.gates.values():
                    g["z"] = -g["z"]
            print(f"[debug] shadow-DR map: {os.path.basename(path)}"
                  + ("  (z flipped to z-up for display)" if flipped else ""),
                  flush=True)
        return self.gates

    def update(self, sd):
        from pilots.fair_pilot.estimate import imu_world
        gates = self._load()
        rs = sd.get("race_status") or {}
        start = rs.get("race_start_boot_time_ms", -1)
        now = rs.get("sim_boot_time_ms", 0) or 0
        live = (start is not None and start >= 0 and now >= start
                and (rs.get("race_finish_time_ns", -1) or -1) < 0)
        imu = sd.get("highres_imu") or {}
        t_us = imu.get("time_usec")
        if t_us is None:
            return None
        dt = 0.0
        if self.imu_us is not None:
            dt = (t_us - self.imu_us) * 1e-6
        self.imu_us = t_us
        if not live:
            self.p = [0.0, 0.0, 0.0]
            self.v = [0.0, 0.0, 0.0]
            self.prev_gate = 0
            self.ticked = False
        elif 0.0 < dt < 0.1:
            yaw = float(sd.get("_sp_heading") or sd.get("_sp_yaw") or 0.0)
            a = 1.0 - math.exp(-dt / 0.27)
            self.att[0] += (float(sd.get("_sp_des_roll") or 0.0) - self.att[0]) * a
            self.att[1] += (float(sd.get("_sp_des_pitch") or 0.0) - self.att[1]) * a
            fw = imu_world(self.att[0], self.att[1], yaw,
                           imu.get("xacc", 0.0), imu.get("yacc", 0.0),
                           imu.get("zacc", 0.0))
            for i, aw in enumerate((fw[0], fw[1], fw[2] - 9.81)):
                self.v[i] += aw * dt
                self.p[i] += self.v[i] * dt
            gid = rs.get("active_gate_index", 0) or 0
            if gid > self.prev_gate:
                g = gates.get(gid - 1)
                gn = gates.get(gid)
                if g is not None:
                    self.p = [g["x"], g["y"], g["z"]]
                    # Drop accumulated velocity error too, or drift regrows
                    # instantly: horizontal speed magnitude kept, direction
                    # re-pointed down the new leg (fair's first-tick trick).
                    if gn is not None:
                        ux, uy = gn["x"] - g["x"], gn["y"] - g["y"]
                        un = math.hypot(ux, uy) or 1.0
                        sp = min(math.hypot(self.v[0], self.v[1]), 8.0)
                        self.v[0] = sp * ux / un
                        self.v[1] = sp * uy / un
                    self.v[2] = 0.0
                self.prev_gate = gid
                self.ticked = True
        yaw = float(sd.get("_sp_heading") or sd.get("_sp_yaw") or 0.0)
        return ((self.p[0], self.p[1], self.p[2], yaw, self.att[1]),
                gates, self.ticked)


_SHADOW = _ShadowDR()


def _pose_source(shared_data):
    """(pose, gates, ticked) from whichever pilot has a position belief.

    fair/cl keep {est, gates} in shared_data; giga keeps a module estimator;
    steady/ace fall back to the viewer's shadow dead-reckoner."""
    vst = shared_data.get("_vins")
    if vst and vst.get("inited"):
        kf = vst["kf"]
        yaw = math.atan2(kf.R[1, 0], kf.R[0, 0])
        pitch = -math.asin(max(-1.0, min(1.0, kf.R[2, 0])))
        return ((kf.p[0], kf.p[1], kf.p[2], yaw, pitch),
                vst.get("gates") or {}, vst.get("prev_gate", 0) > 0)
    for key in ("_fair", "_cl"):
        st = shared_data.get(key)
        if not st or "est" not in st:
            continue
        est = st["est"]
        plant = est.est.plant if hasattr(est, "est") else est.plant
        return ((est.p[0], est.p[1], est.p[2],
                 plant.yaw, getattr(plant, "pitch", 0.0)),
                st.get("gates") or {}, est.last_tick_gate is not None)
    try:
        from pilots.giga_pilot.giga_pilot import _EST
        if _EST is not None and getattr(_EST, "gates", None):
            return ((_EST.p[0], _EST.p[1], _EST.p[2],
                     _EST.plant.yaw, getattr(_EST.plant, "pitch", 0.0)),
                    _EST.gates, _EST.last_tick_gate is not None)
    except Exception:
        pass
    return _SHADOW.update(shared_data)


def _draw_map_gates(img, shared_data):
    src = _pose_source(shared_data)
    if src is None:
        return img
    pose, gates, ticked = src
    shadow = not (shared_data.get("_fair") or shared_data.get("_cl"))
    rs = shared_data.get("race_status") or {}
    chasing = rs.get("active_gate_index", 0) or 0
    n_drawn = 0
    for gid, g in list(gates.items()):
        nx, ny = g.get("cross_dir", [1.0, 0.0])[:2]
        nn = math.hypot(nx, ny) or 1.0
        rx, ry = ny / nn, -nx / nn          # in-plane right vector
        corners = []
        for sr, sz in ((-1, 1), (1, 1), (1, -1), (-1, -1)):
            p = _project(g["x"] + sr * GATE_HALF * rx,
                         g["y"] + sr * GATE_HALF * ry,
                         g["z"] + sz * GATE_HALF, pose)
            if p is None:                    # any corner behind camera: skip
                corners = []
                break
            corners.append(p)
        if not corners:
            continue
        color = (255, 255, 0) if gid == chasing else (160, 160, 40)
        thick = 2 if gid == chasing else 1
        drew = False
        for i in range(4):
            ok, p1, p2 = cv2.clipLine((0, 0, WIDTH - 1, HEIGHT - 1),
                                      corners[i], corners[(i + 1) % 4])
            if ok:
                cv2.line(img, p1, p2, color, thick)
                drew = True
        if drew:
            n_drawn += 1
            u0 = min(max(corners[0][0], 4), WIDTH - 40)
            v0 = min(max(corners[0][1] - 4, 14), HEIGHT - 6)
            cv2.putText(img, f"m{gid}", (u0, v0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    status = (f"map: p=({pose[0]:+.1f},{pose[1]:+.1f},{pose[2]:+.1f}) "
              f"yaw={math.degrees(pose[3]):+.0f} chase=g{chasing} "
              f"vis={n_drawn}"
              + (" [shadow-DR]" if shadow else "")
              + ("" if ticked else " [PRE-TICK]"))
    cv2.putText(img, status, (10, HEIGHT - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 60), 1)
    return img


def _debug_camera_loop(shared_data):
    """Show a 3-panel live view: raw frame | HSV mask | annotated detections.
    Pressing Q or ESC inside the window stops the whole program cleanly."""
    print("[debug] Camera window starting - press Q or ESC in the window to quit.", flush=True)
    while shared_data["running"]:
        frame = shared_data.get("latest_frame")
        if frame is None:
            time.sleep(0.01)
            continue

        # Run the same HSV segmentation the flight controller uses
        mask = gate_mask(frame)
        dets = mask_to_detections(mask, frame.shape)
        annotated = annotate(frame.copy(), dets)
        annotated = draw_map_gates(annotated, shared_data)

        # Stack: raw (left) | mask (centre) | annotated (right)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = cv2.hconcat([frame, mask_bgr, annotated])

        # Status label - gate bearing, distance, area
        if dets:
            best = dets[0]
            label = (f"gate  offset={best.offset_x:+.2f}  dist={best.distance_m:.1f}m  "
                     f"area={best.area:.0f}  opening={'Y' if best.has_opening else 'N'}")
        else:
            label = "no gate detected"
        cv2.putText(combined, label, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        cv2.imshow("AI-GP debug  |  raw  |  HSV mask  |  detections", combined)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):     # Q or ESC closes everything
            shared_data["running"] = False
            break

    cv2.destroyAllWindows()
    print("[debug] Camera window closed.", flush=True)


def start_debug_camera(shared_data):
    """Launch the debug viewer on its own daemon thread so the cv2.imshow loop
    never blocks the control loop. Returns the thread."""
    thread = threading.Thread(target=_debug_camera_loop, args=(shared_data,), daemon=True)
    thread.start()
    return thread


if __name__ == "__main__":
    # Standalone: spin up our own components and run the viewer on the main thread.
    from runtime.setup import setup_components

    SIM_SERVER_UDP_IP = "127.0.0.1"
    SIM_SERVER_UDP_PORT = 14550

    system_boot_ms = int(time.time() * 1000)
    shared_data = {"running": True}

    setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)

    print("Starting live camera debug view. Press Q or ESC to quit.", flush=True)
    _debug_camera_loop(shared_data)
