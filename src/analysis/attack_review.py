"""
Deterministic VISUAL replay of an ATTACK-pilot flight. Feeds the RECORDED session (real camera JPGs,
real per-frame detections, real HIGHRES_IMU stream, real race_status) through the REAL attack pilot
(pilots.attack_pilot.update_attack_control) at the recorded cadence - IMU records drive control ticks
exactly like the live loop, frames land between them - and paints onto each JPG what the pilot saw and
decided:

  * every detection's box (amber) with area-fraction + rough range; the TRACKED gate green (thick)
  * the smoothed aim point (cyan cross), the post-pass AVOID zone (red ellipse)
  * the estimated artificial horizon (grey line from the IMU attitude filter - if this line is wrong,
    every world-frame bearing is wrong)
  * a panel with the full control state: az/el, az/el rates, zgyro, commit, stale, roll raw/slewed,
    yaw command, climb, thrust
  * roll + yaw bars along the bottom, red when pinned at the cap

Because it runs the pilot's own tracker / pass / servo code (not a re-implementation), the overlay is
a faithful reconstruction - and after a code change, re-running it over an OLD session shows what the
NEW code would have decided on the same inputs (open-loop: the trajectory doesn't change, the
decisions do). The pilot's live per-tick CSV (attack_dbg_*.csv) is the numeric twin of this view.

Usage:
    python -m analysis.attack_review                        # newest session -> attack_review.mp4
    python -m analysis.attack_review <session_dir>
    python -m analysis.attack_review <session_dir> --png 40 120    # also dump PNGs, seq 40..120
    python -m analysis.attack_review <session_dir> --no-mp4 --png 40 120
"""

import glob
import json
import math
import os
import sys

import cv2
import numpy as np

from common.camera import WIDTH as W, HEIGHT as H, FY, UPTILT_RAD
from common.paths import DATASETS_DIR
from perception.gate_detection import GateDetection
import pilots.attack_pilot.attack_pilot as ap
from pilots.attack_pilot.config import ATK_PITCH_FWD, ATK_AVOID_RADIUS

# The pilot polls the keyboard and writes a debug CSV live; neither is wanted in a replay.
ap.keyboard.is_pressed = lambda *a, **k: False
ap._dbg_log = lambda row: None

AMBER = (0, 190, 255)
GREEN = (0, 230, 0)
CYAN = (230, 230, 0)
GREY = (150, 150, 150)
RED = (0, 0, 255)
WHITE = (255, 255, 255)


class _Mav:
    def __getattr__(self, _):            # absorb any mav.<whatever>_send the senders call
        return lambda *a, **k: None


class _FakeConn:
    def __init__(self):
        self.target_system = self.target_component = 1
        self.mav = _Mav()


def _px(offx, offy):
    """image-offset (-1..1) -> pixel (x, y)."""
    return int((offx + 1.0) * W / 2.0), int((offy + 1.0) * H / 2.0)


def _dets(rec):
    """Reconstruct the GateDetection list the pilot saw this frame, nearest (biggest) first."""
    out = []
    for d in rec.get("dets", []):
        out.append(GateDetection(
            offset_x=d["offx"], offset_y=d["offy"],
            area=d.get("area_frac", 0.0) * W * H,
            distance_m=d.get("pinhole_dist") or 0.0,
            area_frac=d.get("area_frac", 0.0),
            has_opening=d.get("has_opening", False),
            bbox=tuple(d["bbox"]) if d.get("bbox") else None,
        ))
    out.sort(key=lambda g: g.area, reverse=True)
    return out


def _panel(img, lines):
    """Semi-transparent text panel, top-left."""
    pad, lh = 6, 15
    box = img.copy()
    cv2.rectangle(box, (0, 0), (262, pad * 2 + lh * len(lines)), (0, 0, 0), -1)
    cv2.addWeighted(box, 0.55, img, 0.45, 0, img)
    for i, (txt, col) in enumerate(lines):
        cv2.putText(img, txt, (pad, pad + lh * (i + 1) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)


def _horizon(img, roll, pitch):
    """Grey artificial-horizon line from the pilot's ESTIMATED attitude. The world horizon sits at
    camera-down angle (uptilt + pitch) below centre and rotates by -roll in the image."""
    down = math.tan(UPTILT_RAD + pitch) * FY           # px below image centre at roll 0
    cx, cy = W / 2.0, H / 2.0 + down
    c, s = math.cos(-roll), math.sin(-roll)
    x0, y0 = int(cx - 400 * c), int(cy - 400 * s)
    x1, y1 = int(cx + 400 * c), int(cy + 400 * s)
    cv2.line(img, (x0, y0), (x1, y1), GREY, 1, cv2.LINE_AA)


def annotate(img, dets, data, t_s, frame_id):
    cx = W // 2
    cv2.line(img, (cx, 0), (cx, H), GREY, 1)
    _horizon(img, data.get("_at_est_roll", 0.0), data.get("_at_est_pitch", 0.0))

    # which det is the tracked one: nearest to the pilot's raw lock position
    lock = data.get("_at_track_off")
    locked_i = -1
    if lock is not None and dets:
        lx, ly = lock
        locked_i = min(range(len(dets)),
                       key=lambda i: (dets[i].offset_x - lx) ** 2 + (dets[i].offset_y - ly) ** 2)

    for i, d in enumerate(dets):
        col = GREEN if i == locked_i else AMBER
        thick = 2 if i == locked_i else 1
        if d.bbox:
            x, y, bw, bh = d.bbox
            cv2.rectangle(img, (x, y), (x + bw, y + bh), col, thick)
        else:
            p = _px(d.offset_x, d.offset_y)
            cv2.circle(img, p, 6, col, thick)
        px, py = _px(d.offset_x, d.offset_y)
        cv2.putText(img, f"{d.area_frac:.3f} {d.distance_m:.0f}m", (px + 4, py - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv2.LINE_AA)

    # post-pass avoid zone (the flown-through gate the tracker must hand off from)
    avoid = data.get("_at_avoid_off")
    if avoid is not None:
        cv2.ellipse(img, _px(*avoid), (int(ATK_AVOID_RADIUS * W / 2), int(ATK_AVOID_RADIUS * H / 2)),
                    0, 0, 360, RED, 1)

    # smoothed AIM point (the opening, when located) the controller actually reads
    off = data.get("_at_off_smooth")
    if off is not None:
        cv2.drawMarker(img, _px(*off), CYAN, cv2.MARKER_CROSS, 16, 2)

    heading = data.get("_at_yaw", 0.0)
    pitch = data.get("_at_des_pitch", 0.0)
    gc = data.get("_at_gate_count", 0)
    stale = data.get("_at_stale", 0)
    ox, oy = data.get("_at_off", (0.0, 0.0))

    _panel(img, [
        (f"t={t_s:6.2f}s  frame {frame_id}  [{data.get('attack_regime', '?')}]", WHITE),
        (f"gates passed: {gc}", GREEN if gc else WHITE),
        (f"n_dets={len(dets)}  area={data.get('_at_area', 0.0):.3f}  stale={stale}",
         RED if stale else WHITE),
        (f"aim ox={ox:+.3f}  oy={oy:+.3f}", WHITE),
        (f"commit={data.get('_at_commit', 0.0):.0f}", CYAN),
        (f"heading cmd={math.degrees(heading):+6.1f} deg", CYAN),
        (f"pitch cmd={math.degrees(pitch):+4.2f} deg  thr={data.get('_at_thrust', 0.0):.3f}", WHITE),
        (f"vz_est={data.get('_at_climb', 0.0):+4.1f}  est r/p="
         f"{math.degrees(data.get('_at_est_roll', 0.0)):+4.1f}/"
         f"{math.degrees(data.get('_at_est_pitch', 0.0)):+4.1f}", WHITE),
        (f"pass: armed={int(data.get('_at_pass_armed', False))} block={int(data.get('_at_pass_block', False))}",
         WHITE),
    ])

    # pitch bar along the bottom: centre = 0, full half-width = the creep cap
    by = H - 12
    bx0, half = 10, (W - 20) // 2
    cv2.line(img, (bx0, by), (bx0 + 2 * half, by), GREY, 1)
    cv2.line(img, (bx0 + half, by - 5), (bx0 + half, by + 5), GREY, 1)
    frac = max(-1.0, min(1.0, pitch / ATK_PITCH_FWD))
    cv2.line(img, (bx0 + half, by), (bx0 + half + int(frac * half), by), CYAN, 4)
    return img


def _newest_session():
    s = sorted(glob.glob(os.path.join(DATASETS_DIR, "session_*", "vision_frames.jsonl")),
               key=os.path.getmtime)
    return os.path.dirname(s[-1]) if s else None


def _load_events(session_dir):
    """Merge IMU + race_status (telemetry.jsonl) and vision frames into one epoch-ns-ordered stream."""
    events = []   # (epoch_ns, kind, payload)
    tj = os.path.join(session_dir, "telemetry.jsonl")
    if not os.path.exists(tj):
        raise SystemExit(f"{tj} missing - the replay needs the recorded IMU stream")
    for line in open(tj):
        r = json.loads(line)
        if r["kind"] == "highres_imu":
            events.append((r["recv_time_ns"], "imu", r))
        elif r["kind"] == "race_status":
            events.append((r["recv_time_ns"], "race", r))
    files = {}
    for line in open(os.path.join(session_dir, "frames.jsonl")):
        r = json.loads(line)
        files[r["frame_id"]] = os.path.join(session_dir, r["file"])
    n_img = 0
    for line in open(os.path.join(session_dir, "vision_frames.jsonl")):
        r = json.loads(line)
        if r["frame_id"] in files:
            r["_img"] = files[r["frame_id"]]
            n_img += 1
        events.append((r.get("recv_ns") or r["sim_time_ns"], "frame", r))
    if not n_img:
        raise SystemExit("no frames with both an image and a detection record")
    events.sort(key=lambda e: e[0])
    return events


def run(session_dir, make_mp4=True, png_range=None):
    events = _load_events(session_dir)
    data = {"running": True}
    conn = _FakeConn()

    writer = None
    if make_mp4:
        out_mp4 = os.path.join(session_dir, "attack_review.mp4")
        writer = cv2.VideoWriter(out_mp4, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (W, H))
    png_dir = None
    if png_range is not None:
        png_dir = os.path.join(session_dir, "review_png")
        os.makedirs(png_dir, exist_ok=True)

    t0 = events[0][0]
    n_frames = sum(1 for e in events if e[1] == "frame")
    print(f"=== attack_review: {os.path.basename(session_dir)}  "
          f"({n_frames} frames, {len(events)} events) ===", flush=True)

    seq = -1
    pending = None          # frame rec waiting for the next control tick before we paint it
    for t_ns, kind, rec in events:
        if kind == "race":
            data["race_status"] = {k: rec[k] for k in
                                   ("sim_boot_time_ms", "race_start_boot_time_ms", "race_finish_time_ns")}
            data["race_status"]["active_gate_index"] = rec.get("active_gate_index", 0)
        elif kind == "frame":
            seq += 1
            data["latest_frame_id"] = rec["frame_id"]
            data["latest_frame_seq"] = seq
            data["vision_gates"] = _dets(rec)
            rec["_seq"] = seq
            pending = rec
        elif kind == "imu":
            data["highres_imu"] = rec
            ap.update_attack_control(conn, 0, data)     # a real control tick, at the recorded cadence
            if pending is not None:                      # first tick AFTER a frame = that frame's decision
                img = cv2.imread(pending["_img"]) if "_img" in pending else None
                if img is not None:
                    if img.shape[1] != W or img.shape[0] != H:
                        img = cv2.resize(img, (W, H))
                    t_s = (t_ns - t0) * 1e-9
                    annotate(img, data.get("vision_gates") or [], data, t_s, pending["frame_id"])
                    if writer is not None:
                        writer.write(img)
                    if png_dir is not None and png_range[0] <= pending["_seq"] <= png_range[1]:
                        cv2.imwrite(os.path.join(
                            png_dir, f"seq{pending['_seq']:04d}_g{data.get('_at_gate_count', 0)}.png"), img)
                pending = None

    if writer is not None:
        writer.release()
        print(f"wrote {out_mp4}", flush=True)
    if png_dir is not None:
        print(f"wrote annotated PNGs (seq {png_range[0]}..{png_range[1]}) -> {png_dir}", flush=True)


def main():
    args = sys.argv[1:]
    make_mp4 = "--no-mp4" not in args
    args = [a for a in args if a != "--no-mp4"]
    png_range = None
    if "--png" in args:
        i = args.index("--png")
        png_range = (int(args[i + 1]), int(args[i + 2]))
        del args[i:i + 3]
    session = args[0] if args else _newest_session()
    if session is None:
        raise SystemExit("no session with vision_frames.jsonl")
    run(session, make_mp4=make_mp4, png_range=png_range)


if __name__ == "__main__":
    main()
