"""Carry the drone through a real gate, props off, and record what the
flight code WOULD have done.

    python3 -m hardware.walkthrough --port /dev/ttyTHS1 \
        --config ../config/ladder/vehicle_s15_cam20_75.toml \
        --fy 859 --cx 616.9 --cy 330.0 --cam-tilt 10 --tag level

PROPS OFF. This never calls set_rc, never arms, and needs no transmitter.
It opens the camera, reads attitude from the FC, and runs the real detector
and the real altitude loop over every frame.

WHY. Flight 4 on 2026-09-21 tracked gate 1 cleanly from 7.4 m to 2.5 m and
then climbed a metre into the top bar over the last 1.8 m. The vertical
reference comes from the gate's elevation, and that elevation comes from the
ring's centre row - which is wrong whenever the ring runs off the bottom of
the frame. Nothing about that failure needs a motor to reproduce: it is a
camera, a gate, and a range. So walk it instead of flying it.

WHAT YOU GET

  walk_<stamp>.avi   the raw capture, MJPG, to replay offline through
                     perception.video_probe as many times as you like
  walk_<stamp>.csv   one row per frame: the detection, the geometry it
                     implies, the commit decision, and the throttle the
                     altitude loop would have commanded
  walk_<stamp>/      lossless PNGs either side of every commit transition,
                     because those are the frames worth arguing about

Row N of the CSV is frame N of the video. They cannot drift apart - the loop
is synchronous, one frame in, one row out.

HOW TO WALK IT. One run is enough. Pass --gate-h with the PUBLISHED gate
centre height and the script turns the question around: instead of you
declaring what height you are at so it can grade the elevation, it reports
the height the elevation IMPLIES, and you check that against a tape.

    implied_h_m    = gate height - range*sin(elevation)
    implied_lat_m  = range*sin(azimuth), + is RIGHT of the gate centreline

So move it wherever you like - in, out, high, low, left, right - and those
two numbers have to follow you. Hold it on a tape mark for a second or two
and read them off. That is the whole test, and it does not care where you
stand.

Still do these four things during the run, because each one catches a
different lie:

  1. WALK IN LEVEL at gate centre height, 8 m to through the gate. The
     operational case, and the one that failed in the air. implied_h_m
     should hold at the gate height the whole way in.
  2. HOLD STILL at about 5 m and raise and lower it 0.5 m. implied_h_m must
     follow, in the right direction and by the right amount. A detector
     stuck at the null passes step 1 and fails here - which is the point.
  3. STEP SIDEWAYS 1 m either side of the centreline. implied_lat_m should
     read about -1.0 and +1.0.
  4. TILT IT - roll and pitch it while it sits still. implied_h_m must NOT
     move: the gate has not moved, and the attitude compensation exists to
     take exactly this out. If it drifts with tilt, gate_dz has a sign or
     axis wrong and no amount of flying will fix it.

Keep it LEVEL for steps 1 and 3 - body pitch lands in the elevation one
degree for one degree. The FC attitude is logged so you can check afterwards
rather than trusting your hands.

--lens-h is optional and only for the case where you DO hold one fixed
height start to finish: it adds el_true_deg and el_err_deg so the grading is
done for you.

A/B THE FIX. The detector reconstructs the centre of a vertically clipped
ring from its width (AIGP_GATE_UNCLIP, on by default). To see what that is
buying you, replay one recording both ways and diff the offset:

    python -m perception.video_probe walk_XXX.avi --fy 859 --csv on.csv
    AIGP_GATE_UNCLIP=0 python -m perception.video_probe walk_XXX.avi --fy 859 --csv off.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import time
from pathlib import Path

GATE_OUTER_M = 2.7      # the ring is 2.7 m square; range divides the WIDTH by it


# THE VISION HEIGHT REFERENCE. These MUST match solvers/follower.py - they are
# the filter the aircraft actually flies. They are printed at startup so you
# can eyeball them against the follower before believing a single row.
VERT_EL_GAIN = 0.06      # m of height correction per degree of elevation
VERT_DZ_MAX = 0.35       # hard cap on that correction, m
VERT_DZ_SLEW = 0.35      # m/s the reference may move (a flicker must not move it)
VERT_EL_STALE_S = 0.5    # a detection older than this is not used


def _f(v, nd=3):
    return "" if v is None else f"{v:.{nd}f}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1",
                    help="FC serial port for attitude. 'none' runs camera-only, which "
                         "holds R level and cannot take body pitch out of the elevation")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", default=None, help="GStreamer pipeline or /dev/videoN")
    ap.add_argument("--config", default=None, help="vehicle toml: thrust curve and gains")
    ap.add_argument("--fy", type=float, required=True, help="from camcal_board")
    ap.add_argument("--cx", type=float, default=None, help="principal point col, from camcal_board")
    ap.add_argument("--cy", type=float, default=None, help="principal point row, from camcal_board")
    ap.add_argument("--cam-tilt", type=float, default=10.0, help="mount up-tilt, deg, from camtilt")
    ap.add_argument("--gate-h", type=float, default=None,
                    help="PUBLISHED gate centre height, m (1.35 on the FLAT plan). With "
                         "--lens-h this gives el_true and el_err per frame, so the test has "
                         "ground truth instead of a judgement call")
    ap.add_argument("--lens-h", type=float, default=None,
                    help="height you are CARRYING the lens at, m. Measure it, do not guess")
    ap.add_argument("--seconds", type=float, default=120.0, help="Ctrl+C to stop earlier")
    ap.add_argument("--out", default=None, help="output directory (default ../out/walkthrough)")
    ap.add_argument("--tag", default=None, help="name this run: level / low50 / high50")
    ap.add_argument("--no-video", action="store_true", help="CSV only, no recording")
    ap.add_argument("--png-around-commit", type=int, default=8,
                    help="lossless PNGs either side of a commit transition (0 = off)")
    args = ap.parse_args(argv)

    # The detector reads the principal point from the environment, per call.
    if args.cx is not None:
        os.environ["AIGP_CAM_CX"] = str(args.cx)
    if args.cy is not None:
        os.environ["AIGP_CAM_CY"] = str(args.cy)
    if args.config:
        os.environ["AIGP_VEHICLE_TOML"] = str(Path(args.config))

    import cv2
    import numpy as np
    from hardware.hover import gate_azimuth, gate_dz
    from hardware.runtime import DEFAULT_PIPELINE
    from perception.detectors.hsv_classic import gate_mask
    from perception.gate_detection import (COMMIT_FRAC, UNCLIP_V, commit_reason,
                                           mask_to_detections)
    from raceline.config import load_config
    from raceline.rc_backend import AltitudeLoop, StateEstimate

    cfg = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
    alt = AltitudeLoop(cfg)
    tilt_rad = math.radians(args.cam_tilt)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"walk_{stamp}" + (f"_{args.tag}" if args.tag else "")
    out_dir = Path(args.out) if args.out else Path(__file__).resolve().parents[2] / "out" / "walkthrough"
    out_dir.mkdir(parents=True, exist_ok=True)
    png_dir = out_dir / name
    csv_path, avi_path = out_dir / f"{name}.csv", out_dir / f"{name}.avi"

    print(f"vision height filter: gain {VERT_EL_GAIN} m/deg, cap +-{VERT_DZ_MAX} m, "
          f"slew {VERT_DZ_SLEW} m/s, stale {VERT_EL_STALE_S} s")
    print("   -> CHECK these against solvers/follower.py before trusting a row")
    print(f"commit: ring height >= {COMMIT_FRAC:.2f} of frame, or both edges clipped. "
          f"unclip reconstruction {'ON' if UNCLIP_V else 'OFF'}")

    # Attitude. Without it R is the identity and every elevation is measured
    # against a level aircraft that may not be level - see hardware.camtilt.
    src = br = None
    if args.port not in (None, "none") or args.tcp:
        from hardware.bridge import FcBridge
        from hardware.state import FcStateSource
        br = FcBridge.open(port=(None if args.port == "none" else args.port),
                           tcp=args.tcp, baud=args.baud)
        br.start()
        src = FcStateSource(br)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and br.state().attitude is None:
            time.sleep(0.05)
        if br.state().attitude is None:
            print("ERROR no attitude from the FC after 10 s")
            br.stop()
            return 2
        print("attitude: live from the FC")
    else:
        print("WARNING no --port: R held level. Body pitch is NOT taken out of the "
              "elevation and lands in it one degree for one degree.")

    pipeline = args.camera or DEFAULT_PIPELINE
    cap = cv2.VideoCapture(pipeline, cv2.CAP_V4L2 if pipeline.startswith("/dev/video")
                           else cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"ERROR camera did not open: {pipeline}")
        if br is not None:
            br.stop()
        return 3

    COLS = ["frame", "t",
            "roll_deg", "pitch_deg", "yaw_deg",
            "n_blobs", "off_x", "off_y", "area_frac", "rw_px", "rh_px", "rh_frac",
            "clipped_v", "v_usable", "has_opening", "range_m",
            "el_deg", "el_true_deg", "el_err_deg", "az_deg", "dz_raw_m",
            "implied_h_m", "implied_lat_m",
            "commit", "commit_why",
            "dz_vis_m", "z_est", "vz_est", "z_target", "a_cmd", "thrust_cmd", "throttle_pwm",
            "cam_fps"]
    fh = open(csv_path, "w", newline="")
    w = csv.writer(fh)
    w.writerow(COLS)

    writer, identity_R = None, np.eye(3)
    dz_vis, t_dz = 0.0, None        # the slew-limited reference, the follower's filter
    frame_i = n_det = n_win = 0
    fps, t_win = 0.0, time.monotonic()
    last_commit, recent, last_print = None, [], 0.0
    # THE VERDICT: DRIFT, NOT ABSOLUTE VALUE.
    #
    # You cannot hand-hold the drone at exactly gate height, and you do not
    # need to. Carry it 0.1 m low and the camera SHOULD report a small
    # positive elevation and a small positive dz - that is the system working,
    # not failing. An absolute threshold on dz would fail a correct system for
    # a steady hand being 4 inches off.
    #
    # Flight 4 did not fail by holding a constant offset. It failed by the
    # offset GROWING as the range closed: dz read +0.09 at 8.2 m and +0.33 at
    # 9.4 s, pinned at its +0.35 cap, and she climbed ~1.05 m into the top bar.
    # A clipped ring reads MORE wrong the closer you get, so the signature is a
    # RUNAWAY, and a runaway is visible whatever height you started at.
    #
    # So: sample the command far out (ring clean, trustworthy) and again just
    # before commit, and report the difference. Near minus far is the height
    # the approach would have talked itself into moving. Hold it steady - any
    # steady height - and that number is honest.
    samples = []          # (range_m, dz_vis, implied_h) while the reference is LIVE
    commit_rng, commit_why_first, dz_at_commit = None, None, None
    hit_cap = False
    t0 = time.monotonic()
    print(f"\nrecording -> {csv_path}\n")
    print(f"{'t':>6} {'rng':>6} {'rh%':>5} {'el':>7} {'impl_h':>7} {'lat':>6} {'dz':>6} {'thr':>5}  state")
    try:
        while time.monotonic() - t0 < args.seconds:
            ok, bgr = cap.read()
            if not ok or bgr is None:
                time.sleep(0.005)
                continue
            t = time.monotonic()
            h, wpx = bgr.shape[:2]
            if writer is None and not args.no_video:
                writer = cv2.VideoWriter(str(avi_path), cv2.VideoWriter_fourcc(*"MJPG"),
                                         30.0, (wpx, h))
            frame_i += 1
            n_win += 1
            if t - t_win >= 1.0:
                fps, t_win, n_win = n_win / (t - t_win), t, 0

            dets = mask_to_detections(gate_mask(bgr), bgr.shape)
            det = dets[0] if dets else None
            if det is not None:
                n_det += 1
                box = det.ring_bbox or det.bbox
                # pinhole on the WIDTH: the real gate carries a header board on
                # top, so its pixel HEIGHT is not the ring's size (DVR, 2026-09-16)
                det.range_m = (args.fy * GATE_OUTER_M / float(box[2])
                               if box is not None and box[2] > 4 else None)
                det.t = t

            # attitude, and the state the altitude loop closes on
            roll_d = pitch_d = yaw_d = None
            R, est = identity_R, None
            if src is not None:
                est = src.estimate()
                a = br.state().attitude
                if a is not None:
                    roll_d, pitch_d = a.roll_deg, a.pitch_deg
                    yaw_d = getattr(a, "yaw_deg", None)
                if est is not None:
                    R = est.R
            if est is None:
                est = StateEstimate(p=np.zeros(3), v=np.zeros(3), R=identity_R,
                                    yaw=0.0, omega=np.zeros(3))

            el_deg = az_deg = dz_raw = None
            if det is not None:
                dz_raw, el = gate_dz(det, R, args.fy, tilt_rad, (wpx, h))
                az = gate_azimuth(det, args.fy, tilt_rad, (wpx, h))
                el_deg = math.degrees(el) if el is not None else None
                az_deg = math.degrees(az) if az is not None else None

            # GROUND TRUTH, from the published gate height. The gate centre is
            # a known constant (1.35 m on the FLAT plan) and you measured the
            # height you are carrying it at, so the elevation the camera OUGHT
            # to report is asin(dz / slant range). That range comes from the
            # ring's WIDTH, which is a horizontal measurement - independent of
            # the vertical channel this test is judging. It is the camera's
            # weakest signal, so el_true is soft at long range and tight up
            # close, which is the half of the walk that matters.
            el_true = el_err = None
            if (args.gate_h is not None and args.lens_h is not None
                    and det is not None and det.range_m):
                ratio = (args.gate_h - args.lens_h) / det.range_m
                el_true = math.degrees(math.asin(max(-1.0, min(1.0, ratio))))
                if el_deg is not None:
                    el_err = el_deg - el_true

            # WHERE THE CAMERA THINKS IT IS. Turn the answer around: instead of
            # you telling the script what height you are at so it can grade the
            # elevation, the script says what height the elevation IMPLIES and
            # you check it against a tape. That frees the test from a fixed
            # carry height - move it anywhere, in, out, up, down, sideways, and
            # this number has to follow you.
            #
            #   implied height  = gate height - range*sin(elevation)
            #   implied lateral = range*sin(azimuth), + is RIGHT of the gate
            #
            # Both lean on range, which is the camera's weakest signal - but
            # they lean on it GENTLY: the range multiplies a small sine, so a
            # 3 % range error moves a 0.5 m offset by 15 mm. The error that
            # matters here is in the angle, and the angle is what we are
            # testing. Lateral assumes the nose is on the gate; it is a
            # cross-check, not a survey.
            implied_h = implied_lat = None
            if det is not None and det.range_m is not None:
                if dz_raw is not None and args.gate_h is not None:
                    implied_h = args.gate_h - dz_raw
                if az_deg is not None:
                    implied_lat = det.range_m * math.sin(math.radians(az_deg))

            committed, why = commit_reason(det, bgr.shape)

            # THE REFERENCE THE AIRCRAFT WOULD FLY, by the follower's own rule:
            # use the elevation only while it is usable, cap it, and SLEW it.
            # Losing the gate walks the reference back to zero at the same rate
            # instead of dropping it, so a dropout fades into a vertical speed
            # hold rather than stepping the loop.
            want = 0.0
            if det is not None and not committed and el_deg is not None:
                want = max(-VERT_DZ_MAX, min(VERT_DZ_MAX, VERT_EL_GAIN * el_deg))
            dt = 0.02 if t_dz is None else max(0.0, min(0.1, t - t_dz))
            t_dz = t
            step = VERT_DZ_SLEW * dt
            dz_vis += max(-step, min(step, want - dz_vis))

            if det is not None and det.range_m is not None:
                if committed:
                    if commit_rng is None:
                        commit_rng, commit_why_first, dz_at_commit = det.range_m, why, dz_vis
                else:
                    samples.append((det.range_m, dz_vis, implied_h))
                    if abs(dz_vis) >= VERT_DZ_MAX - 1e-6:
                        hit_cap = True

            z_target = float(est.p[2]) + dz_vis
            # airborne=True on purpose: you are carrying it, and the open-loop
            # takeoff branch would otherwise mask the whole vertical channel
            thr = alt.throttle(t, est, z_target, 0.0, True, True)

            rw = rh = rh_frac = None
            if det is not None and det.ring_bbox is not None:
                _, _, rw, rh = det.ring_bbox
                rh_frac = rh / float(h)

            w.writerow([frame_i, f"{t - t0:.3f}",
                        _f(roll_d), _f(pitch_d), _f(yaw_d),
                        len(dets),
                        _f(det.offset_x if det else None, 4),
                        _f(det.offset_y if det else None, 4),
                        _f(det.area_frac if det else None, 5),
                        rw if rw is not None else "", rh if rh is not None else "", _f(rh_frac, 4),
                        int(det.clipped_v) if det else "", int(det.v_usable) if det else "",
                        int(det.has_opening) if det else "",
                        _f(det.range_m if det else None),
                        _f(el_deg), _f(el_true), _f(el_err), _f(az_deg), _f(dz_raw),
                        _f(implied_h), _f(implied_lat),
                        int(committed), why,
                        f"{dz_vis:.4f}", f"{float(est.p[2]):.3f}", f"{float(est.v[2]):.3f}",
                        f"{z_target:.3f}", f"{alt.a_cmd:.3f}", f"{alt.thrust:.3f}", thr,
                        f"{fps:.1f}"])

            if writer is not None:
                writer.write(bgr)

            # Lossless frames either side of a commit transition. JPEG is not a
            # fair witness to a three-pixel question about where a ring's edge was.
            if args.png_around_commit:
                if last_commit is not None and committed != last_commit:
                    png_dir.mkdir(parents=True, exist_ok=True)
                    for i_, f_ in recent[-args.png_around_commit:]:
                        cv2.imwrite(str(png_dir / f"f{i_:06d}.png"), f_)
                    recent = []
                recent.append((frame_i, bgr.copy()))
                if len(recent) > args.png_around_commit:
                    recent = recent[-args.png_around_commit:]
                last_commit = committed

            if t - last_print > 0.2:
                last_print = t
                if det is not None:
                    state = ("COMMIT:" + why if committed
                             else ("clipped" if det.clipped_v else "clean"))
                    err_s = (f"{el_err:+7.2f}" if el_err is not None
                             else (f"{implied_h:7.2f}" if implied_h is not None else "      -"))
                    lat_s = f"{implied_lat:+6.2f}" if implied_lat is not None else "     -"
                    print(f"\r{t - t0:6.1f} {det.range_m or 0:6.2f} {100 * (rh_frac or 0):5.1f} "
                          f"{el_deg:+7.2f} {err_s} {lat_s} "
                          f"{dz_vis:+6.2f} {thr:5d}  {state:18s}", end="", flush=True)
                else:
                    print(f"\r{t - t0:6.1f} {'no gate':>52s}{'':18s}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        fh.close()
        if br is not None:
            br.stop()

    print(f"\n\nframes {frame_i}, gate seen in {n_det} ({100.0 * n_det / max(frame_i, 1):.0f} %)")

    # ---- THE VERDICT: would she have flown through, or climbed into the bar?
    GATE_HALF_OPENING_M = 0.75      # 1.5 m opening
    DRIFT_PASS_M = 0.15             # a fifth of the half-opening
    F4_EXCURSION_M = 1.05           # d44 flight 4: 0.90 -> 1.95 m over the last 1.8 m
    print("\n" + "=" * 66)
    print("APPROACH VERDICT - flight 4 was a RUNAWAY, so we measure DRIFT")
    print("=" * 66)
    if len(samples) < 20:
        print(f"  NOT ENOUGH DATA: {len(samples)} live frames with a range. Walk it again "
              f"from ~8 m and keep the gate in view.")
    else:
        def med(xs):
            xs = sorted(x for x in xs if x is not None)
            return xs[len(xs) // 2] if xs else None
        near_lo = commit_rng if commit_rng else min(s[0] for s in samples)
        far = [s for s in samples if s[0] >= 6.0]
        near = [s for s in samples if s[0] <= near_lo + 1.5]
        if not far:
            far = sorted(samples, key=lambda s: -s[0])[:max(5, len(samples) // 5)]
        dz_far, dz_near = med([s[1] for s in far]), med([s[1] for s in near])
        h_far, h_near = med([s[2] for s in far]), med([s[2] for s in near])
        drift = dz_near - dz_far
        print(f"  commit fired at        {commit_rng:.2f} m   ({commit_why_first})"
              if commit_rng else "  commit NEVER fired - the reference stayed live all the way in")
        print(f"  dz commanded far out   {dz_far:+.3f} m   (median over {len(far)} frames beyond 6 m)")
        print(f"  dz commanded at commit {dz_near:+.3f} m   (median over {len(near)} frames)")
        if h_far is not None and h_near is not None:
            print(f"  height it believed     {h_far:.2f} m far out -> {h_near:.2f} m at commit")
        print(f"\n  DRIFT {drift:+.3f} m   <- the climb this approach would have flown")
        print(f"  flight 4 flew {F4_EXCURSION_M:+.2f} m and hit the top bar. "
              f"Gate half-opening is {GATE_HALF_OPENING_M:.2f} m.")
        bad = abs(drift) > DRIFT_PASS_M or hit_cap
        if hit_cap:
            print(f"\n  FAIL: dz reached its +-{VERT_DZ_MAX} m cap while the reference was live. "
                  f"That is the flight-4 signature exactly.")
        elif bad:
            print(f"\n  FAIL: drift exceeds {DRIFT_PASS_M:.2f} m. She would move {abs(drift):.2f} m "
                  f"vertically on the way in - do not fly this.")
        else:
            print(f"\n  PASS: the commanded height held within {DRIFT_PASS_M:.2f} m all the way to "
                  f"commit, and froze there. She flies through.")
        if commit_rng is None:
            print("  NOTE: with no commit the last metres ran on an unreconstructable ring. "
                  "Check you walked all the way through the gate.")
    print("=" * 66)
    print(f"  csv   {csv_path}")
    if writer is not None:
        print(f"  video {avi_path}")
    if png_dir.exists():
        print(f"  pngs  {png_dir}")
    print(f"\nnext, on the laptop:")
    print(f"  python -m perception.video_probe {avi_path} --fy {args.fy:.0f} --csv on.csv")
    print(f"  AIGP_GATE_UNCLIP=0 python -m perception.video_probe {avi_path} "
          f"--fy {args.fy:.0f} --csv off.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
