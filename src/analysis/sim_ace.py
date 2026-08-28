"""
Offline closed-loop simulation of ace_pilot: the REAL pilot code (imported, not reimplemented)
flying against the flight-validated dynamics, with synthesized latency-correct vision.

    python -m analysis.sim_ace [--noise] [--t0 0.265] [--plot]

Truth dynamics = the validated models (45-deg per-axis clamp, 96 ms attitude lag, thrust-scaled
horizontal accel + two-term drag, thrust-model vertical + fitted vertical drag, ground contact).
Vision = pinhole projection of the map gates from the truth pose 150 ms ago at 30 Hz, all gates
in FOV (so wrong-gate anchoring is exercised), area consistent with ace's range proxy.

Reports per-gate pass/miss (+-0.61 m opening), minimum altitude (ground scrape check), lap
time. This is where laps get solved from now on - the sim only confirms.
"""

import argparse
import json
import math
import os
import sys
import types

# stub the keyboard module BEFORE the pilot imports it (headless run)
sys.modules.setdefault("keyboard", types.SimpleNamespace(is_pressed=lambda *_: False))

from common.camera import HALF_TAN_X, HALF_TAN_Y, HEIGHT, UPTILT_RAD, WIDTH
import pilots.ace_pilot.ace_pilot as ap
from pilots.ace_pilot.config import ACE_C_RNG

G = 9.81
CLAMP = math.radians(45.0)
ATT_TAU = 0.096
YAW_TAU = 0.15
DRAG_C1, DRAG_C2 = 0.1141, 0.0192
C1V, C2V = 0.35, 0.010
VIS_LAT_S = 0.15
FPS = 30.0
TICK = 1.0 / 90.0


class Det:
    __slots__ = ("offset_x", "offset_y", "area_frac", "has_opening", "bbox")

    def __init__(self, ox, oy, a, has_opening=False, true_ox=None, true_oy=None):
        # offset_x/y = STRUCTURE bbox centre (biased, like the real detector); when the opening
        # is located, bbox encodes the TRUE opening centre (what _aim recovers)
        self.offset_x, self.offset_y, self.area_frac = ox, oy, a
        self.has_opening = has_opening
        if has_opening:
            cx = (true_ox + 1.0) * WIDTH / 2.0
            cy = (true_oy + 1.0) * HEIGHT / 2.0
            self.bbox = (cx - 10, cy - 10, 20, 20)
        else:
            self.bbox = None


def synth_vision(gates, pose, rng=None):
    """Detections from a (possibly delayed) truth pose: all map gates in FOV."""
    x, y, z, psi, th = pose
    dets = []
    for g in gates:
        dx, dy, dz = g["x"] - x, g["y"] - y, g["z"] - z
        r = math.hypot(dx, dy)
        if not (1.0 < r < 45.0):
            continue
        bear = math.atan2(dy, dx) - psi
        bear = (bear + math.pi) % (2 * math.pi) - math.pi
        if abs(bear) > math.radians(44.0):
            continue
        el = math.atan2(dz, r)
        ox = -math.tan(bear) / HALF_TAN_X
        oy = math.tan(UPTILT_RAD - el - th) / HALF_TAN_Y
        if abs(ox) > 1.0 or abs(oy) > 1.0:
            continue
        a = (ACE_C_RNG / r) ** 2
        true_ox, true_oy = ox, oy
        # STRUCTURE-CENTRE bias (real run 115245): the raw detector centre sits off the opening
        # by a per-gate constant (up to ~1 m on the start gate); the opening itself is located
        # only ~60% of frames
        bias = 0.35 * (((g["gate_id"] * 2654435761) % 100) / 100.0 - 0.5)
        oy = oy + bias / max(r / 8.0, 0.7)
        has_open = (rng is None) or (rng.random() < 0.6)
        if rng is not None:
            # noise levels MEASURED from real run 120743: range jitter ~0.6%, offsets ~0.005
            # (the old 10%/0.02 guesses were 3-15x too harsh and made strong anchors look
            # unstable offline when they are fine in reality)
            ox += rng.gauss(0.0, 0.006)
            oy += rng.gauss(0.0, 0.006)
            true_ox += rng.gauss(0.0, 0.005)
            true_oy += rng.gauss(0.0, 0.005)
            a *= 1.0 + rng.gauss(0.0, 0.02)
        dets.append(Det(ox, oy, max(a, 1e-5), has_open, true_ox, true_oy))
    return dets


def main():
    global VIS_LAT_S
    apar = argparse.ArgumentParser()
    apar.add_argument("--noise", action="store_true")
    apar.add_argument("--t0", type=float, default=0.265, help="truth hover collective")
    apar.add_argument("--plot", action="store_true")
    apar.add_argument("--tmax", type=float, default=60.0)
    apar.add_argument("--vislat", type=float, default=VIS_LAT_S)
    apar.add_argument("--thrtau", type=float, default=0.03,
                      help="truth thrust response lag (s); MEASURED ~0.02 on tab7 steps")
    apar.add_argument("--seed", type=int, default=7)
    apar.add_argument("--truthstate", action="store_true",
                      help="feed the pilot TRUTH state each tick (generation only): smooth "
                           "feedback commands with zero vision noise - the recordable tape")
    apar.add_argument("--tape-out", default=None,
                      help="write the flown command sequence (sim-time indexed) to this JSON - the open-loop race tape")
    apar.add_argument("--launchlag", type=float, default=0.4,
                      help="truth ignores commands this long after GO (spool-up; run 120743's early gap)")
    apar.add_argument("--liftgain", type=float, default=1.0,
                      help="truth vertical lift vs the pilot model (tape altimetry, attempt 6: reality flew +1.15 m high at gate 0 = ~1%% excess lift; the closed-loop generation then bakes the counter-trim into the tape)")
    apar.add_argument("--vscale", type=float, default=1.0,
                      help="truth horizontal response scale vs the model (run 120743: estimate ran ~40%% fast on VQ2 - the dynamics fits are VQ1's)")
    apar.add_argument("--visdrop", type=float, default=0.35,
                      help="fraction of frames with NO detection (real detector loses the gate ~50%% of launch)")
    apar.add_argument("--no-anchor", action="store_true")
    apar.add_argument("--prefix", default=None,
                      help="PREFIX LOCK: replay this tape's commands verbatim until --splicet, "
                           "then hand off to the follower. Gates a real flight passed keep their "
                           "exact bytes - rebuilds can never break them again.")
    apar.add_argument("--splicet", type=float, default=0.0)
    apar.add_argument("--dump-trail", default=None, help="write truth [t,x,y,z] samples to this JSON")
    args = apar.parse_args()
    VIS_LAT_S = args.vislat
    ap.ACE_MODE = "vision"   # the sim GENERATES tapes with the closed-loop follower; replaying
                             # the frozen tape.json here (config default "tape") made every
                             # iteration round identical and tested old commands vs new physics
    if args.no_anchor:
        ap.ACE_ANCHOR_GAIN_LAT = 0.0
        ap.ACE_ANCHOR_GAIN_Z = 0.0
        ap.ACE_ANCHOR_GAIN_RNG = 0.0
        ap.ACE_K_HOME = 0.0
        ap.ACE_VSCALE_K = 0.0
    rng = None
    if args.noise:
        import random
        rng = random.Random(args.seed)

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    gates = json.load(open(os.path.join(here, "pilots", "ace_pilot", "course_map.json")))["gates"]
    # race_offsets = our best belief of where REALITY's gates sit relative to the raw map -
    # the sim's truth must use the same belief, or it fights the reality layer
    ro_path = os.path.join(here, "pilots", "ace_pilot", "race_offsets.json")
    rg_floor = 0.0
    if os.path.exists(ro_path):
        ro = json.load(open(ro_path))
        rg = ro.get("global", [0, 0, 0])
        rg_floor = min(0.0, rg[2])
        rgg = {int(k): v for k, v in ro.get("gates", {}).items()}
        for g in gates:
            o = rgg.get(g["gate_id"], (0, 0, 0))
            g["x"] += rg[0] + o[0]
            g["y"] += rg[1] + o[1]
            g["z"] += rg[2] + o[2]

    # capture the pilot's commands instead of sending MAVLink
    cmd = {"roll": 0.0, "pitch": 0.0, "yaw": math.radians(97.1), "thr": 0.0}

    def fake_send(conn, boot, roll, pitch, yaw, thrust):
        cmd["roll"], cmd["pitch"], cmd["yaw"], cmd["thr"] = roll, pitch, yaw, thrust

    ap.send_attitude_setpoint = fake_send
    ap.send_rate_attitude = lambda *a, **k: None

    # truth state
    thr_act = 0.0            # actual thrust follows command through a motor lag
    import random as _rnd
    drop_rng = _rnd.Random(13)
    prefix_cmds = json.load(open(args.prefix))["commands"] if args.prefix else None
    ppi = 0
    px = py = pz = 0.0
    vx = vy = vz = 0.0
    th_t = ph_t = 0.0
    psi_t = math.radians(97.1)
    pose_hist = []          # (t, pose) for vision latency
    data = {"running": True,
            "race_status": {"race_start_boot_time_ms": 0, "sim_boot_time_ms": 1,
                            "race_finish_time_ns": -1}}
    t = 0.0
    tape = []
    frame_id = 0
    next_frame_t = 0.0
    trail = []
    min_alt_after_launch = 1e9
    passes = {}
    prev_along = {}
    while t < args.tmax:
        # ---- sensors ----
        data["highres_imu"] = {"time_usec": int(t * 1e6)}
        if t >= next_frame_t:
            next_frame_t += 1.0 / FPS
            frame_id += 1
            tpast = t - VIS_LAT_S
            pose = pose_hist[0][1] if pose_hist else (px, py, pz, psi_t, th_t)
            for tt, pp in pose_hist:
                if tt <= tpast:
                    pose = pp
                else:
                    break
            data["latest_frame_id"] = frame_id
            if drop_rng.random() < args.visdrop:
                data["vision_gates"] = []          # detector missed this frame (reality does)
            else:
                data["vision_gates"] = synth_vision(gates, pose, rng)
        # ---- pilot tick (the real code) ----
        if args.truthstate and data.get("_ace_live"):
            data["_ace_x"], data["_ace_y"], data["_ace_alt"] = px, py, pz
            data["_ace_vx"], data["_ace_vy"], data["_ace_vz"] = vx, vy, vz
            data["_ace_psi_est"] = psi_t
            data["_ace_th_lag"], data["_ace_ph_lag"] = th_t, ph_t
            data["vision_gates"] = []          # no vision anywhere in truth-state generation
        ap.update_ace_control(None, 0, data)
        if prefix_cmds is not None and t < args.splicet:
            # half-tick tolerance: recorded times are rounded (0.45556 -> 0.4556), a bare
            # <= t skips the same-tick row and replays one tick late
            while ppi < len(prefix_cmds) - 1 and prefix_cmds[ppi + 1][0] <= t + 0.5 / 90.0:
                ppi += 1
            _, cmd["roll"], cmd["pitch"], cmd["yaw"], cmd["thr"] = prefix_cmds[ppi]
            last_prefix_cmd = dict(cmd)
        elif prefix_cmds is not None and t < args.splicet + 2.2 and 'last_prefix_cmd' in dir():
            # SPLICE BLEND: the follower's first post-splice corrections are a step-response
            # SLAM (-25 deg nose-up / -33 roll / thrust chop in one tick, burned into every
            # tape). Slew-limit attitude (90 deg/s) and thrust (0.5/s) away from the last
            # banked command for 0.7 s so the handoff is a smooth carve, never a spike.
            mx_a = math.radians(35.0) * TICK
            mx_t = 0.5 * TICK
            for k, mx in (("roll", mx_a), ("pitch", mx_a), ("thr", mx_t)):
                d = cmd[k] - last_prefix_cmd[k]
                last_prefix_cmd[k] += max(-mx, min(mx, d))
                cmd[k] = last_prefix_cmd[k]
        if args.tape_out is not None:
            tape.append([round(t, 4), round(cmd["roll"], 5), round(cmd["pitch"], 5),
                         round(cmd["yaw"], 5), round(cmd["thr"], 5)])
        # ---- truth dynamics ----
        if t < args.launchlag:
            cmd_roll_eff, cmd_pitch_eff, cmd_thr_eff = 0.0, 0.0, 0.0
        else:
            cmd_roll_eff, cmd_pitch_eff, cmd_thr_eff = cmd["roll"], cmd["pitch"], cmd["thr"]
        r_c = max(-CLAMP, min(CLAMP, cmd_roll_eff))
        p_c = max(-CLAMP, min(CLAMP, cmd_pitch_eff))
        a_att = 1.0 - math.exp(-TICK / ATT_TAU)
        th_t += (p_c - th_t) * a_att
        ph_t += (r_c - ph_t) * a_att
        dpsi = (cmd["yaw"] - psi_t + math.pi) % (2 * math.pi) - math.pi
        psi_t += dpsi * (1.0 - math.exp(-TICK / YAW_TAU))
        thr_act += (cmd_thr_eff - thr_act) * (1.0 - math.exp(-TICK / max(args.thrtau, 1e-3)))
        tw = G * (max(thr_act, 0.0) / args.t0)
        af = tw * math.cos(ph_t) * math.sin(th_t)
        ar = tw * math.cos(th_t) * math.sin(ph_t)
        fwd = (math.cos(psi_t), math.sin(psi_t))
        right = (math.sin(psi_t), -math.cos(psi_t))
        sp = math.hypot(vx, vy)
        drag = DRAG_C1 + DRAG_C2 * sp
        vx += (args.vscale * (af * fwd[0] + ar * right[0]) - drag * vx) * TICK
        vy += (args.vscale * (af * fwd[1] + ar * right[1]) - drag * vy) * TICK
        ge = 1.0 + 0.05 * max(0.0, 1.0 - pz / 3.0)  # ground cushion ADDS lift (modest gain
                                                     # near ground vs 0.26 at altitude)
        a_up = (args.liftgain * ge * G * (max(thr_act, 0.0) / args.t0) ** 1.45
                * math.cos(th_t) * math.cos(ph_t) - G
                - (C1V + C2V * abs(vz)) * vz)
        vz += a_up * TICK
        px += vx * TICK
        py += vy * TICK
        pz += vz * TICK
        floor_z = 0.0 if t < 2.0 else rg_floor   # pad is ELEVATED; arena floor rides the
                                                  # reality z belief (attempt 4: gates sit
                                                  # ~0.8 below the pad plane)
        if pz <= floor_z:           # ground contact
            pz = floor_z
            if vz < 0.0:
                vz = 0.0
            vx *= 0.5               # ground friction kills horizontal motion
            vy *= 0.5
        if t > 2.0:
            min_alt_after_launch = min(min_alt_after_launch, pz)
            if pz <= floor_z + 0.005:
                print(f"CRASH: floor contact at t={t:.2f} pos=({px:+.1f},{py:+.1f}) "
                      f"v={math.hypot(vx, vy):.1f} m/s pitch_cmd={math.degrees(cmd['pitch']):+.0f}")
                break
        pose_hist.append((t, (px, py, pz, psi_t, th_t)))
        if len(pose_hist) > 40:
            pose_hist.pop(0)
        trail.append((t, px, py, pz))
        # (dump handled after the loop)
        # estimate-vs-truth divergence (the sim's superpower: truth is known)
        ex_ = data.get("_ace_x", 0.0) - px
        ey_ = data.get("_ace_y", 0.0) - py
        ez_ = data.get("_ace_alt", 0.0) - pz
        div = math.hypot(ex_, ey_)
        if "div_log" not in data:
            data["div_log"] = []
        data["div_log"].append((t, div, ez_))
        # ---- gate pass check: crossing the gate plane inside the +-0.61 m opening ----
        for g in gates:
            gid = g["gate_id"]
            if gid in passes:
                continue
            cd = g["cross_dir"]
            along = (px - g["x"]) * cd[0] + (py - g["y"]) * cd[1]
            pa = prev_along.get(gid)
            prev_along[gid] = along
            if pa is not None and pa < 0.0 <= along and abs(along - pa) < 2.0:
                perp = -(px - g["x"]) * cd[1] + (py - g["y"]) * cd[0]
                dz = pz - g["z"]
                if abs(perp) > 3.0:
                    continue        # crossed this gate's infinite plane from elsewhere - not an attempt
                ok = abs(perp) <= 0.61 and abs(dz) <= 0.61
                passes[gid] = (t, perp, dz, ok)
        t += TICK
        if (len(gates) - 1) in passes:
            break                      # finish gate crossed - the race clock stops here
        if len(passes) == len(gates) and all(p[3] for p in passes.values()):
            break

    dl = data.get("div_log", [])
    if dl:
        dmax = max(dl, key=lambda r: r[1])
        print(f"est-truth divergence: max horiz {dmax[1]:.2f} m at t={dmax[0]:.1f}; "
              f"per-second: " + " ".join(f"{int(tt)}s:{d:.1f}" for tt, d, _ in dl[::90]))
        print("vertical est-truth (est z - true z) per second: "
              + " ".join(f"{int(tt)}s:{e:+.1f}" for tt, _, e in dl[::90]))
    if args.dump_trail:
        json.dump([[round(a, 3) for a in row] for row in trail], open(args.dump_trail, "w"))
    n_ok = sum(1 for p in passes.values() if p[3])
    print(f"\nsim result: {n_ok}/{len(gates)} gates passed clean, "
          f"{len(passes)} planes crossed, t_end={t:.1f}s, min alt after launch {min_alt_after_launch:.2f} m")
    for gid in sorted(passes):
        tt, perp, dz, ok = passes[gid]
        print(f"  g{gid:2d} t={tt:5.1f}  lateral {perp:+5.2f}  vertical {dz:+5.2f}  "
              f"{'PASS' if ok else 'MISS'}")
    missed_planes = [g["gate_id"] for g in gates if g["gate_id"] not in passes]
    if missed_planes:
        print("  planes never crossed:", missed_planes)

    if args.tape_out is not None:
        json.dump({"columns": ["t", "roll", "pitch", "yaw", "thrust"], "commands": tape},
                  open(args.tape_out, "w"))
        print(f"tape -> {args.tape_out} ({len(tape)} commands, {tape[-1][0]:.1f}s)")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(15, 7))
        axes[0].plot([p[1] for p in trail], [p[2] for p in trail], "r-", lw=1)
        axes[0].scatter([g["x"] for g in gates], [g["y"] for g in gates], c="g", s=50)
        axes[0].set_title("x-y")
        axes[0].set_aspect("equal", adjustable="datalim")
        axes[1].plot([p[0] for p in trail], [p[3] for p in trail], "r-", lw=1)
        axes[1].scatter([passes[g][0] for g in sorted(passes)],
                        [gates[g]["z"] for g in sorted(passes)], c="g", s=40)
        axes[1].set_title("alt vs t")
        out = os.path.join(here, "datasets", "sim_ace_flight.png")
        fig.tight_layout()
        fig.savefig(out, dpi=110)
        print("plot ->", out)


if __name__ == "__main__":
    main()
