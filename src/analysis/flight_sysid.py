"""
BATCH SYSTEM IDENTIFICATION from recorded tape flights - the scientific loop.

    python -m analysis.flight_sysid <dbg.csv> <session_dir> [more pairs ...]

For each flight: simulate the tape's commands through a parameterized dynamics model
(theta = [accel_scale, drag_scale, lift_scale, spool_s, att_tau, yaw_tau]) and score the
simulated pose trajectory against EVERY recorded detection: predicted (bearing, elevation,
distance) of each map gate vs the observed opening (ox, oy, pnp_dist), associated by best
agreement. Nelder-Mead-ish coordinate search minimizes the total robust residual.

Outputs: fitted theta, residual breakdown per gate (map-error localization), and the
reconstructed trajectory (datasets/sysid_track_view.png overlay).

Deterministic sim + metric map + thousands of detections = overdetermined; no eyeballs.
"""

import csv
import json
import math
import os
import sys

from common.camera import HALF_TAN_X, HALF_TAN_Y

G = 9.81
UPTILT = math.radians(20.0)
T0, P = 0.265, 1.45
C1, C2 = 0.1141, 0.0192
C1V, C2V = 0.35, 0.010
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_flight(dbg_path, sess):
    tape_cmds = list(csv.DictReader(open(dbg_path)))
    cmds = []
    for r in tape_cmds:
        try:
            cmds.append((float(r["t"]), math.radians(float(r["roll_deg"])),
                         math.radians(float(r["pitch_deg"])), math.radians(float(r["yaw_deg"])),
                         float(r["thr"])))
        except (ValueError, KeyError):
            continue
    frames = [json.loads(l) for l in open(os.path.join(sess, "vision_frames.jsonl"))]
    # time alignment: dbg t=0 is race GO. Find GO's recv-time from the telemetry race_status
    # stream (sim_boot crosses race_start), then map to the frame clock via each frame's
    # recv_ns. (Aligning to the first frame was seconds off - the client records pre-GO.)
    go_recv = None
    for line in open(os.path.join(sess, "telemetry.jsonl")):
        r = json.loads(line)
        if r.get("kind") == "race_status":
            st = r.get("race_start_boot_time_ms", -1)
            now = r.get("sim_boot_time_ms", 0)
            if st is not None and st >= 0 and now >= st:
                go_recv = r["recv_time_ns"]
                break
    if go_recv is None:
        go_recv = frames[0].get("recv_ns") or frames[0].get("recv_time_ns")
    # frame sim-time at GO: interpolate from the frame whose recv is nearest GO
    fr_near = min(frames, key=lambda f: abs((f.get("recv_ns") or f.get("recv_time_ns")) - go_recv))
    t0_ns = fr_near["sim_time_ns"] - ((fr_near.get("recv_ns") or fr_near.get("recv_time_ns")) - go_recv)
    obs = []
    for f in frames:
        tf = (f["sim_time_ns"] - t0_ns) * 1e-9
        for d in f.get("dets") or []:
            if not (d.get("has_opening") and d.get("bbox") and d.get("pnp_dist")):
                continue
            if not (2.0 < d["pnp_dist"] < 30.0):
                continue
            bx, by, bw, bh = d["bbox"]
            ox = ((bx + bw / 2.0) - 320.0) / 320.0
            oy = ((by + bh / 2.0) - 180.0) / 180.0
            obs.append((tf, ox, oy, d["pnp_dist"]))
    return cmds, obs


def simulate(cmds, theta, t_end):
    a_s, d_s, l_s, spool, att_tau, yaw_tau = theta
    dt = 1.0 / 90.0
    px = py = pz = 0.0
    vx = vy = vz = 0.0
    th = ph = 0.0
    psi = math.radians(97.1)
    thr_act = 0.0
    i = 0
    out = []
    t = 0.0
    while t < t_end:
        while i < len(cmds) - 1 and cmds[i + 1][0] <= t:
            i += 1
        _, r_c, p_c, y_c, tr = cmds[i]
        eff = 0.0 if t < spool else tr
        thr_act += (eff - thr_act) * (1 - math.exp(-dt / 0.03))
        a = 1 - math.exp(-dt / att_tau)
        th += (p_c - th) * a
        ph += (r_c - ph) * a
        psi += ((y_c - psi + math.pi) % (2 * math.pi) - math.pi) * (1 - math.exp(-dt / yaw_tau))
        tw = a_s * G * thr_act / T0
        af = tw * math.cos(ph) * math.sin(th)
        ar = tw * math.cos(th) * math.sin(ph)
        sp = math.hypot(vx, vy)
        drag = d_s * (C1 + C2 * sp)
        vx += (af * math.cos(psi) + ar * math.sin(psi) - drag * vx) * dt
        vy += (af * math.sin(psi) + ar * -math.cos(psi) - drag * vy) * dt
        a_up = l_s * G * (max(thr_act, 0.0) / T0) ** P * math.cos(th) * math.cos(ph) - G \
            - (C1V + C2V * abs(vz)) * vz
        vz += a_up * dt
        px += vx * dt
        py += vy * dt
        pz = max(pz + vz * dt, 0.0)
        if pz == 0.0:
            vz = max(vz, 0.0)
        out.append((t, px, py, pz, psi, th))
        t += dt
    return out


def residual(traj, obs, gates, per_gate=None):
    """Robust total residual: for each observation, distance between the observed measurement
    and the best-matching map gate's prediction from the simulated pose at that time."""
    total = 0.0
    n = 0
    for tf, ox, oy, dist in obs[::3]:
        k = min(int(tf * 90), len(traj) - 1)
        _, px, py, pz, psi, th = traj[k]
        best = None
        best_g = None
        for g in gates:
            dx, dy, dz = g["x"] - px, g["y"] - py, g["z"] - pz
            rng = math.hypot(dx, dy)
            if not (1.5 < rng < 35.0):
                continue
            bear = (math.atan2(dy, dx) - psi + math.pi) % (2 * math.pi) - math.pi
            if abs(bear) > math.radians(50):
                continue
            ox_p = -math.tan(bear) / HALF_TAN_X
            el = math.atan2(dz, rng)
            oy_p = math.tan(UPTILT - th - el) / HALF_TAN_Y
            slant = math.hypot(rng, dz)
            e = math.hypot((ox - ox_p) * 8.0, (oy - oy_p) * 8.0) + abs(dist - slant) * 0.4
            if best is None or e < best:
                best = e
                best_g = g["gate_id"]
        if best is not None:
            r = min(best, 6.0)          # robust cap
            total += r
            n += 1
            if per_gate is not None and best_g is not None:
                per_gate.setdefault(best_g, []).append(best)
    return total / max(n, 1), n


def main():
    pairs = [(sys.argv[i], sys.argv[i + 1]) for i in range(1, len(sys.argv) - 1, 2)]
    gates = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "course_map.json")))["gates"]
    flights = [load_flight(d, s) for d, s in pairs]
    print(f"{len(flights)} flights, obs counts: {[len(o) for _, o in flights]}")

    theta = [1.0, 1.0, 1.0, 0.4, 0.096, 0.15]
    names = ["accel_scale", "drag_scale", "lift_scale", "spool_s", "att_tau", "yaw_tau"]
    steps = [0.1, 0.1, 0.03, 0.15, 0.05, 0.08]

    def score(th):
        tot = 0.0
        for cmds, obs in flights:
            t_end = min(cmds[-1][0], obs[-1][0] if obs else 0) + 0.5
            traj = simulate(cmds, th, t_end)
            r, n = residual(traj, obs, gates)
            tot += r
        return tot / len(flights)

    best = score(theta)
    print(f"initial residual {best:.3f}")
    for sweep in range(4):
        improved = False
        for j in range(len(theta)):
            for sgn in (+1, -1):
                cand = list(theta)
                cand[j] = max(0.01, cand[j] + sgn * steps[j])
                s = score(cand)
                if s < best - 1e-4:
                    theta, best = cand, s
                    improved = True
                    print(f"  sweep {sweep}: {names[j]} -> {theta[j]:.3f}  residual {best:.3f}")
        if not improved:
            steps = [s * 0.5 for s in steps]
    print("\nfitted:", {n: round(v, 3) for n, v in zip(names, theta)})
    print(f"final residual {best:.3f}")

    # per-gate residual localization on the first flight (map-error detector)
    cmds, obs = flights[0]
    traj = simulate(cmds, theta, min(cmds[-1][0], obs[-1][0]) + 0.5)
    pg = {}
    residual(traj, obs, gates, per_gate=pg)
    print("\nper-gate mean residual (high = map error at that gate):")
    for gid in sorted(pg):
        m = sum(pg[gid]) / len(pg[gid])
        print(f"  g{gid:2d}: {m:5.2f}  ({len(pg[gid])} obs)")


if __name__ == "__main__":
    main()
