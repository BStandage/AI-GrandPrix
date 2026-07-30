"""
SMOOTH ANALYTIC RACE TAPE from trajectory.json - pure feedforward, no controller in the loop.

    python -m analysis.make_race_tape

The previous race tapes were RECORDINGS of the closed-loop follower flying the generation sim,
so every anchor correction and feedback twitch (+-30 deg at 2-3 Hz - audited) was burned onto
the tape; reality diverged in the first 40 m every attempt while smooth diagnostic tapes flew
fine. This generator derives commands analytically from the plan:
    yaw    = path tangent (unwrapped, slew-limited)
    pitch/roll = body decomposition of plan accel + drag compensation (validated linear model)
    thrust = vertical demand through the measured lift curve (T0 0.265, p 1.45)
Launch prefix: 1.4 s spool+hop. Every channel is smooth by construction.
Sanity: replays the tape through the truth physics and reports gate crossings.
"""

import json
import math
import os

G = 9.81
T0, P = 0.265, 1.45
C1, C2 = 0.1141, 0.0192
C1V, C2V = 0.35, 0.010
LAUNCH_S = 1.4
LEAN_MAX = math.radians(38.0)
YAW_SLEW = 2.5
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    traj = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "trajectory.json")))
    S = traj["samples"]          # [t, s, x, y, z, vx, vy, vz]
    tmax = S[-1][0]

    def interp(tq):
        lo, hi = 0, len(S) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if S[mid][0] <= tq:
                lo = mid
            else:
                hi = mid
        a, b = S[lo], S[hi]
        f = (tq - a[0]) / max(b[0] - a[0], 1e-6)
        return [a[k] + f * (b[k] - a[k]) for k in range(len(a))]

    cmds = []
    yaw_prev = math.radians(97.1)
    dt = 1.0 / 90.0
    # ---- SOLVED launch prefix: end at the plan's start state (z ~ plan z0, vz ~ 0). A hop
    # that ends 3 m high with climb is an initial-condition error pure feedforward keeps
    # forever (first analytic tape ballooned +5..+20 m from exactly this).
    z_target = S[0][4] if S[0][4] > 0.1 else 0.4
    best = None
    for t_hop in [x * 0.05 for x in range(6, 24)]:
        for t_arr in [x * 0.05 for x in range(4, 20)]:
            z = vz = 0.0
            thr_act = 0.0
            tt = 0.0
            while tt < t_hop + t_arr:
                cmd = 0.33 if tt < t_hop else 0.20
                eff = 0.0 if tt < 0.4 else cmd
                thr_act += (eff - thr_act) * (1 - math.exp(-dt / 0.03))
                a_up = G * (thr_act / T0) ** P - G - (C1V + C2V * abs(vz)) * vz
                vz += a_up * dt
                z = max(z + vz * dt, 0.0)
                if z == 0.0:
                    vz = max(vz, 0.0)
                tt += dt
            err = abs(z - z_target) + abs(vz) * 0.6
            if best is None or err < best[0]:
                best = (err, t_hop, t_arr, z, vz)
    _, T_HOP, T_ARR, zf, vzf = best
    print(f"launch solved: hop {T_HOP:.2f}s + arrest {T_ARR:.2f}s -> z {zf:.2f} vz {vzf:+.2f} "
          f"(target z {z_target:.2f})")
    t = 0.0
    while t < T_HOP + T_ARR:
        thr_l = 0.33 if t < T_HOP else 0.20
        cmds.append([round(t, 4), 0.0, 0.0, round(yaw_prev, 5), thr_l])
        t += dt
    LAUNCH_END = T_HOP + T_ARR
    while t < LAUNCH_END + tmax:
        tq = t - LAUNCH_END
        s0 = interp(max(tq - 0.15, 0.0))
        s1 = interp(min(tq + 0.15, tmax))
        cur = interp(tq)
        span = max(s1[0] - s0[0], 1e-3)
        ax = (s1[5] - s0[5]) / span
        ay = (s1[6] - s0[6]) / span
        az = (s1[7] - s0[7]) / span
        vx, vy, vz = cur[5], cur[6], cur[7]
        sp = math.hypot(vx, vy)
        drag = C1 + C2 * sp
        axc, ayc = ax + drag * vx, ay + drag * vy
        # yaw along the velocity (fall back to previous when slow), slew-limited
        yaw_des = math.atan2(vy, vx) if sp > 0.8 else yaw_prev
        dpsi = (yaw_des - yaw_prev + math.pi) % (2 * math.pi) - math.pi
        yaw = yaw_prev + max(-YAW_SLEW * dt, min(YAW_SLEW * dt, dpsi))
        yaw_prev = yaw
        fwd = (math.cos(yaw), math.sin(yaw))
        rgt = (math.sin(yaw), -math.cos(yaw))
        a_fwd = axc * fwd[0] + ayc * fwd[1]
        a_rgt = axc * rgt[0] + ayc * rgt[1]
        # two-pass consistency: thrust above hover scales horizontal accel too (tw = g*thr/T0)
        pitch = roll = 0.0
        thr = T0
        for _ in range(3):
            a_up = az + (C1V + C2V * abs(vz)) * vz
            lift_req = max((G + a_up) / max(math.cos(pitch) * math.cos(roll), 0.5), 0.5)
            thr = max(0.05, min(0.6, T0 * (lift_req / G) ** (1.0 / P)))
            tw = G * thr / T0
            pitch = max(-LEAN_MAX, min(LEAN_MAX, math.atan2(a_fwd, tw)))
            roll = max(-LEAN_MAX, min(LEAN_MAX, math.atan2(a_rgt, tw)))
        cmds.append([round(t, 4), round(roll, 5), round(pitch, 5), round(yaw, 5), round(thr, 5)])
        t += dt

    out = os.path.join(HERE, "pilots", "ace_pilot", "tape.json")
    json.dump({"columns": ["t", "roll", "pitch", "yaw", "thrust"], "commands": cmds,
               "_src": "analytic feedforward (make_race_tape)"}, open(out, "w"))
    print(f"tape: {len(cmds)} cmds, {cmds[-1][0]:.1f}s -> {out}")

    # ---- sanity replay through truth physics (with spool + ground) ----
    gates = json.load(open(os.path.join(HERE, "pilots", "ace_pilot", "course_map.json")))["gates"]
    px = py = pz = 0.0
    vx = vy = vz = 0.0
    th = ph = 0.0
    psi = math.radians(97.1)
    thr_act = 0.0
    prev_along = {}
    passed = {}
    i = 0
    t = 0.0
    while t < cmds[-1][0]:
        while i < len(cmds) - 1 and cmds[i + 1][0] <= t:
            i += 1
        _, r_c, p_c, y_c, tr = cmds[i]
        eff = 0.0 if t < 0.4 else tr
        thr_act += (eff - thr_act) * (1 - math.exp(-dt / 0.03))
        a = 1 - math.exp(-dt / 0.096)
        th += (p_c - th) * a
        ph += (r_c - ph) * a
        psi += ((y_c - psi + math.pi) % (2 * math.pi) - math.pi) * (1 - math.exp(-dt / 0.15))
        tw = G * thr_act / T0
        af = tw * math.cos(ph) * math.sin(th)
        ar = tw * math.cos(th) * math.sin(ph)
        sp = math.hypot(vx, vy)
        drag = C1 + C2 * sp
        vx += (af * math.cos(psi) + ar * math.sin(psi) - drag * vx) * dt
        vy += (af * math.sin(psi) + ar * -math.cos(psi) - drag * vy) * dt
        a_up = G * (thr_act / T0) ** P * math.cos(th) * math.cos(ph) - G - (C1V + C2V * abs(vz)) * vz
        vz += a_up * dt
        px += vx * dt
        py += vy * dt
        pz = max(pz + vz * dt, 0.0)
        if pz == 0.0:
            vz = max(vz, 0.0)
        for g in gates:
            gid = g["gate_id"]
            if gid in passed:
                continue
            cd = g["cross_dir"]
            along = (px - g["x"]) * cd[0] + (py - g["y"]) * cd[1]
            pa = prev_along.get(gid)
            prev_along[gid] = along
            if pa is not None and pa < 0 <= along and abs(along - pa) < 2:
                perp = -(px - g["x"]) * cd[1] + (py - g["y"]) * cd[0]
                if abs(perp) < 3.0:
                    passed[gid] = (round(t, 1), round(perp, 2), round(pz - g["z"], 2))
        t += dt
    ok = sum(1 for v in passed.values() if abs(v[1]) <= 0.61 and abs(v[2]) <= 0.61)
    print(f"sanity replay: {ok}/{len(gates)} clean, {len(passed)} planes")
    for gid in sorted(passed):
        tt, lat, dz = passed[gid]
        flag = "PASS" if abs(lat) <= 0.61 and abs(dz) <= 0.61 else "MISS"
        print(f"  g{gid:2d} t={tt:5.1f} lat {lat:+5.2f} dz {dz:+5.2f} {flag}")


if __name__ == "__main__":
    main()
