"""
Offline trajectory solver: course_map.json -> trajectory.json

    python -m pilots.ace_pilot.trajectory

Pipeline:
  1. Waypoints: spawn + (pre, center, post) triplet per gate - the pre/post points sit
     ACE_GATE_NORMAL_OFF metres along the gate's measured crossing direction, forcing the spline
     to cross each gate plane near-perpendicular (entry angle shrinks the effective opening).
  2. Catmull-Rom spline through the waypoints, sampled every ACE_SAMPLE_DS metres.
  3. Curvature-limited speed profile: v = min(V_MAX, sqrt(A_LAT / kappa)), then a forward pass
     (accel limit) and backward pass (brake limit) - the classic time-optimal path-speed solve.
  4. Time-parameterize and emit samples [t, x, y, z, vx, vy, vz] plus per-gate arc markers.

The output is feedforward truth for the runtime follower; the follower adds feedback, so solver
smoothness matters more than optimality polish. Bang-bang body-rate output is deliberately NOT
the target - the sim flies attitude setpoints (gain 1.0, 96 ms lag), so speed/heading profiles
are the correct currency.
"""

import json
import math
import os

from pilots.ace_pilot.config import (ACE_A_ACC, ACE_A_BRK, ACE_A_LAT, ACE_GATE_NORMAL_OFF, ACE_TRANSIT_UP, ACE_V_HAIRPIN,
                                     ACE_SAMPLE_DS, ACE_V_FINISH, ACE_V_MAX, ACE_Z_FLOOR)

HERE = os.path.dirname(os.path.abspath(__file__))


def _catmull_rom(pts, ds):
    """Sample a Catmull-Rom spline through pts (list of (x,y,z)) roughly every ds metres."""
    if len(pts) < 2:
        return list(pts)
    ext = [pts[0]] + list(pts) + [pts[-1]]
    out = []
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        seg_len = math.dist(p1, p2)
        n = max(2, int(seg_len / ds))
        for k in range(n):
            u = k / n
            u2, u3 = u * u, u * u * u
            out.append(tuple(
                0.5 * ((2 * p1[j]) + (-p0[j] + p2[j]) * u
                       + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * u2
                       + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * u3)
                for j in range(3)))
    out.append(pts[-1])
    return out


def solve(map_path=None, out_path=None):
    map_path = map_path or os.path.join(HERE, "course_map.json")
    out_path = out_path or os.path.join(HERE, "trajectory.json")
    cmap = json.load(open(map_path))
    gates = cmap["gates"]
    # PER-GATE WAYPOINT CORRECTIONS (gate_offsets.json): the open-loop iteration loop - fly,
    # measure each gate's miss, shift its waypoint the other way, re-solve. Works identically
    # against the offline sim and the real one (the map is never modified; offsets accumulate).
    off_path = os.path.join(HERE, "gate_offsets.json")
    offsets = {}
    if os.path.exists(off_path):
        offsets = {int(k): v for k, v in json.load(open(off_path)).items()}
    # REALITY layer (race_offsets.json, hand-edited from race-attempt feedback): a global shift
    # plus per-gate corrections, SEPARATE from gate_offsets.json so the model-iteration loop
    # (iterate_tape) can never overwrite what reality taught us.
    ro_path = os.path.join(HERE, "race_offsets.json")
    r_glob = (0.0, 0.0, 0.0)
    r_gates = {}
    if os.path.exists(ro_path):
        ro = json.load(open(ro_path))
        r_glob = ro.get("global", [0.0, 0.0, 0.0])
        r_gates = {int(k): v for k, v in ro.get("gates", {}).items()}
    for g in gates:
        o = offsets.get(g["gate_id"])
        if o:
            g = dict(g)
        # applied via a copy below

    # 1) waypoints: spawn + gate centers. Pre/post triplets (ACE_GATE_NORMAL_OFF > 0) are
    # retired - collinear triplets concentrated the whole turn into the short inter-gate join,
    # spiking curvature at every gate (flight 2's near-stop weave). Centers-only spreads each
    # turn over the whole leg.
    centers = []
    for g in gates:
        o = offsets.get(g["gate_id"], (0.0, 0.0, 0.0))
        ro_g = r_gates.get(g["gate_id"], (0.0, 0.0, 0.0))
        centers.append((g["x"] + o[0] + r_glob[0] + ro_g[0],
                        g["y"] + o[1] + r_glob[1] + ro_g[1],
                        g["z"] + o[2] + r_glob[2] + ro_g[2]))
    # VIA points (race_offsets.json "vias": [{"after_gate": k, "frac": 0.5,
    # "offset": [dx, dy, dz]}]): mid-leg shaping points to route around static obstacles
    # (e.g. the display fighter jet on a leg). frac = fraction along the straight
    # gate-k -> gate-k+1 line; offset = world-frame push from that point.
    vias = {}
    if os.path.exists(ro_path):
        for v in json.load(open(ro_path)).get("vias", []):
            vias.setdefault(int(v["after_gate"]), []).append(v)
    wpts = [(0.0, 0.0, 0.0)]
    for i, g in enumerate(gates):
        dx, dy = g["cross_dir"]
        c = centers[i]
        if ACE_GATE_NORMAL_OFF > 0.0:
            wpts.append((c[0] - ACE_GATE_NORMAL_OFF * dx, c[1] - ACE_GATE_NORMAL_OFF * dy, c[2]))
        wpts.append(c)
        if ACE_GATE_NORMAL_OFF > 0.0:
            wpts.append((c[0] + ACE_GATE_NORMAL_OFF * dx, c[1] + ACE_GATE_NORMAL_OFF * dy, c[2]))
        for v in sorted(vias.get(g["gate_id"], []), key=lambda v: v.get("frac", 0.5)):
            if "abs" in v:
                # WORLD-ANCHORED via: obstacles are world-fixed - a frac/offset via slides
                # when a gate correction moves the leg line (corridor slid into the jet wing)
                wpts.append(tuple(v["abs"]))
            elif i + 1 < len(centers):
                f = v.get("frac", 0.5)
                nxt = centers[i + 1]
                wpts.append((c[0] + (nxt[0] - c[0]) * f + v["offset"][0],
                             c[1] + (nxt[1] - c[1]) * f + v["offset"][1],
                             c[2] + (nxt[2] - c[2]) * f + v["offset"][2]))

    # effective gate altitudes WITH all offsets - the cruise/fade pass below must see the
    # same z the waypoints carry (reading raw map z made every z offset silently inert)
    gz_eff = []
    for g in gates:
        o = offsets.get(g["gate_id"], (0.0, 0.0, 0.0))
        ro_g = r_gates.get(g["gate_id"], (0.0, 0.0, 0.0))
        gz_eff.append(g["z"] + o[2] + r_glob[2] + ro_g[2])

    # 2) spline + arc length
    pts = _catmull_rom(wpts, ACE_SAMPLE_DS)
    n = len(pts)
    s = [0.0] * n
    for i in range(1, n):
        s[i] = s[i - 1] + math.dist(pts[i - 1], pts[i])

    # gate arc positions: nearest sample to each gate center
    gate_s = []
    for g in gates:
        c = (g["x"], g["y"], g["z"])
        i_min = min(range(n), key=lambda i: math.dist(pts[i], c))
        gate_s.append(s[i_min])

    # via arc-positions (needed by the curvature pass below and the z pass in 3b)
    spline_z = [p[2] for p in pts]
    via_pts = [v["abs"] for lst in vias.values() for v in lst if "abs" in v]
    via_s = []
    for vp in via_pts:
        i_min = min(range(n), key=lambda i: math.hypot(pts[i][0] - vp[0], pts[i][1] - vp[1]))
        via_s.append(s[i_min])

    # 3) curvature (3-point finite difference on the horizontal path) -> speed cap
    v = [ACE_V_MAX] * n
    for i in range(1, n - 1):
        ax_, ay_ = pts[i - 1][0], pts[i - 1][1]
        bx, by = pts[i][0], pts[i][1]
        cx, cy = pts[i + 1][0], pts[i + 1][1]
        a = math.hypot(bx - ax_, by - ay_)
        b = math.hypot(cx - bx, cy - by)
        c2 = math.hypot(cx - ax_, cy - ay_)
        area2 = abs((bx - ax_) * (cy - ay_) - (by - ay_) * (cx - ax_))
        if a * b * c2 > 1e-9:
            kappa = 2.0 * area2 / (a * b * c2)
            if kappa > 1e-6:
                # (a forced speed floor here burned a +-27 deg alternating-roll ring into
                # the tape - the follower cannot track the corridor bow above the curvature
                # cap; the brake through the bow is the price of the jet dodge)
                v[i] = min(v[i], math.sqrt(ACE_A_LAT / kappa))
    # PER-LEG PACE CAPS (race_offsets "leg_caps": [{"after_gate": k, "v": cap}]): reality
    # traverses some legs slower than the model (measured by camera range-rate); cap the plan
    # so the tape's maneuvers arrive where reality actually is.
    ro2 = json.load(open(ro_path)) if os.path.exists(ro_path) else {}
    for lc in ro2.get("leg_caps", []):
        k = int(lc["after_gate"])
        if k + 1 < len(gate_s):
            for i in range(n):
                if gate_s[k] < s[i] < gate_s[k + 1]:
                    v[i] = min(v[i], lc["v"])

    # HAIRPIN gates (course-heading reversal > 90 deg across the gate) get a local speed cap:
    # the global curvature profile under-slows the apex and the follower exits ~1 m wide
    # (offline sim: gate 14, a 125-deg reversal over 6.6 m, missed by -0.9 at every gain).
    for k, g in enumerate(gates):
        if 0 < k < len(gates) - 1:
            c0, c1 = gates[k - 1]["cross_dir"], gates[k + 1]["cross_dir"]
            dotp = c0[0] * c1[0] + c0[1] * c1[1]
            if dotp < -0.3:                      # true reversal (>107 deg) through this gate
                for i in range(n):
                    if abs(s[i] - gate_s[k]) < 3.0:
                        v[i] = min(v[i], ACE_V_HAIRPIN)
    v[0] = 0.0
    v[-1] = min(ACE_V_FINISH, v[-1])
    # forward (accel) and backward (brake) passes
    for i in range(1, n):
        ds = s[i] - s[i - 1]
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * ACE_A_ACC * ds))
    for i in range(n - 2, -1, -1):
        ds = s[i + 1] - s[i]
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * ACE_A_BRK * ds))

    # PER-LEG SPEED FLOORS (race_offsets "leg_floors": [{"after_gate": k, "v": f}]): applied
    # AFTER the brake pass - the brake propagation kept finding sub-metre kinks near gate
    # aims and dragging mid-leg speed to ~3 (nose-up braking in the jet zone). On a straight
    # leg the follower absorbs the small infeasibility smoothly.
    for lf in ro2.get("leg_floors", []):
        k = int(lf["after_gate"])
        if k + 1 < len(gate_s):
            for i in range(n):
                if gate_s[k] + 2.0 < s[i] < gate_s[k + 1] + 3.0:
                    v[i] = max(v[i], lf["v"])   # through the crossing - no braking AT the gate either

    # 3b) CRUISE-HIGH on transits (steady's doctrine, and the sim-crash fix: the map's z puts
    # gates 4-5 at pad height, so flying the spline z dragged the floor at 10 m/s). Lift the
    # path between gates, fading to the true gate z within ~3 m of each crossing.
    # Near a world-anchored via the plan follows the via's own z instead (the dip under an
    # obstacle) - this pass previously overwrote the corridor and the dip never reached the plan.
    for i in range(n):
        d_gate = min(abs(s[i] - gs) for gs in gate_s)
        lift = ACE_TRANSIT_UP * min(d_gate / 14.0, 1.0)
        # cruise at the HIGHER of the neighbouring gates: a late saturated climb into a high
        # gate arrived 1-1.5 m low regardless of waypoint z (VZ_MAX-insensitive - the inert-
        # offset signature); arriving level or from above costs nothing and always tracks
        z_prev = None
        z_next = None
        s_prev = None
        s_next = None
        for k, gs_ in enumerate(gate_s):
            if gs_ <= s[i]:
                z_prev, s_prev = gz_eff[k], gs_
            elif z_next is None:
                z_next, s_next = gz_eff[k], gs_
        # CONTINUOUS gate-to-gate z line: the old nearest-gate pick stepped z by the full
        # neighbour delta at every leg midpoint (2-3 m in one sample between gates 10-11) -
        # the plan's velocity vector went near-vertical there and horizontal speed collapsed
        # to a hover. Smoothstep between the leg's two gate altitudes instead.
        if z_prev is None:
            z_line = z_next
        elif z_next is None:
            z_line = z_prev
        else:
            u = (s[i] - s_prev) / max(s_next - s_prev, 1e-6)
            u = u * u * (3.0 - 2.0 * u)
            z_line = z_prev + (z_next - z_prev) * u
        cruise = max(v for v in (z_prev, z_next, pts[i][2]) if v is not None)
        # FADE between the leg z-line (exact gate z at each plane) and the cruise base (mid-leg)
        w = min(d_gate / 12.0, 1.0)
        base = (1.0 - w) * z_line + w * cruise
        z_plan = max(base + lift, ACE_Z_FLOOR + r_glob[2])
        # VIA CORRIDOR: blend to the via-shaped spline z within 6 m of a via. Altitude changes
        # are flown with thrust at full speed - never traded for airspeed (curvature caps are
        # horizontal-only, so the dip and the climb out cost no braking).
        for sv in via_s:
            d = abs(s[i] - sv)
            if d < 6.0:
                wv = 1.0 - d / 6.0
                z_plan = (1.0 - wv) * z_plan + wv * max(spline_z[i], ACE_Z_FLOOR + r_glob[2])
        pts[i] = (pts[i][0], pts[i][1], z_plan)

    # 4) time parameterization + velocity vectors
    t = [0.0] * n
    for i in range(1, n):
        ds = s[i] - s[i - 1]
        vm = max(0.5 * (v[i] + v[i - 1]), 0.05)
        t[i] = t[i - 1] + ds / vm
    samples = []
    for i in range(n):
        j0, j1 = max(i - 1, 0), min(i + 1, n - 1)
        seg = math.dist(pts[j0], pts[j1]) or 1.0
        tx = (pts[j1][0] - pts[j0][0]) / seg
        ty = (pts[j1][1] - pts[j0][1]) / seg
        tz = (pts[j1][2] - pts[j0][2]) / seg
        samples.append([round(t[i], 3), round(s[i], 3),
                        round(pts[i][0], 3), round(pts[i][1], 3), round(pts[i][2], 3),
                        round(v[i] * tx, 3), round(v[i] * ty, 3), round(v[i] * tz, 3)])

    out = {"source_map": os.path.basename(map_path),
           "columns": ["t", "s", "x", "y", "z", "vx", "vy", "vz"],
           "v_max": ACE_V_MAX, "a_lat": ACE_A_LAT,
           "gate_s": [round(gs, 2) for gs in gate_s],
           "total_time_s": round(t[-1], 1),
           "total_len_m": round(s[-1], 1),
           "samples": samples}
    json.dump(out, open(out_path, "w"))
    print(f"trajectory: {out['total_len_m']} m in {out['total_time_s']} s "
          f"(avg {out['total_len_m'] / max(out['total_time_s'], 0.1):.1f} m/s), "
          f"{n} samples -> {out_path}")
    return out


if __name__ == "__main__":
    solve()
