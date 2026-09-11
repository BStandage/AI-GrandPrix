"""Kinematic replay of a plan through the real follower Tracker against the
referee's gate geometry.

The plant is a point mass with the measured drag, a thrust-vector budget
(vertical need first, the same rule as the follower) and a first-order
altitude loop; attitude is assumed instantaneous. It is NOT a flight model:
it is the cheap check that a plan's geometry can be followed at all and
that every crossing lands inside the openings. A plan that fails here will
fail in the sim; one that passes still has to be flown.

    python -m raceline.replay out/plans/plan_RACE.json
"""
from __future__ import annotations

import os
import sys

import numpy as np

G = 9.81
DT = 0.01
T_MAX_S = 120.0
THRUST_BUDGET_SHARE = float(os.environ.get("REPLAY_THRUST_SHARE", "0.80"))   # ACHIEVED share of the motors' thrust (the follower commands 0.9; at 70 deg with the mixer saturated the plant delivers less - calibrated against flown outcomes)
ATT_LAG_S = float(os.environ.get("REPLAY_ATT_LAG", "0.06"))   # first-order lag between the commanded and the
                            # achieved thrust vector (attitude loop at ka 600 /
                            # kw 10: tau ~0.1 s plus the 80 ms plant lag).
                            # Without it the replay passed plan_027 25/25 and the
                            # drone ran 1.4 m wide on the g6 exit bend and hit g7
                            # (race_050): instantaneous attitude is not a plant.


def replay(plan_path: str, verbose: bool = False) -> dict:
    os.environ["AIGP_TRAJ"] = plan_path
    from raceline.config import load_config
    from raceline import planner as plan_io, course as course_bridge
    from solvers.follower import Tracker, StateEstimate
    import solvers.follower as _fol
    # The replay judges plans with the follower's feedforward LEAD off: the
    # lead cancels the real attitude lag (race_055: arcs 0.47 -> 0.20 m wide)
    # but this point-mass plant has no attitude state, and with the lead on
    # it cuts every arc inside and fails the plan that flew clean. Judging
    # without the lead keeps the guard on the conservative side.
    _fol.ACC_LEAD_S = float(os.environ.get("REPLAY_LEAD", "0.0"))

    cfg = load_config()
    plan = plan_io.load_plan(plan_path)
    course = course_bridge.load_course(laps=cfg.planner.laps)
    pq = course_bridge.pq_course()
    ref = pq.RaceTracker(course)
    tr = Tracker(plan, cfg)
    tr.started = True
    t_cap = THRUST_BUDGET_SHARE * float(max(cfg.thrust.curve_acc))

    p = plan["pos"][0].copy()
    v = np.zeros(3)
    a_act = np.zeros(3)     # achieved (lagged) thrust acceleration
    t = 0.0
    seen = set()
    events = []
    while t < T_MAX_S and not ref.complete:
        est = StateEstimate(p=p.copy(), v=v.copy(), R=np.eye(3), yaw=0.0)
        a_xy, z_t, vz_ff, yaw_des, done = tr.step(est, ref.event_idx)
        # altitude loop with the follower's own gains (the 6/3 stand-in
        # could not climb into g10-top at the tilt-89 approach speeds)
        az = float(np.clip(cfg.follower.kp_z * (z_t - p[2]) + cfg.follower.kd_z * (vz_ff - v[2]), -8.0, 14.0))
        a = np.array([a_xy[0], a_xy[1], az])
        # thrust-vector budget: vertical need first, horizontal gets the rest
        th_z = max(G + az, 0.0)
        h_cap = float(np.sqrt(max(t_cap * t_cap - th_z * th_z, 0.0)))
        nh = float(np.hypot(a[0], a[1]))
        if nh > h_cap:
            a[:2] *= h_cap / max(nh, 1e-9)
        # attitude lag: the thrust vector follows the command first-order
        a_act += (a - a_act) * (DT / ATT_LAG_S)
        a_eff = a_act.copy()
        vh = float(np.hypot(v[0], v[1]))
        if vh > 1e-3:
            a_eff[:2] -= (cfg.a_drag(vh) / vh) * v[:2]
        # vertical drag on CLIMBS: the plant's coefficient on the world-vertical
        # component is twice the horizontal one (drag_quad_z); without it the
        # guard passed climbs the sim rejected (g1 / g10-top candidates,
        # 2026-09-10). Descents stay as calibrated: with the term on both signs
        # the guard failed the stack drop of the plan that flies clean (L5).
        if v[2] > 0.0:
            a_eff[2] -= cfg.drag_k_xyz()[2] * float(np.linalg.norm(v)) * v[2]
        v += a_eff * DT
        p += v * DT
        t += DT
        hit = ref.update(t, p)
        if hit is not None:
            # record the crossing at the referee's credit tick (sampling on
            # the tracker's arc index lagged through the compact stack and
            # printed g10-top 2.5 m low while the referee credited it)
            k = ref.events_passed - 1
            e = plan["events"][k]
            h = e["heading_rad"]
            nx, ny = -np.sin(h), np.cos(h)
            lat = float((p[0] - e["x"]) * nx + (p[1] - e["y"]) * ny)
            rec = {"k": k, "label": e["label"], "t": t, "lat": lat,
                   "dz": float(p[2] - e["z"]), "along": 0.0,
                   "v": float(np.linalg.norm(v)), "passed": ref.events_passed}
            events.append(rec)
            if verbose:
                print(f"{e['label']:>8} t={t:6.2f} lat {lat:+.2f} dz {rec['dz']:+.2f} "
                      f"v={rec['v']:4.1f}  referee passed {ref.events_passed}")
    # which events were NOT credited: the referee count advances by one per
    # credited event in order, so the first uncredited event index is the
    # count itself when the run is incomplete
    passed = int(ref.events_passed)
    total = int(course.total_events)
    missed = [] if passed >= total else [plan["events"][passed]["label"]]
    if verbose:
        print("RESULT", passed, "/", total, "t", round(t, 1))
    return {"passed": passed, "total": total, "t": t, "events": events, "missed": missed}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    r = replay(sys.argv[1], verbose=True)
    return 0 if r["passed"] == r["total"] else 1


if __name__ == "__main__":
    sys.exit(main())
