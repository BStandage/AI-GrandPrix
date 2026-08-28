"""Kinematic follower harness: replay Tracker guidance against a
double-integrator plant in pure Python. Milliseconds per lap instead of
five minutes per flight; found for the hairpin-stall class of bug where
the SIM is overkill but hand-analysis is too slow.

Not a substitute for the real sim (no Betaflight, no attitude dynamics) -
a bug filter before burning flights.

    python -m raceline.sim_lite out/plans/plan_2412.json
"""
import sys
import numpy as np


def run(plan_path, cfg=None, t_max=200.0, dt=0.01, verbose=True):
    import os
    os.environ.setdefault("AIGP_TRAJ", str(plan_path))
    from raceline.config import load_config
    from raceline import planner as plan_io
    from raceline import course as course_bridge
    from solvers.follower import Tracker, StateEstimate

    cfg = cfg or load_config(os.environ.get("AIGP_VEHICLE_TOML"))
    plan = plan_io.load_plan(plan_path)
    course = course_bridge.load_course(laps=cfg.planner.laps)
    pq = course_bridge.pq_course()
    tracker_ref = pq.RaceTracker(course)      # the referee
    tr = Tracker(plan, cfg)
    tr.started = True

    p = plan["pos"][0].copy()
    v = np.zeros(3)
    a_max = cfg.a_lat_full()
    t = 0.0
    log = []
    while t < t_max and not tracker_ref.complete:
        est = StateEstimate(p=p.copy(), v=v.copy(), R=np.eye(3), yaw=0.0)
        a_xy, z_t, vz_ff, yaw_des, done = tr.step(est, tracker_ref.event_idx)
        az = 6.0 * (z_t - p[2]) + 3.0 * (vz_ff - v[2])
        a = np.array([a_xy[0], a_xy[1], np.clip(az, -6, 6)])
        # crude accel-slew and magnitude limits to mimic the plant
        v += a * dt
        sp = float(np.linalg.norm(v))
        if sp > 12.0:
            v *= 12.0 / sp
        p += v * dt
        tracker_ref.update(t, p)
        log.append((t, *p, float(np.linalg.norm(v)), tr.s[tr.idx]))
        t += dt
        if done:
            break
    if verbose:
        print(f"kinematic replay: {tracker_ref.events_passed}/"
              f"{course.total_events} events in {t:.1f} s")
    return tracker_ref, np.array(log)


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else "../out/plans/plan_2412.json")
