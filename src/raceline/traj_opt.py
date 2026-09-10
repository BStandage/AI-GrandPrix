"""Time-optimal trajectory through the gate sequence (direct multiple shooting,
CasADi + IPOPT).

Decision variables: for each of the K gate-to-gate segments a duration T_k,
N+1 states (position, velocity) and N thrust vectors u (specific force the
motors produce, m/s^2). Dynamics: p' = v, v' = u + g - k_d |v| v with the
measured drag k_d. Constraints: |u| <= u_max, u_z >= u_zmin (no pushing
down), |du/dt| <= slew (the attitude cannot swing the thrust vector faster
than that), |v| <= v_max, and at the end of segment k the drone is ON gate
k's plane inside its opening (with margin) moving through it. Objective:
sum of T_k plus a small smoothness term. Seeded by an existing plan.

The output is a plan JSON the follower flies directly (same format as the
rule planner writes), with the nose heading from the planner's yaw rule.

    python -m raceline.traj_opt --init out/plans/plan_L4_b33_db_s400.json --out out/plans/plan_opt.json
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from raceline import course as cb
from raceline import planner
from raceline.config import load_config

G = 9.81


def _interp_rows(t_src, rows_src, t_q):
    return np.column_stack([np.interp(t_q, t_src, rows_src[:, j]) for j in range(rows_src.shape[1])])


def optimize(init_plan: dict, course, cfg, nodes: int = 24, u_max: float = 33.75,
             u_zmin: float = 1.0, slew: float = 250.0, v_max: float = 12.0,
             lat_margin: float = 0.5, z_margin: float = 0.45, v_cross_min: float = 2.0,
             cross_tilt_deg: float = 25.0,
             max_iter: int = 3000, verbose: bool = True) -> dict:
    import casadi as ca

    k_d = float(cfg.drag_k) if hasattr(cfg, "drag_k") else 0.257
    events = [course.event(k) for k in range(course.total_events)]
    K = len(events)
    N = nodes
    t_src = init_plan["t_arr"]; P_src = init_plan["pos"]; V_src = init_plan["vel"]; A_src = init_plan["acc"]
    ev_t = [e["t"] for e in init_plan["events"]]
    assert len(ev_t) == K, f"init plan has {len(ev_t)} events, course {K}"

    opti = ca.Opti()
    Ts, Xs, Us = [], [], []
    J = 0
    x_prev_end = None
    t_bounds = [0.0] + ev_t
    for k in range(K):
        T = opti.variable()
        X = opti.variable(6, N + 1)
        U = opti.variable(3, N)
        Ts.append(T); Xs.append(X); Us.append(U)
        dur0 = max(t_bounds[k + 1] - t_bounds[k], 0.3)
        opti.subject_to(T >= 0.15)
        opti.set_initial(T, dur0)
        # initial guess along the seed plan
        tq = np.linspace(t_bounds[k], t_bounds[k + 1], N + 1)
        Pq = _interp_rows(t_src, P_src, tq); Vq = _interp_rows(t_src, V_src, tq); Aq = _interp_rows(t_src, A_src, tq)
        sp = np.linalg.norm(Vq, axis=1, keepdims=True)
        Uq = Aq + np.array([0.0, 0.0, G]) + k_d * sp * Vq
        Uq[:, 2] = np.maximum(Uq[:, 2], u_zmin)
        opti.set_initial(X, np.vstack([Pq.T, Vq.T]))
        opti.set_initial(U, Uq[:N].T)
        dt = T / N
        for i in range(N):
            p = X[0:3, i]; v = X[3:6, i]; u = U[:, i]
            def f(p_, v_):
                sp_ = ca.sqrt(ca.sumsqr(v_) + 1e-6)
                return ca.vertcat(v_, u + ca.vertcat(0, 0, -G) - k_d * sp_ * v_)
            k1 = f(p, v)
            k2 = f(p + dt / 2 * k1[0:3], v + dt / 2 * k1[3:6])
            k3 = f(p + dt / 2 * k2[0:3], v + dt / 2 * k2[3:6])
            k4 = f(p + dt * k3[0:3], v + dt * k3[3:6])
            x_next = X[:, i] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            opti.subject_to(X[:, i + 1] == x_next)
            # thrust ceiling, no pushing down, speed cap
            opti.subject_to(ca.sumsqr(u) <= u_max ** 2)
            opti.subject_to(u[2] >= u_zmin)
            opti.subject_to(ca.sumsqr(X[3:6, i + 1]) <= v_max ** 2)
            # thrust-vector slew (attitude rate)
            if i > 0:
                opti.subject_to(ca.sumsqr(U[:, i] - U[:, i - 1]) <= (slew * dt) ** 2)
            J += 1e-4 * ca.sumsqr(U[:, i] - U[:, i - 1]) if i > 0 else 0
        # continuity with the previous segment (state and thrust)
        if x_prev_end is not None:
            opti.subject_to(X[:, 0] == x_prev_end)
            opti.subject_to(ca.sumsqr(U[:, 0] - Us[k - 1][:, N - 1]) <= (slew * dt) ** 2)
        else:
            opti.subject_to(X[0:3, 0] == ca.vertcat(0.0, 0.0, 0.2))
            opti.subject_to(X[3:6, 0] == ca.vertcat(0.0, 0.0, 0.0))
        # gate k at the end of this segment
        e = events[k]
        n = np.array([math.cos(e.heading_rad), math.sin(e.heading_rad), 0.0])
        b = np.array([-math.sin(e.heading_rad), math.cos(e.heading_rad), 0.0])
        c = np.array([e.x, e.y, e.z])
        pe = X[0:3, N]; ve = X[3:6, N]
        d = pe - c
        opti.subject_to(ca.dot(d, n) == 0)
        opti.subject_to(ca.dot(d, b) <= (e.half_w - lat_margin))
        opti.subject_to(ca.dot(d, b) >= -(e.half_w - lat_margin))
        opti.subject_to(d[2] <= (e.half_h - z_margin))
        opti.subject_to(d[2] >= -(e.half_h - z_margin))
        opti.subject_to(ca.dot(ve, n) >= v_cross_min)
        # cross within cross_tilt_deg of the gate normal: a 60 deg crossing
        # shrinks the 1.5 m opening to 0.75 m and a 0.3 m tracking error
        # misses it (batch 10: g7 refused by the referee at 0.3 m off)
        opti.subject_to(ca.dot(ve, n) >= math.cos(math.radians(cross_tilt_deg)) * ca.sqrt(ca.sumsqr(ve) + 1e-6))
        x_prev_end = X[:, N]
        J += T
    opti.minimize(J)
    opts = {"ipopt.max_iter": max_iter, "ipopt.tol": 1e-4, "ipopt.acceptable_tol": 1e-3,
            "ipopt.print_level": 5 if verbose else 0, "print_time": verbose,
            "ipopt.mu_strategy": "adaptive"}
    opti.solver("ipopt", opts)
    t0 = time.time()
    try:
        sol = opti.solve()
        ok = True
    except RuntimeError as ex:
        if verbose:
            print("solver did not converge cleanly:", str(ex)[:120])
        sol = opti.debug
        ok = False
    # unpack
    t_nodes, P, V, U = [], [], [], []
    t_acc = 0.0
    for k in range(K):
        T = float(sol.value(Ts[k])); X = np.array(sol.value(Xs[k])); Uk = np.array(sol.value(Us[k]))
        tt = t_acc + np.linspace(0, T, N + 1)
        sl = slice(0, N + 1) if k == K - 1 else slice(0, N)
        t_nodes.append(tt[sl]); P.append(X[0:3, sl].T); V.append(X[3:6, sl].T)
        Uk_full = np.column_stack([Uk, Uk[:, -1]])
        U.append(Uk_full[:, sl].T)
        t_acc += T
    t_nodes = np.concatenate(t_nodes); P = np.vstack(P); V = np.vstack(V); U = np.vstack(U)
    ev_times = np.cumsum([float(sol.value(T)) for T in Ts])
    return {"ok": ok, "t": t_nodes, "pos": P, "vel": V, "u": U, "event_t": ev_times,
            "total_s": float(ev_times[-1]), "solve_s": time.time() - t0, "k_d": k_d}


def to_plan(res: dict, course, cfg, ds: float = 0.25) -> planner.Plan:
    """Resample the node solution at ds along the arc and package as a Plan."""
    t = res["t"]; P = res["pos"]; V = res["vel"]; U = res["u"]; k_d = res["k_d"]
    sp = np.linalg.norm(V, axis=1, keepdims=True)
    A = U + np.array([0.0, 0.0, -G]) + (-k_d) * sp * V      # kinematic accel dv/dt
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1); s_nodes = np.concatenate([[0.0], np.cumsum(seg)])
    s_q = np.arange(0.0, s_nodes[-1], ds)
    pos = np.column_stack([np.interp(s_q, s_nodes, P[:, j]) for j in range(3)])
    vel = np.column_stack([np.interp(s_q, s_nodes, V[:, j]) for j in range(3)])
    acc = np.column_stack([np.interp(s_q, s_nodes, A[:, j]) for j in range(3)])
    tt = np.interp(s_q, s_nodes, t)
    v = np.linalg.norm(vel, axis=1)
    T = vel / np.maximum(v, 1e-6)[:, None]
    # curvature from the tangent derivative (diagnostic only)
    dT = np.gradient(T, s_q, axis=0)
    kappa = np.linalg.norm(dT, axis=1)
    psi = np.unwrap(np.arctan2(T[:, 1], T[:, 0]))
    dpsi_ds = np.gradient(psi, s_q)
    dkappa_ds = np.gradient(kappa, s_q)
    # events at the segment ends
    events = []
    ev_t = res["event_t"]
    for k in range(course.total_events):
        c = course.event(k)
        i = int(np.argmin(np.abs(tt - ev_t[k])))
        events.append({"event": k, "lap": course.lap_of(k), "label": c.label,
                       "s": float(s_q[i]), "t": float(ev_t[k]), "v": float(v[i]),
                       "x": round(float(c.x), 4), "y": round(float(c.y), 4), "z": round(float(c.z), 4),
                       "heading_rad": round(float(c.heading_rad), 4)})
    yaw, hold = planner._yaw_profile(T, s_q, s_min=float(events[0]["s"]))
    meta = {"planner": "traj_opt (CasADi/IPOPT direct multiple shooting)",
            "frame_violations": 0, "config": getattr(cfg, "sha1", "")[:8],
            "config_path": str(getattr(cfg, "path", "")), "path_length_m": round(float(s_q[-1]), 2),
            "solver_ok": bool(res["ok"]), "solve_s": round(res["solve_s"], 1)}
    return planner.Plan(s=s_q, pos=pos, vel=vel, acc=acc, t=tt, v=v, v_lim=v.copy(), tangent=T,
                        kappa=kappa, dpsi_ds=dpsi_ds, dkappa_ds=dkappa_ds,
                        binding=np.array(["traj_opt"] * len(s_q)), events=events, meta=meta,
                        yaw=yaw, yaw_hold=hold)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--init", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--nodes", type=int, default=24)
    ap.add_argument("--umax", type=float, default=33.75)
    ap.add_argument("--slew", type=float, default=250.0)
    ap.add_argument("--vmax", type=float, default=12.0)
    ap.add_argument("--lat-margin", type=float, default=0.5)
    ap.add_argument("--z-margin", type=float, default=0.45)
    ap.add_argument("--iter", type=int, default=3000)
    ap.add_argument("--cross-tilt-deg", type=float, default=25.0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    cfg = load_config()
    course = cb.load_course(laps=cfg.planner.laps)
    init = planner.load_plan(args.init)
    res = optimize(init, course, cfg, nodes=args.nodes, u_max=args.umax, slew=args.slew,
                   v_max=args.vmax, lat_margin=args.lat_margin, z_margin=args.z_margin,
                   cross_tilt_deg=args.cross_tilt_deg, max_iter=args.iter, verbose=not args.quiet)
    plan = to_plan(res, course, cfg)
    planner.write_plan(plan, args.out)
    try:
        from raceline import render_plan
        render_plan.render(plan, args.out.replace(".json", ".png"))
    except Exception as ex:
        print("render skipped:", ex)
    print(f"traj_opt: {'converged' if res['ok'] else 'NOT converged'}; total {res['total_s']:.2f} s "
          f"(seed {init['events'][-1]['t']:.2f}); solve {res['solve_s']:.0f} s -> {args.out}")
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
