#!/usr/bin/env python3
"""One-command race loop: plan -> fly headless -> report -> archive.

Run from a WSL shell (elodin + Betaflight SITL live there):

    python race.py --config config/vehicle.toml
    python race.py --plan-only          # iterate planner numbers in seconds
    python race.py --traj out/plans/plan_002.json   # refly a saved plan

Interns tune by editing config/vehicle.toml and re-running this. All tuning
is GLOBAL (RESTRICTIONS.md). Race results come from the sim tracker's run
record; predicted times are model predictions, unverified.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO / "src"))

from raceline.config import load_config          # noqa: E402
from raceline import planner as planner_mod      # noqa: E402
from raceline import course as course_bridge     # noqa: E402

BASELINE_S = 225.3   # stop-and-center reference pilot, 24/24 (2026-08-27)


def resolve_path(pathstr):
    """Accept paths relative to the caller's cwd OR to this repo (race.cmd
    runs us with the sim repo as cwd, but users type AI-GrandPrix-relative
    paths). Windows separators tolerated."""
    if pathstr is None:
        return None
    p = Path(pathstr)
    if p.exists():
        return p
    alt = REPO / str(pathstr).replace("\\", "/")
    return alt if alt.exists() else p


def make_plan(cfg, out_path=None):
    p = planner_mod.plan(cfg)
    print(planner_mod.report(p, baseline_s=BASELINE_S))
    out = Path(out_path) if out_path else planner_mod.next_numbered(
        str(REPO / "out" / "plans" / "plan_XXX.json"))
    planner_mod.write_plan(p, out)
    print(f"FILES plan -> {out}")
    try:
        from raceline import render_plan
        render_plan.render(p, out.with_suffix(".png"))
        print(f"      render -> {out.with_suffix('.png')}")
    except Exception as e:
        print(f"      render skipped: {e}")
    return p, out


def run_sim(sim_repo: Path, plan_path: Path, cfg, sim_time_s: float) -> int:
    env = dict(os.environ)
    env.update(
        RACE_SOLVER="solvers.follower",
        AIGP_TRAJ=str(plan_path.resolve()),
        AIGP_VEHICLE_TOML=str(cfg.path.resolve()),
        AIGP_SIM_TIME=f"{sim_time_s:.0f}",
        AIGP_LAPS=str(cfg.planner.laps),   # sim tracker must expect the
                                           # same lap count the plan flies
    )
    cmd = ["uv", "run", "elodin", "run", "sim/main.py"]
    print(f"\nSIM   {' '.join(cmd)}  (cwd={sim_repo}, "
          f"sim_time={sim_time_s:.0f} s, ~0.8x realtime)")
    # Two sim failure modes are handled here, both observed repeatedly:
    # 1. `elodin run` HANGS after "Simulation stopped" -> hard deadline.
    # 2. Betaflight sometimes wedges at boot (bridge never gets a warmup
    #    response; every tick times out) -> detect and RETRY the launch.
    deadline = sim_time_s * 2.0 + 120.0
    for attempt in (1, 2):
        rc = _run_sim_once(cmd, sim_repo, env, deadline)
        for pat in ("elodin run", "render-server", "betaflight_SITL"):
            subprocess.run(["pkill", "-f", pat], capture_output=True)
        if rc != 9:
            return rc
        print(f"\nSIM   dead bridge at boot (attempt {attempt}) - retrying")
    return 9


def _run_sim_once(cmd, sim_repo, env, deadline) -> int:
    import time
    proc = subprocess.Popen(cmd, cwd=sim_repo, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, errors="replace")
    t0 = time.time()
    warmed = False
    stalled = 0
    for line in proc.stdout:
        print(line, end="")
        if "Warmup complete" in line:
            # a wedged Betaflight sometimes answers 1-2 packets of ~500;
            # demand a real handshake before calling the bridge alive
            import re as _re
            m = _re.search(r"\((\d+) responses", line)
            warmed = bool(m and int(m.group(1)) >= 50)
        if "cannot achieve real-time" in line and "99." in line:
            stalled += 1
            if not warmed and stalled > 20:
                proc.kill()
                return 9          # bridge never answered: retryable
        else:
            stalled = 0
        if time.time() - t0 > deadline:
            print(f"\nSIM   note: killed after {deadline:.0f} s deadline "
                  "(results are on disk)")
            proc.kill()
            return 0
    return proc.wait()


def newest_result(sim_repo: Path, known: set) -> Path | None:
    fresh = sorted(set(sim_repo.glob("race_result_*.json")) - known)
    return fresh[-1] if fresh else None


def print_report(rec: dict, plan_events):
    total = rec.get("total_time_s")
    n, ntot = rec["gates_passed"], rec["events_total"]
    if rec["complete"]:
        delta = total - BASELINE_S
        print(f"\nRACE  {n}/{ntot} COMPLETE   total {total:.2f} s  "
              f"(baseline {BASELINE_S} s -> {delta:+.1f})")
    else:
        print(f"\nRACE  INCOMPLETE: {n}/{ntot} events "
              f"(final t {rec['final_t_s']:.1f} s)")
    for lap in rec["lap_times"]:
        print(f"      lap {lap['lap']}: {lap['t_lap']:.2f} s")

    # per-event vs plan, aligned on the first crossing (takeoff offset)
    events = rec["events"]
    if events and plan_events:
        t0_sim = events[0]["t"]
        t0_plan = plan_events[0]["t"]
        rows = []
        for e, pe in zip(events, plan_events):
            d = (e["t"] - t0_sim) - (pe["t"] - t0_plan)
            rows.append((e["lap"], e["gate"], e["t"], d))
        worst = sorted(rows, key=lambda r: -abs(r[3]))[:3]
        print("      event  lap gate      t(s)   dt-vs-plan(s)")
        for lap, gate, t, d in rows:
            mark = "  <-- worst" if any(w[1] == gate and w[0] == lap
                                        and w[3] == d for w in worst) else ""
            print(f"        {lap}   {gate:8s} {t:7.2f}   {d:+6.2f}{mark}")
    nm = rec.get("near_misses", [])
    print(f"      near-misses: {len(nm)}")
    for m in nm:
        print(f"        t={m['t']:.1f} {m['cone']} d={m['dist_xy']} z={m['z']}")


def archive(plan_path: Path, result_path, cfg, rec) -> Path:
    dest = planner_mod.next_numbered(str(REPO / "out" / "races" / "race_XXX"))
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(plan_path, dest / "plan.json")
    png = plan_path.with_suffix(".png")
    if png.exists():
        shutil.copy2(png, dest / "plan.png")
    shutil.copy2(cfg.path, dest / "vehicle.toml")
    if result_path is not None:
        shutil.copy2(result_path, dest / "race_result.json")
    return dest


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="vehicle.toml path")
    ap.add_argument("--plan-only", action="store_true",
                    help="plan + render + report, no sim run")
    ap.add_argument("--traj", default=None,
                    help="refly an existing plan JSON instead of replanning")
    ap.add_argument("--sim-time", type=float, default=None,
                    help="override sim duration (default 2x predicted + 30)")
    ap.add_argument("--keep-db", action="store_true",
                    help="keep the sim's betaflight_dbXXX directory")
    ap.add_argument("--sim-repo", default=None,
                    help="elodin sim repo (default: AIGP_SIM_REPO or sibling "
                         "elodin-sim-aigp)")
    args = ap.parse_args()

    cfg = load_config(resolve_path(args.config))

    if args.traj:
        plan_path = resolve_path(args.traj)
        plan_dict = planner_mod.load_plan(plan_path)
        if plan_dict.get("config_sha1") != cfg.sha1:
            print("NOTE  plan was built with a different vehicle.toml "
                  f"({plan_dict.get('config_sha1', '?')[:8]} != "
                  f"{cfg.sha1[:8]}) - follower gains come from the CURRENT "
                  "config, geometry/speeds from the plan")
        plan_events = plan_dict["events"]
        predicted = plan_dict["predicted"]["total_s"]
        print(f"PLAN  reusing {plan_path} "
              f"(predicts {predicted:.1f} s - model prediction, unverified)")
    else:
        p, plan_path = make_plan(cfg)
        plan_events = p.events
        predicted = p.total_s

    if args.plan_only:
        return 0

    sim_repo = Path(args.sim_repo) if args.sim_repo else course_bridge.SIM_REPO
    if not (sim_repo / "sim" / "main.py").exists():
        print(f"ERROR sim repo not found at {sim_repo} "
              "(set AIGP_SIM_REPO or --sim-repo)")
        return 2

    known_results = set(sim_repo.glob("race_result_*.json"))
    known_dbs = set(sim_repo.glob("betaflight_db*"))
    sim_time = args.sim_time or max(120.0, 2.0 * predicted + 30.0)

    rc = run_sim(sim_repo, plan_path, cfg, sim_time)

    result = newest_result(sim_repo, known_results)
    if result is None:
        print(f"\nERROR no new race_result_*.json in {sim_repo} "
              f"(sim exit code {rc})")
        return rc or 3
    rec = json.loads(result.read_text())
    print_report(rec, plan_events)

    dest = archive(plan_path, result, cfg, rec)
    print(f"FILES archived -> {dest}")

    if not args.keep_db:
        for d in set(sim_repo.glob("betaflight_db*")) - known_dbs:
            shutil.rmtree(d, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
