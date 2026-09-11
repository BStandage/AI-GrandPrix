"""Fly candidate plans headless in the Docker sim and tabulate the referee.

The kinematic replay is a guard, not a plant; the sim is the plant. This runs
each plan through the real follower, Betaflight SITL and referee, one after
another, and reports gates passed, contacts, and the referee time. It is
validation of planner candidates, not per-gate tuning: nothing is edited
between runs, every run is the same solver on a different plan file.

    python -m raceline.batch_fly out/plans/plan_a.json out/plans/plan_b.json ...
    python -m raceline.batch_fly --glob "out/plans/plan_gs*.json"

Results are appended to out/flightlogs/batch_results.csv and printed.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

from raceline.config import AIGP_REPO

SIM_REPO = Path(os.environ.get("AIGP_SIM_REPO", AIGP_REPO.parent / "elodin-sim-aigp"))
CONTAINER = "elodin-sim-aigp-sim-1"
RACE_RE = re.compile(r"\[RACE\] .*gates_passed=(\d+)/(\d+) total_time=([0-9.]+|--)s lap_times=\[([^\]]*)\] status=(\w+)")
CRASH_RE = re.compile(r"\[CRASH\] hit (\S+) frame at t=([0-9.]+)s")


def _compose(*args, env=None, timeout=180):
    return subprocess.run(["docker", "compose", *args], cwd=str(SIM_REPO), env=env,
                          capture_output=True, text=True, timeout=timeout)


def _logs():
    r = subprocess.run(["docker", "logs", CONTAINER], capture_output=True, text=True, timeout=60)
    return (r.stdout or "") + (r.stderr or "")


def fly(plan_path: Path, timeout_s: float = 200.0, solver: str = "solvers.follower") -> dict:
    """Fly one plan; returns the referee result dict."""
    rel = plan_path.resolve().relative_to(AIGP_REPO.resolve())
    env = dict(os.environ)
    env["RACE_SOLVER"] = solver
    env["AIGP_TRAJ"] = "/work/AI-GrandPrix/" + str(rel).replace("\\", "/")
    _compose("down", "--remove-orphans", timeout=120)
    up = _compose("up", "-d", env=env)
    if up.returncode != 0:
        return {"plan": str(rel), "error": up.stderr[-300:]}
    t0 = time.time()
    result = {"plan": str(rel), "passed": None, "total": None, "time_s": None,
              "laps": "", "status": "TIMEOUT", "crash": "", "crash_t": None}
    while time.time() - t0 < timeout_s:
        time.sleep(3.0)
        log = _logs()
        m = RACE_RE.search(log)
        if m:
            result.update(passed=int(m.group(1)), total=int(m.group(2)),
                          time_s=(None if m.group(3) == "--" else float(m.group(3))),
                          laps=m.group(4), status=m.group(5))
            c = CRASH_RE.search(log)
            if c:
                result.update(crash=c.group(1), crash_t=float(c.group(2)))
            break
        if "Traceback" in log:
            result["status"] = "ERROR"
            result["crash"] = log[log.index("Traceback"):][:200].replace("\n", " ")
            break
    _compose("down", "--remove-orphans", timeout=120)
    result["wall_s"] = round(time.time() - t0, 1)
    # first-gate-to-last-gate time from the newest flight log: the referee
    # total includes arming and takeoff, which varied by up to 2 s between
    # identical solvers before the boot-grace hold (2026-09-10)
    try:
        logs = sorted(glob.glob(str(AIGP_REPO / "out" / "flightlogs" / "race_*.csv")), key=os.path.getmtime)
        if logs:
            with open(logs[-1], newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            cred = []
            g_prev = int(rows[0]["gate"])
            for r in rows:
                g = int(r["gate"])
                if g != g_prev:
                    cred.append(float(r["t"])); g_prev = g
            air = [float(r["t"]) for r in rows if float(r["z"]) > 0.2][:1]
            result["log"] = os.path.basename(logs[-1])
            result["liftoff_s"] = round(air[0], 2) if air else None
            result["g0_s"] = round(cred[0], 2) if cred else None
            result["core_s"] = round(cred[-1] - cred[0], 2) if len(cred) >= 2 else None
    except Exception as ex:
        result["log"] = f"log parse failed: {ex}"
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plans", nargs="*")
    ap.add_argument("--glob", default=None)
    ap.add_argument("--timeout", type=float, default=200.0)
    ap.add_argument("--solver", default="solvers.follower", help="RACE_SOLVER module for every flight in this batch")
    args = ap.parse_args()
    plans = [Path(p) for p in args.plans]
    if args.glob:
        plans += [Path(p) for p in sorted(glob.glob(args.glob))]
    if not plans:
        print(__doc__)
        return 2
    out_csv = AIGP_REPO / "out" / "flightlogs" / "batch_results.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    new_file = not out_csv.exists()
    rows = []
    for p in plans:
        model = None
        try:
            model = json.load(open(p))["predicted"]["total_s"]
        except Exception:
            pass
        print(f"flying {p} (model {model}) with {args.solver} ...", flush=True)
        r = fly(p, timeout_s=args.timeout, solver=args.solver)
        r["model_s"] = model
        rows.append(r)
        print(f"   -> {r.get('status')} {r.get('passed')}/{r.get('total')} "
              f"time {r.get('time_s')} crash {r.get('crash') or '-'} ({r.get('wall_s')} s wall)", flush=True)
        with open(out_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["plan", "model_s", "passed", "total", "time_s", "core_s", "g0_s", "liftoff_s", "log", "laps", "status", "crash", "crash_t", "wall_s", "error"])
            if new_file:
                w.writeheader(); new_file = False
            w.writerow({k: r.get(k) for k in w.fieldnames})
    print()
    print(f"{'plan':34s} {'model':>6s} {'passed':>6s} {'time':>6s} {'core':>6s} {'g0':>5s} {'status':>8s} crash  log")
    for r in rows:
        print(f"{r['plan'][-34:]:34s} {str(r.get('model_s')):>6s} {str(r.get('passed')):>6s} {str(r.get('time_s')):>6s} {str(r.get('core_s')):>6s} {str(r.get('g0_s')):>5s} {str(r.get('status')):>8s} {r.get('crash') or '-'}  {r.get('log','')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
