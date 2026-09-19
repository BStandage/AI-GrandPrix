"""
Benchmark: camera mount x lens x lever level x seeds, all in the sim on
vision, one table at the end. Nothing hard-coded: every combination is
solved and flown. Run from src/ with Docker running.

    python -m raceline.benchmark --tilts 20 35 45 --lenses 90 120 --ks 0.33 0.5 --seeds 1 2 3
    python -m raceline.benchmark --tilts 35 --lenses 120 --ks 0.2 0.33 0.5 0.8 --seeds 1 2 3 --out ../out/bench_cam35.csv

For each (tilt, lens): a config is derived from --config with cam_tilt_deg
and cam_hfov_deg replaced, the ladder solves each k (plan + toml), and each
plan is flown --seeds times with AIGP_STATE_SOURCE=deadreckon and the
synthetic camera at that tilt and lens. Results go to --out (CSV, one row
per flight) and a summary table prints: clean count, mean time.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import time
from pathlib import Path

from raceline.config import AIGP_REPO

LADDER_DIR = AIGP_REPO / "config" / "ladder"
PLANS_DIR = AIGP_REPO / "out" / "plans"
RESULT_RE = re.compile(r"->\s+(\w+)\s+(\d+)/(\d+)\s+time\s+([\d.]+|None)")


def derive_config(base: Path, tilt: float, lens: float) -> Path:
    s = base.read_text(encoding="utf-8")
    for key, val in (("cam_tilt_deg", tilt), ("cam_hfov_deg", lens)):
        s, n = re.subn(rf"^({re.escape(key)}\s*=\s*)([-\d.eE+]+)", rf"\g<1>{val:.1f}", s, count=1, flags=re.M)
        if n != 1:
            raise SystemExit(f"{base.name}: {key} not found (the base config must carry the camera keys)")
    out = LADDER_DIR / f"_bench_cam{tilt:.0f}_{lens:.0f}.toml"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(s, encoding="utf-8")
    return out


def solve_rung(cfg: Path, k: float, tilt: float, lens: float):
    """raceline.ladder --k: returns (plan_path, toml_path, model_s)."""
    tag = f"k{int(round(k * 100)):03d}_cam{tilt:.0f}_{lens:.0f}"
    r = subprocess.run([sys.executable, "-m", "raceline.ladder", "--k", f"{k}", "--config", str(cfg),
                        "--cam-tilt", f"{tilt}", "--cam-hfov", f"{lens}"],
                       capture_output=True, text=True, cwd=str(AIGP_REPO / "src"))
    m = re.search(rf"^\s*{re.escape(tag)}\s+[\d.]+\s+([\d.]+)", r.stdout, flags=re.M)
    if not m:
        print(r.stdout[-800:], r.stderr[-800:])
        raise SystemExit(f"ladder did not produce {tag}")
    return PLANS_DIR / f"plan_LADDER_{tag}.json", LADDER_DIR / f"vehicle_{tag}.toml", float(m.group(1))


def fly(plan: Path, toml: Path, tilt: float, lens: float, seed: int, timeout: float) -> dict:
    env = dict(os.environ, AIGP_STATE_SOURCE="deadreckon", AIGP_CAM_TILT_DEG=f"{tilt}", AIGP_CAM_HFOV_DEG=f"{lens}",
               AIGP_SEED=str(seed), MSYS_NO_PATHCONV="1")
    t0 = time.monotonic()
    r = subprocess.run([sys.executable, "-m", "raceline.batch_fly", "--timeout", f"{timeout}", "--config", str(toml), str(plan)],
                       capture_output=True, text=True, cwd=str(AIGP_REPO / "src"), env=env)
    m = RESULT_RE.search(r.stdout)
    status, passed, total, t = (m.group(1), int(m.group(2)), int(m.group(3)), m.group(4)) if m else ("ERROR", 0, 23, "None")
    return {"status": status, "passed": passed, "total": total, "time_s": None if t == "None" else float(t),
            "wall_s": round(time.monotonic() - t0, 1)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--config", default=str(AIGP_REPO / "config" / "vehicle_cam35_120.toml"))
    ap.add_argument("--tilts", type=float, nargs="+", default=[20.0, 35.0, 45.0])
    ap.add_argument("--lenses", type=float, nargs="+", default=[90.0, 120.0])
    ap.add_argument("--ks", type=float, nargs="+", default=[0.33, 0.5])
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--out", default=str(AIGP_REPO / "out" / "benchmark.csv"))
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    n_total = len(args.tilts) * len(args.lenses) * len(args.ks) * len(args.seeds)
    n = 0
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["tilt", "lens", "k", "model_s", "seed", "status", "passed", "total", "time_s", "wall_s"])
        w.writeheader()
        for tilt in args.tilts:
            for lens in args.lenses:
                cfg = derive_config(Path(args.config), tilt, lens)
                for k in args.ks:
                    plan, toml, model_s = solve_rung(cfg, k, tilt, lens)
                    for seed in args.seeds:
                        n += 1
                        r = fly(plan, toml, tilt, lens, seed, args.timeout)
                        row = {"tilt": tilt, "lens": lens, "k": k, "model_s": model_s, "seed": seed, **r}
                        rows.append(row)
                        w.writerow(row)
                        fh.flush()
                        print(f"[{n}/{n_total}] tilt {tilt:.0f} lens {lens:.0f} k {k:.2f} (model {model_s:.1f} s) seed {seed}: "
                              f"{r['status']} {r['passed']}/{r['total']} {r['time_s']}", flush=True)
                cfg.unlink(missing_ok=True)

    print(f"\n{'tilt':>4} {'lens':>4} {'k':>5} {'model':>6} {'clean':>7} {'mean time':>9}")
    for tilt in args.tilts:
        for lens in args.lenses:
            for k in args.ks:
                sel = [r for r in rows if r["tilt"] == tilt and r["lens"] == lens and r["k"] == k]
                clean = [r for r in sel if r["status"] == "COMPLETE"]
                stalled = [r for r in sel if r["status"] in ("TIMEOUT", "ERROR")]
                mean_t = sum(r["time_s"] for r in clean) / len(clean) if clean else float("nan")
                print(f"{tilt:4.0f} {lens:4.0f} {k:5.2f} {sel[0]['model_s']:6.1f} {len(clean):>3}/{len(sel) - len(stalled):<3} {mean_t:9.1f}"
                      + (f"  ({len(stalled)} stalled)" if stalled else ""))
    print(f"\nrows -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
