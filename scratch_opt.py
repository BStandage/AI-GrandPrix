import sys
import time
sys.path.insert(0, "src")
from raceline.config import load_config
from raceline import line_opt, planner

cfg = load_config("config/vehicle.toml")
base = planner.plan(cfg)
print("current line: %.1f s" % base.total_s, flush=True)

# ONE fast optimize_free round first (quick read), then report.
t0 = time.time()
u, fp, p = line_opt.optimize_free(cfg, rounds=1, verbose=True)
print("optimize_free(1 round): %.1f s  (%.0fs, frame_viol %d)"
      % (p.total_s, time.time() - t0, p.meta.get("frame_violations", 0)),
      flush=True)
for lbl in ("g4", "g5", "g6", "g7", "g8", "g9"):
    b = next(e for e in base.events if e["label"] == lbl)
    o = next(e for e in p.events if e["label"] == lbl)
    print("  %-4s %.1f -> %.1f m/s" % (lbl, b["v"], o["v"]), flush=True)
if p.meta.get("frame_violations", 0) == 0:
    planner.write_plan(p, "out/plans/plan_opt.json")
    print("saved out/plans/plan_opt.json", flush=True)
else:
    print("NOT saved (touches a frame)", flush=True)
