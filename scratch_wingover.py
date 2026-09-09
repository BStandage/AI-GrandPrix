import sys
sys.path.insert(0, "src")
import numpy as np
from raceline.config import load_config
from raceline import planner, course as course_bridge

cfg = load_config("config/vehicle.toml")
course = course_bridge.load_course(laps=cfg.planner.laps)

for climb in (0.0, 1.0, 2.0, 3.0):
    cfg.planner.reversal_climb_m = climb
    p = planner.plan(cfg, course)
    s = np.array(p.s)
    # hairpin section g6->g8
    g6 = next(e for e in p.events if e["label"] == "g6")
    g7 = next(e for e in p.events if e["label"] == "g7")
    g8 = next(e for e in p.events if e["label"] == "g8")
    m = (s >= g6["s"]) & (s <= g8["s"])
    vmin = float(np.min(p.v[m]))
    # peak altitude on the hairpin
    zmax = float(np.max(np.array(p.pos)[m, 2]))
    fv = p.meta.get("frame_violations", 0)
    print("climb=%.1f m: total %.1f s | g6-g8 vmin %.2f | g7 v %.2f | "
          "apex z %.2f | frame_viol %d"
          % (climb, p.total_s, vmin, g7["v"], zmax, fv))
