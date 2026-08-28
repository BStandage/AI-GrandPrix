"""Bridge to the elodin sim repo's course module (sim.pq_course).

pq_course is deliberately elodin-free (numpy + stdlib), owns the ONE
map->sim transform, and defines the 24-event crossing sequence - so both
the planner (standalone) and the follower (inside the sim process) consume
the course through it and are guaranteed the same frame.

Inside `elodin run` the module is already importable. Standalone (planner,
tests) we add the sim repo root to sys.path: env AIGP_SIM_REPO, defaulting
to the sibling checkout `../elodin-sim-aigp`.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

AIGP_REPO = Path(__file__).resolve().parents[2]
SIM_REPO = Path(os.environ.get("AIGP_SIM_REPO",
                               str(AIGP_REPO.parent / "elodin-sim-aigp")))


def pq_course():
    try:
        from sim import pq_course as mod
        return mod
    except ImportError:
        pass
    if not SIM_REPO.is_dir():
        raise RuntimeError(
            f"elodin sim repo not found at {SIM_REPO}; set AIGP_SIM_REPO")
    p = str(SIM_REPO)
    if p not in sys.path:
        sys.path.insert(0, p)
    from sim import pq_course as mod
    return mod


def load_course(**kwargs):
    return pq_course().load_course(**kwargs)


def map_path() -> Path:
    return Path(pq_course().DEFAULT_MAP_PATH)
