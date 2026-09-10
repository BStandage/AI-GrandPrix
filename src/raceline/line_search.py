"""Per-gate line search against the planner model.

Coordinate descent over the planner's per-gate knobs (crossing offset along
the bar, crossing height, crossing tilt) plus the reversal and stack shape
constants, scored by the model's total time with frame contacts treated as
infeasible. The model is the planner's own 3D thrust-vector speed profile,
which flew within 0.3 s per leg on 2026-09-10, so a tenth here is close to
a tenth in flight. This is a local search from the rule line, restarted from
a few perturbed starts; it is not a global proof.

Run from the repo root:

    python -m raceline.line_search --sweeps 3 --out out/plans/plan_search.json

Writes the best plan found and prints the knob values so they can be copied
into planner.py as the new rule defaults.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np

from raceline import course as cb
from raceline import planner
from raceline.config import load_config

GATES = ["g0", "g1", "g2", "g3", "g4", "g5", "g6", "g7", "g8", "g9",
         "g10-top", "g10-low"]
CONTACT_PENALTY_S = 50.0
REPLAY_MARGIN_M = 0.45   # required LATERAL clearance at every crossing in the calibrated replay (opening half-width 0.75; real drone ~0.3 wider than the replay)
REPLAY_MARGIN_Z_M = 0.65  # height: the replay's altitude loop overshoots climbs by ~0.5 where the flown follower overshoots ~0.2, so only a near-miss of the bar counts


def _score(cfg, course, knobs):
    """Model total time for one knob set; contacts are infeasible."""
    planner.POSE_LAT_OFFSET_M = {g: v for g, v in knobs["lat"].items() if abs(v) > 1e-9}
    planner.POSE_Z_OFFSET_M = {g: v for g, v in knobs["z"].items() if abs(v) > 1e-9}
    planner.POSE_TILT_OVERRIDE_DEG = dict(knobs["tilt"])
    planner.PRE_STUB_M = dict(knobs.get("pre", {}))
    planner.POST_STUB_M = dict(knobs.get("post", {}))
    planner.REVERSAL_SWING_R_M = knobs["swing_r"]
    planner.STACK_U_R_M = knobs["stack_r"]
    planner.STACKED_STANDOFF_M = knobs["stack_so"]
    cfg.planner.a_lat_margin = knobs.get("margin", cfg.planner.a_lat_margin)
    try:
        p = planner.plan(cfg, course)
    except Exception:
        return math.inf, None
    t = float(p.total_s) + CONTACT_PENALTY_S * int(p.meta.get("frame_violations", 0))
    return t, p


def _bounded(v, lo, hi):
    return max(lo, min(hi, v))


TILT_MAX_DEG = 25.0   # a crossing tilted t deg narrows the opening to 1.5*cos(t); the REAL follower runs ~0.3 m wide at 10 m/s (race_049 clipped g6 at 35 deg), 25 keeps 0.68 m of half-width


def search(cfg, course, sweeps=3, seed=0, verbose=True, frozen=(), knobs=None):
    rng = random.Random(seed)
    if knobs is None:
        knobs = {
            "lat": {g: planner.POSE_LAT_OFFSET_M.get(g, 0.0) for g in GATES},
            "z": {g: planner.POSE_Z_OFFSET_M.get(g, 0.0) for g in GATES},
            "tilt": dict(planner.POSE_TILT_OVERRIDE_DEG),
            "pre": dict(planner.PRE_STUB_M),
            "post": dict(planner.POST_STUB_M),
            "swing_r": planner.REVERSAL_SWING_R_M,
            "stack_r": planner.STACK_U_R_M,
            "stack_so": planner.STACKED_STANDOFF_M,
            "margin": cfg.planner.a_lat_margin,
        }
    best_t, best_p = _score(cfg, course, knobs)
    if verbose:
        print(f"start: {best_t:.2f} s")
    # (path, lo, hi, initial step)
    axes = []
    # Bounds are the GATE OPENING and the plant, not the follower (Brian:
    # "do not limit anything"). What the follower cannot track is found by
    # the replay afterwards and that gate's knobs are frozen (see --replay).
    for g in GATES:
        if g in frozen:
            continue
        axes.append((("lat", g), -0.5, 0.5, 0.15))
        axes.append((("z", g), -0.4, 0.4, 0.12))
        if g not in ("g10-top", "g10-low"):
            axes.append((("tilt", g), -TILT_MAX_DEG, TILT_MAX_DEG, 8.0))
            # straight stub lengths at the crossing (the g4->g5 wobble is
            # the exit stub meeting the junction arc at a different radius)
            axes.append((("pre", g), 0.4, 2.5, 0.4))
            axes.append((("post", g), 0.4, 2.5, 0.4))
    axes.append((("swing_r",), 1.5, 3.5, 0.3))
    if "stack_r" not in frozen:
        axes.append((("stack_r",), 0.7, 1.6, 0.15))
    if "stack_so" not in frozen:
        axes.append((("stack_so",), 0.6, 2.0, 0.2))

    def get(path):
        # a gate without a tilt override follows the planner's rule; that
        # state is None here and is restored by deleting the key
        if len(path) == 2:
            return knobs[path[0]].get(path[1])
        return knobs[path[0]]

    def put(path, v):
        if len(path) == 2:
            if v is None:
                knobs[path[0]].pop(path[1], None)
            else:
                knobs[path[0]][path[1]] = v
        else:
            knobs[path[0]] = v

    n_eval = 1
    for sweep in range(sweeps):
        improved = False
        order = list(axes)
        rng.shuffle(order)
        for path, lo, hi, step in order:
            base = get(path)
            base_num = 0.0 if base is None else base
            h = step * (0.5 ** sweep)
            for trial in (base_num + h, base_num - h):
                v = _bounded(trial, lo, hi)
                if base is not None and abs(v - base) < 1e-9:
                    continue
                put(path, v)
                t, p = _score(cfg, course, knobs)
                n_eval += 1
                if t < best_t - 0.005:
                    best_t, best_p = t, p
                    base = v
                    improved = True
                    if verbose:
                        print(f"  sweep {sweep} {'/'.join(path):14s} = {v:+7.2f} -> {best_t:.2f} s  ({n_eval} evals)")
                else:
                    put(path, base)
        if verbose:
            print(f"sweep {sweep} done: {best_t:.2f} s, {n_eval} evals, improved={improved}")
        if not improved:
            break
    return best_t, best_p, knobs, n_eval


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweeps", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="out/plans/plan_search.json")
    ap.add_argument("--replay", action="store_true",
                    help="replay the result; freeze the knobs of any gate the "
                         "follower misses (and the one before it) and search again")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--freeze", default="",
                    help="comma-separated gate labels whose knobs stay at the rule values "
                         "(the gates the SIM rejected: batch_fly results)")
    args = ap.parse_args()
    cfg = load_config()
    course = cb.load_course(laps=cfg.planner.laps)
    t0 = time.time()
    frozen = set(g for g in args.freeze.split(",") if g)
    rule = {
        "lat": dict(planner.POSE_LAT_OFFSET_M), "z": dict(planner.POSE_Z_OFFSET_M),
        "tilt": dict(planner.POSE_TILT_OVERRIDE_DEG),
        "pre": dict(planner.PRE_STUB_M), "post": dict(planner.POST_STUB_M),
    }
    knobs = None
    for rnd in range(args.rounds if args.replay else 1):
        best_t, best_p, knobs, n_eval = search(cfg, course, args.sweeps, args.seed,
                                               frozen=frozen, knobs=knobs)
        if best_p is None:
            print("no feasible plan")
            return 1
        planner.write_plan(best_p, args.out)
        try:
            from raceline import render_plan
            png = str(Path(args.out).with_suffix(".png"))
            render_plan.render(best_p, png)
        except Exception as ex:   # the render is a courtesy, never a blocker
            print(f"  render skipped: {type(ex).__name__}: {ex}")
        if not args.replay:
            break
        from raceline.replay import replay
        r = replay(args.out)
        worst = max(r["events"], key=lambda e: max(abs(e["lat"]) / REPLAY_MARGIN_M, abs(e["dz"]) / REPLAY_MARGIN_Z_M)) if r["events"] else None
        print(f"round {rnd}: model {best_t:.2f} s, replay {r['passed']}/{r['total']} in {r['t']:.1f} s, "
              f"worst {worst['label'] if worst else '-'} lat {worst['lat']:+.2f} dz {worst['dz']:+.2f}" if worst else "")
        # a pass with less than REPLAY_MARGIN_M to spare at any crossing is
        # treated as a miss: the calibrated replay flips on tenths and the
        # real drone runs 0.3 m wider than it
        over = [e for e in r["events"] if abs(e["lat"]) > REPLAY_MARGIN_M or abs(e["dz"]) > REPLAY_MARGIN_Z_M]
        if r["passed"] >= r["total"] and not over:
            break
        miss = r["missed"][0] if r["missed"] else worst["label"]
        # the gate (and its predecessor) already frozen at rule values and
        # the replay still fails there: the cause is global speed, not a
        # knob - pull the cornering share down a notch and try again
        if miss in frozen and knobs["margin"] > 0.6:
            knobs["margin"] = round(knobs["margin"] - 0.05, 3)
            print(f"  {miss} already frozen: cornering share -> {knobs['margin']:.2f}")
            continue
        if r["missed"]:
            print(f"  replay missed {miss}")
        else:
            print(f"  replay margin violated at {miss} (lat {worst['lat']:+.2f} dz {worst['dz']:+.2f})")
        idx = GATES.index(miss.split("-")[0] if miss not in GATES else miss) if (miss.split("-")[0] in GATES or miss in GATES) else None
        to_freeze = {miss}
        if idx is not None and idx > 0:
            to_freeze.add(GATES[idx - 1])
        for g in to_freeze:
            frozen.add(g)
            for key in ("lat", "z", "tilt", "pre", "post"):
                if g in rule[key]:
                    knobs[key][g] = rule[key][g]
                else:
                    knobs[key].pop(g, None)
        if miss.startswith("g10"):
            # the stack shape itself is what the replay could not fly:
            # fall back to the flown 35 s stack and take it out of the search
            knobs["stack_so"], knobs["stack_r"] = 2.0, 1.35
            frozen.update({"stack_so", "stack_r"})
        print(f"  freezing {sorted(to_freeze)} at rule values, searching again")
    print(f"BEST {best_t:.2f} s after {n_eval} evals in {time.time() - t0:.0f} s -> {args.out}")
    print("knobs:", json.dumps(knobs, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
