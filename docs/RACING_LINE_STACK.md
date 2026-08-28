# Racing-line stack (elodin sim)

One tuning surface, one command. You edit `config/vehicle.toml`, you run
`race.py`, you read the report. Nothing else.

```
config/vehicle.toml ──> raceline.planner ──> out/plans/plan_XXX.json (+ .png)
                              │                        │
      data/course_map.json ───┘                        ▼
                                     raceline.follower (RACE_SOLVER)
                                                       │
                              race_result JSON <── elodin sim (WSL)
```

## Quickstart (WSL shell)

```bash
cd /mnt/c/Users/brian/Documents/GitRepos/elodin-sim-aigp
uv run python ../AI-GrandPrix/race.py                # plan + fly + report
uv run python ../AI-GrandPrix/race.py --plan-only    # iterate numbers in seconds
```

`--plan-only` also works from Windows Python (needs numpy; matplotlib for
the render). Sim env setup: `docs/ELODIN_SIM_SETUP.md`.

## The rules (RESTRICTIONS.md is authority)

- **All tuning is GLOBAL.** vehicle.toml cannot express a per-gate value,
  and no one adds one. The uniform crossing-speed rule
  (`v_gate_mps` within `gate_window_m` of ANY crossing) stays uniform.
- Predicted times from the planner are **model predictions, unverified**.
  The sim tracker's `race_result` is the run record; only real flight
  proves anything.
- Baseline to beat: the stop-and-center reference pilot, **24/24 in
  225.3 s** (2026-08-27).

## What lives where

| Piece | Path |
|---|---|
| Tuning surface | `config/vehicle.toml` (strict loader: typos fail loudly) |
| Planner | `src/raceline/planner.py` |
| Follower (solver) | `src/raceline/follower.py`, `RACE_SOLVER=raceline.follower` |
| One-command loop | `race.py` |
| Course bridge | `src/raceline/course.py` (imports the sim repo's `sim.pq_course`) |
| Tests | `tests/test_raceline.py` (`python -m unittest test_raceline` from `tests/`) |
| Plans / runs | `out/plans/`, `out/races/race_XXX/` (plan + result + exact toml) |

## How the plan is built

Anchors per crossing (`center ± standoff·normal`) with a two-value standoff:
the base value normally, the larger `anchor_standoff_turn_m` on both sides
of any junction whose actual TRAVEL turns more than `turn_angle_deg`
(covers the g10 out-and-back and the g7 switchback with one rule).
Centripetal Catmull-Rom through the anchors, then a speed profile:
pointwise ceiling from tilt (`g·tan(max_tilt)·a_lat_margin` vs curvature),
yaw rate (`max_yaw_rate_rps` vs heading rate), climb/descent slope, and the
uniform crossing window — then forward/backward friction-circle passes for
accel/brake. Timestamps and feedforward accel fall out.

Every plan run prints the CHECK line: the speed-profile minimum and where
it sits. A near-zero minimum somewhere unexpected means a bad plan —
fix the plan, don't tune the follower around it. (The known-real minimum
is the g7 switchback: the course genuinely reverses there.)

## How the follower flies it

Arc-length carrot: nearest path sample -> target `lookahead_m` ahead ->
`a = a_ff + kp_pos·Δp + kd_pos·Δv`, clamped to the tilt circle; altitude
loop with the plan's vz as feedforward; nose points along the path tangent
`yaw_lookahead_m` ahead. State comes through `StateSource` — ground truth
today; the estimator milestone swaps in a new source and touches nothing
else.

## Tuning levers, in the order to try them

1. `v_gate_mps` / `gate_window_m` — crossing speed dominates lap time
   (24 crossings).
2. `max_tilt_deg` (raises corner speed AND follower authority together) —
   but check the plan report's "profile ruled by" line first: on a heavy
   airframe `a_lat_rate_max` (attitude slew) binds before tilt does, and
   raising tilt then does nothing.
3. `a_accel_max` / `a_brake_max` — straight-line ramps.
4. `v_max_mps` — only matters once straights stop being accel-limited.
5. Follower `lookahead_m` / `kp_pos` if cross-track error (the `xtrack`
   debug print) grows before gates get missed.

After every change: `race.py`, read gates + total + the worst dt-vs-plan
events. If gates drop below 24/24, revert the last change first.

## From sim to the real drone

The organizer FAQ (2026-08-28) confirmed the physical interface is the
same shape as the sim's: the Jetson sends **RC commands over UART** and
receives **IMU data** back — nothing else. So the stack splits cleanly:

| Layer | Sim today | Real drone (September) |
|---|---|---|
| `config/vehicle.toml` | SITL plant seeds | **same file** — `[vehicle]`/`[thrust]`/`[limits]` refilled from day-1 sysid + `diff all` on the 8" Archer Block 2 |
| Planner + plan JSON | unchanged | **unchanged** (new course map in, plan out) |
| Follower `Tracker` | unchanged | **unchanged** (pure: state + plan → desired accel/yaw) |
| Follower `StateSource` | ground-truth pose from the sim | **swapped**: estimator on FC IMU (UART) + camera gate fixes |
| RC output | `RCCommand` → sim bridge packets | **swapped**: same channel values written to the FC over UART (Betaflight ≤ 2026.6.1) |
| Scoring | `sim/pq_course.RaceTracker` | the organizers' clock |

The two "swapped" rows are deliberately thin adapters — that was the
design constraint from day one. Nobody retunes the planner or the tracker
to go to hardware; they get a new state source and a new wire.

Sim-only conveniences that do NOT carry: ground truth (obviously),
determinism (one survey flight, then it has to work — RESTRICTIONS.md),
and lockstep timing (the real loop is free-running; command-rate limits
come from the FC link).
