# Racing-line stack (elodin sim)

One tuning surface, one command. You edit `config/vehicle.toml`, you run
`race.py`, you read the report. Nothing else.

```
config/vehicle.toml --> raceline.planner --> out/plans/plan_XXX.json (+ .png)
                              |                        |
      data/course_map.json ---+                        v
                                     solvers.follower (RACE_SOLVER)
                                                       |
                              race_result JSON <-- elodin sim (WSL)
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
  and no one adds one. The crossing-speed rule (`v_gate_mps` within
  `gate_window_m` of a crossing, applied only where the line bends there)
  is one rule for every gate. Line shape beyond the toml is LEARNED by
  the optimizer (`line_opt --free`), never hand-set per gate.
- **The collision referee**: touching any gate frame freezes scoring and
  invalidates the run, regardless of gates already ticked. Plans with
  frame contacts are refused before flight (the CHECK line).
- Predicted times from the planner are **model predictions, unverified**.
  The sim tracker's `race_result` is the run record; only real flight
  proves anything.
- Reference points: the stop-and-center pilot, 24/24 in 225.3 s over 2
  laps (estimate map, 2026-08-27); the current stack, **23/23 in
  29.55 s** over 2 laps with zero contacts on the published map
  (race_160, 2026-09-15). Race length = `[planner] laps` (default 2 =
  23 events: start g0 + 11 crossings per lap).
- The course is PUBLISHED (`data/course_map.json`, organizer table of
  2026-09-15; the 2026-08-27 overhead estimate is kept as
  `data/course_map_overhead_estimate.json`). Labels are traversal order:
  gK = organizer gate K+1, g8 = the double gate (top opening south at
  4.05 m - an estimate, unpublished - then the low opening north), g9 =
  gate 10. The drone starts on the dashed line 7.3 m behind gate 1
  (`meta.start`); the sim spawns there.

## What lives where

| Piece | Path |
|---|---|
| Tuning surface | `config/vehicle.toml` (strict loader: typos fail loudly) |
| Planner | `src/raceline/planner.py` |
| Follower (solver) | `src/solvers/follower.py`, `RACE_SOLVER=solvers.follower` |
| Line optimizer | `src/raceline/line_opt.py` (`--free` = learned crossing poses, needs scipy) |
| Line search (the working one) | `src/raceline/line_search.py` - per-crossing knobs searched against the model with the calibrated replay as guard; its outputs are the seed dicts at the top of `planner.py` |
| Batch validation | `src/raceline/batch_fly.py` - flies plans through the Docker sim and tabulates the referee (`cd src` first) |
| Speed ladder | `src/raceline.ladder` -> `config/ladder/vehicle_<T>s.toml` + `out/plans/plan_LADDER_<T>s.json`, centred crossings, for race-day binary search |
| Course map | `data/course_map.json` (published), loader `src/common/course_map.py`, 3D viewer `viz/course_viewer.html` |
| RC backend (shared loops) | `src/raceline/rc_backend.py` |
| One-command loop | `race.py` |
| Course bridge | `src/raceline/course.py` (imports the sim repo's `sim.pq_course`) |
| Tests | `tests/test_raceline.py` (`python -m unittest test_raceline` from `tests/`) |
| Plans / runs | `out/plans/`, `out/races/race_XXX/` (plan + result + exact toml) |

## How the plan is built

Takeoff is part of the line: a launch anchor just off the deck and a
diagonal blend to cruise, so the profile accelerates from the first
meter. Then anchors per crossing (`center +- standoff*normal`) with a
two-value standoff: the base value normally, the larger
`anchor_standoff_turn_m` on both sides of any junction whose actual
TRAVEL turns more than `turn_angle_deg` (covers the g8 out-and-back and
the g5 switchback with one rule; labels are traversal order on the
published map, g0 = organizer gate 1, gK = gate K+1). The path is a STRAIGHT segment through
every opening (pre -> center -> post is linear - the hole is never
curved) with centripetal Catmull-Rom between gates. The base planner
crosses along each gate's normal; `line_opt --free` may LEARN the
crossing pose instead (offset in the opening, heading within
`pose_angle_max_deg` of the normal, standoff scales), found by search
against predicted time with contacts priced at 50 s. The speed profile:
pointwise ceiling from tilt (`g*tan(max_tilt)*a_lat_margin` vs
curvature), attitude slew (`a_lat_rate_max` vs curvature change), yaw
rate, climb/descent slope, and the crossing window where the line bends
- then forward/backward friction-circle passes for accel/brake.
Timestamps and feedforward accel fall out.

Every plan run prints the CHECK line: the speed-profile minimum and where
it sits. A near-zero minimum somewhere unexpected means a bad plan -
fix the plan, don't tune the follower around it. (The known-real minimums are the g5
switchback and the g8 stack: the course genuinely reverses there.)

## How the follower flies it

Progress along the line is INTEGRATED (velocity projected on the path
tangent, refined by a local nearest search - a plain global nearest
search cuts corners), then an arc-length carrot `lookahead_m` ahead:
`a = a_ff + kp_pos*deltap + kd_pos*deltav` with the velocity-error term
capped (`v_err_max`) so rejoining the line slow never demands a lunge.
The RC backend is thrust-vector control in ACRO: attitude points the
accel vector (tilt-error P plus `kw_att` body-rate damping; sticks are
RATE commands), throttle supplies the vector's magnitude at the achieved
tilt off the measured thrust curve, capped at hover-minus when riding
high. The nose follows the path tangent `yaw_lookahead_m` ahead but goes
NEUTRAL inside gate windows (a railed yaw demand steals motor authority
at the crossing). State comes through `StateSource` - ground truth
today; the estimator milestone swaps in a new source and touches nothing
else.

## Tuning levers, in the order to try them

1. `v_gate_mps` / `gate_window_m` - crossing speed dominates lap time
   (a dozen windows per lap; straight-through gates already sprint).
2. `max_tilt_deg` (raises corner speed AND follower authority together) -
   but check the plan report's "profile ruled by" line first: on a heavy
   airframe `a_lat_rate_max` (attitude slew) binds before tilt does, and
   raising tilt then does nothing.
3. `a_accel_max` / `a_brake_max` - straight-line ramps.
4. `v_max_mps` - only matters once straights stop being accel-limited.
5. Follower `lookahead_m` / `kp_pos` if cross-track error (the `xtrack`
   debug print) grows before gates get missed.

After every change: `race.py`, read gates + total + the worst dt-vs-plan
events. If gates drop or a [CRASH] appears, revert the last change
first.

## From sim to the real drone

The organizer FAQ (2026-08-28) confirmed the physical interface is the
same shape as the sim's: the Jetson sends **RC commands over UART** and
receives **IMU data** back - nothing else. So the stack splits cleanly:

| Layer | Sim today | Real drone (September) |
|---|---|---|
| `config/vehicle.toml` | SITL plant seeds | **same file** - `[vehicle]`/`[thrust]`/`[limits]` refilled from day-1 sysid + `diff all` on the 8" Archer Block 2 |
| Planner + plan JSON | unchanged | **unchanged** (new course map in, plan out) |
| Follower `Tracker` | unchanged | **unchanged** (pure: state + plan -> desired accel/yaw) |
| Follower `StateSource` | ground-truth pose from the sim | **swapped**: estimator on FC IMU (UART) + camera gate fixes |
| RC output | `RCCommand` -> sim bridge packets | **swapped**: same channel values sent as MSP RC frames over `/dev/ttyTHS1` at 115200 (Betaflight 4.5.x on the Archer; organizer libraries `~/target/msp/msp.py`, `msp_rc.py`) |
| Attitude loop | our thrust-vector loop at 1 kHz on ground truth (ACRO) | **open decision**: MSP is polled at 30-50 Hz, and the organizers say flight-rate loops belong inside Betaflight -> ANGLE mode with angle setpoints from us is the candidate; the follower needs that output variant |
| Scoring | `sim/pq_course.RaceTracker` | the organizers' clock |

The two "swapped" rows are deliberately thin adapters - that was the
design constraint from day one. Nobody retunes the planner or the tracker
to go to hardware; they get a new state source and a new wire.

Sim-only conveniences that do NOT carry: ground truth (obviously),
determinism (one survey flight, then it has to work - RESTRICTIONS.md),
and lockstep timing (the real loop is free-running; command-rate limits
come from the FC link).

Optimizer evolution (multi-start, topology enumeration, full optimal
control): `docs/OPTIMIZER_ROADMAP.md`.
