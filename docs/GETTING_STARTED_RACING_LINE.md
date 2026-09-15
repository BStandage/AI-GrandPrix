# Getting started: fly the racing line

A walkthrough for a new dev, from zero to a scored race in the sim.
Follow it top to bottom; every step says what you should see before moving on.

**Just want to WATCH a flight, right now, without any setup?** Install
Docker Desktop, clone both repos side by side (step 0), then double-click
`run_race_docker.cmd` in the sim repo. It builds the whole environment in
a container (first run ~10 min), flies the newest racing-line plan, and
opens the viewer for you. You do NOT need the WSL setup, uv, or a
Betaflight build for this - those are only for headless development runs.
Commands like `elodin run sim/main.py` in the sim repo's README assume
that full setup exists and will fail with Betaflight build errors without
it; that failure means "do step 0 or use Docker," not that you broke
something.

Race length comes from `[planner] laps` in the toml; the default is the
September format, 2 laps = **23 events**: the start crossing of g0, then
11 crossings per lap (10 gates, the stacked gate g8 counted twice), the
last one being g0 as the finish line. The course is the organizers'
**published** map (`data/course_map.json`, 2026-09-15): 10 gates on an
85 x 165 ft floor, the drone starts on the dashed line 7.3 m behind gate
1. Code labels are traversal order, so **gK = organizer gate K+1**: g0 is
gate 1, g8-top/g8-low is the double gate 9, g9 is gate 10.

**The mental model, in one paragraph:** there are two repos side by side.
`AI-GrandPrix` (this one) owns the course map, the planner, the follower, and
the config. `elodin-sim-aigp` is the simulator (it runs in WSL with a real
Betaflight flight controller in the loop). You tune exactly ONE file -
`config/vehicle.toml` - and run exactly one command - `race.py` - which plans
a racing line, flies it headless, and prints lap times and gates from the
sim's run record. That's the whole loop.

```
config/vehicle.toml --> planner --> out/plans/plan_XXX.json (+ .png to eyeball)
                                            |
                                            v
                              follower (RACE_SOLVER=solvers.follower)
                                            |
        race report + archive <-- elodin sim + Betaflight SITL (WSL)
```

---

## 0. Prerequisites

Never touched a drone or heard of Betaflight? Read
`docs/WHAT_IS_BETAFLIGHT.md` first (10 minutes) - it explains what the
flight controller does, why sticks are rates not angles, and what hardware
the September race actually uses.

You need both repos as siblings (the code assumes this layout, overridable
with `AIGP_SIM_REPO`). Ask Brian for access, then:

```
cd <wherever>/GitRepos
git clone https://github.com/bstandageusf/AI-GrandPrix.git
git clone https://github.com/BStandage/elodin-sim-aigp.git
```

```
.../GitRepos/AI-GrandPrix/        <- this repo (planner, follower, config, race.py)
.../GitRepos/elodin-sim-aigp/     <- the sim (elodin + Betaflight SITL, runs in WSL)
```

Two environments are involved:

| Side | Used for | Needs |
|---|---|---|
| Windows Python 3.11+ | tests, planning, the PNG render | `numpy`, `matplotlib` |
| WSL (Ubuntu) | actually flying | full sim setup - follow `docs/ELODIN_SIM_SETUP.md` once |

No WSL, or on a Mac, or the setup guide fought you? The sim repo has a
Docker path: `run_race_docker.cmd` (Windows) or `docker compose up --build`
(any OS) runs the identical sim in a container, repos bind-mounted, editor
attaching at localhost:2240. First build takes ~10 minutes; after that it
behaves like the WSL setup. One warning that has cost real hours: a
running sim container holds port 2240, and headless WSL runs will wedge
until you `docker compose down`. Close your watch sessions.

Quick check that the WSL side is ready (run from Windows):

```
wsl -e bash -lc "cd /mnt/c/Users/<you>/Documents/GitRepos/elodin-sim-aigp && uv run python -c 'import numpy; print(\"ok\")' && ls betaflight/obj/main/betaflight_SITL.elf"
```

You should see `ok` and the `.elf` path. If the `.elf` is missing, the sim
setup guide covers building Betaflight SITL.

---

## 1. Run the unit tests (10 seconds, no sim needed)

From Windows (or WSL, either works):

```
cd AI-GrandPrix/tests
python -m unittest test_raceline -v
```

Expect 25 tests. The one to know about is
`test_tracker_replay_completes_all_events`: it feeds the planner's own
trajectory through the sim's gate tracker and demands every crossing in
order and direction. If the planner ever produces a path that wouldn't
score, this test fails before you waste a sim run. Known state
(2026-09-15): three standoff-rule tests (`test_degenerate_leg_uses_heading_comparison`,
`test_travel_reversal_triggers_turn_standoff`,
`test_lateral_accel_within_planner_budget`) are stale from the September
planner changes and fail; the other 22 pass.

---

## 2. Make a plan - no sim required

Easiest (works with zero Windows Python setup - it runs in the WSL env):

```
race.cmd --plan-only          (from a terminal in AI-GrandPrix)
```

or, if you have numpy on Windows Python:

```
cd AI-GrandPrix
python race.py --plan-only
```

This reads `config/vehicle.toml` + `data/course_map.json` and prints
something like:

```
PLAN  23 events, 244 m path, config 187cdd88
      predicts total 26.3 s (lap0 12.2s  lap1 12.0s) - model prediction, unverified; baseline 225.3 s
CHECK frame contacts: 0 samples (MUST be 0 - the referee crashes the run on contact)
CHECK speed-profile minimum: 3.31 m/s at s=93.0 m (nearest event: g8-top, +1.2 m along-path)
      binding constraint there: tilt/curvature; kappa=0.86 1/m ...
      profile ruled by: tilt/curvature 86%, accel-slew 7%, climb/descent 5%
      ...per-event crossing speeds...
FILES plan -> out/plans/plan_004.json
      render -> out/plans/plan_004.png
```

(`out/plans/plan_RACE.json` is THE flown plan - the naming convention is
that plan_RACE is always the current race plan and the branch name
carries its time. Numbered plans are candidates.)

Three things to know:

- Predicted times are model predictions, unverified. Only a run's
  tracker record (and at the September race, only real flight) counts.
- The CHECK lines are your plan sanity gate. Frame contacts must be 0 or
  race.py refuses to fly the plan. The speed-profile minimum names the
  slowest point and where it sits: a low minimum around the g8 stack or
  the g5 hairpin is expected (the course genuinely reverses there); a
  near-zero minimum anywhere else means the planner produced a kinked
  path - fix the plan (planner params), do not fly it and then tune the
  follower around the kink.
- Open the PNG. `out/plans/plan_XXX.png` shows the course top-down with
  the path colored by planned speed, the v_min marked with a red x, and the
  full v(s) profile underneath. Thirty seconds of eyeballing catches what
  numbers hide.

Iterating on planner/limit values with `--plan-only` takes ~2 seconds per
cycle. Do your rough thinking here before spending 5-minute sim runs.

---

## 3. Fly it

Double-click `race.cmd` in the AI-GrandPrix root (or run it from any
Windows terminal). It hops into WSL for you and runs the whole loop.

Equivalent, from a WSL shell, if you prefer living there:

```
cd /mnt/c/Users/<you>/Documents/GitRepos/elodin-sim-aigp
uv run python ../AI-GrandPrix/race.py
```

(Want to WATCH the flight instead of racing headless? The sim repo's
`run_race.cmd solvers.follower` starts the sim with your newest plan and
opens the Elodin editor viewport. That's for eyeballs - lap times you
quote should come from headless `race.cmd` runs.)

What happens, in order:

1. A fresh plan is built and reported (same as step 2).
2. The sim launches headless: `uv run elodin run sim/main.py` with
   `RACE_SOLVER=solvers.follower` and the plan path in `AIGP_TRAJ`.
   Runtime is sized automatically from the predicted time (~0.8x realtime,
   so expect roughly 4-6 minutes).
3. While flying you'll see three kinds of lines:
   - `[GATE] lap 0 g3 (event 3) at t=12.41s ...` - the tracker scoring a
     crossing. Count these; you want one per crossing (23 over two
     laps: event 0 is the start crossing of g0, events 11 and 22 are the
     lap-closing crossings of g0).
   - `[CRASH] hit g4 frame at t=... - run INVALID, scoring frozen` - the
     COLLISION REFEREE. Touching any gate frame invalidates the whole run
     on the spot; everything the drone does afterward is unscored
     wandering. If you see this line, the run is over no matter what
     flies next.
   - `[RL] t= 12.0 s= 34.5 p=(...) v=2.87 xtrack=0.21 ...` - the follower's
     5 Hz heartbeat. `xtrack` is cross-track error to the plan; happy is
     <=0.3 m cruising.
4. When the sim ends, race.py finds the new `race_result_XXX.json` and
   prints the report:

```
RACE  23/23 COMPLETE   total 29.55 s  (baseline 225.3 s -> -195.7)
      lap 0: 15.72 s   lap 1: 13.84 s
      event  lap gate      t(s)   dt-vs-plan(s)
        0   g0         ...
        ...
      near-misses: 0
FILES archived -> out/races/race_000
```

(those are the real numbers of race_160, 2026-09-15, the first clean
two-lap run on the published map.) Read it as:

- `23/23 COMPLETE` (and `crashed: false`) is the only line that matters
  first. The tracker is ordered - one missed gate blocks all scoring after
  it, so `7/23` usually means one bad corner, not sixteen. A frame contact
  invalidates the run outright even if every gate ticked. The referee
  records only the FIRST contact; everything the drone does afterwards
  is unscored wandering (and the follower, which is never told, will
  keep fighting the frozen gate - see `MAX_RETRIES_PER_GATE` in the
  follower).
- `dt-vs-plan` is per-crossing time versus the plan, aligned at the
  first gate (so takeoff time doesn't pollute it). The three worst are
  flagged - that's where the follower is losing time to tracking, or the
  plan is optimistic.
- Everything is archived to `out/races/race_XXX/`: the plan, the PNG,
  the result JSON, and the exact `vehicle.toml` that produced it. Any
  number you quote is reproducible from that folder.

Useful flags (all pass straight through `race.cmd` too):

```
race.cmd --traj out/plans/plan_004.json   # refly a saved plan
race.cmd --sim-time 300                   # force a longer run
race.cmd --keep-db                        # keep the sim's flight DB for debugging
```

---

## What actually happens when you run race.cmd

The whole pipeline, in order. Nothing is hidden; every stage is one file
you can read.

```
race.cmd
  |
  | hops into WSL, runs race.py
  v
[1] LOAD CONFIG        config/vehicle.toml (strict loader, typos fail loudly)
  v
[2] LOAD COURSE        data/course_map.json (published) -> sim.pq_course
                       -> the gate crossings in sim coordinates: the
                       start crossing of g0 + [planner] laps x 11 (the
                       stacked gate g8 counts twice per lap, g0 closes
                       each lap). The sim origin is the map's start
                       line, 7.3 m behind g0.
  v
[3] PLAN THE LINE      src/raceline/planner.py, ~2 seconds, offline:
                       a. anchors: launch just off the deck, a diagonal
                          blend up to cruise (takeoff is part of the
                          line), then for every crossing points before/
                          at/after the opening (wider standoff where the
                          travel turns hard - one global rule covers the
                          g8 out-and-back and the g5 switchback)
                       b. path: a STRAIGHT segment through every opening
                          (pre -> center -> post is linear; the hole is
                          never curved), smooth spline everywhere else.
                          The base planner crosses along each gate's
                          normal; the optimizer (line_opt --free) may
                          LEARN the crossing pose instead - offset in the
                          opening, heading up to pose_angle_max_deg off
                          the normal, and standoff lengths - all found by
                          search, nothing per-gate by hand
                       c. speed limit at every point = min of:
                          - v_max
                          - tilt (corner accel = g * tan(max_tilt_deg))
                          - attitude slew (a_lat_rate_max vs curvature change)
                          - yaw rate (nose must keep up with the path)
                          - climb/descent rate on slopes
                          - v_gate near a crossing, but ONLY where the
                            line actually bends there (straight-through
                            gates are flown at full speed)
                       d. forward pass (can it accelerate that fast?) and
                          backward pass (can it brake in time?) sharing
                          one tilt budget
                       e. timestamps + feedforward accel fall out
  v
[4] PLAN ARTIFACTS     out/plans/plan_XXX.json (the trajectory contract)
                       out/plans/plan_XXX.png  (speed-colored line - look at it)
                       terminal: predicted lap times (model prediction,
                       unverified) + the CHECK line (slowest point + which
                       physical limit rules the profile)
  v
[5] LAUNCH THE SIM     elodin physics + a real Betaflight flight
                       controller, in lockstep, headless. Env vars carry
                       the plan: RACE_SOLVER=solvers.follower,
                       AIGP_TRAJ=<plan>, AIGP_VEHICLE_TOML=<config>
  v
[6] EVERY PHYSICS TICK (the actual flying, src/solvers/follower.py)
                       sensors -> StateSource (ground truth today,
                                  estimator on the real drone)
                       -> Tracker: progress along the line is INTEGRATED
                          (velocity projected on the path tangent, then a
                          local nearest-point refinement - a plain global
                          nearest search cuts corners); aim at a carrot a
                          little ahead; desired accel = plan feedforward
                          + position/velocity correction (with caps on
                          how hard retries and rejoins may pull)
                       -> rc_backend (thrust-vector control, ACRO):
                          attitude points the accel vector (tilt-error P
                          plus body-rate damping; sticks are RATE
                          commands), THROTTLE carries the vector's
                          magnitude at the achieved tilt, capped at
                          hover-minus whenever the drone rides high.
                          Yaw follows the path tangent but goes neutral
                          inside every gate window.
                       -> Betaflight: sticks -> motor speeds
                       -> physics moves the drone
                       meanwhile the referee (sim/pq_course.py) checks
                       every position against the NEXT expected opening:
                       right order, right direction, through the 1.5 m
                       hole - only then does a gate count. Touching any
                       frame at any time freezes scoring: run invalid.
  v
[7] REPORT + ARCHIVE   race_result JSON -> terminal report: gates N/M,
                       lap times, per-gate delta vs the plan (worst 3
                       flagged). Everything that produced the number
                       (plan + result + exact vehicle.toml) is copied to
                       out/races/race_XXX/ so it is reproducible. (Known
                       wart: the sim overwrites race_result_000.json
                       instead of numbering up, which can make race.py
                       miss the result and skip the archive - fix
                       pending.)
```

Two takeaways:

- The SOLUTION is found offline in step [3], in 2 seconds, from the map
  and the toml. The flight only tracks it. That is why the same planner
  works on an unseen September course: new map in, new line out.
- The tracker in step [6] is ordered: miss one gate and nothing after it
  scores. Clean early gates beat a fast ragged lap.

## 4. Your first tune

The workflow is always: **edit `config/vehicle.toml` -> `race.py` -> compare
against the last archive.** Nothing else is a tuning surface; if you find
yourself wanting to edit planner or follower code to go faster, the number
you want is probably missing from the toml - raise that instead.

**Working alongside others**: `config/vehicle.toml` is the team baseline -
don't churn it with experiments. Copy it and iterate on your own file:

```
copy config\vehicle.toml config\dev_yourname.toml
race.cmd --config config\dev_yourname.toml
```

Every archived run records which config produced it, so comparing your
variant against the baseline is just two `out/races/` folders side by
side. When your variant beats the baseline *at 23/23*, propose copying it
back into `vehicle.toml`.

Know where the baseline sits first: the race config runs with the limits
essentially open (`max_tilt_deg` 89, `v_max_mps` 20, `v_gate_mps` = v_max,
`a_lat_rate_max` 165) and the plan is ruled by tilt/curvature - so the
levers below are for pulling speed OUT (a safer line) far more often than
for adding it. A safe first experiment: fly a ladder rung
(`cd src && python -m raceline.ladder --targets 40`, then
`race.py --config config/ladder/vehicle_40s.toml --traj out/plans/plan_LADDER_40s.json`)
and compare its `dt-vs-plan` against the race plan's. Compare: gates
(still 23/23, no crash?), total, and the worst `dt-vs-plan` events.
**If gates drop or a [CRASH] appears, revert the last change before touching
anything else.**

The levers, in the order they usually pay off:

1. `max_tilt_deg` / `a_lat_margin` - corner speed and follower authority
   together (the profile is ruled by tilt/curvature today)
2. `a_lat_rate_max` - attitude slew; the binding limit on a heavy airframe
3. `a_accel_max` / `a_brake_max` - straight-line ramps
4. `v_max_mps` (and `v_gate_mps`, kept equal to it) - matters only once
   straights stop being accel-limited
5. `[follower] lookahead_m`, `kp_pos` - only if `xtrack` grows or gates
   get clipped; fix tracking, then go back to speed levers

The loader is strict: a typo'd key fails loudly instead of silently doing
nothing, so edit boldly.

---

## 5. What if you want to make your own solver?

Tuning the toml changes how the EXISTING race solver flies. Write your own
solver when you want different behavior entirely: a vision-based pilot, a
measurement flight, a wild experiment. A solver is one file with one
function:

```python
def autopilot(update: SensorUpdate) -> RCCommand:
```

The sim calls it every physics tick. `update` carries everything the
drone knows (IMU, baro, camera frame, ground-truth pose for now, next
expected gate); you return 4 RC stick values plus the arm switch. That is
the whole contract - same one the real drone uses in September.

The path:

```
cd src/solvers
cp _template.py my_solver.py     # 80 commented lines that already fly
```

Open `my_solver.py`. It arms, takes off, and hovers in front of gate 0.
Everything you keep is labeled; the part you replace is marked
`YOUR BRAIN GOES HERE` - it computes one thing, a desired horizontal
acceleration, and the shared loops turn that into sticks:

- `raceline.rc_backend` gives you the proven throttle, tilt, and yaw
  loops. Do not reinvent them; every solver flying today uses them.
- `raceline.course` gives you every gate position and crossing direction.
- `raceline.planner` gives you a full timed racing line if you want one.

Run it:

```
RACE_SOLVER=solvers.my_solver AIGP_SIM_TIME=60 uv run elodin run sim/main.py
```

(from the sim repo in WSL - this NEEDS the full WSL setup from step 0;
use `run_race.cmd solvers.my_solver` to watch it in the editor, or
`run_race_docker.cmd solvers.my_solver` if you only have Docker). The gate tracker scores every solver
automatically - a `race_result_XXX.json` appears no matter who is flying.

Three rules, non-negotiable:

1. Numbers go in `config/vehicle.toml`, not in your code. If your solver
   needs a knob that does not exist, add the key to the schema in
   `raceline/config.py` - the strict loader is what keeps tuning sane.
2. Nothing per-gate, ever (`RESTRICTIONS.md`).
3. If it flies well, it proves it the same way as everyone: gates scored
   by the tracker, complete with zero frame contacts, on a `race_result` record.

The knob-to-key table and the full solver list live in
`src/solvers/README.md`.

## 6. The rules (read RESTRICTIONS.md, seriously)

- **Every toml parameter is global.** The schema cannot express a
  per-gate value, and nobody adds one. "g4 keeps clipping so nudge g4" is
  the banned move - the fix must improve the *rule* (standoff logic,
  gains, limits) for every gate at once. The per-crossing offsets that do
  exist in `planner.py` are OUTPUTS of `raceline.line_search` (a search
  against the model with the replay guard), recorded as seeds for the
  next search - never hand-set, and never a per-gate fix for a flown miss.
  The September course is published, but the sim follower flies on ground
  truth and the real drone will not; margins you spend in the sim are
  gone on the day.
- **Language discipline:** planner numbers are "the model predicts,
  unverified." The sim tracker's record is a sim result. Only real flight
  validates.
- Reference points: the stop-and-center pilot, 24/24 in 225.3 s (2 laps,
  estimate map, 2026-08-27); the current stack, **23/23 in 29.55 s** over
  two laps with zero contacts on the published map (race_160,
  2026-09-15; laps 15.72 + 13.84, model 26.3). Beat the second one.

## 7. The speed ladder (race-day binary search)

`cd src && python -m raceline.ladder --targets 60 50 40 35` builds one
plan + toml per target model time: every crossing through the centre of
its opening, the envelope (tilt, v_max, slew, a_lat_margin) scaled by one
level k. On the day: fly the slowest rung; clean -> the fastest; fails ->
the midpoint (`--k`). Details and the heat procedure:
`src/PQ_PROCEDURE.md`.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `solvers.follower needs AIGP_TRAJ=...` | You launched the sim directly without a plan. Use `race.py`, or set `AIGP_TRAJ` yourself. |
| `elodin sim repo not found at ...` | Repos aren't siblings. Set `AIGP_SIM_REPO=/path/to/elodin-sim-aigp`. |
| `ConfigError: ... unknown or missing` | Typo or deleted key in vehicle.toml - the message names it. |
| Sim starts but drone never lifts | Betaflight eeprom/build issue - `docs/ELODIN_SIM_SETUP.md`, rebuild + `configure_betaflight.py`. A run spamming `Arming disabled: BOOTGRACE` is a boot flake: kill it and relaunch. |
| `io port 2240 is already in use` | Another sim owns the render port - usually a Docker sim container someone left up (`docker compose down`), or a zombie `elodin render-server` (kill it). |
| `No simulation ticks executed` at boot | elodin boot flake (roughly 1 in 4 launches under heavy cycling). Kill leftovers and relaunch; no config change needed. |
| WSL runs get slow / commands fail with odd exit codes | WSL itself is wedged (hours of sim cycling does this). From PowerShell: `wsl --shutdown`, wait a few seconds, rerun. |
| `INCOMPLETE: N/23` | Find the first missing `[GATE]` line; watch `xtrack` just before it. Plan issue (CHECK line / PNG kink) vs tracking issue (xtrack blows up) tells you which layer to look at. If a `[CRASH]` line precedes it, everything after is the follower fighting a frozen gate - the crash is the only finding. |
| `No module named 'raceline'` | You ran `python -m raceline.<x>` from the repo root. The package lives in `src/`: `cd src` first (plan paths then need `../`). `race.py` works from the root. |
| Render skipped | matplotlib missing in that env - harmless; plan JSON is unaffected. |
| Windows-side `ModuleNotFoundError: numpy` | `pip install numpy matplotlib`, or just do everything from the WSL uv env. |

Deeper design reference (planner math, follower layers, file formats):
`docs/RACING_LINE_STACK.md`.
