# Getting started: fly the racing line

A walkthrough for a new dev, from zero to a scored 2-lap race in the sim.
Follow it top to bottom; every step says what you should see before moving on.

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

Expect **14 tests, OK**. The one to know about is
`test_tracker_replay_completes_all_events`: it feeds the planner's own
trajectory through the sim's gate tracker and demands all 24 crossings in
order and direction. If the planner ever produces a path that wouldn't
score, this test fails before you waste a sim run.

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
PLAN  24 events, 249 m path, config 3cee2540
      predicts total 88.2 s (lap0 41.4s  lap1 44.9s) - model prediction, unverified; baseline 225.3 s
CHECK speed-profile minimum: 0.30 m/s at s=72.8 m (nearest event: g7, -3.6 m along-path)
      ...per-event crossing speeds...
FILES plan -> out/plans/plan_004.json
      render -> out/plans/plan_004.png
```

Three things to internalize here:

- **Predicted times are model predictions, unverified.** Only a run's
  tracker record (and at the September race, only real flight) counts.
- **The CHECK line is your plan sanity gate.** It reports the slowest point
  on the speed profile and where it sits. A near-zero minimum at the **g7
  switchback is expected** - the course genuinely reverses direction there.
  A near-zero minimum anywhere ELSE means the planner produced a kinked
  path: fix the plan (planner params), do not fly it and then tune the
  follower around the kink.
- **Open the PNG.** `out/plans/plan_XXX.png` shows the course top-down with
  the path colored by planned speed, the v_min marked with a red x, and the
  full v(s) profile underneath. Thirty seconds of eyeballing catches what
  numbers hide.

Iterating on planner/limit values with `--plan-only` takes ~2 seconds per
cycle. Do your rough thinking here before spending 5-minute sim runs.

---

## 3. Fly it

**Double-click `race.cmd` in the AI-GrandPrix root** (or run it from any
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
3. While flying you'll see two kinds of lines:
   - `[GATE] lap 0 g3 (event 3) at t=12.41s ...` - the tracker scoring a
     crossing. Count these; you want 24.
   - `[RL] t= 12.0 s= 34.5 p=(...) v=2.87 xtrack=0.21 ...` - the follower's
     1 Hz heartbeat. `xtrack` is cross-track error to the plan; happy is
     <=0.3 m cruising.
4. When the sim ends, race.py finds the new `race_result_XXX.json` and
   prints the report:

```
RACE  24/24 COMPLETE   total 97.31 s  (baseline 225.3 s -> -128.0)
      lap 0: 45.92 s
      lap 1: 47.10 s
      event  lap gate      t(s)   dt-vs-plan(s)
        0   g0         4.61    +0.00
        ...
      near-misses: 0
FILES archived -> out/races/race_000
```

Read it as:

- **`24/24 COMPLETE` is the only line that matters first.** The tracker is
  ordered - one missed gate blocks all scoring after it, so `17/24` usually
  means one bad corner, not seven.
- **`dt-vs-plan`** is per-crossing time versus the plan, aligned at the
  first gate (so takeoff time doesn't pollute it). The three worst are
  flagged - that's where the follower is losing time to tracking, or the
  plan is optimistic.
- **Everything is archived** to `out/races/race_XXX/`: the plan, the PNG,
  the result JSON, and the exact `vehicle.toml` that produced it. Any
  number you quote is reproducible from that folder.

Useful flags (all pass straight through `race.cmd` too):

```
race.cmd --traj out/plans/plan_004.json   # refly a saved plan
race.cmd --sim-time 300                   # force a longer run
race.cmd --keep-db                        # keep the sim's flight DB for debugging
```

---

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
side. When your variant beats the baseline *at 24/24*, propose copying it
back into `vehicle.toml`.

A safe first experiment: crossing speed dominates lap time (24 gate windows
per race), so in `[limits]` try `v_gate_mps = 3.0 -> 3.5`, then re-run.
Compare: gates (still 24/24?), total, and the worst `dt-vs-plan` events.
**If gates drop below 24/24, revert the last change before touching
anything else.**

The levers, in the order they usually pay off:

1. `v_gate_mps` / `gate_window_m` - speed carried through crossings
2. `max_tilt_deg` - corner speed and follower authority together
3. `a_accel_max` / `a_brake_max` - straight-line ramps
4. `v_max_mps` - matters only once straights stop being accel-limited
5. `[follower] lookahead_m`, `kp_pos` - only if `xtrack` grows or gates
   get clipped; fix tracking, then go back to speed levers

The loader is strict: a typo'd key fails loudly instead of silently doing
nothing, so edit boldly.

---

## 5. The rules (read RESTRICTIONS.md, seriously)

- **Every parameter is global.** The schema cannot express a per-gate
  value, and nobody adds one. "g4 keeps clipping so nudge g4" is the banned
  move - the fix must improve the *rule* (standoff logic, gains, limits)
  for every gate at once. The September qualifier is an unseen course with
  ~15 min/day of practice; per-gate anything teaches us nothing.
- **Language discipline:** planner numbers are "the model predicts,
  unverified." The sim tracker's record is a sim result. Only real flight
  validates.
- Baseline to beat: the stop-and-center reference pilot, **24/24 in
  225.3 s**.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `solvers.follower needs AIGP_TRAJ=...` | You launched the sim directly without a plan. Use `race.py`, or set `AIGP_TRAJ` yourself. |
| `elodin sim repo not found at ...` | Repos aren't siblings. Set `AIGP_SIM_REPO=/path/to/elodin-sim-aigp`. |
| `ConfigError: ... unknown or missing` | Typo or deleted key in vehicle.toml - the message names it. |
| Sim starts but drone never lifts | Betaflight eeprom/build issue - `docs/ELODIN_SIM_SETUP.md`, rebuild + `configure_betaflight.py`. |
| `INCOMPLETE: N/24` | Find the first missing `[GATE]` line; watch `xtrack` just before it. Plan issue (CHECK line / PNG kink) vs tracking issue (xtrack blows up) tells you which layer to look at. |
| Render skipped | matplotlib missing in that env - harmless; plan JSON is unaffected. |
| Windows-side `ModuleNotFoundError: numpy` | `pip install numpy matplotlib`, or just do everything from the WSL uv env. |

Deeper design reference (planner math, follower layers, file formats):
`docs/RACING_LINE_STACK.md`.
