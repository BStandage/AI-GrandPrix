# AI-GrandPrix

Sim repo: `elodin-sim-aigp`, checked out next to this one.

## Status (2026-09-16)

- Course: `data/course_map.json` (published). 10 gates, gate 9 double, 23 crossings over 2 laps.
- Follower flies a plan on dead reckoning: FC attitude and accel, baro altitude, camera fixes on any gate it can match to the map. No ground truth anywhere in the loop.
- Sim, noisy detector, 35 deg mount + 120 deg lens: the levers at about k = 0.4 flew clean 4 of 4 in 50 s. Every crossing is a lateral fix; the climb rate is a lever because a climbing turn crosses off centre.
- Archer (from its blackbox): Betaflight 4.4.3, acc_1G 2048, baro yes, no mag, ANGLE mode.
- Never flown on the real drone.

## The plan: levers in, time out

`config/vehicle_cam35_120.toml` holds the camera (35 deg mount, 120 deg
lens; change to what `camcal` measures) and the five levers at their race
values: tilt, attitude slew, lateral margin, top speed, climb rate. One
number k moves all five between a safe floor (k = 0) and the race values
(k = 1); the planner solves the line and the time is whatever comes out.
From `src`:

```
python -m raceline.ladder --k 0.5 --config ../config/vehicle_cam35_120.toml
```

writes `config/ladder/vehicle_k050.toml` and `out/plans/plan_LADDER_k050.json`
and prints the model time. Fly that rung in the sim on vision (Docker running):

```
AIGP_STATE_SOURCE=deadreckon AIGP_CAM_TILT_DEG=35 AIGP_CAM_HFOV_DEG=120 python -m raceline.batch_fly --timeout 300 --config ../config/ladder/vehicle_k050.toml ../out/plans/plan_LADDER_k050.json
```

| k | model time | sim on vision |
|---|---|---|
| 0.2 | 63 s | not flown |
| 0.33 | 56 s | 1 clean of 2, one sim stall |
| 0.5 | 49 s | 2 clean of 3 (47 s) |
| 0.8 | 36 s | not flown |
| 1.0 | 30 s | fails |

Binary search on k: top of the search is the safe end, bottom is k = 1.

Benchmark every mount, lens and k in one go (from `src`, Docker running; rows stream to `out/benchmark.csv`):

```
python -m raceline.benchmark --tilts 20 35 45 --lenses 90 120 --ks 0.33 0.5 --seeds 1 2 3
```

How it all works, for anyone: `docs/HOW_IT_FLIES.md`, `docs/how_it_flies.png`, `docs/how_it_flies.pptx`; with links into the code: `docs/HOW_IT_FLIES_DETAILED.md`.

Other sim commands, from `src`:

```
python -m raceline.batch_fly ../out/plans/plan_RACE.json          # the 29 s plan on ground truth
python -m raceline.batch_fly --solver solvers.seeker --timeout 400 ../out/plans/plan_RACE.json   # fallback, ACRO only in sim
python -m perception.video_probe ../event_files/archer_AIGP.mkv --still 1.6   # detector on the real video, width jitter
```

`AIGP_CAM_NOISE=0` = perfect detector (diagnostics only). Traces: `out/flightlogs/race_NNN.csv`, `dr_NNN.csv`.

## Archer (Orin, from `src`, props off until the last line)

```
python3 -m hardware.bench --port /dev/ttyTHS1 info
python3 -m hardware.bench --port /dev/ttyTHS1 rc-test --props-off
python3 -m hardware.bench --port /dev/ttyTHS1 arm-test --props-off
python3 -m hardware.bench --port /dev/ttyTHS1 drift --seconds 60
python3 -m hardware.camcal --dist 6.0 --dz <m> --port /dev/ttyTHS1        # prints --fy --cam-hfov --cam-tilt
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> --fy <px> --cam-hfov <deg> --pilot follower --config ../config/ladder/vehicle_k033.toml --traj ../out/plans/plan_LADDER_k033.json --dry-run
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> --fy <px> --cam-hfov <deg> --pilot follower --config ../config/ladder/vehicle_k033.toml --traj ../out/plans/plan_LADDER_k033.json --arm
```

`--map-north here`: drone on the start line pointing along gate 1 when the runtime starts. `--pilot seeker` = fallback.
The pilot arms and flips MSP OVERRIDE and ANGLE on the radio; MSP owns the four sticks only (`set msp_override_channels_mask = 15`, CLI, once). The pilot can always take the sticks back.

## Race day

1. Fly the lowest k that is clean in the sim. Clean twice, jump to the fastest k you brought.
2. Fail, fly the midpoint. Each heat halves the interval.
3. A complete slow run beats an incomplete fast one.

## Docs

| | |
|---|---|
| Bench steps, day 1, the 15 min slot | `src/PQ_PROCEDURE.md` |
| Course, Orin, firmware facts | `src/PQ_SPECS_INTAKE.md` |
| Sim setup | `docs/ELODIN_SIM_SETUP.md`, `docs/GETTING_STARTED_RACING_LINE.md` |
| Stack design, Betaflight | `docs/RACING_LINE_STACK.md`, `docs/WHAT_IS_BETAFLIGHT.md` |
| Solvers | `src/solvers/README.md` |
