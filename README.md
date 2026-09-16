# AI-GrandPrix

Sim repo: `elodin-sim-aigp`, checked out next to this one.

## Status (2026-09-16)

- Course: `data/course_map.json` (published). 10 gates, gate 9 double, 23 crossings over 2 laps.
- Follower flies a plan on dead reckoning: FC attitude and accel, baro altitude, camera fixes on any gate it can match to the map. No ground truth anywhere in the loop.
- Sim, noisy detector, 20 deg camera: 60 s rung clean, 50 s rung clean. 40 s rung needs a 35 deg mount. plan_RACE fails.
- Archer (from its blackbox): Betaflight 4.4.3, acc_1G 2048, baro yes, no mag, ANGLE mode.
- Never flown on the real drone.

## Sim (Docker running, from `src`)

```
AIGP_STATE_SOURCE=deadreckon python -m raceline.batch_fly --timeout 300 ../out/plans/plan_LADDER_60s.json
AIGP_STATE_SOURCE=deadreckon AIGP_CAM_TILT_DEG=35 AIGP_CAM_HFOV_DEG=120 python -m raceline.batch_fly ../out/plans/plan_LADDER_40s.json
python -m raceline.batch_fly ../out/plans/plan_RACE.json          # ground truth
python -m raceline.batch_fly --solver solvers.seeker --timeout 400 ../out/plans/plan_RACE.json   # fallback, ACRO only in sim
python -m raceline.ladder --targets 60 50 40 35                     # rebuild rungs
python -m perception.video_probe ../event_files/archer_AIGP.mkv     # detector on the real video
```

`AIGP_CAM_NOISE=0` = perfect detector (diagnostics only). Traces: `out/flightlogs/race_NNN.csv`, `dr_NNN.csv`.

## Archer (Orin, from `src`, props off until the last line)

```
python3 -m hardware.bench --port /dev/ttyTHS1 info
python3 -m hardware.bench --port /dev/ttyTHS1 rc-test --props-off
python3 -m hardware.bench --port /dev/ttyTHS1 arm-test --props-off
python3 -m hardware.bench --port /dev/ttyTHS1 drift --seconds 60
python3 -m hardware.camcal --dist 6.0 --dz <m> --port /dev/ttyTHS1        # prints --fy --cam-hfov --cam-tilt
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> --fy <px> --cam-hfov <deg> --pilot follower --traj ../out/plans/plan_LADDER_60s.json --dry-run
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> --fy <px> --cam-hfov <deg> --pilot follower --traj ../out/plans/plan_LADDER_60s.json --arm
```

`--map-north here`: drone on the start line pointing along gate 1 when the runtime starts. `--pilot seeker` = fallback.

## Race day

1. 60 s rung. Clean twice, next rung up.
2. Fail, midpoint (`raceline.ladder --k`).
3. A complete slow run beats an incomplete fast one.

## Docs

| | |
|---|---|
| Bench steps, day 1, the 15 min slot | `src/PQ_PROCEDURE.md` |
| Course, Orin, firmware facts | `src/PQ_SPECS_INTAKE.md` |
| Sim setup | `docs/ELODIN_SIM_SETUP.md`, `docs/GETTING_STARTED_RACING_LINE.md` |
| Stack design, Betaflight | `docs/RACING_LINE_STACK.md`, `docs/WHAT_IS_BETAFLIGHT.md` |
| Solvers | `src/solvers/README.md` |
