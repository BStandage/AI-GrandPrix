# AI-GrandPrix

Sim repo: `elodin-sim-aigp`, checked out next to this one.

## Status (2026-09-16)

- Course: `data/course_map.json` (published). 10 gates, gate 9 double, 23 crossings over 2 laps.
- Follower flies a plan on dead reckoning: FC attitude and accel, baro altitude, camera fixes on any gate it can match to the map. No ground truth anywhere in the loop.
- Sim, noisy detector: `config/vehicle_cam35_120.toml` holds the camera and the envelope the estimator tolerates; `race.py --plan-only --config` solves the plan. 35 deg mount + 120 deg lens: 50 s plan clean 3 of 3 (flies in 50 s). Every crossing is a lateral fix; the climb rate is capped because a climbing turn crosses off centre.
- Archer (from its blackbox): Betaflight 4.4.3, acc_1G 2048, baro yes, no mag, ANGLE mode.
- Never flown on the real drone.

## The race plan (50 s, 35 deg camera, 120 deg lens)

The camera and the flight envelope the estimator tolerates live in
`config/vehicle_cam35_120.toml`. The planner solves the plan from it, no
target time. Repo root:

```
python race.py --plan-only --config config/vehicle_cam35_120.toml --out out/plans/plan_CAM35_120.json
```

Fly it in the sim on vision, from `src` (Docker running):

```
AIGP_STATE_SOURCE=deadreckon AIGP_CAM_TILT_DEG=35 AIGP_CAM_HFOV_DEG=120 python -m raceline.batch_fly --timeout 260 --config ../config/vehicle_cam35_120.toml ../out/plans/plan_CAM35_120.json
```

Faster or slower: edit the envelope in that toml (`max_tilt_deg`,
`a_lat_rate_max`, `a_lat_margin`, `v_max_mps`, `vz_up_max`), solve, fly
three seeds (`AIGP_SEED=1 2 3`). A different camera: change `cam_tilt_deg`
and `cam_hfov_deg` to what `camcal` measured, same two commands.

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
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> --fy <px> --cam-hfov <deg> --pilot follower --config ../config/vehicle_cam35_120.toml --traj ../out/plans/plan_CAM35_120.json --dry-run
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> --fy <px> --cam-hfov <deg> --pilot follower --config ../config/vehicle_cam35_120.toml --traj ../out/plans/plan_CAM35_120.json --arm
```

`--map-north here`: drone on the start line pointing along gate 1 when the runtime starts. `--pilot seeker` = fallback.
The pilot arms and flips MSP OVERRIDE and ANGLE on the radio; MSP owns the four sticks only (`set msp_override_channels_mask = 15`, CLI, once). The pilot can always take the sticks back.

## Race day

1. First flight: the plan above with the envelope as committed (50 s in the sim).
2. Clean twice: raise the envelope in the toml, solve, fly. Fail: lower it.
3. A complete slow run beats an incomplete fast one.

## Docs

| | |
|---|---|
| Bench steps, day 1, the 15 min slot | `src/PQ_PROCEDURE.md` |
| Course, Orin, firmware facts | `src/PQ_SPECS_INTAKE.md` |
| Sim setup | `docs/ELODIN_SIM_SETUP.md`, `docs/GETTING_STARTED_RACING_LINE.md` |
| Stack design, Betaflight | `docs/RACING_LINE_STACK.md`, `docs/WHAT_IS_BETAFLIGHT.md` |
| Solvers | `src/solvers/README.md` |
