# PQ procedure: bench, cage, race

Spec: `docs/specs/20260818_PQ_Technical_Spec_0001.pdf`; the published
course and the Orin quickstart PDFs are alongside it.

- 15 min per day on the real track. 2 laps = a complete run; incomplete
  runs rank by gates passed.
- Track 85 x 165 ft, 10 gates, 1.5 m inner opening, gate 9 is the double.
  `data/course_map.json` is the published table (gK = organizer gate K+1).
- Start = the dashed line 7.3 m behind gate 1 (`meta.start`).
- No survey step: the race is the published map + a ladder rung.

## Day 0: bench (props OFF, drone on the bench)

1. USB-C to the micro-USB port, `ssh dcl@192.168.55.1` (pw `dcl`). If it
   hangs, give the laptop's RNDIS interface 192.168.55.100/24.
2. `sudo ~/target/bringup-check.sh`, then `ls /dev/video0`.
3. `sudo ~/target/msp/setup_jetson_uart.sh --apply` once per board.
4. Save the Betaflight `diff all` to the laptop AND the Jetson before any
   change. Read `map`, `msp_override_channels_mask`, the failsafe.
5. Our link, from `AI-GrandPrix/src` (needs pyserial, numpy, opencv):

```
python3 -m hardware.bench --port /dev/ttyTHS1 info                 # firmware, modes, blockers, ANGLE on AUX2
python3 -m hardware.bench --port /dev/ttyTHS1 rc-test --props-off  # FC must echo our sticks in MSP_RC
python3 -m hardware.bench --port /dev/ttyTHS1 arm-test --props-off # ARMED flag seen, then disarm
python3 -m hardware.bench --port /dev/ttyTHS1 telemetry --hz 20 --seconds 20
```

   Note the attitude rate and RTT it prints: that is the link budget.
6. Numbers the runtime needs, from `telemetry` and a tape measure:

| Measure | How | Runtime flag |
|---|---|---|
| accel counts per g | `acc` z at rest (the runtime also measures it before takeoff) | `--acc-lsb-per-g` (default auto) |
| pitch sign | push the nose down; if pitch reads positive | `--pitch-nose-down-positive` |
| map north | heading while pointing along gate 1 | `--map-north <deg>` |
| camera tilt | horizon row in a level frame, or the mount angle | `--cam-tilt <deg>` (default 20) |
| focal length | lens spec or a checkerboard | `--fy <px>` (default 1000) |
| lens field of view | lens spec | `--cam-hfov <deg>` (default 90) |

7. Camera: `~/target/live-view-imu.py --msp /dev/ttyTHS1`, page at
   http://192.168.55.1:8080/. Then the detector on a real gate: HSV
   thresholds in `perception/detectors/hsv_classic.py` (used by `hardware/runtime.py`)
   must find the ring and the range from ring size must match the tape.
8. Dry run, props off, FC stays disarmed:

```
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north <deg> --cam-tilt <deg> --fy <px> --pilot follower --traj ../out/plans/plan_LADDER_60s.json --dry-run
```

   The log shows the estimate, the detections and the sticks it would
   send. Carry the drone toward a gate: the fix residual must shrink.
9. `sudo shutdown -h now`, wait for the LED, then pull power.

## Camera mount

Set the mount as high as the lens allows: 30-35 deg up with a 120 deg
lens (that geometry flies plan_RACE in the sim), about 25 deg with a
90 deg lens. Measure the tilt after mounting and pass it as `--cam-tilt`.
Range from the ring size is used for the fix; bearing alone is not
enough today.

## Day 1: cage (manual piloting allowed)

1. All-up weight and hover throttle -> `mass_kg`, `hover_pwm`.
2. Short roll/pitch steps (<= 0.5 s) -> `a_lat_rate_max`. The 8" Archer
   is slew-limited before it is tilt-limited.
3. Throttle sweep -> `curve_pwm` / `curve_acc`.
4. First armed run: the 60 s rung in the cage or on the track.

```
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north <deg> --cam-tilt <deg> --fy <px> --pilot follower --traj ../out/plans/plan_LADDER_60s.json --arm
```

Fallback if the estimator cannot hold a fix: `--pilot seeker` (no plan,
gate to gate on the camera, 60-90 s laps in the sim).

## Race day: the ladder

Rungs are `config/ladder/vehicle_<T>s.toml` + `out/plans/plan_LADDER_<T>s.json`
(built by `python -m raceline.ladder --targets 60 50 40 35` from `src`;
a midpoint: `--k 0.4`, see `config/ladder/ladder.json` for k per target).

1. 60 s rung first. Clean twice, fly the fastest rung you brought.
2. Fails, fly the midpoint of the last clean and the last failed rung.
3. plan_RACE only after the 35 s rung has held twice.
4. On the scoring days fly the fastest rung that was clean twice. A
   complete slow run outranks every incomplete fast one.

| Min | Action |
|-----|--------|
| 0-3 | Power up, ssh, `bench info` (blockers, battery), camera alive. |
| 3-8 | Heat 1: the rung the search calls for. |
| 8-11 | Read the runtime log: gates counted, fix residuals, where it drifted. |
| 11-15 | Heat 2 only if heat 1 was clean; otherwise repeat or the midpoint. |

Nothing is hand-edited between heats. A rung that fails goes down the
ladder, not into the planner.

## Abort

- Fix residuals grow past an opening, or the count disagrees with the
  eye: land.
- Frame contact: the run is void; land, drop a rung.
- MSP stream stalls: the bridge disarms on a stale link; know the FC's
  failsafe from the `diff all` before the first heat.
- Detector dead or camera black: fix vision before anything faster than
  the 60 s rung.
