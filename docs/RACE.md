# RACE.md — the exact commands, in order

Julian (d43) first. If he makes gates, Sally (d44). **Sally has no padding:**
on her, abort at the first doubt, before g1 and after it.

Everything is already on both aircraft (branch `sim-angle-loop`, plan
`plan_STACK_s15_cam20_75.json`). Step 0 is only a safety net.

---

## 0 — Laptop, once (repo root). Only if anything changed or you are unsure.

```
scripts/sync_drone.sh d43
scripts/sync_drone.sh d44
```

Expect `== done` for each.

---

## 1 — Julian (d43): dry run in the pits, no gate, NOT armed

```
ssh d43
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=611.9 AIGP_CAM_CY=394.5 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 835.5 --cam-tilt 10 --map-north here \
    --vert vision --dry-run
```

Must show, within 20 s:

| line | means |
|---|---|
| `CROSSINGS: counted 0.75 m past the gate plane` | the race build is the one running |
| `PAD: no gate in view; bias n=...` every 2 s, n climbing | bias learner alive |
| `[RACELINE] released at t=2.xs` | the release rule works |
| `stk=(..., >1500 pitch, ...)` after the release | the tracker asks to go forward |
| no `Traceback` | build is sound |

Ctrl-C. A Traceback = stop, send it.

---

## 2 — Julian (d43): the flight

On the start line, pointed at g0. Throttle stick FULLY DOWN. Props on.

```
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=611.9 AIGP_CAM_CY=394.5 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 835.5 --cam-tilt 10 --map-north here \
    --vert vision --arm
```

**Leave him STILL for at least 10 s after `LIVE: waiting for the pilot`.**
Every 2 s he prints:

```
PAD: gate at 7.5 m, +12.3 deg ABOVE, offset x -0.03; bias=(+0.23,+0.04) n=210
```

Need: a range near 7.5, **ABOVE**, offset x near 0, n climbing past ~200.
`PAD: no gate in view` on the line = do NOT arm; check what the camera faces.

Then, on the transmitter: **ARM -> MSP OVERRIDE on -> ANGLE row 5.**
The clock starts when the FC reports armed + override. You get:

```
accel bias learned on the pad: [0.23 0.04 0.01] m/s^2 body (330 samples)
```

### What the screen shows, in order

| phase | line |
|---|---|
| takeoff | `AIRBORNE`, `gate 0 ... ABOVE me -> CLIMBING` |
| release, by 4 s | `[RACELINE] released at t=2.3s (over g0: ...)` |
| approach | `gate 0 at 6.0 m: LEVEL, holding 1.35 m`, `fixes=` climbing |
| commit, ~3.5 m | `gate 0 COMMIT at 3.5 m ... flying through`, then `[RACELINE] COMMIT: holding throttle 12xx` |
| through | `gate 0 CROSSED` about half a second after he is |
| next | `gate 1 IN SIGHT ...` and the same again |

### ABORT = MSP OVERRIDE OFF, throttle stick near hover

Before g1 counts, abort on ANY of:
- no `released at` line by 5 s
- visible left-right rocking
- off the gate's centre by more than half a metre at 3 m out
- `holding` rising after COMMIT
- the elevation degrees GROWING as the range shrinks

After g1 counts: let him fly. Take over only to land him when the run is
plainly over. Never let the plan run to its end: the pilot lands, always.

---

## 3 — Between attempts (laptop, repo root)

```
scripts/pull_flight.sh d43
```

Read the narration for two minutes before deciding anything. Do not fly a
second attempt to find out what happened in the first.

---

## 4 — Sally (d44): only if Julian made gates

Same two steps with her numbers. **No padding: abort at the first doubt.**

Dry run in the pits:

```
ssh d44
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=616.9 AIGP_CAM_CY=330.0 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 859 --cam-tilt 10 --map-north here --heading-drift-dpm 5.0 \
    --vert vision --dry-run
```

The flight, on the line, still 10 s, PAD lines good, then
**ARM -> MSP OVERRIDE on -> ANGLE row 2** (row 2 on Sally, not 5):

```
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=616.9 AIGP_CAM_CY=330.0 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 859 --cam-tilt 10 --map-north here --heading-drift-dpm 5.0 \
    --vert vision --arm
```

Pull her log with `scripts/pull_flight.sh d44`.

---

## Switches (only if a dry run or a flight says so; put in front of the command)

| switch | effect |
|---|---|
| `AIGP_ACCEL_BIAS=0` | no bias learning on the pad |
| `AIGP_COMMIT_STRAIGHT=0` | full dead-reckoned lateral loop inside the commit range |

Do not use `AIGP_COMMIT_HOLD=0` (that is the barometer hold).
