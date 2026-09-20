# Cage flight 2 - d45 "Randy The Vanguard"

Three flights, one battery, in this order. Stop at the first one that does not
look right: each proves the thing the next one assumes.

**ABORT, ALWAYS: MSP OVERRIDE off.** The sticks come back instantly and the
program stops itself. Keep the throttle stick near hover position throughout
so taking over is a handoff and not a drop.

---

## What changed since flight 1

Flight 1 went to the ceiling. Four things were wrong and all four are fixed:

| what was wrong | now |
|---|---|
| the speedometer read zero for the first third of a second after takeoff, so nothing braked | it watches the accelerometer, which answers every tick instead of ten times a second |
| pitch was mirrored - Betaflight reads positive NOSE DOWN | fixed everywhere, and `tiltcheck` PASSed at 30 deg in all four directions |
| `hover.py` never applied the accelerometer scale it asked for | measured at rest at startup, and it refuses to start if the aircraft is not still |
| nothing stopped a runaway | a hard ceiling on the RAW barometer: above it, throttle to minimum and disarm |

**None of these have flown.** That is what flight 1 is for.

---

## 0. Before the slot

**On the laptop:**

```
cd ~/GitRepos/AI-GrandPrix
scripts/sync_drone.sh d45 cam20_75
```

**Fresh battery.** Stop below 22.0 V - the board reset three times at 21.87.

**Props on.** Look at all four hubs.

**Put the aircraft down LEVEL.** Shim it if you have to. Flight 1's log read
`accel at rest [-1.57 0.62 9.6]`, which is an aircraft sitting noticeably
nose-up on its legs, and that error feeds dead reckoning directly.

**Nose pointed at the gate** for flights 2 and 3. The camera is bolted down.

---

## 1. Plain hover - does it stop where it is told

```
cd ~/AI-GrandPrix/src
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.4 --seconds 20 \
  --ceiling 1.0 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

1. Run it. It prints the accelerometer figures, then `LIVE: waiting for the pilot`.
2. Throttle stick **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** It lifts off by itself.
5. Hands on the sticks, throttle near hover, touch nothing.
6. It climbs to 0.4 m, holds 20 s, descends, disarms itself.

**The one number that matters is `vz` on the climb lines.** In flight 1 it
read `+0.60` during a 3.5 m/s climb, which is why nothing braked. It must now
read something real.

| good | bad |
|---|---|
| a gentle lift | it leaps |
| `at altitude (0.4x m) ... holding` | it sails past 0.4 |
| `vz` reads real numbers while climbing | `vz` near zero while climbing |
| lands and disarms itself | bounces at knee height |
| | `CEILING HIT` - the backstop fired, something is still wrong |

Write down the `HOVER THROTTLE` figure, but do not read much into it: at 0.4 m
the aircraft is in ground effect and it reads low. The table tests - 1294 and
1285 against a fitted 1291 - remain the better numbers.

**If this is not clean, stop. Do not fly 2 or 3.**

---

## 2. Autonomous takeoff, then vision holds the gate's height

The first time vision has ever flown this aircraft.

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 25 \
  --gate-z --fy 824 --cam-tilt 20 \
  --gate-z-min 0.6 --gate-z-max 1.8 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

The ceiling defaults to **2.8 m** here, because it has to clear what vision is
allowed to ask for and not just `--alt`.

Same six human steps. **Nose on the gate before you arm.**

**What the aircraft may do: change its throttle.** That is all. Roll, pitch and
yaw stay centred exactly as in flight 1, so **no axis exists that can carry it
toward the gate**. Takeoff and landing are flown on the barometer; vision only
has the aircraft during `hold`.

It climbs to 1.2 m on the barometer, prints `gate acquired`, and moves the
target toward the gate's centre height (1.35 m) at no more than 0.30 m/s,
clamped to 0.6-1.8 m. Lose the gate and it prints `gate lost` and returns to
1.2 m.

The live line gains a section:

```
t= 12.0 hold  z= 1.31 (target 1.34) vz=+0.04 thr=1297 a_cmd=+0.18 | GATE el= +0.8 rng= 3.4
```

**`el` is the test. It should trend toward zero and stay there** - that means
the aircraft found the gate's height by eye. `rng` prints for information
only: the controller never uses it, deliberately, because on the bench it read
2.4, then 9.8, then 140 m inside ten seconds.

| good | bad |
|---|---|
| `gate acquired` within a second or two | `gate: none` throughout - yaw at the gate |
| `el` settles near 0 | `el` drifts away from 0 |
| target sits around 1.3-1.4 | target pinned at 1.80 or 0.60 |

Repeated `gate lost` / `gate acquired` is survivable, but note how often.

---

## 3. Vision also centres it left and right

Only if 2 was clean.

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 25 \
  --gate-z --gate-roll --fy 824 --cam-tilt 20 \
  --gate-z-min 0.6 --gate-z-max 1.8 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

It now also slides sideways until the gate is dead ahead. **Aim the nose at
the gate before arming**: it travels about `range x tan(how far off your aim
is)`, so 4 m and 15 degrees out is roughly 1 m of drift. Aim well and it
barely moves.

Roll is capped at 4 degrees - the stick never leaves 1475..1525.

**`az` should trend to zero and `roll` settle near zero.**

Still no pitch, so distance to the gate is uncontrolled and it can drift
slowly toward or away. That is the one axis left, and it needs live PnP to
separate lateral offset from heading error.

---

## Landing

| when | do |
|---|---|
| normal | nothing - it lands and disarms itself after `--seconds` |
| early, gently | **MSP OVERRIDE off**, fly it down on the sticks |
| emergency | **Ctrl+C** - motors off at once. From 1.2 m that is a drop |

---

## Before you leave the cage

- [ ] `HOVER THROTTLE` from flight 1
- [ ] did `el` reach zero in flight 2
- [ ] every `.csv` from `~/AI-GrandPrix/out/flightlogs/`
- [ ] battery voltage at the end
- [ ] anything that looked wrong, written down while you still remember it

---

## If something fails

| symptom | means |
|---|---|
| `CEILING HIT` | it ran away again. Send me the CSV before flying anything else |
| `ERROR accelerometer not at rest` | it was moving or being held. Put it down and restart |
| `accelerometer at rest` over 1 m/s^2 | not level, or the attitude is wrong. Re-run `tiltcheck` |
| it leaps off the ground | add `--takeoff-pwm 1320` |
| bounces at knee height when landing | stale code - re-sync |
| `waiting for the pilot` forever | MSP OVERRIDE is not reaching the FC. Check the switch, and check `--help` lists the gate options so you know the sync took |

---

## Next, if all three are clean

The plan, at **k 0.10** - the 67-second model, not k 0.25.

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 20 --fy 824 --cam-hfov 76 --heading-drift-dpm 3.0 --pilot follower \
  --config ../config/ladder/vehicle_k010_cam20_75.toml \
  --traj ../out/plans/plan_LADDER_k010_cam20_75.json --arm
```

**`--map-north here` reads the heading at startup and builds the entire map
from it.** Point the nose down gate 1's line before you start, and put the
aircraft on the real start line: the bench check showed detections being
correctly rejected when the geometry did not match the map.

Camera fixes are worth trusting. The 2026-09-20 bench residual check tracked
range smoothly from 4.4 m in to 1.8 m and back out, and fixed with 1 to 6 cm
of residual.
