# Cage flight 3 - Sally, 8pm

**The question tonight: does the barometer work now?**

Everything else follows from that. With the corrected thrust curve Sally
hovers at ~1227 PWM and draws ~17 A, and at 17 A her barometer was clean.
Randy's garbage readings came at 35-55 A, while the wrong curve had him
climbing. If that theory is right, she can hold a real altitude and the course
is back on.

**ABORT, ALWAYS: MSP OVERRIDE off.** Throttle stick near hover position
throughout.

---

## 7:15 - bench, props off, ~12 minutes

### 1. Sync (2 min, laptop)

```
cd ~/GitRepos/AI-GrandPrix
scripts/sync_drone.sh d44 cam20_75
```

Brings the corrected curve and the new attitude logging.

### 2. Settle the drift (1 min)

Sally on a surface checked with a level.

```
ssh d44
cd ~/AI-GrandPrix/src
python3 -m hardware.tiltcheck --port /dev/ttyTHS1
```

**Read the first line only.**

| roll/pitch | means | do |
|---|---|---|
| **not 0** | the FC's idea of level is tilted | Betaflight **Setup -> Calibrate Accelerometer** on a level surface, 3 min |
| **about 0** | the FC is honest; the drift is the foam | nothing to fix - and the ESTIMATE is honest, which matters more |

Tilt her 30 degrees each way while you are there and check it says PASS. That
confirms her pitch sign, which has been assumed from Randy and never measured.

### 3. Override test (3 min, transmitter on, PROPS OFF)

```
python3 -m hardware.override_test --port /dev/ttyTHS1
```

**This has never been run on d44.** It is the only abort there is. Randy is in
one piece because his works. Do not fly her without it.

### 4. Her camera mount tilt (5 min, optional but cheap)

Only needed for flight 2. Level aircraft, checkerboard or any target whose
height you can measure:

```
python3 -m hardware.camtilt --fy 824 --cy 360 \
  --dist <m> --lens-h <m> --target-h <m>
```

`fy 824` is Randy's and close enough for this - the tilt is set mostly by
where the target lands relative to the principal point. **Five degrees of tilt
error is 0.26 m of height offset at 3 m**, which is fine for a cage test and
not fine for a course.

If there is no time, use `--cam-tilt 20` and accept the bias.

---

## 8:00 - FLIGHT 1: does she hold an altitude

Props on, fresh battery, aircraft level, foam on the FC.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.6 --seconds 20 \
  --ceiling 1.6 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

**No `--no-baro` this time.** That is the whole point.

1. Run it. Wait for `LIVE: waiting for the pilot`.
2. Throttle **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** She lifts off by herself.
5. Hands on the sticks. Touch nothing.
6. Climbs to 0.6 m, holds 20 s, lands, disarms.

### Then read the log before anything else

```
tail -20 ../out/flightlogs/hover_*.csv
```

| column | what good looks like |
|---|---|
| `z` | smooth, sits near 0.60, no metre-sized jumps |
| `vz` | small, no oscillation |
| `throttle` | settles near **1227** |
| `amps` | around **17** |
| `roll_deg` / `pitch_deg` | NEW. Tells you whether she is really leaning or holding level and translating anyway |

### The decision

| | |
|---|---|
| holds 0.6 m | **the barometer theory was right.** Go to flight 2, and the course is back on |
| ragged but under ~0.3 m of wander | flyable. Go to flight 2, watch it |
| metre-sized jumps again | the barometer is genuinely broken on both aircraft. Fall back to `--no-baro` and vision for height |

---

## FLIGHT 2: gate centring - only if flight 1 held

### Put the gate at 4 m, not 2-3

`--gate-z` nulls on the gate's CENTRE, and a 2.7 m gate does not fit in this
camera's vertical view until about 3.1 m away:

| distance | camera sees, height | 2.7 m gate |
|---|---|---|
| 2.0 m | 1.75 m | **clipped** |
| 2.5 m | 2.18 m | **clipped** |
| 3.0 m | 2.62 m | **clipped** |
| **3.5 m** | 3.06 m | fits |
| **4.0 m** | 3.50 m | fits, with margin |

A clipped gate has a FALSE centre - the blob's middle sits wherever the
visible part is - and the bias MOVES as the aircraft climbs, because the
clipping shifts. That is feedback with an unpredictable sign, in a loop that
has never flown, with a net in the way. At 4 m the whole gate is in frame and
the centre is honest.

### The net between is untested

Nobody has run the detector through netting. Check it in the props-off dry run
below before you fly: if `gate: none` or the numbers jump around, the mesh is
confusing the HSV thresholds and flight 2 is off for tonight.

### Do the props-off check first, at the real geometry

Hold her where she will hover, pointed at the gate:

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30   --gate-z --fy 824 --cam-tilt 20 --dry-run
```

Lower her by hand -> `el` goes **positive**. Raise her -> **negative**. If
those are backwards or jumpy, stop.

### The flight

**Point the nose at the gate before arming.** The camera is bolted down.

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 25 \
  --gate-z --fy 824 --cam-tilt 20 \
  --gate-z-min 0.6 --gate-z-max 1.8 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Ceiling defaults to **2.8 m** - it has to clear what vision may ask for.

**What she may do: change her throttle.** That is all. Roll, pitch and yaw stay
centred, so **no axis exists that can carry her toward the gate.** Takeoff and
landing are barometric; vision only has her during `hold`.

**`el` is the test. It should trend toward zero and stay there** - that means
she found the gate's height by eye. `rng` prints for information only; the
controller never uses it, deliberately, because on the bench it read 2.4, then
9.8, then 140 m inside ten seconds.

| good | bad |
|---|---|
| `gate acquired` within a second or two | `gate: none` throughout - yaw at the gate |
| `el` settles near 0 | `el` drifts away from 0 |
| target sits around 1.3-1.4 | pinned at 1.80 or 0.60 |

Worst case is bounded: a detector lying at a false +40 degrees for 25 seconds
moves the target to the 1.8 m clamp and stops.

---

## NOT tonight: `--gate-roll`

The gate sits directly in front with a net between. `--gate-roll` slides the
aircraft sideways until the gate is dead ahead - and with the gate already
centred there is nothing for it to correct, so it would do nothing useful and
could only find its own edge cases. Any motion it did command would be toward
a net 2-3 m away.

Save it for a space where being wrong is cheap.

---

## Landing

| when | do |
|---|---|
| normal | nothing - she lands and disarms herself |
| early, gently | **MSP OVERRIDE off**, fly her down |
| emergency | **Ctrl+C** - motors off at once. From 1.2 m that is a drop |

---

## Before you leave

- [ ] every `.csv` from `~/AI-GrandPrix/out/flightlogs/`
- [ ] did `z` behave in flight 1 - the answer to tonight's question
- [ ] did `el` reach zero in flight 2
- [ ] `roll_deg`/`pitch_deg` during the hover - settles the drift question
- [ ] battery voltage at the end

---

## What is protecting her

| | |
|---|---|
| thrust curve | **measured in flight**, confirmed on two aircraft to 1 PWM |
| barometer readings implying over 12 m/s | rejected, last good value held |
| the altitude loop | closes on accel+baro fused, not the raw sensor |
| climb demand | capped at 12 m/s^2 regardless of the error |
| ceiling | raw barometer, throttle to minimum, disarm |
| vision target | slew-limited 0.30 m/s, clamped 0.6-1.8 m, released on gate loss |
| MSP OVERRIDE | the one that has never failed |

The first line is the one that changed today, and it is the one that would
have prevented every crash.
