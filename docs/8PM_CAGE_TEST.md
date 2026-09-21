# 8pm cage test - Sally

**Two separate capabilities, and we want both.**

**A. Can she hold an assigned altitude?** A pure hover - you name a height,
she holds it on the barometer. This is what the course needs between gates,
and it is the thing that has never once worked on this team's aircraft.

**B. Can she hold a position relative to a gate, by eye?** A three-axis null -
elevation for height, azimuth for left-right, range for distance. Because the
gate's centre height is known, holding that null is an ABSOLUTE altitude that
never touches the barometer. This is the fallback if A keeps failing, and it
is what every gate crossing depends on regardless.

They are independent. A can fail and B can still work.

**ABORT, ALWAYS: MSP OVERRIDE off.** Throttle stick near hover position
throughout.

---

## The space

2x2 m cage, gate 2-3 m beyond the net, net invisible to the camera.

**Fly from the BACK wall.** That puts you 4-5 m from the gate, and a 2.7 m
gate does not fit in this camera's vertical view until 3.1 m. Too close and
the gate is clipped, its centre is a lie, and the bias moves as she climbs.

**You have about 1 m of room in any direction.** That is roughly 3 seconds of
drift. Every flight below is 10-15 seconds for that reason - the answers all
arrive in the first 3.

---

## Step 1 - sync (2 min, laptop)

```
cd ~/GitRepos/AI-GrandPrix
scripts/sync_drone.sh d44 cam20_75
```

Brings the corrected thrust curve, the attitude logging, `--gate-pitch`, and
the barometer re-zero.

**How you know it took:** when you arm, the line reads

```
armed, MSP OVERRIDE on, modes ...: altitude re-zeroed, lifting off
```

The words **`altitude re-zeroed`** are new. The zero used to be taken at
startup and then the program waited for you, which is open-ended - and the
barometer drifts 0.25 m/min at rest. Sally's bench run read -0.51 m one second
after zeroing. It now re-zeroes the instant you arm, while she is still on the
ground, which is the only moment it is true.

---

## Step 2 - the drift question (1 min, props off)

Sally on a surface checked with a level.

```
ssh d44
cd ~/AI-GrandPrix/src
python3 -m hardware.tiltcheck --port /dev/ttyTHS1
```

**Read the first line.**

| roll/pitch | means |
|---|---|
| **not 0** | the flight controller's idea of level is tilted. Betaflight **Setup -> Calibrate Accelerometer**, and you get most of your cage back |
| **about 0** | the FC is honest, the drift is the foam. Nothing to fix |

Worth the minute in a 2x2 cage: drift is what ends every flight tonight.

**Already have this?** The 30-inch table dry run logs `roll_deg`/`pitch_deg`
while she sits still. Same answer, no extra run:

```
tail -20 ../out/flightlogs/hover_*.csv
```

---

## Step 3 - vision check at the real geometry (2 min, PROPS OFF)

Hold her at the back wall, nose on the gate, where she will actually hover.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30 \
  --gate-z --gate-roll --gate-pitch --fy 824 --cam-tilt 20 --dry-run
```

**No `AIGP_CAM_CX/CY` on Sally.** Those are where the optical axis lands on
ONE camera's sensor - Randy's sits 12.8 px left and 8.8 px up of centre.
Sally's will be off by a different amount in a different direction, so
borrowing his can DOUBLE the error rather than remove it. Unset means the
image centre, which is the honest default and caps the bias near a degree -
about 0.07 m at 4 m.

`--cam-tilt 20` IS hers, measured tonight at 40 and 70 inches. `--fy 824` is
Randy's and barely matters here: an `fy` error SCALES the computed angle, and
scaling zero still gives zero, so it vanishes at the null the loop lives on.

Wait for `gate acquired`, then move her by hand:

| you do | expect |
|---|---|
| **lower** her | `el` goes **positive** |
| **raise** her | `el` goes **negative** |
| step **left** of the gate | `az` and `roll` **positive** |
| step **right** | both **negative** |
| move **toward** the gate | `pitch` goes **negative** - backing away |
| move **away** | `pitch` stays **0.0** - forward is capped off by design |

**If any of those are backwards, stop and tell me.** If the gate is never
found, check the yaw and the lighting before anything else.

Note the hit rate it prints at the end.

---

## Step 4 - FLIGHT 1: pure hover at 0.6 m, 10 seconds

Props on, fresh battery, aircraft level, foam on the FC.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.6 --seconds 10   --ceiling 1.6 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

1. Run it. Wait for `LIVE: waiting for the pilot`.
2. Throttle **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** She lifts off by herself.
5. Hands on the sticks. Correct drift if you need to, otherwise touch nothing.

Low and short, to see that the corrected curve behaves before asking for
anything. If this is clean, go straight to flight 2 - do not spend a battery
repeating it.

---

## Step 5 - FLIGHT 2: pure hover at gate height, 20 seconds

**This is capability A, and it is a result in its own right.**

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.35 --seconds 20   --ceiling 2.4 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

**1.35 m is the course's gate centre height.** If she holds that, she can hold
the altitude the race actually needs.

### Read the log

```
tail -25 ../out/flightlogs/hover_*.csv
```

| column | what good looks like |
|---|---|
| `z` | sits near **1.35**, no metre-sized jumps |
| `z` wander over the hold | **under 0.2 m** is good, under 0.5 m is usable |
| `vz` | small, and NOT oscillating - a sine wave means the gains are wrong |
| `throttle` | settles near **1227** |
| `amps` | around **17** |
| `roll_deg` / `pitch_deg` | is she really leaning, or level and translating anyway |

It also prints `HOVER THROTTLE = NNNN PWM` at the end. **Write it down** -
that is the third independent measurement of the corrected curve.

| outcome | means |
|---|---|
| holds 1.35 within ~0.2 m | **capability A works.** The barometer recovered with the curve, and the course is a real possibility |
| wanders 0.5 m or so | usable, and vision height would tighten it |
| metre-sized jumps | the barometer is genuinely broken at this airframe's vibration. Does NOT block flight 3 - that one does not use it |

---

## Step 6 - FLIGHT 3: the vision hold, 15 seconds

**Back wall. Nose on the gate. Arm.**

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 15 \
  --gate-z --gate-roll --gate-pitch --fy 824 --cam-tilt 20 \
  --gate-z-min 0.6 --gate-z-max 1.8 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Ceiling defaults to **2.8 m** here - it has to clear what vision is allowed to
ask for, not just `--alt`.

She takes off on the barometer to 1.2 m, then vision takes all three axes
during `hold`.

The live line reads:

```
t= 5.0 hold  z= 1.33 (target 1.34) vz=+0.03 thr=1229 | GATE el= +0.6 rng= 4.2 az= -1.1 roll=-0.3 hold= 4.4 pitch=-0.1
```

**Success is `el` and `az` trending to zero and staying there.** That is her
holding a known position relative to a gate, by eye.

| good | bad |
|---|---|
| `gate acquired` within a second or two | `gate: none` - yaw at the gate |
| `el` settles near 0 | `el` drifts away from 0 |
| `az` settles near 0 | `az` grows - she is sliding sideways |
| `pitch` mostly 0, occasionally negative | `pitch` pinned at -4 - she is being pushed back hard, check the range |
| target between 1.3 and 1.4 | pinned at 1.80 or 0.60 |

### All three axes, deliberately

`--gate-roll` and `--gate-pitch` have never flown. We are flying them anyway,
because cage time is scarcer than caution and a one-axis result does not
answer the question. Staging them would cost a flight each to learn less.

The bounds are what make that a reasonable trade rather than a gamble: roll is
capped at 4 degrees, pitch may only BACK AWAY, the height target is
slew-limited and clamped, and losing the gate releases all three within
half a second. The worst any of them can do is small and slow.

In a 2x2 cage the range hold is also the only thing opposing the drift that
ends every flight, so the full set may well buy you more seconds than it
costs.

---

## Step 7 - RANDY, if there is time and battery left

**Last, deliberately.** Sally's three flights are the session; Randy is upside.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.35 --seconds 20   --ceiling 2.4 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Three things at once, which is why it is worth a battery:

**It is the discriminating experiment.** Randy is the aircraft that produced
`-3.86 m` while sitting at 0.3. If his barometer is clean at 17 A now, the
theory is confirmed on the aircraft that showed the problem - much stronger
evidence than Sally, who was never obviously broken.

**It shakes down his repair.** The arm has not flown since it was fixed. A
cage is where you want to find a bad repair.

**His camera is the calibrated one.** `fy 824`, `cx 627.2`, `cy 368.8`, tilt
measured at 20 degrees on that airframe. Sally is flying vision on those
numbers as approximations. So if her vision hold in flight 3 sat at a
consistent offset, run the vision command on Randy and see whether the offset
goes away - that separates a calibration error from a real one.

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 15   --gate-z --gate-roll --gate-pitch --fy 824 --cam-tilt 20   --gate-z-min 0.6 --gate-z-max 1.8   --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

**Randy DOES get `AIGP_CAM_CX/CY`** - unlike Sally, they are his own measured
numbers. That is exactly what makes this comparison worth running.

| Randy's barometer | means |
|---|---|
| clean at 1.35 m | it was the over-thrust all along. Both aircraft are course-capable |
| still jumping | his sensor is genuinely worse - damaged, or mounted differently. Sally becomes the race aircraft and Randy needs the FC soft-mounted |

---

## What is protecting her

| | |
|---|---|
| thrust curve | **measured in flight**, two aircraft, 1 PWM apart |
| vision height target | slew-limited 0.30 m/s, clamped 0.6-1.8 m |
| roll | capped **4 deg** - the stick never leaves 1475..1525 |
| **pitch toward the gate** | capped at **ZERO deg**. It may only back away |
| range readings | median-filtered over 1 s, anything outside 1-12 m discarded |
| gate lost | all three axes release, back to the barometer target |
| barometer over 12 m/s | rejected, last good value held |
| climb demand | capped at 12 m/s^2 |
| ceiling | raw barometer, throttle to minimum, disarm |
| MSP OVERRIDE | the one that has never failed |

---

## Landing

| when | do |
|---|---|
| normal | nothing - she lands and disarms herself |
| early | **MSP OVERRIDE off**, fly her down |
| emergency | **Ctrl+C** - motors off at once |

## Before you leave

- [ ] every `.csv` from `~/AI-GrandPrix/out/flightlogs/`
- [ ] did she hold 1.35 m in flight 2, and to what
- [ ] `HOVER THROTTLE` from flight 2
- [ ] did `el` and `az` reach zero in flight 3
- [ ] `roll_deg`/`pitch_deg` - settles the drift question
- [ ] battery voltage
