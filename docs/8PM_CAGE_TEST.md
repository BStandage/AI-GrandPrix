# 8pm cage test - Sally

**What we are actually testing: can she hold a position relative to a gate,
by eye?**

Not a hover. A three-axis null against the gate - elevation for height,
azimuth for left-right, range for distance. And because the gate's centre
height is known, holding that null IS an absolute altitude. **The barometer
stops mattering.** That is the point.

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

Brings the corrected thrust curve, the attitude logging, and `--gate-pitch`,
which was written this evening and has never flown.

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

---

## Step 3 - vision check at the real geometry (2 min, PROPS OFF)

Hold her at the back wall, nose on the gate, where she will actually hover.

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30 \
  --gate-z --gate-roll --gate-pitch --fy 824 --cam-tilt 20 --dry-run
```

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

## Step 4 - FLIGHT 1: plain hover, 10 seconds

Props on, fresh battery, aircraft level, foam on the FC.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.6 --seconds 10 \
  --ceiling 1.6 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

1. Run it. Wait for `LIVE: waiting for the pilot`.
2. Throttle **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** She lifts off by herself.
5. Hands on the sticks. Correct drift if you need to, otherwise touch nothing.

**This exists to answer one question: does the barometer work now?** With the
corrected curve she sits at ~1227 PWM and ~17 A, which is where her barometer
was clean. Randy's garbage came at 35-55 A while the wrong curve had him
climbing.

### Read the log before flying again

```
tail -20 ../out/flightlogs/hover_*.csv
```

| column | good |
|---|---|
| `z` | smooth, near 0.60, no metre-sized jumps |
| `throttle` | settles near **1227** |
| `amps` | around **17** |
| `roll_deg` / `pitch_deg` | NEW - is she really leaning, or level and translating anyway |

| outcome | then |
|---|---|
| holds 0.6 m | the barometer theory was right. Good news for the course |
| jumps by metres | it is genuinely broken. Does NOT block tonight - step 5 does not use it |

---

## Step 5 - FLIGHT 2: the actual test, 15 seconds

**Back wall. Nose on the gate. Arm.**

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 15 \
  --gate-z --gate-roll --gate-pitch --fy 824 --cam-tilt 20 \
  --gate-z-min 0.6 --gate-z-max 1.8 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

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
- [ ] did `z` behave in flight 1
- [ ] did `el` and `az` reach zero in flight 2
- [ ] `roll_deg`/`pitch_deg` - settles the drift question
- [ ] battery voltage
