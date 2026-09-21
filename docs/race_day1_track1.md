# Race day 1, track 1 - Sally, 20 minutes

**The number to beat: 5 gates in 1:48.** That is 21.6 s per gate. Our plan does
8.3. We are not short of speed - we are short of a finished lap.

**The goal: fly as slow as we can and MAKE THE GATES.**

**ABORT, ALWAYS: MSP OVERRIDE off.** Throttle stick near hover position
throughout, so taking over is a handoff and not a drop.

**Two people.** One on the transmitter, finger on MSP OVERRIDE, eyes on the
aircraft. One on the laptop reading numbers. The person on the sticks never
looks at the screen.

---

## Read this first: we are NOT commanding an altitude

Every flight that failed on this team asked for a height against the
barometer. **This one never does.** Verified in the code today, not assumed:

| where | what it does |
|---|---|
| `follower.py:380` | tracker returns an ABSOLUTE `z_target` = 1.35 from the plan |
| `follower.py:668` | landing sets an ABSOLUTE `z_target` from the park point |
| **`follower.py:676-678`** | **`if VERT_VISION:` OVERWRITES both** with `z_target = est.p[2] + dz_vis`. This is the LAST write before the altitude loop on line 679 |
| `rc_backend.py:127` | `err = z_target - est.p[2]` -> **`est.p[2]` cancels** -> `err = dz_vis` |

Every absolute height is overwritten before it reaches the loop, **in every
phase, including landing.** The error term is purely "how far up or down to be
level with the gate I can see". With no gate in view `dz` fades to zero and it
becomes a VERTICAL SPEED HOLD - the only mode that has ever flown clean here.

### Two places the barometer still touches it - know them

- `rc_backend.py:131`: `if not airborne and vz < 0.7: return takeoff_pwm`, and
  `airborne = est.p[2] >= 0.0`. A SUSTAINED negative height would command an
  open-loop climb. That `p[2]` is the FUSED height (accel + baro, outliers
  rejected), not raw - a single spike like Randy's -3.86 cannot do it.
- The landing disarm test `est.p[2] < 0.10`. The barometer decides WHEN TO
  STOP, not how much throttle.

---

## And the throttle is clamped

Randy's crash commanded `a_cmd +32.9` and got **1837 PWM**. That is now
unreachable.

| condition | `a_cmd` | PWM |
|---|---|---|
| hover | 0.0 | **1228** |
| vision error saturated (`dz` +-0.35, `kp_z` 9.0) | +3.15 | 1290 |
| tilt compensation at the plan's 10 deg cap | - | +3 |
| `vz` reads -1.0 m/s wrongly | +4.0 | 1305 |
| `vz` reads -2.0 m/s wrongly (its clamp) | +8.0 | 1377 |
| **EVERYTHING wrong at once** | **+12.0 (capped)** | **1410** |

`CLIMB_ACC_MAX = 12.0` caps the acceleration and `MAX_THRUST_G = 2.0` is a
**hard ceiling on the actuator, not on any estimate**. 1410 PWM is the most
this aircraft can be told to do today, whatever any sensor claims.

---

## What Sally has actually done

Be honest about this, because it decides what we fly first.

| | |
|---|---|
| **Has flown** | two clean hovers, `--no-baro`: climb 0.6 s, then HOLD VERTICAL SPEED. Inside 0.11 m/s for five seconds, no oscillation, 1227 PWM, 17 A |
| **Has never flown** | a commanded altitude. Not once, on any aircraft here |
| **Has never flown** | vision height, any gate crossing, any course |
| **Last autonomous flight** | broke a leg |
| **Has never run** | today's code, or today's calibration |

---

## The aircraft - d44 "Sally the Brave"

Calibrated today. These are HERS. Do not paste Randy's in.

| | |
|---|---|
| `--fy` | **859** |
| `--cam-hfov` | **73** |
| `--cam-tilt` | **20** (agreed at 40 and 70 in) |
| `AIGP_CAM_CX/CY` | **616.9 / 330.0** |
| `--heading-drift-dpm` | **5.0** |
| mask 15 | verified |
| ANGLE | aux row 2 |

---

## The plan - FLAT, s15

`plan_FLAT_s15_cam20_75.json`. The stacked gate's top opening is skipped, so
every crossing is at 1.35 m and the aircraft holds ONE height start to finish.

| | |
|---|---|
| Crossings | 10 per lap, 2 laps, plus the return through the start gate = **21** |
| Path | 243.3 m |
| v_max | **1.5 m/s** |
| Tilt cap | **10 deg** (down from 20) |
| Predicted | **173.6 s** full, **80.2 s** for lap 1 |
| Frame violations | **0** |
| Every z in the plan | **1.35** - verified on the aircraft |

### Why s15

**Nothing scores for being fast. Only gates completed score.** s15 is still
2.6x the competitor's gate rate - margin we do not need to spend.

Heading drift is measured per MINUTE, so a slower plan accumulates MORE of it.
Sally reads 5.0 deg/min, the worst in the fleet.

| rung | v_max | s/gate | full | lap 1 | drift residual by end of lap 1 | cross-track at 25 m |
|---|---|---|---|---|---|---|
| **s15** | **1.5** | **8.3** | **173.6 s** | **80.2 s** | **3.3 deg** | **1.46 m** |
| s20 | 2.0 | 6.1 | 127.8 s | 59.5 s | 2.5 deg | 1.08 m |
| s25 | 2.5 | 4.8 | 101.1 s | 46.9 s | 2.0 deg | 0.85 m |
| k000 | 5.0 | 3.0 | 63.9 s | 33.2 s | 1.4 deg | 0.60 m |

Gate half-width is 0.75 m. **The entire cost of s15 over s20 is 0.8 degrees of
heading drift - about 0.38 m at 25 m.** Against that it buys:

- **more camera fixes** - the gate stays in frame longer, and fixes are the
  only thing correcting position drift
- **10 deg of tilt instead of 12** - attitude error feeds dead reckoning
  directly, so less tilt is less of it
- lower current (the barometer behaves near 17 A), more pilot reaction time,
  less crash energy

Battery is not the constraint - another team flew 23 gates over a 5 minute run.

**Do not go slower than this.** Heading drift is per MINUTE, so every extra
second buys more of the one error nothing on the aircraft corrects.

---

## The clock

| | | |
|---|---|---|
| 0:00 | set up, Sally on the start line, LEVEL, nose down gate 1's line | 2 min |
| 0:02 | **FLIGHT 0 - RANDY hovers** | 3 min |
| 0:05 | **GATE A** - read the log, decide | 1 min |
| 0:06 | **FLIGHT 1 - SALLY, the course** | 6 min |
| 0:12 | battery, debrief | 3 min |
| 0:15 | **FLIGHT 2 - SALLY, the course again** | 5 min |

If anything runs long, **drop flight 2, not flight 0.**

---

## FLIGHT 0 - RANDY hovers, 15 s

**Randy, not Sally.** His camera is broken and `--no-baro` never opens the
camera, so he can prove today's code without risking the only calibrated
aircraft we have.

```
ssh d45
cd ~/AI-GrandPrix/src
python3 -m hardware.hover --port /dev/ttyTHS1 --no-baro \
  --alt 1.2 --takeoff-pwm 1250 --climb-s 0.6 --seconds 15 \
  --ceiling 2.0 --config ../config/ladder/vehicle_s15_cam20_75.toml --arm
```

1. Run it. Wait for `LIVE: waiting for the pilot`.
2. Throttle **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** He lifts off by himself.
5. Hands on the sticks. He WILL drift and he does NOT hold a height - correct
   his position if you need to, but leave the throttle alone.

`--no-baro` holds vertical SPEED. He will sink slowly. **That is the mode
working, not failing.**

### What this proves, and what it does not

| proves | does NOT prove |
|---|---|
| today's synced code runs | Sally's forward drift |
| the corrected thrust curve | Sally's barometer under load |
| takeoff, the vertical-speed hold, the abort path | anything about her camera |

Randy's ANGLE is **aux row 5**, Sally's is row 2. Do not cross them.

---

## GATE A - the decision, 60 seconds

```
tail -20 ../out/flightlogs/hover_*.csv
```

| column | good |
|---|---|
| `vz` | small and steady, **inside ~0.15 m/s**, not oscillating |
| `throttle` | near **1227** |
| `amps` | near **17** |
| `z` | drifts slowly - **expected**, do not read it as a fault |

### If Randy fails, does Sally still fly?

**It depends entirely on WHAT failed.** Randy shares the CODE and the THRUST
CURVE with Sally. He does not share his sensors, his airframe or his aux rows.
Ask one question: does the symptom implicate something SHARED, or something
that is only his?

| what Randy did | shared or his | Sally |
|---|---|---|
| `vz` steady, ~1227 PWM, ~17 A | - | **GO** |
| `CEILING HIT` but `vz` trace otherwise clean | **HIS** - he is the aircraft that read -3.86 m. The ceiling reads the RAW barometer | **GO** |
| drifts off, but holds vertical speed | **HIS** - airframe trim, repaired arm | **GO** |
| never armed / `waiting for the pilot` | **HIS** - his ANGLE is aux row 5, not row 2 | **GO** |
| mechanical: vibration, a leg, a motor | **HIS** | **GO** |
| **`vz` oscillates** | **SHARED** - gains or the filter | **STOP** |
| **throttle settles far from 1227** | **SHARED** - the thrust curve | **STOP** |
| **climbs away with vz reading near zero** | **SHARED** - the launch detector | **STOP** |
| **crashes or python throws** | **SHARED** until proven otherwise | **STOP** |
| anything ambiguous | unknown | **hover Sally first**, below |

### The tie-breaker, 2 minutes

If you cannot tell whose fault it was, do not guess and do not skip to the
course. Fly the same hover on SALLY and read the same columns:

```
ssh d44
cd ~/AI-GrandPrix/src
python3 -m hardware.hover --port /dev/ttyTHS1 --no-baro   --alt 1.2 --takeoff-pwm 1250 --climb-s 0.6 --seconds 15   --ceiling 2.0 --config ../config/ladder/vehicle_s15_cam20_75.toml --arm
```

This is a mode she HAS flown clean twice. If she flies it and Randy did not,
it was his. Costs one battery and buys certainty.

**Never skip straight to the course on an unexplained failure.** The course is
243 m at head height; the hover is 15 s over the start line.

**Do not debug on the clock.** Read it, decide, move.

---

## FLIGHT 1 - SALLY, the course

**Setup matters more than the command:**

- Aircraft on the **REAL start line**, not near it
- **Nose pointed straight down gate 1's line.** `--map-north here` reads the
  heading at startup and builds ALL 21 GATES from it. Get this wrong and the
  whole map is rotated
- Aircraft **LEVEL**. The startup check refuses if she is not still

```
ssh d44
cd ~/AI-GrandPrix/src
AIGP_VERT=vision AIGP_CAM_CX=616.9 AIGP_CAM_CY=330.0 \
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 20 --fy 859 --cam-hfov 73 --heading-drift-dpm 5.0 \
  --pilot follower --vert vision \
  --config ../config/ladder/vehicle_s15_cam20_75.toml \
  --traj ../out/plans/plan_FLAT_s15_cam20_75.json --arm
```

Same six human steps as flight 0.

**`AIGP_VERT=vision` is not optional.** Without it she commands 1.35 m on the
barometer, which is the thing that has never worked. If the banner does not
show the vision vertical, stop and fix it before arming.

### Call outs from the laptop, loudly

- `ev0`, `ev1`... - which gate she is heading for
- `res=` - fix residual. Under ~1 m is healthy
- `fixes=` - climbing means the camera is working

### What counts as a win

| got to | verdict |
|---|---|
| **gate 1** | first autonomous gate this team has ever flown |
| **6 gates** | **beats them** |
| **10 (lap 1)** | doubles their gates in 74 percent of their time |
| **21** | the whole thing |

Gate 1 is 7.3 m out - about **5 s of flying** after takeoff at this speed.

---

## ABORT IMMEDIATELY IF

- she heads for a wall or a net
- she climbs past head height
- she oscillates rather than tracks
- anything at all feels wrong

You get a free retry. You do not get a free airframe.

---

## Landing

| when | do |
|---|---|
| normal | nothing - she lands and disarms herself |
| early, gently | **MSP OVERRIDE off**, fly her down |
| emergency | **Ctrl+C** - motors off at once. From 1.35 m that is a drop |

**Stop below 22.0 V.** The board reset three times at 21.87.

---

## FLIGHT 2 - repeat, or step up

If flight 1 was clean, fly it again and bank a second result. Three flights is
the floor for reading anything across runs.

If she completed a lap comfortably, `s20`, `s25` and `k000` are all on her -
same command, swap **BOTH** the `--config` and the `--traj`.

**Bank a completed lap before chasing a faster one.**

---

## Known risks - fly knowing these

None block the attempt. All argue for a quick finger on the switch.

| | |
|---|---|
| **Everything is a first** | vision height, the course, and every gate crossing are all firsts, at the same time. If it goes wrong it will probably go wrong in the first 10 seconds |
| **Barometer still reaches `vz`** | the position error is clean, but the damping term's `vz` is baro-fused and clamped to the barometer's own slope. `note_throttle` / `baro_trusted` are NOT wired into `runtime.py`, so the automatic fallback cannot fire on the course. Worst case it costs 1377 PWM, not 1837 |
| **Sally drifts forward on centred sticks** | still unexplained - FC trim or the foam prop guards |
| **Detector hallucinates** | claims a gate in 99.6 percent of frames including with the lens covered. Bounded in the vertical channel to +-0.35 m at 0.35 m/s, so it cannot run away, but a persistent false lock parks her 0.35 m off centre |
| **Principal point is thin** | only 7 of 20 calibration views survived. `cy` error cancels in the vision-height path; `cx`'s -1.54 deg bearing bias does not |

---

## Before you leave the track

- [ ] every `.csv` from `~/AI-GrandPrix/out/flightlogs/` on **both** aircraft
- [ ] how far she got, **in gates**
- [ ] `vz` and `HOVER THROTTLE` from flight 0
- [ ] `res=` and `fixes=` from the course run
- [ ] `roll_deg` / `pitch_deg` - settles the drift question
- [ ] battery voltages
- [ ] what looked wrong, written down NOW rather than remembered later

---

## What is protecting her

| | |
|---|---|
| commanded altitude | **there isn't one** - `AIGP_VERT=vision`, verified in code |
| throttle | **hard ceiling 1410 PWM** (2.0 g actuator cap). Randy's crash was 1837 |
| climb demand | capped at 12 m/s^2 whatever the error says |
| vision height target | slew-limited 0.30 m/s, clamped 0.6-1.8 m, released on gate loss |
| tilt | capped **10 deg** by the plan |
| speed | capped **1.5 m/s** by the plan |
| thrust curve | **measured in flight**, two aircraft, 1 PWM apart |
| barometer over 12 m/s | rejected, last good value held |
| implausible camera fixes | rejected beyond 4 m; gates past 15 m not fixed on |
| ceiling | raw barometer, throttle to minimum, disarm |
| MSP OVERRIDE | **the one that has never failed** |
