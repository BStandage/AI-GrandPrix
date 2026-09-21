# Track plan - 23 minutes, 2026-09-20

**Goal: get Randy through gate 1.** Everything else is a bonus. Sally hovers
only if there is time left over - do not let her eat the session.

**Two people.** One on the transmitter with a finger on MSP OVERRIDE and eyes
on the aircraft, one on the laptop reading numbers. The person on the sticks
never looks at the screen.

**ABORT, ALWAYS: MSP OVERRIDE off.** Throttle stick near hover position the
whole time so taking over is a handoff, not a drop.

---

## The clock

| | | |
|---|---|---|
| 0:00 | set up, Randy on the start line, LEVEL | 3 min |
| 0:03 | **FLIGHT 1 - Randy hover** | 2 min |
| 0:05 | **GATE A** - read the `z` column | 1 min |
| 0:06 | **FLIGHT 2 - Randy, course at k 0.10** | 5 min |
| 0:11 | debrief, battery | 4 min |
| 0:15 | **FLIGHT 3 - Sally hover**, if she is ready | 5 min |
| 0:20 | pack up, get the logs off | 3 min |

If anything runs long, **drop Sally**. Randy on the course is the session.

---

## FLIGHT 1 - Randy hover

Props on, fresh battery, aircraft **level** on the ground, foam on the FC.

```
cd ~/AI-GrandPrix/src
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.4 --seconds 20 \
  --ceiling 1.0 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

1. Run it. Wait for `LIVE: waiting for the pilot`.
2. Throttle **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** It lifts off by itself.
5. Hands on the sticks. Touch nothing.
6. It climbs to 0.4 m, holds 20 s, lands, disarms.

**Purpose: prove the barometer behaves with props running.** Everything else
today already works on a bench. This is the one thing that does not.

---

## GATE A - the decision, 60 seconds

```
tail -15 ../out/flightlogs/hover_*.csv
```

Look at the **`z` column only.**

| `z` | decision |
|---|---|
| smooth ramp 0 to 0.4, then holds | **GO for the course** |
| ragged, but swings under ~0.3 m | **GO, and watch it like a hawk** - altitude will wander |
| metre-sized jumps | **STOP.** No course. Fly Sally's hover instead and send me both logs |

Also worth a glance: `vz` should read real numbers during the climb, and it
should have landed rather than bounced.

**Do not debug the barometer on the clock.** Read the column, decide, move.

---

## FLIGHT 2 - Randy, the course at k 0.10

This is the session's objective.

**Setup, and it matters more than the command:**

- Aircraft on the **real start line**, not near it
- **Nose pointed down gate 1's line.** `--map-north here` reads the heading at
  startup and builds the ENTIRE map from it. Get this wrong and every gate is
  rotated
- Aircraft **LEVEL**. The startup check will refuse if it is not still

```
AIGP_CAM_CX=627.2 AIGP_CAM_CY=368.8 \
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 20 --fy 824 --cam-hfov 76 --heading-drift-dpm 3.0 --pilot follower \
  --config ../config/ladder/vehicle_k010_cam20_75.toml \
  --traj ../out/plans/plan_LADDER_k010_cam20_75.json --arm
```

Same six human steps as flight 1.

### What success looks like

**Gate 1 is 7.3 m out and about 3 s in.** Getting through it is the win.
The plan is 2 laps and 23 gates; nobody expects that today.

| got to | verdict |
|---|---|
| gate 1 | **the objective.** First autonomous gate |
| gates 2-3 | better than expected |
| further | genuinely good |

### Abort immediately if

- it heads for a wall or a net
- it climbs past head height
- it oscillates rather than tracks
- anything at all feels wrong - you get a free retry, you do not get a free airframe

### Call outs from the laptop, loudly

- `ev0`, `ev1`... - which gate it is heading for
- `res=` - fix residual. Under ~1 m is healthy
- `fixes=` - climbing means the camera is working

---

## FLIGHT 3 - Sally hover, only if time

Her card is `docs/D44_FLIGHT_CARD.md`. **Two things must happen first and both
are on it:** the override test (never run on d44) and `tiltcheck`. If those
have not been done, **do not fly her today** - the override test is the only
abort there is.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.4 --seconds 20 \
  --ceiling 1.0 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Hover only. No camera, no course.

---

## Before you leave the track

- [ ] every `.csv` from `~/AI-GrandPrix/out/flightlogs/` on both aircraft
- [ ] how far Randy got, in gates
- [ ] battery voltages
- [ ] what looked wrong, written down now rather than remembered later

---

## What is protecting the aircraft

| | |
|---|---|
| barometer readings implying over 12 m/s | rejected, last good value held |
| the altitude loop | closes on accel+baro FUSED, not the raw sensor |
| climb demand | capped at 12 m/s^2, 2.2 g, whatever the error says |
| ceiling | raw barometer, throttle to minimum and disarm |
| implausible camera fixes | rejected beyond 4 m; gates past 15 m not fixed on |
| MSP OVERRIDE | the only one that has never failed |

Five of those were written today because something got past the other four.
Fly it like the sixth is the one you are relying on.
