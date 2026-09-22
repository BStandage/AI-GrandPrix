# Race card — 2026-09-22 — 11:48, 27 minutes, two aircraft

**The goal is g0 and g1.** Two gates puts us in the finals. Attempt 1 flies
the whole FLAT plan; everything after g1 is a bonus.

Both aircraft carry the same code and a 10 deg camera. They differ ONLY in the
numbers in the table. Sync both, dry-run both, fly the one with the cleaner
dry run, keep the other on a charged battery as the spare.

**ABORT, ALWAYS: MSP OVERRIDE off.** Throttle stick near hover position
throughout, so taking over is a handoff and not a drop.

**Two people.** One on the transmitter, finger on MSP OVERRIDE, eyes on the
aircraft. One on the laptop reading numbers. The person on the sticks never
looks at the screen.

---

## The numbers — per aircraft, do not mix

| | **d43 "King Julian"** | **d44 "Sally the Brave"** |
|---|---|---|
| ssh | `d43` | `d44` |
| `--fy` | **835.5** | **859** |
| `AIGP_CAM_CX` | **611.9** | **616.9** |
| `AIGP_CAM_CY` | **394.5** | **330.0** |
| `--cam-tilt` | **10** | **10** |
| heading drift | omit the flag | `--heading-drift-dpm 5.0` |
| **ANGLE switch** | **aux row 5** | **aux row 2** |
| record | 4 flights, level hovers, no gate | flight 4 tracked to 2.5 m, top bar |

---

## Timeline

| when | what |
|---|---|
| on arrival | `scripts/sync_drone.sh d43` and `scripts/sync_drone.sh d44` from the laptop (Sally already has it as of 08:20) |
| done | eye check: g1 is right of g0, matches the map |
| then | DRY RUN on both aircraft, one after the other, on the start line |
| if any flying space exists before 11:48 | ONE test approach: take off, track toward a gate, pilot takes over at ~4 m, land. The only test that reaches the airframe. |
| 11:48 | attempt 1 on the aircraft with the cleaner dry run |
| between attempts | pull the log, two minutes, decide; swap aircraft if one is damaged |

---

## 1 — Sync

From the repo root on the laptop:

```
scripts/sync_drone.sh d43
scripts/sync_drone.sh d44
```

---

## 2 — Eye check: DONE

g1 is to the RIGHT of g0, confirmed on the track 2026-09-22 morning. That
matches the map (1.2 m right). The plan's line to g1 and the gate
association are consistent with the real course.

---

## 3 — DRY RUN in the pits, then the PAD lines on the start line

The start line is only ours inside the slot, so the dry run happens in the
pits with no gate: it still proves the build runs (no Traceback), the FC
link, the bias readout, the release at 2 s and the sticks. Same command as
the flight with `--dry-run` in place of `--arm`. Expect `PAD: no gate in
view; bias n=...` every 2 s and `[RACELINE] released at t=2.xs`.

Then, INSIDE the slot, the gate check costs nothing: start the flight command
(`--arm`), set him on the line pointed at g0, and while you wait the 10 s for
the bias he prints every 2 s:

```
PAD: gate at 7.5 m, +12.3 deg ABOVE, offset x -0.03; bias=(+0.23,+0.04) n=210
```

Range about 7.5, ABOVE (he is on the floor), offset x near 0, n climbing.
That is g0 found and the elevation live. Then arm.

**Julian:**

```
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=611.9 AIGP_CAM_CY=394.5 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 835.5 --cam-tilt 10 --map-north here \
    --vert vision --dry-run
```

**Sally:**

```
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=616.9 AIGP_CAM_CY=330.0 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 859 --cam-tilt 10 --map-north here --heading-drift-dpm 5.0 \
    --vert vision --dry-run
```

Twenty seconds of output must show ALL of these, else do not fly it:

| line | means |
|---|---|
| `CROSSINGS: counted 0.75 m past the gate plane` at startup | the race-day build is the one running |
| `gate 0 IN SIGHT at ~7.5 m, N deg ABOVE me -> CLIMBING` | detector + elevation live (above is right: he is on the floor) |
| `[RACELINE] released at t=2.xs (over g0: ...)` | the release rule works |
| heartbeat `fixes=` climbing, `unm=0`, `res=` under 0.1 | the map matches the real g0 |
| heartbeat `bias=(+0.xx,+0.xx) n=` climbing | the bias learner runs on the pad |
| `stk=(..., >1500 pitch, ...)` after the release | the tracker asks to go forward |
| no `Traceback`, ever | the build is sound |

Ctrl-C, then fly. A dry run that shows a Traceback is a bug fixed in minutes;
a first flight that shows it is a crash.

---

## 4 — The flight

**Julian:**

```
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=611.9 AIGP_CAM_CY=394.5 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 835.5 --cam-tilt 10 --map-north here \
    --vert vision --arm
```

**Sally:**

```
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=616.9 AIGP_CAM_CY=330.0 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_STACK_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 859 --cam-tilt 10 --map-north here --heading-drift-dpm 5.0 \
    --vert vision --arm
```

**Set him on the start line. Leave him STILL for at least 10 s after the
`LIVE: waiting for the pilot` line.** He is measuring his accelerometer bias.

Throttle **fully down** → **ARM** → **MSP OVERRIDE on** → **ANGLE row 5 on
Julian, row 2 on Sally**. The clock starts when the FC reports armed + override.

On arm you get:

```
accel bias learned on the pad: [0.23 0.04 0.01] m/s^2 body (330 samples)
```

Numbers around 0.1-0.3 and a few hundred samples is right.

---

## 5 — What he does, and what the screen shows

| phase | what he does | what you see |
|---|---|---|
| takeoff, 0-1 s | 1250 punch open loop until airborne, then height from gate 0's elevation only. Barometer never in the loop. | `AIRBORNE`, `gate 0 ... ABOVE me -> CLIMBING` |
| start hold, 2-4 s | holds over the start while he climbs out; released when up 2 s and within 0.6 m, or at 4 s regardless within 1.5 m | `[RACELINE] released at t=2.3s (over g0: ...)` or `(timed out on g0: ...)` |
| approach, to ~3.5 m | planned straight line at up to 1.5 m/s, max 8 deg lean, nose on the gate, every frame fixes his position from the gate's bearing, height from its elevation | `gate 0 at 6.0 m: LEVEL, holding 1.35 m`, `fixes=` climbing, `res=` small |
| COMMIT, ~3.5 m | ring spans 85 % of the frame for 3 frames, map within 6 m. Height reference frozen (vertical-speed hold, flat on race_006/007). NO more fixes. Steers only toward the gate centre's line from the map on dead reckoning, at most 3.5 deg of lean (the plan jogs 1.2 m to g1's line in its last 1.5 m). Nose on the crossing heading. | `gate 0 COMMIT at 3.5 m ... holding 1.35 m, flying through`, roll within ~20 of 1500, `holding` flat |
| through | crossing counted 0.75 m PAST the plane, never early; straight and level until then | `gate 0 CROSSED` about half a second after he is through |
| gate 1 | fixes and height steering resume, nose swings to g1, 10 m straight ahead, same sequence | `gate 1 IN SIGHT ...` |
| after g1 | the plan turns right to g2 | your discretion, every second |
| finish | only the pilot lands him; no auto-disarm under vision | — |

**No released line by 5 s: abort.** **Roll stick swinging after COMMIT, or
`holding` rising after COMMIT: abort.**

---

## 6 — The slot plan

**Before g1 counts, protect the aircraft.** A strike there ends the day with
nothing, and a second attempt needs an aircraft. MSP OVERRIDE OFF at the first
wobble, the first off-centre approach at 3 m, or no released line by 5 s.

**After g1 counts, let him fly.** The two gates are banked the moment the
referee has them; a strike after that costs the airframe, not the result.
Go for g2 and whatever follows. Take over only to land him when the run is
plainly over.

**Abort if the degrees figure GROWS as the range shrinks** on the approach.
That is the clipped-ring runaway. It has not happened at 10 deg.

---

## 7 — Between attempts

From the laptop:

```
scripts/pull_flight.sh d43        # or d44
```

Read the narration for two minutes, then decide. Do not fly a second attempt
to find out what happened in the first. Swap aircraft if one is damaged; the
other has the same code and its own numbers above.

---

## Switches, only if a dry run or a flight says so

| put in front of the command | effect |
|---|---|
| `AIGP_ACCEL_BIAS=0` | no bias learning on the pad: yesterday's estimator |
| `AIGP_COMMIT_STRAIGHT=0` | committed = the full dead-reckoned lateral loop instead of the gentle gate-centred steer |
| `AIGP_COMMIT_HOLD=0` | committed = the old barometer vertical-speed hold instead of the held hover throttle (do not) |

---

## The build that flies: sim-angle-loop (branch tip)

Latest full-course result in the Betaflight sim, ANGLE mode, harsh
barometer, real IMU noise, plan_STACK_s15: **20 of 23 gates**, lap one
complete in 114 s, hairpin at -0.08 m, the stacked gate's top opening at
+0.04 m and its low opening at -0.02 m, every other gate within 0.11 m.
The plan is plan_STACK (the FLAT plan skipped the top opening and the
referee stops crediting there). The reversal at the stack is 1.69 m south
of it. The real detector, run over the organizer's lap video, detects the
red gates on 100 % of frames and switches to the next gate the frame after
passing one; the stacked gate reads as ONE tall blob, so the detector now
recognises a stack and aims the height at the upper or lower ring.

Superseded by the above; kept for the record:

Flown in the Betaflight sim in ANGLE mode with the harsh barometer model and
a real IMU's noise: g0, g1, g2, g3, g4 crossed at +0.01, 0.00, -0.01, +0.05,
-0.02 m of centre, height 1.4-1.9 m throughout, no strike. The vertical
channel never uses the barometer: the gate's elevation before commit, the
pad-calibrated accelerometer for damping, the hover throttle held through
the gate. `bias=(...) n=` on the heartbeat is that calibration - the pad wait
matters.

## What changed since yesterday's flights (all in this build, none flown)

1. Release: up 2 s and over the start, or 4 s regardless. No velocity check, no vertical dwell.
2. Accelerometer bias learned on the pad, subtracted in flight.
3. No position fixes on a committed gate.
4. Committed = zero roll, pitch along the nose.
5. No climb feedforward after commit under vision.
6. Commit only when the map puts the gate within 6 m.
7. Crossings counted 0.75 m past the plane.
8. Tilt cap 10 -> 8 deg.
9. No auto-disarm at the plan's finish under vision.
10. Bias readout on every heartbeat line.
