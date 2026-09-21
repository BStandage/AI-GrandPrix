# Track session 2 - debrief

**21 September 2026, 17:20 to 17:35.** Sally, d44. Four autonomous course
flights in fourteen minutes. No damage.

---

## One paragraph

**The first autonomous gate approaches this team has ever flown.** Every
flight failed vertically and every flight succeeded laterally, and the four
failures had four different causes - each one only visible once the previous
one was fixed. By the fourth flight she tracked the line from 7.4 m out to
2.5 m from the gate with 0.22 m of cross-track error and a 0.02 m fix
residual, and clipped the top bar on the final metre. Camera fixes went from
15 on the first flight to 256 on the last. Nothing that broke was the
controller; every cause was a piece of geometry or bookkeeping that had never
been checked in flight.

---

## The session at a glance

| # | log | what we changed first | what happened |
|---|---|---|---|
| 1 | `race_002` | nothing - first run | **vertical runaway.** Climbed away, ~1 m over the top of the gate |
| 2 | `race_003` | rebuild clipped gate centre from width | **held gate centre 4 s**, then climbed when close |
| 3 | `race_004` | reject gate when it overfills frame; camera moved to 10 deg | **altitude limit cycle.** Bounced off the ground and reclimbed, repeatedly |
| 4 | `race_005` | latch `airborne` | **best flight.** Tracked to 2.5 m, then clipped the top bar |

---

## Flight 1 - the vertical runaway

`AIGP_VERT=vision`, s15, `--cam-tilt 20`.

She took off correctly onto gate centre, then climbed away and went about a
metre over the top of the gate.

```
t=2.0   z=1.68  zt=2.03   +0.35   <- at the cap
t=3.0   z=1.82  zt=2.17   +0.35   <- at the cap
t=4.0   z=2.67  zt=3.01   +0.34   <- at the cap
t=6.0   z=4.02  zt=4.11   +0.09   <- gate leaving frame
t=8.0   z=4.90  zt=4.97           <- plateau, gate gone
```

`dz_vision` sat at its **+0.35 m maximum for 35 % of the flight**. It never
once said "you are level". It could not.

### Cause: the camera cannot see a gate at its own height

The camera was tilted **UP 20 deg** with a **45.5 deg vertical field**. The
frame therefore covers elevations **-2.75 deg to +42.75 deg**. A gate at the
aircraft's own height is at 0 deg - **2.75 deg from the bottom edge of the
image** - so its lower half is cut off at every useful range:

| range | gate visible |
|---|---|
| 4 m | 63 % |
| 7 m | 72 % |
| 9 m | 79 % |

A clipped gate has a false centre: the blob's centroid sits above the true
centre because the bottom is missing. So elevation reads positive, the
vertical reference saturates upward, and the aircraft climbs - **and climbing
cuts off more of the gate**, so it runs away. To see a gate centred in this
camera it would have to be 2.55 m above her at 7 m, which on a flat course at
gate height is geometrically impossible.

**Nobody had ever worked out the vertical frame budget.** The calibration doc
says to re-adjust the hinge above ~30 deg "where gates at your own altitude
drop out of frame", but that threshold assumed a wider camera. At 45.5 deg
the real threshold is ~23 deg, and the gate's own half-height eats the
remaining margin immediately.

### Fix

The ring is **square** - 2.7 x 2.7 m per the course map - and the WIDTH is
never clipped. So when the box runs off the bottom of the frame the true
height in pixels IS the width, and the centre is reconstructed from the
unclipped edge. Exact for a square gate, not a fudge factor. Verified
offline: a bottom-clipped ring read 41.5 px (2.8 deg) low, corrected to
0.5 px.

---

## Flight 2 - held gate centre, then lost it on the approach

She **held 1.37, 1.52, 1.42, 1.48 m for four seconds** against a 1.35 m gate
centre. First time that has ever happened.

Then she climbed, starting exactly as `det` fell to 2.87 m and then 1.81 m.

### Cause: the guard switched itself off when it was needed most

The reconstruction was guarded on **exactly one edge** being cut
(`at_bottom != at_top`). At 2 m a 2.7 m gate subtends **68.6 deg** against a
45.5 deg field: it overfills the frame and clips at BOTH edges. So the
correction disabled itself precisely where the bias was worst, and the raw
centroid took back over.

The old cage cards already knew the number - "a 2.7 m gate does not fit in
this camera's vertical view until about 3.1 m" - and it was never connected to
this code path.

### Fix

When both edges are cut there is no unclipped edge to rebuild from, so the
detection carries no usable elevation. Say so (`v_usable=False`) rather than
guess: the follower then fades its vertical reference out and holds the height
it already had.

### Also seen, unexplained

`airborne at t=0.8s (z=0.51 vz=+9.52)`. She is not doing 9.5 m/s off the pad.
Flight 1 read +6.10, flight 3 +4.41, flight 4 +2.85. The launch seed is
over-reading, and it is why throttle slammed to 1035 in the first second.

---

## Flight 3 - the altitude limit cycle

Camera physically moved to **10 deg** (the mount had shifted), both-edge
rejection in.

She oscillated violently, bouncing off the ground and reclimbing to gate
height, over and over. **We never had constant thrust.**

```
t=3.0  z=+1.60          t=4.0  z=-0.30  air=False  thr=1250
t=5.0  z=+1.52          t=6.0  z=-1.44  air=False  thr=1250
t=7.0  z=+1.64          t=8.0  z=-0.85  air=False
```

### Cause: the controller kept re-entering TAKEOFF

`AltitudeLoop` returns an **open-loop `takeoff_pwm`** whenever `not airborne`,
and `airborne` was recomputed every tick as `est.p[2] >= min_alt_translation_m`
- which is **0.0**. Under prop wash the height estimate swings straight
through zero, so the controller was thrown out of closed loop and back into
takeoff mode several times a second, each time commanding 1250 PWM with no
feedback at all.

**Measured: 27.3 % of all control ticks in flight 3 sat at exactly 1250 PWM.**
`air=False` and `thr=1250` appear together on every bounce in the log.

This exact path had been flagged before the session and dismissed as unlikely,
on the grounds that the FUSED height would not go negative from a single
spike. It was not a single spike. The estimate was below zero for **29 % of
the flight**.

### Fix

An aircraft that has left the ground has left it. `airborne` now **latches**
for the run: once true it stays true, so a lying barometer can no longer
re-arm the takeoff branch. `reset_state()` clears it between runs.

---

## Flight 4 - almost through

The best flight of the session.

```
det  7.39 -> 6.88 -> 7.46 -> 6.65 -> 6.07 -> 5.25 -> 4.31 -> 3.51 -> 2.48 m
z    0.35    1.04    0.52    0.88    1.06    0.97    1.10    1.01    0.89
xtrack 0.63  0.88    0.45    0.56    0.64    0.49    0.50    0.22    0.35
res  0.03    0.02    0.02    0.02    0.03    0.02    0.03    0.04    0.02
```

`air=True` throughout. Throttle steady 1232-1271. **No 1250 punches** - the
latch held, and the share of ticks pinned at 1250 fell from 27.3 % to 4.8 %.
The vertical excursion narrowed from a 4.73 m span to 2.52 m.

Then at `det=1.81 m` she corrected upward - 0.89 -> 1.44 -> 1.91 - and clipped
the top bar.

### Cause: the correction was real, and far too late

She genuinely WAS low: holding ~1.0 m against a 1.35 m gate centre. But a
vertical correction begun 1.8 m from a gate has nowhere to go except into a
bar. At 1.5 m/s that is barely a second of warning.

### Fix, built but NOT yet flown

**Commit through the gate.** Inside ~3.9 m the vertical reference is released
and she flies through on the height she already has.

Measured by **apparent size**, not `range_m` - range read 12.6, 22.9 and
**59.5 m** on this same flight while she was two metres from a gate, whereas
the ring's height in pixels is a direct observation.

| range | ring fills | steers height? |
|---|---|---|
| 5.0 m | 70 % | yes |
| 4.5 m | 77 % | yes |
| **4.0 m** | **86 %** | **committed** |
| 3.5 m | 96 % | committed |

At 4.5 m she was holding 1.01-1.10 m - 0.25-0.35 m below gate centre, well
inside the 0.75 m half-opening. She would have gone through.

---

## What worked

| | |
|---|---|
| **The abort** | four flights, four aborts, **no damage**. It has still never failed |
| **The lateral channel** | never the problem, on any flight. `res` median 0.08, `xtrack` to 0.22 m |
| **Camera fixes** | **15 -> 60 -> 237 -> 256** across the four flights |
| **The vision architecture** | `z_target = est.p[2] + dz` means the barometer cancels out of the error. This is why a sensor that was below zero for half of flight 4 did not fly the aircraft into the ground |
| **Takeoff** | correct on every flight. The plan, the thrust curve and `vz_up_max` put her on gate centre every time |
| **Iteration speed** | four flights, four causes, four fixes, fourteen minutes |

---

## Lessons

**1. Work out the frame budget before trusting a camera to hold a height.**
Vision altitude was designed, reviewed, bench-checked and flown before anyone
asked the simplest question: when the aircraft is level with a gate, where
does that gate land in the image? The answer was "2.75 degrees off the bottom
edge", and everything downstream followed from it. One line of trigonometry,
never done.

**2. A guard that disables itself in the worst case is worse than no guard.**
The clipped-gate reconstruction was conditioned on exactly one edge being cut,
so it switched off at close range - the only place it mattered. It looked like
a safety condition and behaved like an off switch.

**3. Do not recompute a latched physical fact from a noisy sensor.**
"Are we airborne" is not a question that should be re-asked at 50 Hz from a
barometer that swings +-1.5 m. The aircraft left the ground once. Everything
else was bookkeeping that the sensor got a vote in, and it voted wrong 29 % of
the time.

**4. Prefer a direct observation to a derived one.**
Range is the camera's worst signal and read 59.5 m at two metres. The ring's
height in pixels needs no range, no calibration and no model. Every close-range
decision now keys off apparent size instead.

**5. The height estimate is not merely noisy - it is wrong.**
Flight 1's estimate peaked at **6.85 m** for a flight observed at roughly
2.9 m. Under props this number should not be trusted in absolute terms at all,
which is precisely the argument for a control law where it cancels.

**6. Fixing one bug is the only way to see the next.**
Four flights, four causes, and none of the later three was visible until the
earlier one was gone. Hardware iteration beat analysis here - the whole session
was fourteen minutes and each flight cost about three.

---

## Still open

| | |
|---|---|
| **She flies ~0.35 m low** | holds ~1.0 m against a 1.35 m gate centre. `--cam-tilt 10` is an ESTIMATE - the mount moved mid-session and was never re-measured. `camtilt` is five minutes and would buy back the margin |
| **The commit fix is unflown** | built, unit-tested, NOT yet on the aircraft |
| **Launch seed over-reads** | vz reads +2.85 to +9.52 at liftoff. Never explained |
| **Height estimate swings +-1.5 m** | the barometer under prop wash. Bounded now, not fixed |
| **`runtime.py` has no ceiling** | the raw-barometer backstop exists only in `hover.py`. The pilot is the only ceiling on the course |
| **`note_throttle` / `baro_trusted` unwired** | both barometer protections live only in `hover.py`; the automatic fallback cannot fire on the course |
| **2 pre-existing test failures** | `test_raceline.TestStandoffRule`, unrelated to anything flown today, present before this session |

---

## Numbers from the session

| | |
|---|---|
| flights | 4, in 14 minutes |
| damage | **none** |
| best cross-track | **0.22 m** |
| best fix residual | **0.02 m** |
| camera fixes, first -> last flight | **15 -> 256** |
| ticks pinned at 1250 PWM, F3 -> F4 | **27.3 % -> 4.8 %** |
| vertical excursion span, F3 -> F4 | **4.73 m -> 2.52 m** |
| closest approach under control | **1.8 m from the gate** |
| gates completed | **0** |

---

## Next session, in order

1. **`camtilt` on Sally.** Five minutes, and it is the difference between
   flying through the middle of a gate and flying through the top of one.
2. **Sync the commit fix** and re-fly the same plan. It is the only thing
   between flight 4 and a completed gate.
3. **Wire `note_throttle` / `baro_trusted` into `runtime.py`.** Ten lines,
   mirroring `hover.py`, and it arms the automatic fallback on the course.
4. **Explain the launch seed.** `vz=+9.52` off the pad is not physical and it
   drives the throttle hard in the first second of every flight.
5. **Then the course.** Flight 4 was one fix away from a gate.
