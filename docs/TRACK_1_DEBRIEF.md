# Track session 1 - debrief

**20-21 September 2026.** First autonomous flights on the real aircraft.

Randy flew once and broke his arm. Sally flew twice and hovered cleanly both
times. Between those two facts sits the finding the whole day had been
pointing at, and it came out of Randy's crash log.

---

## One paragraph

**The aircraft was being told to climb all day.** Our thrust curve came from
the organizers' flight logs, which measure thrust *per kilogram of their
aircraft*. Theirs is heavier than ours, so at any given throttle ours
accelerates harder - commanding 1 g delivered about 1.45 g. Every flight
climbed away and each time we blamed an instrument. Three of those instruments
really were broken and all three were worth fixing, but none of them was why
the drones flew into things. Once the curve was corrected from flight data,
Sally hovered on the first attempt with no oscillation at all.

---

## The flights

**Three flights on the track. Randy got one.**

| # | aircraft | what we asked for | what happened |
|---|---|---|---|
| 1 | **Randy** | `--no-baro`, hold vertical speed | climbed steadily to **5.1 m**, drifted left, hit the net. **Arm broken.** His only flight |
| - | - | **thrust curve corrected from flight 1's own log** | - |
| 2 | Sally | `--no-baro`, gentle | **clean hover.** No oscillation |
| 3 | Sally | same again | **clean hover.** Confirmed |

Randy's arm was repaired. Sally was never damaged.

### What came before, in the cage

Context for why Randy was flying `--no-baro` at all. Earlier the same day, in
the cage:

- a hover at 0.76 m climbed to about 3.3 m and fell. **Camera destroyed** -
  replaced and recalibrated (`fy 824`, `hfov 75.8`, RMS 0.281 px, tilt 20 deg)
- two hovers at 0.40 m with a 1.0 m ceiling were caught and disarmed
  automatically at 1.06 and 1.29 m
- the barometer was found to read `-0.10 -> -3.86 -> -0.49 -> +1.00` with
  props running, so it was taken out of the loop entirely

Every one of those was read as an instrument fault. They were real faults. The
aircraft was also being told to climb the whole time, and nobody had asked
whether the thrust model was right.

---

## Randy's flight - how he broke his arm

Worth walking through, because the log contains the answer to the whole day.

We had just taken the barometer out of the loop entirely (`--no-baro`) because
it was producing nonsense with props running. Instead the aircraft held
**vertical speed** at zero using the accelerometer, which we had proved was
accurate.

It climbed anyway:

```
climb done after 0.45 s, vz=+2.79
t=1.0   vz=+2.07   thr=1209   a_cmd=-4.00
t=2.0   vz=+1.23   thr=1228   a_cmd=-3.06
t=3.0   vz=+1.17   thr=1231   a_cmd=-2.93
t=4.0   vz=+1.19   thr=1230   a_cmd=-2.99
```

The controller was asking for the maximum descent it was allowed, `-4 m/s^2`,
and the aircraft climbed at a steady 1.2 m/s regardless. It drifted sideways
and reached the net before the pilot's abort could bring it down.

**That is the finding.** A commanded `-4 m/s^2` that produces no descent means
the throttle we chose for "4 m/s^2 below hover" was actually **hover**. Two
numbers from that same log, and they agree:

| | curve said | aircraft actually did |
|---|---|---|
| vertical speed CONSTANT at ~1228 PWM | 6.74 m/s^2 | **9.81** - exactly cancelling gravity |
| 1350 PWM reached 2.79 m/s in 0.45 s | 12.97 m/s^2 | **16.0** |

The old curve needed 1291 and 1407 PWM for those. That is **-63 and -57 PWM**.
A single 60 PWM shift explains both.

```
hover_pwm    1291 -> 1228
takeoff_pwm  1350 -> 1289   (1.32 g)
curve_acc = [1.18, 5.87, 13.03, 21.86, 31.03, 44.35, 58.61]
```

---

## Sally's two - the hover that worked

Applied the corrected curve, synced, and flew her gently: 1.12 g of takeoff
for 0.6 s, then hold vertical speed at zero.

```
climb done after 0.60 s, vz=+1.37
t=1.0   vz=+0.31
t=2.0   vz=+0.07
t=3.0   vz=+0.07
t=4.0   vz=-0.01
t=5.0   vz=+0.02
t=6.0   vz=+0.01
```

**A 1.37 m/s climb arrested in under a second, then vertical speed held inside
0.11 m/s for five seconds with no oscillation.**

### How high

Her barometer, which behaved throughout this flight, puts her at:

| | |
|---|---|
| after the takeoff burn | **0.17 m** |
| peak | **0.73 m** |
| where she was when the pilot took over | **0.56 m** |

So roughly **half a metre to three quarters**, slowly sinking. Deliberately
low - this was the first flight after Randy's crash.

### The number we came for

**Steady hover throttle: median 1227 PWM, range 1224-1229.**

The corrected curve predicts **1228**. That was derived from *Randy's crash
log*, an hour earlier, on a *different aircraft*. **One PWM apart.** The
thrust model is now measured rather than assumed.

### Current draw tells its own story

| | |
|---|---|
| Sally, hovering correctly | **17 A** |
| Randy, climbing on the wrong curve | **35-55 A** |

Randy was drawing two to three times the current because he was being told to
climb. Which brings us to the barometer.

---

## The barometer: less broken than we thought

Randy's barometer produced this with props running, while sitting at about
0.3 m:

```
-0.10   ->   -3.86   ->   -0.49   ->   +1.00
```

A 3.8 m step in 100 ms is 37 m/s. The altitude loop believed it was four
metres low and commanded 4.35 g.

We added foam over the sensor. It did not help, so we concluded the cause was
vibration or electrical noise and took the barometer out of the loop.

**Sally's barometer, in the same conditions, was clean:**

```
0.51  0.51  0.51  0.62  0.69  0.69  0.71  0.71  0.72  0.73  0.70  0.65  0.60  0.57  0.55
```

A smooth, believable trace of a gentle sink.

**The difference is current.** Sally was drawing 17 A. Randy was drawing
35-55 A, because the wrong curve had him climbing hard. The barometer failure
was largely a **symptom of the over-thrust**, not an independent fault.

**So fixing the thrust curve may have fixed the barometer too.** That is the
first thing to test next session: a normal altitude-hold hover on Sally.

The prop protection is a candidate too - it changes the airflow around the
whole frame, and Sally carries the same foam. Neither explanation is settled;
both are cheap to test.

---

## Four other bugs, all real, none the cause

Each of these was found and fixed during the day. Every one would have bitten
us eventually. None of them is why an aircraft hit a net.

**Pitch was mirrored.** Betaflight on this firmware reports pitch positive
NOSE DOWN; we assumed the opposite. The error is `g*(cos 2t - 1)`: **exactly
zero when the aircraft is level**, -2.3 m/s^2 at 20 degrees, -5.2 at 31. Every
check we had ever run was done flat on a bench. The same rotation carries the
accelerometer into the world frame for dead reckoning, so the follower would
have integrated forward acceleration *backwards* and been lost within seconds
of the start line. Found by tilting the aircraft by hand
(`hardware.tiltcheck`), which now exists because of this.

**The speedometer could not see a launch.** Vertical speed was held at zero
until three BAROMETER samples read above 0.25 m - about 0.3 s after liftoff,
by which time a 1.3 g takeoff is doing 2.5 m/s - and then counted up from
zero. The altitude loop's brake saw nothing to brake. Now it watches the
accelerometer, which answers every tick.

**`hover.py` never applied the accelerometer scale it asked for.** The
argument existed and nothing read it. Harmless until the filter started
integrating, then it became a constant phantom acceleration.

**The flight controller reports no vertical speed at all.** `MSP_ALTITUDE`'s
vario field reads exactly 0.00 m/s forever on this firmware. We were passing
that straight through. It also meant the airborne latch that gates dead
reckoning could never fire - the follower would have flown the whole course
believing it was parked on the start line.

---

## What we got right

**The abort works.** MSP OVERRIDE off returned control instantly, every single
time, including on the flight that hit the net. It is the reason Randy's repairs
cost us an evening rather than the competition. The bench test that proved it
paid for itself twice in one day.

**The ceiling backstop works.** Flights 2 and 3 were caught and disarmed
automatically at 1.06 and 1.29 m. It reads the raw barometer and trusts no
filter, which is exactly right for a day when every clever layer agreed with
itself and was wrong.

**The camera works.** After Randy's camera was replaced and recalibrated
(`fy 824`, `hfov 75.8`, RMS 0.281 px, mount tilt 20 deg), the bench residual
check tracked a gate smoothly from 4.4 m in to 1.8 m and back out, and fixed
position with **1 to 6 cm** of residual. That is the one part of the stack
that has never misbehaved.

---

## Lessons

**1. Three identical symptoms are a pattern, not three faults.**
Flights 1, 2 and 3 all climbed further than commanded. Each was diagnosed as a
different instrument. The third one should have prompted the question "is the
*model* wrong?" rather than a fourth sensor fix. That is what put Randy in the
net.

**2. A bench cannot check a thrust curve.**
We "confirmed" `hover_pwm 1291` on a table, props off, and it agreed to 3 PWM.
What that confirmed was the controller's arithmetic. The aircraft was never
asked to lift itself. The only test of a thrust model is thrust.

**3. Specific thrust is per kilogram, and the kilogram was someone else's.**
The blackbox curve was excellent data about the organizers' aircraft. Carrying
it to ours without asking whether the masses matched is the whole error, and
it is the same mistake as copying calibration numbers between airframes.

**4. A bench is not a small version of flight.**
The pitch sign was wrong for months and passed every check, because the error
is exactly zero in the one attitude a bench ever tests. Test the thing in the
state it will actually be in.

**5. Put a dumb limit around the clever thing.**
Every layer of the altitude system was individually reasonable and they were
all wrong together. The ceiling does not reason at all, which is why it worked.

---

## Numbers from the session

| | |
|---|---|
| hover throttle, measured in flight | **1227 PWM** (median, 1224-1229) |
| thrust curve shift applied | **-60 PWM** |
| Sally's hover height | **0.17 m to 0.73 m**, sinking slowly |
| vertical speed held | **within 0.11 m/s**, no oscillation |
| hover current | **17 A** |
| `--no-baro` drift | **0.11 m/s per 1.5 s** - about a metre over a 15 s hold |
| accelerometer at rest | **0.00 m/s^2**, both aircraft |
| camera fix residual (bench) | **1-6 cm** |
| Randy | 1 flight, 1 crash, arm repaired |
| Sally | 2 flights, 2 clean hovers, undamaged |

---

## Next session, in order

**1. Work out why Sally drifts forward.** She translated the whole hover on
centred sticks. Two candidates, and they are distinguishable in thirty
seconds:

| cause | FC on a level surface | in the hover |
|---|---|---|
| flight controller level trim | roll/pitch **not** 0 | really leans, so it accelerates |
| foam prop protection | roll/pitch **~0** | holds level, translates from asymmetric airflow |

Run `hardware.tiltcheck` on a surface checked with a level and read the first
line. If the FC is honest, it is aerodynamic. `hover.py` now logs the ACTUAL
roll and pitch alongside the commanded sticks, so the next flight answers this
by itself.

Either way it matters beyond the drift: a 2 degree attitude error is
`0.34 m/s^2` of acceleration our dead reckoning does not know about, which is
about 2.7 m of phantom position by the time you reach gate 1 - wider than the
opening. If it IS the trim, Betaflight **Setup -> Calibrate Accelerometer** on
a level surface fixes it.

**2. Normal altitude-hold hover on Sally** (no `--no-baro`). With the
corrected curve she sits at 17 A, where her barometer behaves. If it holds
altitude, we have a flyable aircraft and the course is back on.

**3. Run the override test on Sally.** It has still never been run on d44. It
is the only abort there is.

**4. Then the course, at k 0.10.** Gate 1 is 7.3 m out and about 3 s in.
Getting through it is the objective; the plan's 23 gates are not.

**Still open and not blocking:** the gate detector reports a gate in 99.6
percent of frames including with the lens covered, and its range estimate read
2.4, then 9.8, then 140 m inside ten seconds. The association layer rejects
these correctly - the bench residual check proved that - but the detector
itself needs offline work on recorded footage.
