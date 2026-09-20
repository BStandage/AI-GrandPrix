# 20 September 2026 - first autonomous flight

d45 flew itself for the first time. It went to the ceiling, the pilot took
control back, and we broke a camera. The thrust model was proved correct, the
safety system worked exactly as designed, and we found and fixed the bug that
caused it.

---

## The bug, in plain words

**The drone's speedometer read zero for the first third of a second after
takeoff.**

The brake is "I'm going up too fast, ease off". With the speedometer reading
zero, the drone believed it was sitting still. So it never braked. It carried
on at the speed it had already built, straight up.

It was never confused about **where** it was - the altimeter was fine
throughout. It was confused about **how fast it was getting there**.

The speedometer was calculated from the altimeter, and the altimeter only
reports about ten times a second. By the time enough readings had arrived to
say "you are moving", the aircraft was already doing 2.5 m/s.

Here are four lines from the flight log. `z` is height, `vz` is the
speedometer:

```
t= 1.0   z= 1.93 m   vz= +0.60 m/s      <- really climbing at ~3.5
t= 2.0   z= 3.28 m   vz= -3.69 m/s      <- three metres up, still rising,
                                           and the speedometer says falling
```

## The fix

The aircraft has an accelerometer that answers **every single tick**, hundreds
of times a second, instead of ten. It felt the launch instantly. We simply
were not listening to it for this.

Now we are, and the speedometer starts from the speed the push has already
produced instead of from zero. Tested offline against the same launch:

| | reported speed | true speed |
|---|---|---|
| old | 1.16 m/s | 3.20 m/s |
| new | 3.21 m/s | 3.20 m/s |

**And a dumb backstop.** `hover.py` now takes `--ceiling`, default one metre
above the target. It watches the raw altimeter - no filters, no controller, no
clever logic - and if the aircraft is above it, the throttle goes to minimum
and it disarms. Today every clever layer agreed with itself and every clever
layer was wrong, so the backstop deliberately trusts none of them.

---

## What worked

| | |
|---|---|
| **The abort** | Pilot flipped MSP OVERRIDE off and had the aircraft back instantly. This is the whole reason today cost a camera and not an airframe. The bench test we ran yesterday paid for itself. |
| **The thrust model** | It reached 0.88 m in 0.7 s. The curve we fitted from the organizers' blackbox predicted 0.74 s. The single biggest unknown going in, and the flight closed it. |
| **Takeoff behaviour** | Lifted off gently at 1.32 g, exactly as commanded. |
| **Landing logic** | Never got to prove itself, but the limit-cycle bug found on the bench yesterday was already fixed. |

## What we lost

- **The camera on d45.** It fell about 2.25 m.
- About half a cage session.

## What it costs us

**d45's camera calibration is now void.** `fy = 830`, `cx = 613.1`,
`cy = 387.0` and the 20 degree mount tilt were all measured on *that specific
camera in that specific mount*. A replacement is a different lens in a
different position. Every one of those numbers has to be measured again -
`docs/CAMERA_CALIBRATION.md`, about 20 minutes.

Flying the old numbers with a new camera puts a constant, invisible bias into
every gate fix.

---

## Lessons

**1. Never let a fast decision depend on the slowest sensor.**
The altimeter is the slowest thing on the aircraft, and it was the sole judge
of "are we moving yet". Everything built on top of it was correct and the
whole stack was still wrong.

**2. A passing bench test proves less than it looks.**
This is the **third** bug in two days where the aircraft sat on a bench and
everything read fine. Sitting still is exactly what all three broken paths did
correctly:

- `MSP_BOXNAMES` returned nothing, so the software could not see its own
  flight modes - the override test passed anyway, because the *aircraft* was
  fine and only our view of it was blind
- the flight controller reports no vertical speed at all, always exactly
  0.00 - invisible until something had to move
- today: the speedometer lagged, which only matters while accelerating

**3. Put a dumb limit around the clever thing.**
Every layer of the altitude system was individually reasonable. The hard
ceiling does not reason at all, which is the point.

**4. Fly the smallest thing first.** We did, and it is why we still have an
aircraft. A 0.76 m hover in a cage found a bug that would have ended a race
run.

---

## Where that leaves us

**Proved on hardware:** thrust curve, hover throttle, takeoff behaviour,
override abort, camera calibration procedure, dead-reckoning ground hold.

**Fixed today, not yet flown:** launch detection, the hard ceiling, vision
altitude hold.

**Still open:** all-up weight is measured (1.751 kg) but the plan has never
flown; the gate detector reported a gate in 99.6 percent of frames including
with the lens covered, and its range estimate read 2.4, 9.8 and then 140 m
within ten seconds - that needs real footage and offline work, not cage time.

**Next aircraft up:** full setup from `docs/NEW_DRONE_SETUP.md`, its own
camera calibration, its own flight card. Nothing transfers between airframes.
