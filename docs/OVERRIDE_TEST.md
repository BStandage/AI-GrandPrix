# MSP override test

Run once per aircraft. Nothing flies autonomously until it passes.
About 5 minutes. Easier with two people: one holding the transmitter, one reading
the screen.

---

## The one idea behind this test

Two different things can move the drone's sticks:

- **The TRANSMITTER** - the radio in a human's hands
- **The JETSON** - the computer bolted to the drone, sending stick values down
  a wire to the flight controller

Only one of them is in charge at a time. A switch on the transmitter decides which:

| MSP OVERRIDE switch | Sticks come from | Switches come from |
|---|---|---|
| **OFF** | the TRANSMITTER | the TRANSMITTER |
| **ON** | the JETSON | the TRANSMITTER |

So when the switch is **ON**, a human waggling the transmitter's sticks does
nothing - the flight controller is ignoring them and using the Jetson's
numbers instead. The human still owns the switches, which is how they stop it.

**This test asks one question: with the Jetson still sending, does flipping
that switch OFF give the sticks back to the TRANSMITTER?**

If the answer is no, there is no way to stop a runaway drone, because our
software cannot disarm it and a crashed Jetson cannot stop itself.

---

## Why it might fail

Betaflight ships with `msp_override_channels_mask = 11`. That value lets the
Jetson write an AUX channel - possibly the very channel carrying the MSP
OVERRIDE switch. The Jetson can then hold the switch ON from the drone's side,
and moving the real switch on the transmitter does nothing.

`15` covers the four sticks and no AUX channel. Setting it is a claim. This
test is the proof.

---

## What you need

- The aircraft, **PROPS OFF**
- Flight battery plugged in
- The transmitter, turned on and bound to this aircraft
- An SSH session to the drone

You do NOT need a cable to the flight controller, and you do NOT need
Betaflight Configurator open.

---

## Setup

1. **Take the props off.** Look at all four hubs.
2. Plug in the flight battery.
3. Turn the transmitter on.
4. Put both switches **down/off**: ARM off, MSP OVERRIDE off.
5. SSH in:

```
ssh d45
cd ~/AI-GrandPrix/src
```

6. Check the flight controller is hearing the transmitter:

```
python3 ~/target/msp/msp_bench.py --port /dev/ttyTHS1 info
```

The line `Arming blocked by:` must **not** include `RX_FAILSAFE`. If it does,
the transmitter is off or not bound to this aircraft. Fix that first - nothing below
will work.

---

## Start the test

```
python3 -m hardware.override_test --port /dev/ttyTHS1
```

It prints a header once, then ONE line that updates in place:

```
 who has the sticks         | armed | override | ANGLE | roll pitch  yaw  thr | 1 2 3 |
 --------------------------------------------------------------------------------------
 TRANSMITTER has the sticks | no    | off      | ON    | 1500 1500 1500  988 | _ _ _ |
```

| Column | Means |
|---|---|
| who has the sticks | `TRANSMITTER` or `JETSON` - who the flight controller is listening to |
| armed | is the aircraft armed |
| override | is the MSP OVERRIDE switch on |
| ANGLE | is ANGLE mode active (it must be - see the note at the end) |
| roll pitch yaw thr | what the flight controller actually has on those four channels |
| 1 2 3 | the three tests. `_` becomes `X` as each one passes |

The four numbers are the thing to watch. When the TRANSMITTER has the sticks
they follow your sticks. When the JETSON has them they sit at 1123 1234 1345 -
values our program picked precisely because nobody's sticks land there.

---

## Test 1: the JETSON takes the sticks

| Step | The person holding the TRANSMITTER does | The screen should show |
|---|---|---|
| 1 | Flip **MSP OVERRIDE ON** (channel 9) | `JETSON has the sticks`, `override ON`, and `1.X` |
| 2 | Waggle the sticks | the `ch1-4` numbers do **NOT** move |

If the numbers still follow the sticks, override did not engage - check which
channel that switch is actually on.

---

## Test 2: the TRANSMITTER gets them back

**This is the test that matters. The other two are context.**

| Step | The person holding the TRANSMITTER does | The screen should show |
|---|---|---|
| 3 | Flip **MSP OVERRIDE OFF** | `TRANSMITTER has the sticks`, and `2.X` |
| 4 | Waggle the sticks | the `ch1-4` numbers follow them again, immediately |

The program is still sending stick values the whole time. That is exactly the
point: control has to come back **while the Jetson is still talking**, because
in a real flight it will be.

---

## Test 3: the TRANSMITTER can still disarm

| Step | The person holding the TRANSMITTER does | The screen should show |
|---|---|---|
| 5 | Flip **MSP OVERRIDE ON** again | `JETSON has the sticks` |
| 6 | Pull throttle **fully down** | |
| 7 | Flip **ARM ON** (channel 5) | `armed YES` |
| 8 | Flip **ARM OFF** | `armed no`, and `3.X` |

Nothing spins. The props are off and the program holds throttle at minimum
throughout.

---

## Finish

9. Both switches **off**.
10. Press `Ctrl+C`.

---

## The result

```
  1. JETSON took the sticks      PASS
  2. TRANSMITTER got them back   PASS
  3. TRANSMITTER disarmed        PASS
  flight modes seen              ANGLE, ARM, MSP OVERRIDE

  RESULT: PASS - the TRANSMITTER can take control back.
```

| Outcome | What to do |
|---|---|
| **PASS** | This aircraft may fly autonomously. Record it. |
| **INCOMPLETE** | Test 2 never happened. **Do not fly it.** Check `get msp_override_channels_mask` reads 15 in the Betaflight CLI. |
| **`MIXED - WRONG MASK!`** seen at any point | The flight controller had only SOME of channels 1-4 from the Jetson. That is exactly what a wrong mask looks like. **Stop.** |
| **`ANGLE was never active`** warning | Fix before flying. See below. |

---

## About ANGLE

The screen shows `ANGLE ON` or `ANGLE OFF`. It must be ON.

In ANGLE mode a stick position means "lean this far". In ACRO it means "rotate
at this rate". Our software sends lean angles. If the aircraft is in ACRO it
reads them as rotation rates and does something completely different from what
the plan asked for.

On these drones ANGLE is hard-coded always-on via an aux row, so it should
read ON the whole time without touching anything. If it reads OFF, the aux row
did not take - see `docs/NEW_DRONE_SETUP.md` section 9.

---

## One thing to know before flying

When the person on the transmitter flips MSP OVERRIDE off **in flight**, their
throttle stick takes effect instantly. If it is sitting at the bottom, the
aircraft drops.

So during any autonomous flight they should hold the throttle stick at roughly
**hover position**, so taking over is a handoff and not a fall. Practise it
once during the cage hover.

---

## Safety notes

- This program **never** raises throttle above minimum and **never** asks the
  flight controller to arm. The only thing that arms the aircraft is the
  switch on the transmitter.
- Only one program at a time can use `/dev/ttyTHS1`. Close any other SSH
  window running `fc-info`, `fc-telem` or `camtilt`, or this will not start.
- Throttle is not used to work out who owns the sticks: a transmitter's throttle at
  rest reads about 988 and ours sends 1000, too close to tell apart. That
  throttle is covered at all is proven by the mask reading 15.

---

## Record it

```
DRONE d__        override test:  PASS / INCOMPLETE        date ______
  MSP OVERRIDE switch is channel ____   and is the ______ switch on the transmitter
  ARM switch is channel ____            and is the ______ switch on the transmitter
  ANGLE seen ON:  yes / no
```

Put tape on those two switches. They are the only two anyone touches in
flight.

---

## Where this sits

**Before:** `docs/NEW_DRONE_SETUP.md` - mask 15, ANGLE aux row, camera
calibration.

**After:** the autonomous hover in the cage -

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30 \
    --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

**Then:** the course - `docs/FLY_IT.md`.
