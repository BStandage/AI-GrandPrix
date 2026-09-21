# Flight variable glossary

Every symbol that appears on screen or in a log, in the order you meet them.

**One warning first: `az` means two different things.** In the `GATE` section
of `hover.py` it is **azimuth**, the gate's bearing left or right. In the
`[RL]` line of `hardware.runtime` it is **vertical acceleration**. Same two
letters, unrelated quantities. Read which line you are looking at.

---

## The two you asked about

| | |
|---|---|
| **`el`** | **ELEVATION.** How far above the aircraft the gate's centre sits, in DEGREES. `+` means the gate is higher than you, so climb. `0` means you are level with it. This is the number `--gate-z` drives to zero, and it is the whole reason the vision hold works without a barometer. |
| **`az`** | **AZIMUTH.** How far to the right the gate's centre sits, in DEGREES. `+` right, `-` left, `0` dead ahead. This is what `--gate-roll` drives to zero. |

Both are measured from the aircraft, with the camera's 20 degree up-tilt and
the aircraft's own attitude already taken out. They describe where the gate is
relative to the drone, not where it is in the picture.

---

## `hover.py` - the live line

```
t= 12.0 hold  z= 1.31 (target 1.34) vz=+0.04 thr=1297 a_cmd=+0.18 | GATE el= +0.8 rng= 3.4 az= -1.1 roll=-0.3 hold= 4.4 pitch=-0.1
```

| symbol | unit | meaning |
|---|---|---|
| `t` | s | seconds since liftoff |
| `phase` | | `climb`, `hold` or `descend` |
| `z` | m | height above where it was zeroed. Accel+baro fused, not the raw sensor |
| `target` | m | the height it is trying to hold. Vision moves this when `--gate-z` is on |
| `vz` | m/s | vertical speed. `+` climbing. **Computed by us** - the flight controller reports 0.00 forever |
| `thr` | PWM | throttle being sent. 1000 is minimum, 2000 maximum, **1228 is hover** |
| `a_cmd` | m/s^2 | vertical acceleration being asked for, gravity excluded. `0` = hold, `+` = climb harder |

The `GATE` section only appears with `--gate-z`:

| symbol | unit | meaning |
|---|---|---|
| `el` | deg | elevation - see above. **The test.** |
| `rng` | m | camera's distance to the gate, from its apparent size. **Our worst signal** - it read 2.4, then 9.8, then 140 m in ten seconds on a bench |
| `az` | deg | azimuth - see above |
| `roll` | deg | roll angle being commanded. Capped at 4 |
| `hold` | m | the distance `--gate-pitch` is holding |
| `pitch` | deg | pitch angle being commanded. **Negative only** - it may back away, never advance |
| `dz` | m | how far above you the gate centre is, in metres. `rng * sin(el)`. Display only - the controller never uses it, because it would put the bad range back in |

---

## `hover.py` - the log file columns

Same as above, plus:

| column | meaning |
|---|---|
| `z_target` | the `(target ...)` from the live line |
| `thrust_cmd` | specific thrust asked for, m/s^2. Hover is 9.81 |
| `throttle`, `roll`, `pitch`, `yaw` | the STICK values sent, 1000-2000. `roll`/`pitch` here are commands, usually 1500 |
| `roll_deg`, `pitch_deg` | the aircraft's ACTUAL attitude from the FC. Different thing from the two above - this is what it did, those are what it was told |
| `yaw_deg` | heading in the map frame |
| `armed` | 1 or 0 |
| `vbat` | battery volts. **Stop below 22.0** |
| `amps` | current. Hover is ~17 A. Randy drew 35-55 climbing on the wrong curve |
| `gate_seen` | 1 if the camera had a gate that tick |
| `gate_el_deg`, `gate_rng`, `gate_dz` | the GATE values above |

---

## `hardware.runtime` - the course

```
t=  1.0 ev0/lm0 p=( +0.2, +1.9,-0.17) yaw= 89 det= 4.38m fixes=2 res= 0.42 rej=0 unm=25 stk=(1500,1500,1350,1500) arm=1800 link 33Hz/11ms healthy=True
```

| symbol | unit | meaning |
|---|---|---|
| `ev0` | | the gate it is flying to next. `ev0` is gate 1 |
| `lm0` | | which map gate the camera matched this frame |
| `p=(x,y,z)` | m | where it believes it is, in the map frame |
| `yaw` | deg | heading, counter-clockwise from the map's +x |
| `det` | m | camera's range to the gate it can see |
| `fixes` | count | camera position fixes accepted so far. Climbing = the camera is working |
| `res` | m | **fix residual** - how far the camera's idea of position sits from the estimate. **Under 1 m is healthy.** The bench check got 1-6 cm |
| `rej` | count | fixes rejected as implausible - a jump of more than 4 m |
| `unm` | count | detections that matched no gate. Climbing is normal when the geometry does not match the map |
| `stk` | PWM | the four sticks: roll, pitch, throttle, yaw |
| `arm` | PWM | 1800 armed, 1000 disarmed |
| `link` | | flight controller update rate and round-trip time |

And the `[RL]` follower line:

| symbol | unit | meaning |
|---|---|---|
| `s` | m | distance travelled along the planned line |
| `v` | m/s | speed |
| `xtrack` | m | **cross-track error** - how far off the planned line it is, sideways |
| `zt` | m | the height the plan wants right now |
| `tilt` | deg | how far it is leaning |
| `az` | m/s^2 | **vertical acceleration** here, NOT azimuth |
| `air` | | whether the follower thinks it is airborne |
| `DR err` | m | dead-reckoning error, sim only - there is no truth on hardware |

---

## Throttle and thrust

| term | meaning |
|---|---|
| **PWM** | the number sent to the flight controller for each stick, 1000 to 2000. Throttle 1000 is idle, 2000 is full |
| **`hover_pwm`** | throttle at which thrust exactly cancels gravity. **1228**, measured in flight on two aircraft |
| **`takeoff_pwm`** | throttle held during the first moment of liftoff. **1289**, which is 1.32 g |
| **`curve_pwm` / `curve_acc`** | the thrust curve: a table of throttle against the acceleration it produces. Measured in flight, 2026-09-21 |
| **specific thrust** | thrust per kilogram, in m/s^2. Hover is 9.81. The whole point: it is **per kilogram**, so a curve from a heavier aircraft is wrong on ours |
| **g** | 9.80665 m/s^2. "1.32 g" means thrust is 1.32 times the aircraft's weight |
| **TWR** | thrust to weight ratio |

---

## Camera

| term | meaning |
|---|---|
| **`fy`** | focal length in pixels. Converts pixel offsets to angles, and apparent size to range. **824** on Randy |
| **`cx`, `cy`** | **principal point** - where the lens's optical axis actually lands on the sensor, in pixels. NOT the image centre. Randy: 627.2, 368.8 against a centre of 640, 360 |
| **`hfov` / `vfov`** | horizontal and vertical field of view, degrees. 76 and 47 |
| **`cam-tilt`** | how far the camera points UP relative to the airframe. **20 deg**, measured on both |
| **RMS** | reprojection error from a calibration, in pixels. **Under 0.5 is good.** Randy got 0.281 |
| **boresight** | a constant bearing bias from a wrong principal point. Never looks random, so it reads as bad tuning |

---

## Modes and control

| term | meaning |
|---|---|
| **MSP** | the protocol the Jetson uses to talk to the flight controller |
| **MSP OVERRIDE** | the switch that decides whether the sticks come from the TRANSMITTER or the JETSON. **Flipping it off is the abort** |
| **ANGLE** | flight mode where a stick position means "lean this far". Our software assumes it |
| **ACRO** | flight mode where a stick position means "rotate at this rate". Our sticks mean something else entirely in ACRO |
| **mask 15** | `msp_override_channels_mask`. 15 covers the four sticks and no switch, so the Jetson can never hold the override switch on from its own side |
| **DR** | **dead reckoning** - working out position by integrating the accelerometer. Drifts, which is why the camera exists |
| **ZUPT** | zero-velocity update. Holding velocity at zero while the aircraft is known to be on the ground, so accelerometer bias cannot integrate into phantom motion |
| **`k`** | ladder rung, 0 to 1. Scales tilt, speed and margins between a safe floor and race values. **k 0.10 is the cautious one** |

---

## Safety numbers

| | |
|---|---|
| `--ceiling` | height above which it cuts throttle and disarms. Reads the RAW barometer and trusts no filter |
| `--vz-max` | vertical speed that aborts the flight, `--no-baro` only |
| `--gate-z-min/max` | hard clamp on where vision may put the height target |
| `--gate-pitch-fwd` | cap on tilt TOWARD the gate. **Zero by default** |
| 12 m/s | barometer readings implying more than this are thrown away |
| 12 m/s^2 | most climb any altitude error may buy, whatever it believes |
