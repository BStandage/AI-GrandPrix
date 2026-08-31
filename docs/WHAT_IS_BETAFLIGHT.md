# What is a Betaflight controller? (drone + hardware primer)

Background reading for devs who write code for this project but have never
touched a drone. No prior RC/drone knowledge assumed. For how to actually
run a race, see `docs/GETTING_STARTED_RACING_LINE.md`.

**The 30-second version:** a quadcopter is four motors and zero natural
stability - it would flip over in milliseconds without a computer
constantly correcting it. That computer is the **flight controller (FC)**,
and **Betaflight** is the open-source firmware it runs (the de-facto
standard for racing drones). Betaflight's job is small and fast: read the
gyro thousands of times per second and adjust motor speeds so the drone
rotates exactly as fast as the "pilot" asks. Everything smarter than that -
holding a position, following a trajectory, seeing gates - is NOT
Betaflight's job. It's ours. Our code is the pilot: it sends Betaflight the
same four stick values a human's radio would send.

---

## 1. How a quadcopter actually moves

Four fixed-pitch propellers, two spinning clockwise and two
counter-clockwise, arranged in an X. Every motion is a combination of
spinning some motors faster than others:

| Motion | How | In practice |
|---|---|---|
| Climb / descend | all four faster / slower (collective thrust) | throttle |
| Roll / pitch (tilt) | speed up motors on one side | tilts the thrust vector |
| Yaw (spin in place) | speed up the CW pair vs the CCW pair (torque imbalance) | rotates the nose |
| **Translate** | **tilt first, then thrust pulls you sideways** | there is no "strafe motor" |

The last row is the one software people trip on: **a quad moves
horizontally only by tilting**. Want to accelerate east at 3 m/s^2? Tilt
east by `atan(3/9.81) ~ 17 deg` and the tilted thrust vector does it. That's
why `max_tilt_deg` in `vehicle.toml` IS the lateral-acceleration budget
(`a_lat = g*tan(tilt)`), and why yaw is aerodynamically almost useless -
yawing points the nose (and the camera, which is why we still control it)
but moves the drone nowhere.

Consequence for control: tilt is also how you STOP. Braking = tilting
backwards. The planner's friction-circle math exists because one tilt
budget is shared between cornering and accelerating/braking.

## 2. What the flight controller does (and doesn't)

The FC is a small board with a microcontroller and an IMU (gyroscope +
accelerometer), plus usually a barometer, sitting between "intent" and
motors:

```
   sticks (4 numbers)        gyro (1000s of Hz)
          |                        |
          v                        v
   +-------------------------------------+
   |  Betaflight: PID rate loop + mixer  |   "make the measured spin rate
   +-------------------------------------+    match the commanded spin rate"
          |
          v
   4 motor speed commands
```

Betaflight is a RACING firmware. Out of the box it does **not** know where
it is, does not hold position, has no GPS waypoints, no "return home"
worth using. It is a very fast, very good **rate controller**: you say
"rotate at X rad/s about each axis, with thrust Y", it makes that happen.
Deliberately dumb, extremely reliable - which is exactly why it's the
right bottom layer, in the sim and (per the September spec) on the real
drone.

Everything above rate control is our stack:

| Layer | Who | Where |
|---|---|---|
| Race line + speed profile | us (planner) | `src/raceline/planner.py` |
| Trajectory tracking (carrot -> desired accel) | us (follower Tracker) | `src/raceline/follower.py` |
| Attitude loop (desired accel -> tilt -> stick values) | us (follower RC backend) | `src/raceline/follower.py` |
| Rate loop + motor mixing | **Betaflight** | `betaflight_SITL.elf` (sim) / the FC board (real) |
| Physics | elodin (sim) / reality | - |

## 3. The interface: RC channels

Betaflight's input is what a hobby radio sends: a list of channels, each a
PWM-style value in **microseconds, 1000-2000, center 1500**. Channel order
is AETR (Aileron, Elevator, Throttle, Rudder) - see `solver/api.py` in the
sim repo:

| Ch | Name | Meaning | Notes (measured on this build) |
|---|---|---|---|
| 0 | Roll | roll rate command | +stick tilts toward -y at yaw 0 |
| 1 | Pitch | pitch rate command | +stick = nose down / accelerate +x at yaw 0 |
| 2 | Throttle | collective thrust | hover ~ **1240** (measured); takeoff 1330 |
| 3 | Yaw | yaw rate command | +stick = yaw RIGHT (world yaw decreases), ~0.31 rad/s per 60 PWM |
| 4 | AUX1 | **arm switch** | >=1700 = armed |
| 5+ | AUX2.. | mode switches | unused by us (see gotchas) |

This RC-values contract is not just a sim convenience - the organizers
confirmed (FAQ, 2026-08-28) that the real race interface is exactly this:
the Jetson sends RC control commands to the FC over UART and receives IMU
data back. Nothing else. No attitude, no position - what you practice
against in the sim is what you get in September.

Two protocol facts that bite newcomers:

- **Arming**: motors will not spin until AUX1 goes high *while throttle is
  low*. That's why every solver starts with a scripted sequence: ~0.5 s
  disarmed -> arm at idle throttle -> fly. Disarming (AUX1 low) is also how
  we "land": kill the motors near the ground.
- **Sticks are rates, not angles** (see modes, next).

## 4. Flight modes: rate (acro) vs angle - and why we fly rate

- **Angle mode**: stick position = tilt angle; let go and it self-levels.
  Requires the FC to *estimate its attitude* from accel+gyro.
- **Rate / acro mode**: stick position = rotation RATE. Nothing
  self-levels; whoever commands the sticks owns the attitude. This is what
  racing pilots use, and what we use.

We fly **rate mode** and close the attitude loop ourselves (a
thrust-vector P controller: compare current body z-axis to desired,
command the rate that rotates one into the other). Two reasons:

1. It matches racing reality - full authority, no built-in leveling
   fighting the trajectory.
2. Measured necessity in this sim: Betaflight's own attitude estimate in
   SITL was untrustworthy (enabling angle mode produced a diverged
   estimate and the quad never lifted). Our loop uses the sim's
   ground-truth quaternion today; the estimator milestone swaps that
   source, not the loop.

## 5. Betaflight in OUR sim (what's actually running)

When you run `race.py`, WSL is executing a real, unmodified-firmware-logic
Betaflight compiled for PC: **SITL** (software-in-the-loop),
`betaflight/obj/main/betaflight_SITL.elf`. The elodin sim and Betaflight
run in **lockstep**: every physics tick, the sim sends a sensor packet
(gyro/accel/baro in Betaflight's expected FRD frame) plus our RC packet,
and gets back four motor commands that drive the physics. Same firmware
behavior as a real FC, deterministic timing.

Configuration lives in `eeprom.bin`, generated by
`scripts/configure_betaflight.py` (sim repo). Hard-won facts baked in
there and in the bridge - don't rediscover these:

- **Softened rate PIDs** (p/i/d 30/55/20, yaw 30/55; measured 2026-08-29):
  stock PIDs (45/80/30) churn the motors so hard on this torque-rich
  airframe that average output floors near hover and throttle-down cannot
  descend (flown: altitude pinned at 1.9 m with throttle commanded 100
  below hover). The measured attitude-slew ceiling barely cares (25.5 at
  30/55/20 vs 27.8 at stock); descent authority matters more. The slew
  numbers in the tomls were measured on THIS tune - re-run
  solvers.sysid_slew after any PID change.
- **Airmode OFF**: with the wobble above, airmode's mixing turned
  oscillation into un-throttleable lift.
- **Sensor-frame signs are subtle** (FLU->FRD conversions in
  `sim/sensors.py`): two upstream sign bugs were found and fixed there,
  and yaw gyro keeps the empirically-stable sign on this build. If you
  ever see a permanent ~1.5 rad/s wobble or "level but thinks it's
  inverted" - you're in this territory; read `sim/sensors.py` history
  before touching anything.

## 6. What a solver gets to see

Every physics tick the sim hands the solver a `SensorUpdate`
(`solver/api.py`): ground-truth pose/velocity (sim-only luxury), IMU
(gyro/accel), baro, mag - each with freshness flags - plus the latest FPV
camera frame (640x360, matching the VADR-TS-002 camera spec) and the race
context (next expected gate event). The follower currently uses
ground-truth state through its `StateSource` layer; the whole point of
that layer is that the physical drone won't have ground truth - an
estimator (IMU dead-reckoning + vision fixes) will implement the same
interface.

## 7. The September hardware (physical qualifier)

From the PQ spec (VADR-TS-004, 2026-08-18) plus the organizer FAQ answers
(2026-08-28) - this is what the sim is a rehearsal for:

| Item | Spec / FAQ |
|---|---|
| Runs | 2-lap autonomous; only the final 2 days score |
| Track | 60 x 21 m footprint; 3 slots/day + a training cage; manual piloting allowed |
| Gates | 2.70 m outer frame, 1.5 m opening; double gate = ONE gate flown through twice; depth 260 vs 140 mm under review |
| Airframe | 8" Archer Block 2, 8x4.1 props (motors/ESCs/battery/weight TBD); 4 drones/team |
| Flight controller | **Betaflight <= 2026.6.1** - full config access (rates/PIDs/filters), no reflashing; extract with CLI `diff all` |
| FC <-> Jetson | **UART: RC commands down, IMU data back - that's the entire interface** (no attitude, no pose). Exactly the sim's contract. |
| Companion computer | NVIDIA Orin NX 16 GB, 25 W mode, root access (our code runs here) |
| Camera | rolling-shutter Arducam, 1920x1080 @ 60 fps, exposure/gain controllable, Orin-side CLOCK_MONOTONIC timestamps; **intrinsics NOT provided** (we calibrate on-site); IMU pose in the airframe IS provided |
| Humans | human-in-flight = disqualification |

Open questions are tracked in `src/PQ_SPECS_INTAKE.md` (angle limit /
ACRO permission, UART protocol/rates and failsafe behavior, training-cage
details, gate depth) - treat anything not in the table above as
unconfirmed.

What transfers from sim to real: the whole architecture (plan -> carrot ->
attitude loop -> RC sticks -> Betaflight) and the toml-driven tuning
discipline. What does NOT transfer: ground-truth state (estimator
required), the exact plant numbers in `[vehicle]`/`[thrust]` (day-1 sysid
replaces them - the toml is designed for exactly that swap), and sim
determinism (one survey flight of data, then it has to work -
RESTRICTIONS.md).

## Glossary

| Term | Meaning |
|---|---|
| FC | flight controller (board + firmware) |
| Betaflight | open-source racing-drone FC firmware |
| SITL | software-in-the-loop: the firmware compiled to run on a PC against a simulator |
| IMU | inertial measurement unit: gyro (rotation rate) + accelerometer |
| Acro / rate mode | sticks command rotation rates; no self-leveling |
| Angle mode | sticks command tilt angles; FC self-levels (we don't use it) |
| Airmode | keeps stabilization authority at zero throttle (disabled here) |
| Arming | AUX1 high + throttle low -> motors allowed to spin |
| PWM 1000-2000 | the value range of every RC channel; 1500 = centered stick |
| Mixer | Betaflight's map from (rates, thrust) to 4 motor outputs |
| FRD / FLU / ENU | axis conventions (Forward-Right-Down etc.) - the source of every sign bug ever |
| Hover PWM | throttle value where climb rate = 0 (measured 1240 here) |
| SITL lockstep | sim and firmware advance one tick at a time, in sync |
