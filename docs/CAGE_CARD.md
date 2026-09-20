# Cage card - d45

Two tests, one battery. Plain hover first, vision second. Read the whole card
before you plug anything in.

**Abort, any time, any test: MSP OVERRIDE off.** You have the sticks instantly
and the program stops itself. Hold your throttle stick at hover position the
whole flight so taking over is a handoff, not a drop.

---

## 0. Before you walk over - laptop, 1 min

```
cd ~/GitRepos/AI-GrandPrix
scripts/sync_drone.sh d45 cam20_75
```

Nothing below exists on the drone until this runs.

Take a **fresh battery**. The board reset three times yesterday at 21.87 V.
Stop flying below 22.0 V.

---

## 1. Bench check, PROPS OFF, 1 min - do not skip

This is the only thing standing between a sign error and the net. Point the
nose at the gate, hold the aircraft still, run:

```
ssh d45
cd ~/AI-GrandPrix/src
AIGP_CAM_CX=613.1 AIGP_CAM_CY=387.0 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.76 --seconds 20 \
  --gate-z --fy 830 --cam-tilt 20 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --dry-run
```

Wait for `gate acquired`, then move the aircraft by hand and watch `GATE`:

| you do | `el` should | `dz` should |
|---|---|---|
| hold it level, gate straight ahead | near 0 | near 0 |
| **lower** the aircraft toward the floor | go **positive** | go **positive** (climb) |
| **raise** it above the gate centre | go **negative** | go **negative** (descend) |
| cover the lens | - | `gate: none`, target returns to 0.76 |

**`dz` positive means "the gate centre is above me, climb".** If it moves the
wrong way, STOP and do test 2 only. Do not fly the vision test.

Also note the hit rate it prints at the end - `N/M frames had a gate`. Below
about half and the gate is hard to see from where you are standing.

---

## 2. TEST ONE - plain hover on the barometer

Props on. Cage clear. This one proves the thrust model on the real 1.751 kg.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.76 --seconds 20 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Human steps:

1. Run the command. It prints `LIVE: waiting for the pilot`.
2. Throttle stick **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** It lifts off.
5. Hands on the sticks, throttle at hover, do nothing else.
6. It climbs, holds 0.76 m for 20 s, descends, disarms itself.

Good looks like: a gentle lift (1350 PWM = 1.32 g, not a leap),
`at altitude (0.7x m) ... holding`, `thr=` near 1290, and **it lands instead
of bouncing at knee height**.

At the end it prints `HOVER THROTTLE = NNNN PWM`. **Write that number down.**
Within 40 of 1291 and the whole plan's braking is trustworthy.

It holds altitude but **not position** - it will drift with any air movement.
That is what ends this flight, not altitude.

---

## 3. TEST TWO - autonomous altitude from the gate

Same as above plus `--gate-z`. **Point the nose at the gate before you arm.**
The camera is fixed and tilted up 20 deg, so yaw is your only aim.

```
AIGP_CAM_CX=613.1 AIGP_CAM_CY=387.0 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.76 --seconds 25 \
  --gate-z --fy 830 --cam-tilt 20 --gate-z-min 0.40 --gate-z-max 1.40 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Same six human steps as test one.

**What the aircraft is allowed to do:** change its throttle. That is all.
Roll, pitch and yaw are centred exactly as in test one, so there is no axis
that can carry it toward the gate. Takeoff and landing are always flown on the
barometer; vision only has the aircraft during `hold`.

**What to expect.** It climbs to 0.76 m on the barometer, then prints
`gate acquired`, and the target starts moving toward the gate's centre height,
at most 0.40 m/s, clamped between 0.40 and 1.40 m. If it loses the gate it
prints `gate lost` and goes back to 0.76 m.

The live line gains a section:

```
t= 12.0 hold     z= 0.98 (target 1.02) vz=+0.11 thr=1301 a_cmd=+0.44 | GATE el= +1.2 rng= 3.4 dz=+0.07
```

- `el` - degrees the gate centre sits above the aircraft. **This is the number
  the test is about. It should trend toward zero.**
- `rng` - metres to the gate
- `dz` - how far to climb to be level with the gate centre

**Success is `el` settling near 0 and staying there.** That means the aircraft
found the gate's height by eye and held it.

---

## 3b. TEST THREE - add roll centring (only if test two was clean)

Adds `--gate-roll`: the aircraft also slides sideways until the gate is dead
ahead. Now it holds height AND lateral position by eye.

```
AIGP_CAM_CX=613.1 AIGP_CAM_CY=387.0 python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.76 --seconds 25   --gate-z --gate-roll --fy 830 --cam-tilt 20   --gate-z-min 0.40 --gate-z-max 1.40   --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

**Aim the nose at the gate before arming.** The aircraft slides sideways by
about `range x tan(how far off your aim is)` - 4 m and 15 deg out is roughly
1 m of travel. Aim well and it barely moves.

The live line gains `az` (degrees the gate is to the right) and `roll` (the
angle commanded). **Success is `az` trending to zero and `roll` settling near
zero.** Roll is capped at 4 deg - the stick never leaves 1475..1525.

Still no pitch: it does not control distance, so it can still drift slowly
toward or away from the gate. That is the one axis left, and it is next.

Compared with test one this drifts LESS, not more - a plain hover has no
horizontal control at all.

---

## 4. Landing

| when | do |
|---|---|
| normal | nothing - it lands and disarms itself after `--seconds` |
| early, gently | **MSP OVERRIDE off**, fly it down on the sticks |
| emergency | **Ctrl+C** - motors off immediately. From 0.76 m that is a drop |

---

## 5. Before you leave the cage

- [ ] `HOVER THROTTLE` from test one, written down
- [ ] both `.csv` files in `~/AI-GrandPrix/out/flightlogs/`
- [ ] **tape-measure the gate centre height off the floor** - one number, and
      it is the only way to check what the camera claimed against truth
- [ ] battery voltage at the end

---

## If something is wrong

| symptom | it means |
|---|---|
| `gate: none (0/N)` throughout | the detector never saw it. Yaw at the gate, check lighting |
| `el` drifts away from zero instead of toward it | sign or calibration. Land, fly test one only |
| target pinned at 1.40 or 0.40 | the clamp is holding it. The range estimate is off |
| it slides sideways and keeps going | your nose was well off the gate. Land, re-aim, retry |
| `roll` pinned at +/-4.0 | azimuth is large - the gate is near the frame edge |
| `camera did not open` | another process has it, or the pipeline is wrong |
| it bounces at knee height on landing | the airborne latch did not take - you are on stale code, re-sync |
