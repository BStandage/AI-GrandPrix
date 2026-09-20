# d45 flight card

Everything measured on d45, and the exact commands with its numbers already
in them. Copy-paste these; do not retype the numbers.

**Another aircraft needs its own card.** None of these values transfer.

---

## d45's numbers

| | value | measured |
|---|---|---|
| `--fy` | **830** | checkerboard, 20 views, RMS 0.165 px |
| `--cam-hfov` | **75** | 74.9 measured, confirms the organizers |
| `--cam-tilt` | **20** | 19.4 deg at 70 in, 20.6 at 40 in |
| `AIGP_CAM_CX` | **613.1** | optical axis, NOT the image centre |
| `AIGP_CAM_CY` | **387.0** | same |
| `--heading-drift-dpm` | **3.0** | on the floor, warmed up |
| override test | **PASS** | 2026-09-20, all three parts |
| hover dry run | **PASS** | 2026-09-20, 1350 = 1.32 g |
| `hover_pwm` 1291 | **CONFIRMED** | table test, first clean sample 1294 (+3 PWM) |
| barometer resolution | **0.076 m** | 1 Pa steps; accurate to 2 cm, quantised to 8 |
| FC vario | **DEAD** | reads 0.00 always; we compute vertical speed ourselves |
| mask | **15** | verified after reboot |
| ANGLE | always on | aux row 5 |
| all-up weight | **not weighed** | 1.59 kg bare, 1.7 assumed |

Leaving `AIGP_CAM_CX/CY` off carries a constant 1.8 deg bearing bias into
every camera fix - about 0.26 m of lateral error at 8 m, always the same way.

---

## 1. Sync, from the laptop

```
cd ~/GitRepos/AI-GrandPrix
scripts/sync_drone.sh d45 cam20_75
```

Do this after ANY change on the laptop. It is the single most common way to
fly stale code.

## 2. Pre-flight, props off, no transmitter needed

```
ssh d45
cd ~/AI-GrandPrix/src
```

**Flight controller alive:**
```
python3 ~/target/msp/msp_bench.py --port /dev/ttyTHS1 info
```
Sensors list GYRO, ACC, BARO. `Arming blocked by:` will say RX_FAILSAFE with
no transmitter - expected.

**Modes decode correctly** (this was broken until 2026-09-20):
```
python3 -c "
from hardware.bridge import FcBridge
br = FcBridge.open(port='/dev/ttyTHS1'); st = br.fc.status()
print('active:', st.active_modes, '| override:', st.msp_override, '| angle:', st.angle_mode)
br.stop()"
```
Must list ANGLE. Must NOT print an empty list.

**Camera:**
```
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=RG10 \
  --stream-mmap --stream-count=100 --stream-to=/dev/null
```

**The whole stack, dry, with the real camera numbers:**
```
AIGP_CAM_CX=613.1 AIGP_CAM_CY=387.0 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --dry-run
```
Prints the takeoff thrust in g and logs the throttle it would send. Nothing
arms, nothing spins.

It will sit in `climb` at `thr=1350` with `z=0.00` for the whole run and end
with "no steady hover samples". That is the pass. The drone never leaves the
floor, so it never leaves takeoff throttle - `rc_backend.AltitudeLoop.throttle`
holds it until z > 0.25 m or vz > 0.7 m/s. What you are checking is the header
line: altitude zeroed without an error, and the takeoff thrust printing about
1.3 g and NOT raising the hard-launch warning. The jitter on `z` is the
barometer's resolution - see below.

**Better version of this test, no transmitter, no props (verified 2026-09-20):**
put the drone on a table instead of leaving it on the floor, and set `--alt` to
the table height. It exercises the whole phase machine on real barometer
motion and it reads back the hover throttle.

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.76 --seconds 20   --config ../config/ladder/vehicle_k025_cam20_75.toml --dry-run
```

Drone on the FLOOR, start it, wait for `altitude zeroed`, THEN lift it onto a
0.76 m table and leave it. Put it back on the floor at `hold done, descending`.

Read the **first** `hold` sample, not the `HOVER THROTTLE` median at the end.
Sitting on a table the aircraft cannot climb to close the error, so the
integrator winds up for the whole hold and drags the median high - d45 printed
1315 against a true 1294. The first sample is taken before any windup.

d45, 2026-09-20: first hold sample `thr=1294`, `a_cmd=+0.14` against
`hover_pwm = 1291`. Three PWM. The measured thrust curve and its inversion are
correct.

The barometer reports in 1 Pa steps, so `z` only ever takes values about
0.076 m apart: floor read -0.08/0.00, the 0.762 m table read 0.66/0.75.
Midpoint to midpoint that is 0.74 m - accurate to 2 cm, but never finer than
8 cm. Do not chase altitude errors smaller than that.

## 3. The residual check - do this BEFORE you have a transmitter

The one test that proves camera -> estimator end to end, and it needs no
props and no transmitter. Put a real gate in front of the drone.

```
AIGP_CAM_CX=613.1 AIGP_CAM_CY=387.0 \
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 20 --fy 830 --cam-hfov 75 --heading-drift-dpm 3.0 \
  --pilot follower \
  --config ../config/ladder/vehicle_k025_cam20_75.toml \
  --traj ../out/plans/plan_LADDER_k025_cam20_75.json \
  --dry-run
```

Worked, all of:

1. prints the accelerometer scale measured at rest
2. prints `map north = FC heading now: <deg>`
3. logs detections when the gate is in view
4. **carry the drone toward the gate and the fix residual SHRINKS**

If the residual does not shrink, the camera numbers or the map heading are
wrong and no amount of flying will fix it. Stop and tell me the numbers.

---

## 4. With a transmitter: the override test

Props off. Full procedure in `docs/OVERRIDE_TEST.md`.

```
python3 -m hardware.override_test --port /dev/ttyTHS1
```

Must print PASS. Already passed once on d45 (2026-09-20); re-run after any
Betaflight change.

## 5. With a transmitter and props: the cage hover

```
AIGP_CAM_CX=613.1 AIGP_CAM_CY=387.0 \
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30 \
  --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Pilot: throttle down, ARM on, MSP OVERRIDE on. Hold the throttle stick near
hover position the whole time, so taking over is a handoff and not a drop.

It prints the hover throttle it settled at. Compare with 1291 from the
organizers' blackbox: within 40 PWM and the thrust model is confirmed.

## 6. On the start line

Same as step 3 with `--dry-run`, then `--arm`. Point the drone along gate 1
before starting - `--map-north here` reads the heading at startup and gets the
whole map from it.

Start with **k 0.10** (67 s model, 21.5 deg tilt), not k 0.25.

---

## After every flight

```
python -m raceline.debrief ../out/flightlogs/hw_follower_<stamp>.csv
```

Three flights in, `--aggregate` tells you whether anything is off the same way
every time. `docs/DEBRIEF_FIRST_STEPS.md`.

---

## Known-open on d45

- All-up weight not measured. 1.7 kg assumed from 1.59 bare. Sets every
  braking point in the plan.
- Betaflight reports no vertical speed on this aircraft. `MSP_ALTITUDE`'s
  vario field reads exactly 0.00 m/s forever, confirmed by hand over a dozen
  0.76 m lifts while `alt` tracked every one. Nothing depends on it any more -
  `FcStateSource` runs its own accel+baro observer - but if you bench another
  aircraft, check `fc_vario_alive` before assuming its vario works.
- With the transmitter OFF, MSP OVERRIDE reads ON (ch9 failsafe is high).
  Worth deliberately testing what Betaflight failsafe does in that state,
  props off, before trusting it in the air.
