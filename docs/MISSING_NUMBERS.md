# Numbers we still need

Updated 2026-09-19.

## Still missing

| # | Number | Where | How long |
|---|---|---|---|
| 1 | override test passes | bench + transmitter | 10 min |
| 2 | `mass_kg` | scale | 2 min |
| 3 | `--cam-tilt` | wall + tape | 15 min |
| 4 | `--fy` | wall/checkerboard | same trip (`--cam-hfov` = **75, CONFIRMED 2026-09-20**) |
| 5 | heading drift | bench, one command | 1 min |

All five are bench work. **None need a flight session.**

## Already measured

| Number | Value | Source |
|---|---|---|
| `hover_pwm` | 1291 | blackbox, 4 flights agreed within 5 PWM |
| `curve_pwm` / `curve_acc` | measured | blackbox, residual 0.29 m/s^2 |
| `a_lat_rate_max` | 150 | blackbox, peak roll rate |
| `vz_up_max` | 1.25 | matches the human lap's 1.27 p95 |
| `--cam-hfov` | **75** | confirmed with the organizers 2026-09-20 |

---

## 1. Override test

Props off, battery in, transmitter on.

```
ssh d44
cd ~/AI-GrandPrix/src
python3 -m hardware.override_test --port /dev/ttyTHS1
```

1. Flip MSP OVERRIDE **on** -> screen says `MSP has the sticks`.
2. Flip it **off** -> screen says `PILOT has the sticks`. **This is the test.**
3. ARM on, then off -> `armed` goes YES then no.

Prints PASS or INCOMPLETE. INCOMPLETE means do not fly.

## 2. mass_kg

1. Drone on the scale. Battery, Orin, camera, props on.
2. Read kg.

Config says 0.9, a sim value. It sets braking distance in every plan.

## 3. --cam-tilt

1. Level the drone: `fc-telem`, shim until roll and pitch are within +-1.
2. Put it **1 m** from a wall, facing it. Measure lens-to-wall = **D**.
3. `~/target/live-view.py`, open `http://192.168.18.195:8080/`.
4. Mark where the **top edge** of the picture lands on the wall, and the
   **bottom edge**. Midpoint of the two = the optical centre.
5. Measure centre height = **H**, lens height = **L**.
6. Repeat at 2 m. The two answers must agree within 2 degrees.

```
cam_tilt = atan( (H - L) / D )
```

## 4. --fy and --cam-hfov

Same setup, do not move anything.

7. Mark the **left edge** and the **right edge** of the picture on the wall.
8. Measure between them = **W**.

```
hfov = 2 * atan( (W/2) / D )
fy   = 640 / tan(hfov/2)
```

Cross-check against the real gate: drone 5-8 m back, clamped, three runs.

```
python3 -m hardware.camcal --dist 6.0 --port /dev/ttyTHS1
```

Never closer than 5 m. At 2.3 m we got fy 952 against 776 at 6 m.

## 5. Heading drift

```
python3 -m hardware.bench --port /dev/ttyTHS1 drift --seconds 60
```

Dead still for the full minute. Over 1 deg/min, fly only the slowest rung.

---

## Send me

```
mass_kg   = ?
D, H, L   = ?    at 1 m, and again at 2 m
W         = ?
drift     = ? deg/min
override  = PASS / INCOMPLETE
```

I put them in the configs and rebuild the plans.

---

## Two configs, do not mix them

- `config/vehicle_cam35_75.toml` and `config/ladder/vehicle_k*_cam35_75.toml`
  - **the Archer.** Measured thrust curve.
- `*_sim.toml` - **sim flights only.** The SITL plant's curve.

The sim's plant has its own physics. Give the follower the real aircraft's
curve in the sim and it mis-commands.
