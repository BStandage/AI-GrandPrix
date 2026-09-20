# Camera calibration, step by step

**Do Method 0. The rest of this file is fallback and cross-checks.**

Five numbers come out of this, and every camera fix in flight depends on them:

| Number | Flag | What it does |
|---|---|---|
| focal length in pixels | `--fy` | how far away a gate is |
| horizontal field of view | `--cam-hfov` | turns a pixel position into a direction |
| principal point | `AIGP_CAM_CX/CY` | where the optical axis really is, which is NOT the image centre |
| camera mount tilt | `--cam-tilt` | which way the camera points relative to the airframe |

Get `--cam-tilt` wrong by 5 degrees and every gate is misplaced by 0.7 m at
8 m. That is the one to be careful with.

**None of these transfer between aircraft, or survive a camera swap.** They
describe one lens in one mount. d45 broke its camera on 2026-09-20 and every
number below had to be measured again.

---

## Before anything: is the airframe level, and are its signs right?

The tilt is measured against the airframe, so a body that is pitched or rolled
goes straight into the answer.

```
ssh d45
cd ~/AI-GrandPrix/src
python3 -m hardware.tiltcheck --port /dev/ttyTHS1
```

Tilt it 30 degrees each way, holding each pose still. It must say **PASS**.

This is not optional politeness. Betaflight on this firmware reports pitch
positive NOSE DOWN, we assumed the opposite for months, and the error is
exactly zero when the aircraft is flat - which is how every calibration is
done. d45 measured -5.24 m/s^2 at 31 degrees on 2026-09-20. If a new airframe
reads the other way, `--pitch-nose-up-positive` flips it back.

Then get it physically level: shim with folded paper or tape until roll and
pitch read within a degree of zero.

```
python3 ~/target/msp/msp_bench.py --port /dev/ttyTHS1 telemetry
```

If they never sit still, calibrate the accelerometer once in Betaflight
(**Setup -> Calibrate Accelerometer**, on a surface checked with a level) and
re-check.

---

## Method 0: the checkerboard - THIS ONE

Twenty minutes, and the only method that gives the principal point and the
distortion. This is what d45 was calibrated with on 2026-09-20: 20 views,
RMS 0.165 px.

### 0a. Print the board

`out/caltarget/checkerboard_letter_25mm.png`, at **100 percent** - no "fit to
page". Tape it FLAT to stiff card. A curled board quietly ruins the answer.

**Measure one square with a ruler and use the real number.** Printers scale by
a percent or two, and every length here is proportional to it.

### 0b. Capture

```
python3 -m hardware.camcal_board grab
```

It shows nothing and needs no display. Aim for **20 views**.

**Move, then STOP, then let it capture.** It will not save a frame until the
board has held still for two consecutive frames and the board area is sharp -
it tells you which one it is waiting on. This is not fussiness: the sensor has
a rolling shutter, so it reads the image one row at a time and anything moving
comes out SKEWED rather than merely blurred. A skewed board fits no camera
model at all, which is how d45 produced a 5.9 px RMS on 2026-09-20 while the
drone was being walked around a fixed board.

**Moving the drone instead of the board is fine** - only the relative pose
matters - as long as you stop before each capture.

**Tilt in BOTH axes, and do not keep yaw constant.** This is the one that
decides whether the solve works:

| what you do | what it foreshortens | what it measures |
|---|---|---|
| yaw the drone / angle the board left-right | horizontally | **fx** |
| pitch the drone / angle the board top-bottom | vertically | **fy** |
| spin the board in its own plane | nothing - still face-on | little |

Oblique in one axis only leaves the other focal length free to drift, and an
`fx`/`fy` split is precisely that happening. d45 got 642 / 1210 on 2026-09-20.

**The rule of thumb: if the board looks like a rectangle in frame, that view
is worthless.** You want a trapezoid - near edge clearly bigger than the far
edge.

Make the views genuinely different:

- near (0.4 m) and far (1.5 m)
- board tilted left, right, up, down - 20 to 40 degrees, not flat on
- board in each corner of the frame as well as the middle

Twenty copies of the same view calibrates nothing. The variety is the
measurement.

### 0c. Solve

```
python3 -m hardware.camcal_board solve --square-mm 25.0
```

Use YOUR measured square size. It prints `fy`, `hfov`, the principal point and
the distortion, and an RMS reprojection error.

**RMS under about 0.5 px is good.** d45 got 0.165. Above 1.0, the captures
were too alike or the board was not flat - capture more variety and re-solve.

### 0d. The mount tilt

Now that `fy` and `cy` are known, the tilt comes from one target at a known
height:

```
python3 -m hardware.camtilt --dist 1.78 --lens-h 0.94 --target-h 1.25     --fy 830 --cy 387
```

- `--dist` lens to target, horizontally along the floor, in metres
- `--lens-h` lens height off the floor
- `--target-h` height of the target centre off the floor
- `--fy`, `--cy` from step 0c

All in metres, all heights from the same floor. Keep the aircraft level.

Do it at **two different distances** and the answers should agree within about
a degree. d45 read 19.4 at 70 inches and 20.6 at 40 inches, so 20.

### 0e. Write them down

Put them straight into that aircraft's flight card. The principal point is the
one people forget: leaving it at the image centre carried a constant 1.8 degree
bearing bias on d45, about 0.26 m of lateral error at 8 m, always the same
way - which is exactly the kind of error that looks like bad tuning forever.

---

## Fallbacks and cross-checks

Everything below predates Method 0. Use it to sanity-check a number, or if the
checkerboard is unavailable. **Method A cannot give you the principal point or
the distortion**, and on d45 its frame-edge marks implied a 37.6 degree
vertical field of view where the checkerboard measured 46.9.

## Method A: the wall

Nothing to print, nothing to detect. You mark where the edges of the picture
land on a wall and measure with a tape.

### A1. Set it up

1. Point the drone at a wall, roughly square on, **2 to 3 m** back. Any wall.
   Textured is easier than blank, because you can see exactly where the edge
   falls.

2. The camera points up about 35 degrees, so with the drone level the picture
   lands high on the wall. **Prop the nose down** until the middle of the
   picture sits at a comfortable height. That is fine - the pitch is measured
   and subtracted later.

3. Check roll is still near 0, and note the pitch:

```
fc-telem
```

Write the pitch down. Ctrl+C.

4. Find out which way pitch is signed on your board: push the nose down further
   and watch the number. **Write down whether it goes up or down.** Nothing
   later works without this.

### A2. Measure D

5. Measure from the **lens** to the wall, straight out, perpendicular. Call it
   **D**.

6. Check you are square: measure lens-to-wall at the left edge mark and at the
   right edge mark (step 8). If those two differ by more than a few cm, the
   drone is angled - straighten it and start again.

### A3. Mark the edges

7. Start the camera:

```
~/target/live-view.py
```

Open `http://192.168.18.195:8080/` on a laptop on the same WiFi. (`.195` is
d44; d45 is `.14`.)

8. One person watches the stream, another slides a finger along the wall. Stop
   when the finger is exactly at the **left edge** of the picture. Mark the
   wall with tape. Repeat for the **right edge**.

9. Measure between the two marks. Call it **W**.

10. Mark where the **middle** of the picture lands: halfway between your two
    marks, level with the centre of the image. Measure its height off the
    floor. Call it **H**.

11. Measure the **lens** height off the floor. Call it **L**.

### A4. What you now have

D, W, H, L, and the pitch. That is everything:

```
hfov     = 2 * atan( (W/2) / D )
fy       = (1280/2) / tan(hfov/2)
cam_tilt = atan( (H - L) / D )  +  (how far the nose was pitched down)
```

Hand those five numbers over and the arithmetic gets done for you.

**Why this beats pointing at a gate:** the baseline is the full frame width
instead of a blob a quarter that size, so the same tape-measure error matters
about four times less.

---

## Method B: the real gate

This is `camcal`, and it is the one that also proves the detector sees the real
gate's red.

1. Put the drone **5 to 8 m** from the gate. Not closer: at 2-3 m the gate
   fills the frame, lens distortion is worst there, and the answer comes out
   wrong. We measured `fy` 952 at 2.3 m against 776 at 6 m - the near one is
   the bad one.

2. Square on, looking through the opening, gate upright.

3. Measure the horizontal distance along the floor, lens to ring centre. Call
   it **dist**.

4. Measure **dz** = ring centre height minus lens height. Negative if the ring
   centre is lower than the lens.

5. Clamp or weight the drone. **Do not move it between runs.** Two of our runs
   at a nominal 6 m disagreed by 7 % because the drone shifted.

6. Run it, on the drone, from `~/AI-GrandPrix/src`:

```
python3 -m hardware.camcal --dist 6.0 --dz 0.4 --port /dev/ttyTHS1
```

Use your real numbers. Leave `--ring-m` off: the gate's outer width, 2.70 m, is
already the default.

7. Run it **three times without touching anything**. The three `fy` values
   should agree within about 2 %.

8. Move to 3 m and 10 m, re-measuring `dist` each time. `fy` should come out the
   same. That is the real check that the whole thing is consistent.

### Reading the output

```
frames 299, detections 299, capture 1280x720
frame width 349.0 px (spread 330..352), centre (598, 561) px
fy    = 775.6 px            -> --fy 776
hfov  = 79.1 deg            -> --cam-hfov 79
range = 6.00 m at --fy 776 (must equal --dist 6.0)
tilt  = ... deg             -> --cam-tilt ...
```

- **detections close to frames** - it sees the gate in every frame. Good.
- **spread** - the width jitter. 330..352 is about +-3 %, which is the target.
  A spread like 159..1120 means the detection is unstable; look at the picture.
- **range must equal dist** - it is computed from your own inputs, so it is a
  self-check, not a result.

### If it says no ring detected

The HSV thresholds do not match the red in front of it. Look at what the camera
actually sees with `~/target/live-view.py` first. Do not adjust numbers blind.

### If the FC attitude is missing

```
no attitude from the FC; tilt will assume the body is level
```

Only one program can hold `/dev/ttyTHS1` at a time. Close other SSH sessions
using it, check `fc-info` works, then run camcal again. With that message
showing, `--cam-tilt` is only right if the drone really is level.

---

## Method C: a protractor on the hinge

Worth it as a cross-check, not as the answer.

1. Level the drone (`fc-telem`, roll and pitch about 0).
2. Hold the protractor against a flat face of the camera that is square to the
   lens.
3. Read the angle above horizontal.

If your gauge reads **90 for straight forward**, then 35 degrees up reads
**55**.

**The catch:** the camera's case is not necessarily square to its optical axis,
usually within a couple of degrees. So a protractor tells you the hinge is
roughly right; it does not tell you what the lens is actually doing. Use it to
set the hinge, use Method A or B for the number you fly with.

A digital angle gauge is better than a protractor for this, and a phone
inclinometer app is about as good as either.

---

## What to do with the numbers

They go into every flight command:

```
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 35 --fy 776 --cam-hfov 79 \
  --pilot follower --config <rung toml> --traj <rung plan> --dry-run
```

And into the planner, so the plan knows what the camera can see:

```
python -m raceline.ladder --k 0.33 --config ../config/vehicle_cam35_75.toml \
  --cam-tilt 35 --cam-hfov 79
```

One thing not to do: **do not deliberately understate the field of view to be
safe.** It is used in two places. In the planner a smaller number is
conservative, but in the estimator it makes every gate appear less off-axis
than it really is, which corrupts every fix. Give the estimator the true
number, and get your safety margin from a lower k.

---

## What we have measured

**d45, 2026-09-20, checkerboard, 20 views, RMS 0.165 px** - and then the
camera broke and all of it became history:

| | |
|---|---|
| `--fy` | 830 |
| `--cam-hfov` | 75 (74.9 measured; organizers say 75) |
| `AIGP_CAM_CX/CY` | 613.1 / 387.0 in a 1280x720 frame - 27 px off centre in both axes |
| `--cam-tilt` | 20 (19.4 at 70 in, 20.6 at 40 in) |

Earlier, by the gate method: `fy` 720-780 at 6 m, and 952 at 2.3 m. The near
one is distortion - do not calibrate at 2-3 m.

**Open problem, not a calibration one.** The detector finds a gate in about
99.6 percent of frames *including with the lens covered*, and its range
estimate read 2.4, then 9.8, then 140 m within ten seconds (d45, 2026-09-20).
Something in the room passes the HSV thresholds. That needs recorded footage
and `perception.video_probe`, not more tape-measure work.
