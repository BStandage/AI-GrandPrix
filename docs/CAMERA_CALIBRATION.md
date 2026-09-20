# Camera calibration, step by step

Three numbers come out of this, and every camera fix in flight depends on them:

| Number | Flag | What it does |
|---|---|---|
| focal length in pixels | `--fy` | how far away a gate is |
| horizontal field of view | `--cam-hfov` | turns a pixel position into a direction |
| camera mount tilt | `--cam-tilt` | which way the camera points relative to the drone |

Get `--cam-tilt` wrong by 5 degrees and every gate is misplaced by 0.7 m at 8 m.
That is the one to be careful with.

Two ways to do this. **Method A (wall)** needs a tape measure and a wall.
**Method B (gate)** needs the real gate. Do A first, then check with B.

---

## Before either method: get the drone level

1. SSH in:

```
ssh d44
```

2. Watch the attitude:

```
fc-telem
```

You get a table updating a few times a second. Look at the `roll` and `pitch`
columns.

3. Shim the drone with folded paper or tape until **roll** reads between -1 and
   +1. Roll matters more than pitch for this - a rolled drone tilts the whole
   image sideways and ruins the horizontal measurement.

4. Ctrl+C to stop.

If roll and pitch never sit still, the FC's own idea of level may be off.
Fix that once, on a surface you have checked with a spirit level (a phone level
app on the frame is fine):

5. Betaflight -> **Setup** tab -> **Calibrate Accelerometer**. Do not move the
   drone while it runs.
6. Re-run `fc-telem`. It should read about 0 / 0.

---

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

## Where we got to (2026-09-19)

- `--fy` roughly 720-780 at 6 m. The 952 from 2.3 m is distortion, discard it.
- `--cam-hfov` roughly 80 measured; organizers say 75-85.
- `--cam-tilt` never measured - the FC attitude was not reading during camcal.
- Detector found the real gate in 100 % of frames at 6 m, width jitter +-3 %.
- It also found a false gate in 100 % of frames with no gate in the room, so
  something red in there passes the thresholds.
