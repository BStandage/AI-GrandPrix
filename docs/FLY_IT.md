# Fly it: drone on the start line to wheels up

Exact steps. Follow them in order. Every step says what "it worked" looks like.

Two machines: your **laptop** builds the plan, the **drone** flies it. `d44` and
`d45` are the SSH shortcuts; password `dcl`.

Nothing here has been flown yet. Treat every number as unproven.

---

## A. Once per drone, before race day

### A1. Get our code onto the drone

The Jetson ships with `~/target` (the organizers' tools). Our code is separate.

From your laptop:

```
scp -r ~/GitRepos/AI-GrandPrix dcl@<drone ip>:~/
```

Check it landed:

```
ssh d44 "ls ~/AI-GrandPrix/src"
```

Worked: you see `hardware`, `raceline`, `perception`, `seeker`, `solvers`.

### A2. Check the Python it needs

```
ssh d44 "python3 -c 'import numpy, cv2, serial; print(cv2.__version__)'"
```

Worked: it prints a version, no traceback.

If `cv2` fails: use the system python3. Do **not** `pip install opencv-python` —
JetPack's build has GStreamer and the pip wheel does not.

### A3. Measure the camera

Needs a real gate and a tape measure. On the drone, from `~/AI-GrandPrix/src`:

```
python3 -m hardware.camcal --dist 6.0 --dz <ring centre height minus lens height> --port /dev/ttyTHS1
```

Worked: it prints `--fy`, `--cam-hfov` and `--cam-tilt`.

Repeat at 3 m and 10 m. The range it prints must match the tape at all three.
**Write the three numbers down — every later command needs them.**

No detection at all means the HSV thresholds are wrong for this lighting; fix
`perception/detectors/hsv_classic.py` before going further.

---

## B. Build the plan for the k you want

On your **laptop**, from `AI-GrandPrix/src`. Use the `--cam-tilt` and
`--cam-hfov` that `camcal` measured:

```
python -m raceline.ladder --k 0.5 --config ../config/vehicle_cam35_120.toml --cam-tilt 35 --cam-hfov 120
```

Worked: it prints the camera tilt cap and the model time, and writes two files:

```
config/ladder/vehicle_k050_cam35_120.toml     <- the rung's limits
out/plans/plan_LADDER_k050_cam35_120.json     <- the plan
```

The filename carries the k and the camera: `k050` = k 0.5, `cam35_120` = 35 deg
mount, 120 deg lens. Change `--k` and you get a different pair.

k is one number from 0 (timid) to 1 (the config's race limits). Lower k, slower
and safer. **Fly the lowest k first.**

Copy both files to the drone:

```
scp ../config/ladder/vehicle_k050_cam35_120.toml dcl@<drone ip>:~/AI-GrandPrix/config/ladder/
scp ../out/plans/plan_LADDER_k050_cam35_120.json dcl@<drone ip>:~/AI-GrandPrix/out/plans/
```

---

## C. On the start line

### C1. Place the drone

Put it on the start line **pointing along gate 1**. `--map-north here` reads the
flight controller's heading the moment the runtime starts and calls that
direction map north. Point it wrong and the whole map is rotated.

### C2. Dry run first, props OFF

Props off. The FC stays disarmed and receives neutral sticks; nothing spins.

On the drone, from `~/AI-GrandPrix/src`:

```
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 35 --fy 1000 --cam-hfov 120 \
  --pilot follower \
  --config ../config/ladder/vehicle_k050_cam35_120.toml \
  --traj ../out/plans/plan_LADDER_k050_cam35_120.json \
  --dry-run
```

Replace `--cam-tilt`, `--fy` and `--cam-hfov` with your camcal numbers.

Worked, all four:

1. It prints the accelerometer scale it measured at rest.
2. It prints `map north = FC heading now: <deg>`.
3. The log shows the estimate, detections and the sticks it *would* send.
4. Carry the drone toward a gate: the fix residual **shrinks**.

If the residual does not shrink, the camera numbers or the map heading are
wrong. Stop here. Do not arm.

### C3. The armed run

Props on. Everyone clear. Pilot holding the transmitter.

Same command, `--dry-run` becomes `--arm`:

```
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here \
  --cam-tilt 35 --fy 1000 --cam-hfov 120 \
  --pilot follower \
  --config ../config/ladder/vehicle_k050_cam35_120.toml \
  --traj ../out/plans/plan_LADDER_k050_cam35_120.json \
  --arm
```

It prints `LIVE: waiting for the pilot` and then does nothing at all until the
pilot acts. The plan clock starts only when the FC reports both switches.

The pilot, in this order:

1. Throttle fully down.
2. **ARM** on (channel 5 high).
3. **MSP OVERRIDE** on (channel 9 high).

ANGLE is always on — it is hard-coded on both drones, not switched.

Worked: it prints `armed, MSP OVERRIDE on: flying` and the drone takes off.

### C4. Stopping it

Any of these ends the run:

- Pilot flips **MSP OVERRIDE off** — the sticks come back to the pilot and the
  runtime stops.
- Pilot **disarms**.
- `--max-s` expires (240 s default).
- The FC link goes stale.

**Abort is the human pilot's switch.** The runtime cannot arm, cannot disarm, and
cannot hold the override on.

---

## D. After the flight

```
python -m raceline.debrief ../out/flightlogs/hw_follower_<stamp>.csv
```

Draws where the drone believed it was over the plan, and logs the per-gate
numbers. After three flights, `--aggregate` says whether anything is off the
same way every time. See `docs/DEBRIEF_FIRST_STEPS.md`.

---

## The ladder, on the day

1. Fly the lowest k that was clean in the sim.
2. Clean twice, jump to the fastest k you brought.
3. It fails, fly the midpoint of the last clean and the last failed k.
4. A complete slow run beats an incomplete fast one.

Fly the rungs you built and tested. Do not interpolate a new k at the track: in
the sim k 0.9 failed 3 of 3 while 0.8 and 1.0 both flew clean.

---

## Before any of this counts

The MSP override bench test (`src/PQ_PROCEDURE.md`, day 0 step 4) has **not been
run on either drone**. Until the human pilot has taken the sticks back with MSP still
sending, and disarmed with MSP still sending, nothing above should be armed.
