# Gate-centring bench check

Run on the bench, PROPS OFF (or on — nothing spins either way). No transmitter
needed. About 15 minutes with two people: one holding/tilting the aircraft,
one reading the screen and calling out numbers.

---

## The one idea behind this test

`hover.py`'s `--gate-z` and `--gate-roll` fly the aircraft by nulling two
numbers: **elevation** (how far above the aircraft the gate's centre sits)
and **azimuth** (how far left/right it sits). Both come from the camera's
pixel offset PLUS the aircraft's own attitude — the camera's mount tilt and
whatever the aircraft is doing (roll, pitch) have to be subtracted out before
"which way is the gate" means anything.

That attitude subtraction is exactly the kind of thing that can be wrong and
look fine — it only shows up once the aircraft actually tilts. (`tiltcheck.py`
exists for the same reason, for the accelerometer: a wrong sign read zero
error sitting still and real trouble the moment it moved.)

**This test asks: put a target at a position we've measured by hand, then
tilt the aircraft without moving the target. Does the reported
elevation/azimuth stay put, matching what the tape measure says it should
be — or does it drift?**

If it drifts with tilt, the compensation math is wrong, and that's worth
knowing on the bench, not in the cage.

---

## No gate needed

The detector just looks for a red/orange blob (see
`perception/detectors/hsv_classic.py`) and uses its centre — it doesn't need
a ring or a hole. Any solid piece of the right colour works: our test paper
is **HSV (6, 255, 255), 180mm × 120mm**, mounted **landscape** (180mm side
horizontal). If your team uses a different swatch, that's fine — just re-measure
its width for `--target-width` below.

---

## What you need

- The aircraft (props optional — this never spins them)
- The camera connected and working
- The Jetson, and ideally the flight controller connected too (`--port`) so
  real attitude is used — without it the test can only check the level case
- The red/orange test paper, mounted flat and rigid (taped to cardboard or a
  stand — a curled sheet distorts the shape)
- A tape measure or ruler
- For the off-centre step: something to check a right angle (a carpenter's
  square, a book's corner, or a 3-4-5 string triangle)
- A straight reference line (floor tape, or a table edge)

You do NOT need Betaflight Configurator, a battery in the aircraft, or a
transmitter.

---

## Setup

1. Tape a straight reference line on the floor or table.
2. Prop the aircraft up level, nose pointed exactly along the line. It needs
   to stay in this spot the whole test — you'll tilt it by hand later, not
   move it.
3. Get the camera's known numbers — `--fy` and `--cam-tilt` — from whoever
   last ran `hardware.camcal` on this aircraft (or re-run it if unsure).
4. **Place the target dead ahead first** (not off to the side yet):
   - Put the paper somewhere along the reference line, landscape orientation.
   - Since the camera is mounted `--cam-tilt` degrees above the body's nose,
     a target level with the lens shows up near the *bottom* of the frame,
     not the centre. To land it near the centre (and to match where a real
     gate sits relative to a level, hovering aircraft), raise the paper
     above lens height by roughly:

     ```
     height above lens ≈ distance × tan(cam_tilt)
     ```

     e.g. at 1.8 m out with a 20° mount, that's about 66 cm above the lens.
   - Measure the real result — don't just trust the formula:
     - **`--fwd`**: tape-measure distance from the lens to the paper's
       centre, along the reference line.
     - **`--dz`**: the paper centre's height minus the lens's height. Measure
       both up from the same flat surface (floor or table) and subtract.
     - **`--right`**: `0` for this first placement, by construction.
     - **`--target-width`**: the paper's horizontal width as mounted (`0.18`
       for the 180mm-landscape example).

---

## Run it

```
ssh d44
cd ~/AI-GrandPrix/src
python3 -m hardware.gatecheck --port /dev/ttyTHS1 --fy 824 --cam-tilt 20 \
    --fwd 1.8 --right 0.0 --dz 0.66 --target-width 0.18
```

Use your own measured `--fwd`/`--dz` and this aircraft's real `--fy`/`--cam-tilt`
— the numbers above are only the worked example.

It prints a header with the expected elevation/azimuth (computed from your
measurements), then a live line that updates in place:

| Column | Means |
|---|---|
| `roll` / `pitch` | the aircraft's actual attitude right now (needs `--port`) |
| `el` / `el_err` | measured elevation, and its error against the expected value |
| `az` / `az_err` | same, for azimuth |
| `rng` / `rng_err` | range readout (informational only — this codebase never trusts range for control) |
| `area%` | how much of the frame the target fills — a sanity check the camera sees it at all |
| verdict | `ok` / `SUSPECT` / `FAIL` for that instant |

---

## The three checks, in order

### 1. Level and centred
With the aircraft sitting still and level, `el_err`/`az_err` should both sit
near 0 (within a few degrees). **If this is already wrong, stop** — it means
`--fy`, `--cam-tilt`, or one of your tape measurements is off, not a sign bug.
Fix that before moving on.

### 2. Tilt it, target still doesn't move
Hold the aircraft still in its spot and rock it by hand: roll left, roll
right, pitch nose-up, pitch nose-down, holding each pose a couple of seconds.
**The target hasn't moved, so the true answer can't change.** Watch whether
`el_err`/`az_err` stay flat or grow with tilt.

- Stay flat → the attitude compensation is working.
- Grow with tilt → suspect a wrong sign or swapped axis in the mount-tilt
  rotation. The screen will point this out at the end.

### 3. Off to one side
Move the paper off the reference line. At its new spot, use your square to
find the point on the line closest to it, and measure:
- `--right`: the perpendicular distance from the line to the paper
- `--fwd`: distance along the line from the lens to that perpendicular point
  (not straight to the paper)
- `--dz`: same height-difference method as before

Re-run with the new numbers, repeat the level check and the tilt check. This
is the only step that actually exercises azimuth being nonzero — skipping it
means azimuth's sign was never really tested.

---

## Reading the final result

Ctrl+C whenever you're satisfied. It prints:

```
  worst elevation error: +1.8 deg (roll +0, pitch +25)
  worst azimuth error:   +0.9 deg (roll +30, pitch +0)
  range error (informational only, never used for control): median +0.15 m, worst +0.40 m over 143 samples

  PASS: both stay within 3.0 deg of the measured geometry, including while tilted
```

| Outcome | What it means |
|---|---|
| **PASS** | Elevation and azimuth stayed within tolerance, including while tilted. The centring math is trustworthy for this aircraft/camera pairing. |
| **FAIL, worst case at nonzero roll/pitch** | Error grows with tilt — suspect a sign or axis bug in the mount-tilt rotation. Do not fly `--gate-z`/`--gate-roll` until this is fixed. |
| **FAIL, worst case near level too** | A constant offset even level points at `--fy`/`--cam-tilt`/your measurements being off, not a sign bug — recheck the tape measure and the camcal numbers first. |
| **FAIL: too few detections** | The camera didn't see the target enough to judge anything. Fix lighting/framing/HSV thresholds before re-testing. |

---

## Record it

```
DRONE d__     gatecheck:  PASS / FAIL       date ______
  fy ____   cam-tilt ____ (from camcal, date ______)
  worst el error ____ deg     worst az error ____ deg
  tested dead-ahead: yes/no      tested off-centre: yes/no      tested tilted: yes/no
```

---

## Where this sits

**Before:** `hardware.camcal` (measures `--fy`/`--cam-tilt` in the first place).

**After:** `hover.py --gate-z` / `--gate-roll` in the cage — this test exists
so that flight is the second time this math runs, not the first.
