# Walk-through test — prove the gate crossing without flying it

**Props off. Nothing arms. No transmitter.** You carry the drone through a real
gate and the flight code tells you whether it would have flown through or hit
the bar.

This exists because of **d44 flight 4, 2026-09-21**: Sally tracked gate 1
cleanly from 7.4 m to 2.5 m, then climbed a metre over the last 1.8 m and
clipped the top bar. Nothing about that failure needs a motor to reproduce. It
is a camera, a gate, and a range.

---

## What actually went wrong, in one picture

The camera points **up**. A gate at your own height therefore sits **below** the
optical axis, and its ring runs off the **bottom** of the frame. The visible
centroid then sits above the ring's true centre, the vertical loop reads that as
*"the gate is above me"*, and it climbs — which cuts off more of the gate, which
makes it climb harder. A runaway.

The fix: the ring is **square** (2.7 × 2.7 m) and its **width is not clipped**,
so the true centre is rebuilt from the width. Exact for a square gate, not a
fudge factor. When even that is impossible, the aircraft **commits**: it holds
the height the gate gave it while it could still see all of it, and flies
through.

---

## The three commit triggers

Not "the gate leaves the frame" — it never leaves, it **overfills**.

| trigger | range | kind |
|---|---|---|
| `both_edges` | ~3.2 m | geometry. A 2.7 m ring exceeds the 45.5° vertical field. No unclipped edge left to rebuild from. Not negotiable — it is the lens. |
| `width_clipped` | ~1.8 m | the ring is wider than the frame, so the width can't rebuild anything either |
| `size` ≥ 0.85 frame | **~3.8 m** | policy, `AIGP_GATE_COMMIT_FRAC`. Deliberately earlier than the geometric floor — a correction begun at 1.8 m has nowhere to go but into a bar. |
| `lost` | any | no detection. The reference **fades** at 0.35 m/s, it does not step. |

A **singly**-clipped ring is not a commit — that gets reconstructed.

---

## Before you walk

1. **Camera at 10°.** Sally's wall marks, lens at 34-3/8": **41-7/16"** at 40",
   **46-11/16"** at 70".
2. Confirm it. Board taped at 50" (1.270 m) — **move the drone, not the board**:

```
python3 -m hardware.camtilt --fy 859 --cy 330.0 --dist 1.016 --lens-h 0.8731 --target-h 1.270
python3 -m hardware.camtilt --fy 859 --cy 330.0 --dist 1.778 --lens-h 0.8731 --target-h 1.270
```

Both must read ~10° and agree within 2°. If they don't, the drone moved or it
isn't level. Fix that first.

3. `scripts/sync_drone.sh d44`
4. Props off. Battery in — we need FC attitude.
5. Mark a spot **8 m straight out** from the gate, dead centre.

---

## The walk

```
ssh d44
cd ~/AI-GrandPrix/src

python3 -m hardware.walkthrough --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 859 --cx 616.9 --cy 330.0 --gate-h 1.35
```

Hold the lens **as close to gate centre height as is comfortable** — exact is
not required, see below — **level**, nose on the gate. Walk straight in from the
8 m mark, through the gate, at about the speed it flies. `Ctrl+C`.

Two people: one carries, one reads the screen.

### You do not need to hold 1.35 exactly

The verdict measures **drift, not absolute value.**

Carry it 0.1 m low and the camera *should* report a small positive elevation and
a small positive `dz`. That is the system working. An absolute threshold would
fail correct code because someone's arms were four inches off.

Flight 4 did not fail by holding a constant offset. It failed by the offset
**growing** as the range closed — `dz` +0.09 at 8.2 m, pinned at its +0.35 cap
by 9.4 s. A clipped ring reads *more* wrong the closer you get. That is a
runaway, and a runaway shows up whatever height you started at.

**Hold it steady — any steady height — and the number is honest.**

---

## What you get

| file | |
|---|---|
| `walk_<stamp>.csv` | one row per frame. **Row N is frame N of the video** — the loop is synchronous, they cannot drift apart |
| `walk_<stamp>.avi` | raw MJPG capture, to replay offline as many times as you like |
| `walk_<stamp>/` | lossless PNGs either side of every commit transition |

Live on screen, and in the CSV:

- `el_deg` — the gate's elevation, the number the vertical channel flies on
- `implied_h_m` — **the height the camera thinks you are at.** Check it against
  a tape. This is the flight-4 smoking gun.
- `implied_lat_m` — how far right of the gate centreline it thinks it is
- `dz_vis_m` — the height it is being *commanded* to move
- `commit` / `commit_why` — whether the reference is live, and which trigger fired
- `throttle_pwm`, `a_cmd`, `thrust_cmd` — the real `AltitudeLoop` output
- `roll_deg` / `pitch_deg` — how level you actually held it

---

## The verdict

Printed at the end:

```
==================================================================
APPROACH VERDICT - flight 4 was a RUNAWAY, so we measure DRIFT
==================================================================
  commit fired at        3.79 m   (size)
  dz commanded far out   -0.018 m
  dz commanded at commit -0.010 m
  height it believed     1.39 m far out -> 1.37 m at commit

  DRIFT +0.008 m   <- the climb this approach would have flown
  flight 4 flew +1.05 m and hit the top bar. Gate half-opening is 0.75 m.

  PASS: the commanded height held within 0.15 m all the way to
  commit, and froze there. She flies through.
==================================================================
```

**PASS** = drift under 0.15 m and `dz` never reached its ±0.35 cap.

### Validated against flight 4

Replayed through the real detector at both mounts:

| mount | unclip | commit | drift | height it *believed* | cap hit | verdict |
|---|---|---|---|---|---|---|
| **20°** | **OFF** ← flight 4 | 3.79 m | +0.113 | **0.88 → 0.83 m** | **yes** | **FAIL** |
| 20° | ON | 3.79 m | −0.007 | 1.28 → 1.32 m | no | PASS |
| 10° | ON | 3.79 m | +0.008 | 1.39 → 1.37 m | no | PASS |

Row 1 is the bug: the camera believed it was at **0.88 m when the truth was
1.35 m** — thought it was half a metre low, and climbed. The test catches it.

10° with the fix off also passes, because at 10° the ring barely clips before
commit fires. The mount and the fix cover each other.

---

## Also worth doing in the same run

Each catches a different lie:

| | do | pass |
|---|---|---|
| 1 | the walk in, above | drift under 0.15 m |
| 2 | stop at ~5 m, raise and lower it 0.5 m | `implied_h_m` follows, right direction, right amount |
| 3 | step 1 m left, then 1 m right | `implied_lat_m` reads ≈ −1.0 then ≈ +1.0 |
| 4 | **hold still and tilt it** — roll and pitch | `implied_h_m` must **not** move |

Step 2 is the important one. A detector stuck at zero passes the walk-in
perfectly and fails here — which is the point of doing it.

Step 4 checks the attitude compensation. `gate_dz` is attitude-invariant to
0.00° in simulation under roll, pitch and yaw; step 4 checks that on the real
aircraft with the real FC. If `implied_h_m` drifts when you tilt it, `gate_dz`
has a sign or axis wrong and nothing downstream will save you.

---

## Offline: A/B the fix

```
python -m perception.video_probe walk_XXX.avi --fy 859 --csv on.csv
AIGP_GATE_UNCLIP=0 python -m perception.video_probe walk_XXX.avi --fy 859 --csv off.csv
```

Diff `off_y`. The annotated video draws:

- **magenta line** — the row the height is actually steered on
- **dashed yellow** — the raw centroid it would have used without the rebuild.
  The gap between them *is* the false climb.
- **grey line** — the optical axis (not the middle of the picture; on d44 those
  are 30 px apart)
- **COMMIT banner** — which trigger fired, and the ring size as a % of frame

---

## Race-day narration

`hardware.runtime` now writes a `.log` beside the CSV saying what the aircraft
**believed**, in sentences, so you can put it next to what you saw. It is event
driven — a line appears when something changes, not 50 a second.

Flight 4, replayed through it:

```
   0.2  gate 1 IN SIGHT at 7.7 m, LEVEL with me
   1.2  AIRBORNE at 0.89 m
   6.4  gate 1 at 5.0 m: LEVEL, holding 0.97 m
   7.4  gate 1 at 3.6 m: 0.17 m BELOW me -> EASING DOWN, holding 1.02 m
   8.2  gate 1 is 0.09 m ABOVE me -> CLIMBING (at 3.1 m)
   8.6  gate 1 at 2.9 m: 0.14 m ABOVE me -> CLIMBING, holding 0.94 m
   9.6  gate 1 at 1.9 m: 0.29 m ABOVE me -> CLIMBING, holding 1.31 m
  12.0  gate 1 LOST - fading the height reference out, holding vertical speed
  12.2  THROTTLE 1450 - near the 1380+ ceiling, the loop is asking for a lot of lift
```

Three lines from 8.2 s and the whole failure is visible. With the fix in,
**COMMIT fires at 3.8 m — before the 8.2 s line ever happens.**

---

## Do not fly if

- drift exceeds 0.15 m, or `dz` hits its ±0.35 cap
- `implied_h_m` disagrees with a tape by more than ~0.15 m
- `implied_h_m` moves when you tilt the aircraft
- the two `camtilt` distances disagree by more than 2°
