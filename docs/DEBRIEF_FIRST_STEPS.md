# Debrief: first steps on the real drone

What to do with `raceline.debrief` the first time the Archer flies. The tool
reads no ground truth, so everything here works on the real airframe with no
extra equipment.

Nothing below has been done yet. There are zero hardware flights, so every
number the tool has produced so far is synthetic.

## 0. Before the drone leaves the bench

Confirm the log carries the debrief columns. Props off, FC disarmed:

```
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here --cam-tilt <deg> \
  --fy <px> --cam-hfov <deg> --pilot follower \
  --config ../config/ladder/vehicle_k050_cam35_120.toml \
  --traj ../config/ladder/plans/plan_LADDER_k050_cam35_120.json --dry-run
```

`head -1 out/flightlogs/hw_*.csv` must show `cross_ev, cross_lat, cross_dz,
fix_cx_sum, fix_al_sum`. No columns, no debrief.

## 1. The cheapest calibration check there is

Still on the ground, after `camcal`. Carry the drone slowly at a real gate
from about 8 m, as in PQ_PROCEDURE day 0 step 8, then:

```
python -m raceline.debrief ../out/flightlogs/hw_follower_<stamp>.csv
```

Read the bottom-right panel. The camera and dead reckoning are looking at the
same gate from the same place, so **both innovations should sit near zero**. If
`along` is already 0.2 m with the drone barely moving, the range scale is wrong
before anything has flown: re-run `camcal`, check `--fy`, and check the gate
width the range divides by. Fix it here, not in the air.

There are no crossings on this log. That is expected.

## 2. First flights

Fly the slowest rung, `k 0.5` for the 35/120 camera. Debrief after every
flight:

```
python -m raceline.debrief            # the newest log
```

Three flights minimum before reading anything across runs. Two runs cannot
establish a bias and the tool will refuse to call one.

## 3. Read the aggregate

```
python -m raceline.debrief --aggregate
```

Take the findings in this order:

1. **Whole-course innovation.** One sign at every gate is calibration, not
   drift. `along` is the range scale (`--fy`, gate width). `cross` is the
   boresight or the yaw reference (`--cam-tilt`, `--map-north`). Fix this
   first: it is one number and it moves every other number on the page.
2. **PROVEN rows.** It believed it was outside an opening it flew through.
   That is position error measured with no truth, and it is the largest
   honest number you have. Look at which leg fed it.
3. **CONSISTENT rows.** Same offset at the same gate every run. The line is
   planned through the gate centre, so suspect in this order: the estimate,
   the map, the line.

A blind leg has no innovation bars at all, because no fix arrives on it. Drift
through the hairpin and the stacked gate only becomes visible in the believed
offset at the gate that ends the leg. That bar is the blind turn's report card.

## 4. Changing something

One change per flight, and re-fly before the next one. Two changes at once and
the next aggregate cannot attribute either.

Do not carry a nudge over from the sim. A bias measured in the sim is a bias of
the sim's synthetic camera; the per-gate pose offsets in `planner.py` were
searched that way, did not transfer, and are switched off today.

## What this cannot tell you

- Where the drone actually was. There is no truth on the Archer. The crossing
  offset is a *belief* against a known bound, not a measurement of position.
- Anything about a leg the drone did not finish.
- Anything from one run. Three is the floor, and the floor is not a lot.
