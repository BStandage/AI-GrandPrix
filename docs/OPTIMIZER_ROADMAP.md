# Line optimizer roadmap: escaping local minima

UPDATE (2026-09-15): the course is now the organizers' PUBLISHED map and
the labels below are the OLD estimate-map ones - read "g7 switchback" as
today's g5 hairpin and "the stacked gate" as g8; 24 crossings is now 23
events. Item 1 exists in practice as `raceline.line_search` (multi-seed,
per-crossing knobs, calibrated replay as the guard) and produced every
plan_RACE since 2026-09-09; `line_opt --free` needs scipy, which the
project venv does not carry. The 29.55 s published-map plan is a
relabelled seed of the 28.30 s estimate-map search, so a fresh search on
the new geometry is the cheapest second still on the table.

UPDATE (2026-08-29): the precondition below is met - the Betaflight tune
sprint landed and the slew ceiling is measured (48.8 m/s^3 at race
amplitude, was 5.3). The search space also grew past this doc's status
quo: `--free` now optimizes LEARNED CROSSING POSES per event (lateral
offset, heading within pose_angle_max_deg of the gate normal, standoff
scales) alongside the free points - diagonal crossings and short/long
run-ins are discovered by search, not hand rules. Items 2 and 3 below
are unchanged and still the path; the basins they hunt are now worth
several seconds instead of ~1.

Status quo (2026-08-28): `raceline/line_opt.py --free` is a LOCAL search -
Powell/coordinate descent from a straight-line initial guess. It provably
lands in local minima: at the g7 switchback we measured two distinct
basins (through-the-opening-backward-and-return at 82.75 s vs
beyond-the-gate-and-around at 83.83 s), and the optimizer only found the
first because its starting guess rolled downhill into that basin. With 24
crossings and discrete topology choices at each (which side, through vs
around, over vs under at the stacked gate), the landscape has hundreds of
basins and we explore exactly one.

Three escalation steps, in order of effort. Timing note: do these AFTER
the Betaflight tune sprint raises the slew ceiling - today's basins
differ by ~1 s while slew holds ~40 s; at higher agility, line
differences amplify and basin-hunting pays several times more.

## 1. Multi-start (hours of work)

Run the existing local optimizer from N randomized free-point seeds, keep
the best validated result. Embarrassingly parallel, ~3 min per start.
Expected: 1-3 s on today's envelope, more later. No new math; a loop and
a seed generator around `optimize_free`.

## 2. Junction topology enumeration (a day)

The discrete choices only matter at a few junctions: the g7 switchback,
the stacked-gate turnaround, the lap transition. Enumerate the 2-3
candidate topologies at each (about a dozen combinations total), polish
each with the local optimizer, price them all with the same cost
function. This is a principled global search over exactly the choices
that move time - it is what we did by hand for g7, automated. Candidate
generators are geometric rules (through / around-left / around-right),
never per-gate constants.

## 3. Full optimal control (a week)

Drop the spline parameterization entirely: solve the drone's control
inputs directly for minimum time through the gate sequence (direct
collocation / CPC-style), with measured limits (slew, tilt, thrust curve)
as constraints and gate openings plus frame clearance as path
constraints. No basins by construction of the path family; wider turns,
diagonal acceleration, and split-S shapes all emerge from the solver.
The plan JSON contract stays identical - the follower never knows which
optimizer produced its trajectory.

Prerequisites for 3 to be worth it: measured slew after the BF tune (the
constraint set must be honest), and the frame-clearance checker (already
built, `planner.frame_violations`) as the constraint oracle.

## Risk policy, orthogonal to all three

The cost function prices TIME only. Race-day doctrine may prefer slower,
lower-variance lines (example: the g7 through-the-gate trick saves ~0.5
s/lap but triples passes through the tightest 0.60 m corridor). Encode
such policies as global cost terms (e.g. price wrong-direction opening
transits), never by editing a specific gate's line.

Measured note (2026-08-28): a 6-start random multi-start on the 2-lap
problem was WON by the unperturbed seed; all perturbed starts found worse
basins. Blind restarts do not pay here - the basins are topology choices,
so item 2 is the productive next step.
