"""Offline racing-line planner: course map + vehicle.toml -> timed trajectory.

Pipeline (all GLOBAL parameters, nothing per-gate):
  1. ANCHORS - per crossing k with center c and required-direction normal n:
     (c - d_pre*n, c, c + d_post*n). Standoffs follow the two-value rule:
     the base standoff normally, the larger turn standoff on BOTH sides of a
     junction whose consecutive crossing headings differ by more than
     turn_angle_deg. The g10 out-and-back (headings ~180 deg apart) falls out
     of this rule - the planner never names a gate.
  2. PATH - centripetal Catmull-Rom through the anchors (interpolating, no
     overshoot loops on uneven spacing), resampled to uniform arc length.
  3. SPEED PROFILE - pointwise ceiling
        v_lim = min( v_max,
                     sqrt(a_lat / kappa),            a_lat = margin*g*tan(tilt)
                     yaw_rate_max / |dpsi/ds|,       nose-follows-tangent
                     vz_up / slope, vz_down / |slope|,
                     v_gate within gate_window of ANY crossing center )
     then a forward pass (accel) and backward pass (brake), both on the
     friction circle a_long = a_budget * sqrt(1 - (v^2*kappa/a_lat)^2), so
     braking before corners and accelerating out fall out of the math.
  4. TIMESTAMPS + feedforward accel by differentiating v * tangent.

Output: a Plan (arrays + provenance) and a JSON file with a stable contract
so a better optimizer can replace this module without touching the follower.

All predicted times are MODEL PREDICTIONS, unverified (RESTRICTIONS.md).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from raceline import course as course_bridge
from raceline.config import G, VehicleConfig, load_config, AIGP_REPO

PLAN_VERSION = 1
PARK_ALT_M = 0.8
# Share of the brake budget the run-out is allowed to use. The race is already
# scored past the last gate, so there is no reason to stop at 100% authority -
# and the profile did exactly that once the last crossing sped up, leaving no
# margin at all for the real vehicle. Braking with less than full authority
# back-propagates into a slower, calmer final crossing.
RUNOUT_BRAKE_FRAC = 0.55
_DENSE_DS = 0.05      # spline pre-sampling resolution (m)
_DEDUP_M = 0.15       # drop pre/post anchors this close to their neighbor
# Largest share of a leg the two standoff anchors sitting on it may consume
# between them (see standoffs()). This is what keeps a turn WIDE AND GRADUAL
# rather than a pivot: the anchors bend the spline, and once they eat too
# much of the leg the spline has to double back to reach them in order.
# Swept on this course at tilt 30 / v_max 2.8, measuring each leg's path
# length against its straight-line distance (1.0 = straight, >1.6 = visibly
# looping):
#     0.60 -> mean 1.39x, 155.8 m, 122.2 s   (loops before g4, g5, g8)
#     0.50 -> mean 1.33x, 149.3 m, 119.6 s
#     0.30 -> mean 1.13x, 130.4 m, 106.9 s   <- smooth, matches a clean line
#     0.22 -> mean 1.07x but CRASHES at g5 - too little room to line up
# LOWERED 0.30 -> 0.20 after the drone still overshot g5 in the real sim and
# had to re-enter. At 0.30 the g4->g5 leg still swung 1.35x its straight-line
# distance, and that excursion let the plan accelerate to the full gate cap
# (3.00 m/s) right at the g5 crossing - so any real-world tracking lag turned
# straight into lateral error at the worst moment. Re-swept at the current
# config (tilt 30 / kd_pos 2.8 / short lookahead), which tracks far better
# than when 0.22 crashed earlier:
#     0.30 -> g4->g5 detour 1.35x, g5 crossed at 3.00 m/s, lap 56.3 s
#     0.20 -> detour 1.08x, g5 crossed at 2.21 m/s, lap 50.3 s   <- chosen
#     0.15 -> detour 1.06x, lap 50.1 s (no real gain over 0.20)
# Tightening the path is strictly better than slowing the gates here: it fixes
# the excursion AND lowers the crossing speed as a side effect AND is 6 s
# faster, where lowering v_gate_mps to 2.0 instead cost 5 s and left the
# excursion untouched.
# RAISED 0.20 -> 0.36 for the g6->g7 reversal. An earlier sweep of this
# number found it made NO difference to that corner (radius stuck at
# 0.13 m from 0.20 all the way to 0.60) - but that was measured while the
# reversal clearance routing was accidentally disabled. With the routing
# restored the budget matters again, because it sets how much room g7's
# entry anchor gets to line up: 0.20 -> 0.43 m corner / 1.04 m/s;
# 0.28 -> 0.48 / 1.19;  0.36 -> 0.56 / 1.29;  0.45 -> 0.60 / 1.39 but
# +0.5 s of lap. 0.36 buys a 30% wider corner for 0.1 s.
# RAISED 0.36 -> 0.45 after the real sim clipped a g9 gate edge. At 0.36 the
# budget was silently capping g9's standoff anchors (its two neighbouring legs
# are only ~8.1 m, so the pair got ~1.47 m instead of the 2.0 m configured) and
# the spline then had to bend THROUGH the hoop: radius at the g9 crossing was
# only 2.83 m, i.e. 21 deg of bank while threading a 1.5 m wide frame. 0.45
# restores the full standoff and straightens the crossings that were tightest:
#   R at gate (m)   g3   g5   g7   g8   g9
#     0.36         3.5  2.9  4.6  2.1  2.8
#     0.45         5.4  4.0  8.0  2.7  7.0   <- chosen
#     0.48         4.6  2.4  4.6  3.9  4.5   (spline starts oscillating again)
# g6->g7 (what 0.36 was originally chosen to protect) also improves slightly:
# corner Rmin 0.53 -> 0.61 m, vmin 1.30 -> 1.37 m/s. Cost: +0.5 s of lap.
LEG_BUDGET_FRAC = 0.45
# A junction whose ARRIVAL turn exceeds this is a genuine reversal: the
# incoming leg would otherwise cross the gate plane outside the opening,
# through the frame post, so build_anchors routes it around the outer
# frame instead. Kept SEPARATE from turn_angle_deg (which only sizes
# standoffs, and is now set far lower) and never inferred from the
# standoff magnitude - the leg-budget scaling can shrink a reversal's
# standoff below the base value, which silently switched this routing off
# at g7 and left a 0.13 m cusp there. 100 deg reproduces the behaviour
# this logic was written and measured against.
REVERSAL_CLEARANCE_DEG = 100.0
# Where the reversal clearance anchor sits, relative to the gate centre.
# LATERAL is along the gate bar, on the arrival side, and must clear the
# outer frame (half of 2.7 m) with room to spare. BACK is how far upstream
# of the gate plane it sits, and it is what decides whether the path CURVES
# into the opening or jogs into it: at the original 0.5 m the spline had to
# move 2.55 m sideways while advancing only 0.42 m, which is a corner
# (measured radius 0.47 m, speed collapsing to 1.0 m/s) rather than a turn.
# Standoff for a STACKED pair (two openings at the same XY, separated in z).
# Kept SEPARATE from anchor_standoff_turn_m because this is a vertical drop,
# not a corner. Sharing that number coupled two unrelated problems: shrinking
# it to tighten g10 was also shrinking g7's entry standoff and undoing that
# corner's fix. Decoupled, g7 holds at R~0.55 m / 1.30 m/s across this whole
# sweep. On g10 (swing / over- and under-shoot of the gate altitudes / time):
#   2.0  -> 2.31 m, 0.16/0.16, 3.32 s   (was sharing anchor_standoff_turn_m)
#   1.2  -> 1.47 m, 0.11/0.11, 3.02 s
#   0.5  -> 0.70 m, 0.05/0.05, 2.53 s   <- chosen at the time
#   0.35 -> 0.53 m, 0.03/0.02, 2.44 s   (better, but close to the edge)
#   0.2  -> CRASHES at g10-top
# RAISED 0.5 -> 2.0 to round the TOP->BOTTOM transition. At 0.5 the path had
# to be back on the gate normal within half a metre of each opening, so the
# two places where horizontal flight meets the vertical drop were pivots in
# place: radius 0.27 m, and the drone crawled the transition at 1.05 m/s.
# Re-swept with the standoff now independent of anchor_standoff_turn_m:
#   standoff  Rmin   vmin   t10    g10-low crossing
#     0.5     0.27   1.05   2.54s      0.044
#     1.5     0.43   1.33   3.13s      0.033
#     2.0     0.50   1.43   3.31s      0.030   <- chosen
#     2.5     0.55   1.50   3.52s      0.024   (+0.2 s more for +0.05 m)
# A sideways BOW on the transition was also tried (0.8-3.0 m). It swung the
# path out as intended but made everything worse - Rmin no better than 0.34 m
# and vmin collapsing to 0.32 m/s at every setting - so it was removed.
# NOTE vz_down_max was also tried here and does NOTHING: the plan saturates
# at exactly -3.00 m/s, but accel-slew (56%) and tilt/curvature (33%) are
# what actually cap this leg, so 3.0 -> 6.0 changed not one measured value.
# Anchor-shape rules (2026-09-09, from the g9->g10-top hook and the
# g10-low->g0 kink in plan_RACE; both are global geometry, no per-gate).
# ASYM_APEX_MAX_DEG: a single circular arc fits a junction only when the
# chord makes the same angle with both headings. When the two angles
# differ by more than this, the "apex" bow lands on the wrong side of the
# leg (g9->g10-top: 12 vs 48 deg put the bow due WEST of g9, and the path
# folded back south into the gate). Such junctions get no bow.
# DOGLEG_S: a leg whose headings are aligned but whose chord runs well off
# them (g10-low -> g0: both north, chord 47 deg off) is an S, two equal
# arcs. Without anchors for that shape the spline runs the chord straight
# and kinks inside the 2 m stub at the far gate (5 m/s dip before g0).
ASYM_APEX_MAX_DEG = 20.0
DOGLEG_S = True
DOGLEG_MIN_DEG = 25.0
DOGLEG_SAG_SCALE = 1.0   # Catmull-Rom overshoots sparse bows; <1 tames it
POSE_ANGLE_MAX_DEG = 35.0
POSE_TILT_OVERRIDE_DEG = {'g5': 0.0, 'g6': -25.0, 'g10-top': -30.0, 'g4': 14.0}   # flown 32.1 s knobs
STACK_LOOP = False   # see build_anchors: g10-top -> g10-low as a banked descending half-loop
EARLY_CLIMB = True   # see build_anchors: climb right after the previous gate, turn level into the next
CLIMB_RAMP = False   # ON is the straight climb (g9->top 1.47 -> 1.26 s model) - the replay cannot score climbs, so it is a flight test, not a search default; linear height ramp along the leg instead of the step (see build_anchors)
POSE_Z_OFFSET_M = {'g4': -0.24, 'g8': 0.24, 'g9': 0.45, 'g10-top': 0.0, 'g10-low': 0.4}   # flown 32.1 s knobs; g10-top at 0 (offsets of -0.25..-0.4 took the bar in batch_1)
POST_STUB_M = {'g6': 1.2}     # gate label -> straight exit length (m); g6: a 2 m stub south then a 90 deg bend west was the dip-and-rise into the g7 loop
PRE_STUB_M = {'g0': 0.5}   # g0: the default 2 m entry stub sat BEHIND the takeoff blend anchor (y 1.0 vs 1.8) and folded the first 3 m of the line (race_061 crossed g0 at 4 m/s, 0.55 s behind the model). gate label -> straight approach length (m); g10-top: the turn from the g9 arc must finish BEFORE the gate (race_026 crossed 0.9 m right of centre)
POSE_LAT_OFFSET_M = {'g4': -0.5, 'g5': -0.5, 'g6': -0.5, 'g8': -0.34, 'g10-top': 0.2, 'g10-low': -0.19}   # the flown 32.1 s knobs (race_061) are the rule now
                           # tilt toward the bisector of the incoming and
                           # outgoing chords by up to this. The 1.5 m opening
                           # seen at angle a is 1.5cos(a)-0.26sin(a) wide:
                           # 1.32 m at 20 deg, i.e. 0.66 half-width minus the
                           # 0.15 drone = 0.51 m of tracking budget (measured
                           # error 0.27-0.30). 0 = every gate on its normal.
CORNER_STUB_TILT = True    # tilt the corner-arc stub as well as adding the arc
                           # end. Swept with the 15 deg pose cap (2026-09-09):
                           # untilted -> the g6 exit kinks (profile minimum
                           # 2.65 m/s right at the gate, +0.2 s); tilted ->
                           # smooth, g6 referee-geometry clearance 0.30 m,
                           # the same as the lane gates g1/g2.
REVERSAL_SIDE_FLIP = True   # feature/rip: the NORTH swing searched to 27.33 vs 28.30 south (replay 25/25 both). was False. route reversal clearance on the OTHER side
# Reversal LOOP as an explicit arc of this radius (0 = the old single
# clearance anchor, which made a 0.9 m hook into g7 that the drone flew
# 0.6-1.0 m wide of, missing g7 on 2026-09-09). 2.0 m is the loop Brian
# drew: top of the loop 4 m above g7, west point 2.5 m west of it.
REVERSAL_LOOP_R_M = 2.5
# REVERSAL SWING: approach a reversal gate with ONE
# constant-radius turn on the FAR side of the gate (for g7, arriving from
# g6 in the north-east and crossing east: a right-hand circle whose top is
# g7, so the line swings SOUTH of g7). The arrival-side loop above is
# geometrically an S with a U on the end (south out of g6, right to go
# west, then 180 deg left to come back east) and it carried the two
# tightest bends on the lap outside the stack: r 0.7 m at 2.0 m/s out of
# g6 and r 1.6-1.8 m into the loop; flown 4.4 s against 3.5 planned. The
# swing has no bends: a straight from the previous gate's stub tangent
# onto the circle, then the arc all the way round to the crossing. About
# the same model time at r 3-3.5 m; its case is trackability.
REVERSAL_SWING = True
REVERSAL_OVERTOP = False   # vertical U over the reversal gate instead of the horizontal swing (see build_anchors)
OVERTOP_H_M = 2.5
OVERTOP_B_M = 1.0
OVERTOP_STANDOFF_M = 1.5
REVERSAL_SWING_R_M = 1.5   # feature/rip sweep with the g7 cap off: r1.0 29.14 (frame), 1.5 29.48 (frame), 2.0 29.84 clean (v_min 7.3), 2.5 30.22, 3.0 30.60; overtop 29.61 but a 0.45 m fold at 2.4 m/s
SWING_CROSS_OFFSET_M = 0.0   # crossing bias on a swing: 0.25 toward the far side put the path 0.6 m off-centre at g7's plane (tilted crossing) and touched the frame
SWING_SAMPLE_M = 1.25        # arc anchor spacing on the swing (2.5 m rippled r 2.9..5.2 on an r 3.5 circle)
V_REVERSAL_SWING_MPS = 20.0  # cap OFF (feature/rip): the 4 m disc also caught the straight passing beside g7; the arc's curvature sets the crossing speed
                             # holds ~6.7; the 3.5 m/s loop cap is not needed)
REVERSAL_CROSS_OFFSET_M = 0.25  # see build_anchors: aim inside the loop by the follower's wide error
_LAST_REVERSAL = []          # reversal flags of the last build_anchors() call
_LAST_LABELS = []
GATE_SPEED_CAP = {}          # gate label -> speed cap (m/s) within REVERSAL_WINDOW_M of it (the follower's carrot is 0.3 s of speed: slower = tighter tracking)
V_REVERSAL_MPS = 3.5         # speed cap through a reversal gate (replay: the follower
REVERSAL_WINDOW_M = 4.0      # left the 2.5 m loop at 9.3 m/s and crossed g7 0.45 m wide)
# Loop points are placed this many degrees BEFORE the gate along the loop,
# which ends AT the gate with the crossing tilted toward the next gate
# (REVERSAL_CROSS_MAX_DEG). 180 deg of turning instead of 270 - the drone
# does not have to cross a gate square (Brian, 2026-09-09).
REVERSAL_LOOP_ANGLES_BACK = (120.0, 80.0)
REVERSAL_CROSS_MAX_DEG = 25.0
REVERSAL_LEADIN_M = 1.5
REVERSAL_LOOP_BACK_M = 1.6   # loop bottom ON the approach stub (0.5 stepped back into the stub and kinked)
                            # (topology probe for the optimizer seed)
CORNER_ARC = True        # corner-arc anchors at sharp junctions (g6 exit)
CLIMB_RUNIN_M = 0.0      # extra LEVEL run-in before a gate approached with
                         # > 1 m of climb/descent (stub kept straight so the
                         # climb finishes before the frame); 0 = off
CORNER_R_M = 2.5         # corner-arc radius at sharp junctions; swept 2.5/3.5/4.5 on
                         # g6->g7: 2.77/2.93/3.11 s, turn starts at the gate at 2.5
STACKED_STANDOFF_M = 0.5   # Brian: start the turn back at the gate; the U begins 0.5 m past g10-top (was 2.0 -> 1.0 -> 0.5), calibrated replay 25/25
# STACK U (Brian, 2026-09-09 night): the stacked pair is flown as a U in
# the VERTICAL plane, not a vertical drop with the reversal at the bottom.
# Apex anchor STACK_U_EXTRA_M beyond the standoffs at mid height: the path
# goes out past the top gate while already descending, reverses its
# horizontal velocity at the apex (still falling), and comes back through
# the low gate descending. The thrust vector points back-and-up the whole
# way, so the throttle never has to go to zero - race_039 hung 0.6 s at
# 3.7 m at zero throttle with the motors on their floor, then rose into
# g10-low's top bar.
STACK_U = True    # priced by the 3D thrust-vector ceiling now (race_041's 3 s was the old horizontal-loop pricing at 1.7 m/s plus the yaw demand at idle)
STACK_U_R_M = 1.35       # round U (1.0 with a 0.5 m standoff collapsed the pair under the searched offsets)
                         # semi-axis = half the gate spacing); a single apex
                         # anchor folded the spline to r 0.2-0.3 m whatever
                         # its distance, a sampled half-ellipse keeps r ~1-1.5
STACK_U_SAMPLES = 6
V_TILT_SMOOTH_M = 0.0   # moving-minimum window on the tilt/curvature ceiling (see _speed_profile); 1.5 m cost 4 s on the lap and did not remove the g4 ripple - off
CLEARANCE_LATERAL_M = 2.7 / 2.0 + 1.2
CLEARANCE_BACK_M = 2.5



def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Anchors
# ---------------------------------------------------------------------------

def standoffs(centers_xy: List[Tuple[float, float]], headings: List[float],
              base: float, turn: float,
              thresh_rad: float) -> Tuple[List[float], List[float], List[bool]]:
    """Per-event (pre, post) standoff distances under the two-value rule.

    The junction turn is measured on the actual TRAVEL, not just the crossing
    headings: out-turn = exit heading vs the leg direction to the next
    crossing, in-turn = leg direction vs the next entry heading. A planar
    switchback (leg opposes the entry heading) therefore triggers the larger
    standoff even when the two crossing headings are nearly perpendicular.
    A degenerate XY leg (the stacked out-and-back) falls back to comparing
    the crossing headings directly and widens both sides.

    LEG BUDGET: gate k's exit anchor and gate k+1's entry anchor both live on
    the SAME leg. If they sum to more than the leg is long they cross over
    each other, and the spline - which must interpolate them in order - has
    to double back on itself. That renders as a teardrop loop before the
    gate (the drone over-pivots, flies away from the gate, then comes back)
    instead of the wide gradual turn the standoff was meant to buy.
    Measured on this course: a 5.0 m turn standoff on both ends of the 9.9 m
    g3->g4 leg took that leg from 1.13x to 2.24x its straight-line distance.
    So cap the PAIR at LEG_BUDGET_FRAC of the leg and scale both ends down
    together when they don't fit - wide where there is room, never looping
    where there isn't. Global rule, no per-gate anything.
    """
    n = len(headings)
    pre = [base] * n
    post = [base] * n
    # True where the arrival at gate k+1 is a genuine REVERSAL (see
    # REVERSAL_CLEARANCE_DEG). build_anchors uses this to decide whether to
    # route around the outer frame. It must NOT be inferred from the standoff
    # magnitude: the leg-budget scaling below can shrink a reversal's standoff
    # under the base value, which silently disabled that routing.
    reversal = [False] * n
    for k in range(n - 1):
        dx = centers_xy[k + 1][0] - centers_xy[k][0]
        dy = centers_xy[k + 1][1] - centers_xy[k][1]
        leg_len = math.hypot(dx, dy)
        if leg_len < 1.0:
            # Stacked pair: the two openings share an XY position and are
            # separated in z alone, so this is a vertical transition, not a
            # horizontal corner. It gets its OWN standoff - the turn standoff
            # is sized for cornering room on a real leg, and reusing it here
            # coupled the two problems (tightening g10 was shrinking g7's
            # entry and undoing its corner fix).
            turn_in = abs(wrap_pi(headings[k + 1] - headings[k]))
            out_big = in_big = turn_in > thresh_rad
            turn_out = turn_in
            if out_big:
                post[k] = STACKED_STANDOFF_M
            if in_big:
                pre[k + 1] = STACKED_STANDOFF_M
            reversal[k + 1] = False   # vertical approach: no frame to dodge
            continue
        else:
            leg = math.atan2(dy, dx)
            turn_out = abs(wrap_pi(leg - headings[k]))
            turn_in = abs(wrap_pi(headings[k + 1] - leg))
            out_big = turn_out > thresh_rad
            in_big = turn_in > thresh_rad
        if out_big:
            post[k] = turn
        if in_big:
            pre[k + 1] = turn
        # Only for a real horizontal leg: the clearance routing exists because
        # an incoming leg would cross the gate plane outside the opening and
        # clip the frame post. The stacked pair is approached vertically, so
        # that geometry never arises there - and routing it around the frame
        # anyway re-inflated its loop from 2.56x to 4.17x (+4.1 s on the lap).
        reversal[k + 1] = (leg_len >= 1.0
                           and turn_in > math.radians(REVERSAL_CLEARANCE_DEG))
        # Fit the pair to the leg. Skipped for a degenerate XY leg (the
        # stacked pair is separated in z, so its XY length says nothing
        # about the room available).
        if leg_len >= 1.0:
            budget = LEG_BUDGET_FRAC * leg_len
            total = post[k] + pre[k + 1]
            if total > budget:
                scale = budget / total
                post[k] *= scale
                pre[k + 1] *= scale
    return pre, post, reversal


# Half-width of each opening the LANE may claim when straightening a run
# of aligned gates: 0.60 referee-effective window (0.75 - 0.15 drone
# radius) minus measured tracking error on a straightened lane. 0.40 was
# sized on <= 0.16 m of error (flight 2026-09-03); the aerobatic plant
# flies 0.27-0.30 m of lateral error at g1/g2 (race_001/race_002.csv,
# 2026-09-09), and 0.40 + 0.28 = 0.68 > 0.60 touched g1's post three
# flights running. 0.25 leaves 0.07 m at the measured error. The g0-g3
# stagger is ~1 m; crossing toward the hole edges instead of the centers
# turns the r~1.5-4 m interpolation wiggle into r~15+ m arcs, which is
# what lets those gates run near v_max. Re-widen only from a measured
# smaller tracking error, never from a plan.
_LANE_USE_M = 0.25
_LANE_ALIGN_RAD = math.radians(25.0)


def _lane_points(events) -> List[np.ndarray]:
    """Crossing points for runs of >=3 heading-aligned gates: fit the
    best single diagonal through the run and shift each crossing toward
    it, clamped to +-_LANE_USE_M along its own gate bar. Best-effort by
    design - where one line cannot stab every hole, a partially
    straightened lane still multiplies the local bend radius. Gates
    outside aligned runs keep their exact centers."""
    pts = [np.array([e.x, e.y, e.z], dtype=float) for e in events]
    runs = []
    n = len(events)

    # RE-ENABLED (chord version): a run of aligned gates (g0-g3) is not
    # collinear - g1 juts ~1 m off the g0->g3 line - so crossing the exact
    # centers makes the path weave, and the drone slows for that curvature.
    # This shifts each interior gate toward the g0->g3 CHORD (minimising the
    # weave, not maximising speed to the hole edge), so the lane flies nearly
    # straight and fast. The earlier EDGE-max version overshot g2's post; the
    # chord version pulls toward center-line and is far less overshoot-prone.

    def _joins(a, b):
        # aligned headings AND travel along them: the lap-1 g10-low ->
        # lap-2 g0 leg matches headings (both ~north) but travels ~34
        # deg off them - a dogleg, not a lane. Grouping it dragged the
        # lap-2 g0-g3 diagonal 8 m east and crawled lap-2 g0/g1 at
        # 3.05-3.11 vs lap-1's 5.2
        dx, dy = b.x - a.x, b.y - a.y
        leg = math.hypot(dx, dy)
        if leg < 1.5:
            return False
        if abs(wrap_pi(b.heading_rad - a.heading_rad)) >= _LANE_ALIGN_RAD:
            return False
        travel = math.atan2(dy, dx)
        return (abs(wrap_pi(travel - a.heading_rad)) < _LANE_ALIGN_RAD
                and abs(wrap_pi(travel - b.heading_rad)) < _LANE_ALIGN_RAD)

    i = 0
    while i < n:
        j = i + 1
        while j < n and _joins(events[j - 1], events[j]):
            j += 1
        if j - i >= 3:
            ev = events[i:j]
            # Straightest feasible corridor, not a speed-max diagonal.
            # Target the CHORD between the run's first and last gate
            # centers and pull every interior gate as far toward that
            # straight line as its opening safely allows. This MINIMISES
            # the weave: the old best-fit-minimax line pushed crossings
            # to opposite opening edges (g1 flown at -1.03, then a hard
            # swing east that overshot g2's +0.41 post and crashed). Endpoints are pinned so the chord
            # is anchored to the real gates the drone must hit.
            a = np.array([ev[0].x, ev[0].y])
            b = np.array([ev[-1].x, ev[-1].y])
            chord = b - a
            clen = float(np.hypot(chord[0], chord[1]))
            u = chord / max(clen, 1e-9)
            v = np.array([-u[1], u[0]])
            t_lane = np.array([u[0], u[1]])
            for k, e in enumerate(ev):
                if i + k == 0 or k == 0 or k == len(ev) - 1:
                    # launch gate and both run endpoints stay put; they
                    # define the straight line the interior bows toward
                    runs.append((i + k, t_lane))
                    continue
                rel = np.array([e.x, e.y]) - a
                lat = float(np.dot(rel, v))          # signed dist to chord
                along = float(np.dot(rel, u))
                target_lat = 0.0                      # the chord itself
                want = target_lat - lat
                off = max(-_LANE_USE_M, min(_LANE_USE_M, want))
                xy = a + along * u + (lat + off) * v
                pts[i + k] = np.array([xy[0], xy[1], e.z])
                runs.append((i + k, t_lane))
        i = j
    # lane_dir[k]: unit XY tangent of the fitted diagonal for gates inside
    # an aligned run, else None. build_anchors lays THOSE gates' pre/post
    # anchors along the lane instead of the gate normal - the normal-
    # aligned triplet was yanking the spline off the diagonal and back at
    # every crossing, an S-wiggle inside each gate window that the slew
    # ceiling priced as a dip ("slowing inside every gate for no reason",
    # ). The lane runs within _LANE_ALIGN_RAD of each
    # crossing heading, so crossing direction stays legal.
    lane_dir = [None] * n
    for k, t in runs:
        lane_dir[k] = t
    return pts, lane_dir


def build_anchors(course, cfg: VehicleConfig):
    """Anchor chain: takeoff -> (pre, center, post) per event -> park.

    Returns (anchors[N,3], center_anchor_idx[len(events)]).
    """
    p = cfg.planner
    events = [course.event(i) for i in range(course.total_events)]
    lane, lane_dir = _lane_points(events)
    pre_d, post_d, reversal = standoffs([(c.x, c.y) for c in events],
                                        [c.heading_rad for c in events],
                                        p.anchor_standoff_m,
                                        p.anchor_standoff_turn_m,
                                        math.radians(p.turn_angle_deg))
    pre_d = list(pre_d)
    post_d = list(post_d)
    for k, c in enumerate(events):
        if c.label in PRE_STUB_M:
            pre_d[k] = PRE_STUB_M[c.label]
        if c.label in POST_STUB_M:
            post_d[k] = POST_STUB_M[c.label]
    # STACKED pair as a banked half-loop: the hover-and-drop cusp is what
    # this airframe cannot fly (race_027: 1.5 s at 4.5 m with the throttle
    # at zero, then a 3 m/s drop and a bounce through g10-low 0.4 m high).
    # A descending banked turn keeps speed, so the thrust vector's vertical
    # component drops below the throttle floor and the drone can sink.
    reversal = list(reversal)
    if STACK_LOOP:
        for k in range(1, len(events)):
            a, b = events[k - 1], events[k]
            if (abs(wrap_pi(b.heading_rad - a.heading_rad)) > math.radians(150.0)
                    and math.hypot(b.x - a.x, b.y - a.y) < 1.0):
                reversal[k] = True
    # Reversal-gate crossing target moved to the INSIDE of the loop by the
    # follower's measured steady-state error: the pure-pursuit carrot runs
    # a curve wide by about L^2/(2R) (0.40-0.45 m in the replay of the
    # 2.5 m g7 loop; the real drone hit g7's outside post twice on
    # 2026-09-09). Aim inside so the tracked path passes through the
    # centre. The referee still scores the true gate.
    for k, c in enumerate(events):
        if c.label in POSE_LAT_OFFSET_M and not reversal[k]:
            bar = np.array([-math.sin(c.heading_rad), math.cos(c.heading_rad), 0.0])
            lane[k] = lane[k] + POSE_LAT_OFFSET_M[c.label] * bar
        if c.label in POSE_Z_OFFSET_M:
            lane[k] = lane[k] + np.array([0.0, 0.0, POSE_Z_OFFSET_M[c.label]])
    for k in range(1, len(events)):
        if reversal[k] and REVERSAL_CROSS_OFFSET_M > 0.0:
            c = events[k]
            bar = np.array([-math.sin(c.heading_rad), math.cos(c.heading_rad), 0.0])
            side = math.copysign(1.0, float(np.dot(bar, lane[k - 1] - lane[k])))
            if REVERSAL_SIDE_FLIP:
                side = -side
            if REVERSAL_SWING:
                side = -side      # the swing's inside is the FAR side
            lane[k] = lane[k] + side * (SWING_CROSS_OFFSET_M if REVERSAL_SWING
                                        else REVERSAL_CROSS_OFFSET_M) * bar
    # Reversal-gate crossing target moved to the INSIDE of the loop by the
    # follower's measured steady-state error: the pure-pursuit carrot runs
    # a curve wide by about L^2/(2R) (0.45 m in the replay of the 2.5 m
    # g7 loop, and the real drone hit g7's outside post twice). Aim inside
    # so the tracked path passes through the centre. The referee still
    # scores the true gate.
    for k in range(1, len(events)):
        if reversal[k] and REVERSAL_CROSS_OFFSET_M > 0.0:
            c = events[k]
            bar = np.array([-math.sin(c.heading_rad), math.cos(c.heading_rad), 0.0])
            side = math.copysign(1.0, float(np.dot(bar, lane[k - 1] - lane[k])))
            if REVERSAL_SIDE_FLIP:
                side = -side
            if REVERSAL_SWING:
                side = -side      # the swing's inside is the FAR side
            lane[k] = lane[k] + side * (SWING_CROSS_OFFSET_M if REVERSAL_SWING
                                        else REVERSAL_CROSS_OFFSET_M) * bar

    # Junction arc parameters (k -> k+1): (radius, turn sign) for bent
    # non-reversal junctions, else None. Used twice: the apex anchor
    # mid-leg, and ROTATING each gate's pre/post stub onto the arc - a
    # straight 2 m stub along the gate heading forces the spline to
    # re-bend hard right after it (the residual dips after g3 / around
    # g4).
    # The chord of an arc segment of length d deviates from the end
    # tangent by d/(2r), so tilting the stub by that angle lays it on
    # the arc. Answer to the question: no - the physics floor for the
    # r~7 m top arcs is ~5.4 m/s; everything below that was kink.
    junc: List[Optional[Tuple[float, float]]] = [None] * len(events)
    for k in range(len(events) - 1):
        if reversal[k + 1]:
            continue
        a2, b2 = lane[k][:2], lane[k + 1][:2]
        cvec = b2 - a2
        L = float(np.hypot(cvec[0], cvec[1]))
        turn = abs(wrap_pi(events[k + 1].heading_rad - events[k].heading_rad))
        if L <= 3.0 or not (math.radians(25.0) < turn < math.radians(140.0)):
            continue
        chord_ang = math.atan2(cvec[1], cvec[0])
        phi_a = abs(wrap_pi(chord_ang - events[k].heading_rad))
        phi_b = abs(wrap_pi(events[k + 1].heading_rad - chord_ang))
        phi = 0.5 * (phi_a + phi_b)
        if phi <= math.radians(10.0):
            continue
        if abs(phi_a - phi_b) > math.radians(ASYM_APEX_MAX_DEG):
            continue          # no single arc fits: leave the stubs straight
        ta = np.array([math.cos(events[k].heading_rad),
                       math.sin(events[k].heading_rad)])
        tb = np.array([math.cos(events[k + 1].heading_rad),
                       math.sin(events[k + 1].heading_rad)])
        cross = ta[0] * tb[1] - ta[1] * tb[0]
        junc[k] = (L / (2.0 * math.sin(phi)), 1.0 if cross > 0 else -1.0)

    def _rot(v: np.ndarray, ang: float) -> np.ndarray:
        ca, sa = math.cos(ang), math.sin(ang)
        return np.array([ca * v[0] - sa * v[1],
                         sa * v[0] + ca * v[1], v[2]])

    def _stub_on_arc(center, n_dir, d, target):
        """Stub end d from center, laid on the circular arc that leaves
        center tangent to n_dir and passes through target (chord angle
        theta): the point d along that arc sits at chord-to-tangent
        angle d*sin(theta)/|chord|. A straight stub says "fly the gate
        normal for 2 m, then turn"; this starts the turn at the gate.
        Falls back to straight when the target is behind or dead ahead.
        Global geometry, every junction, no per-gate anything."""
        v = np.array([target[0] - center[0], target[1] - center[1]])
        L = float(np.hypot(v[0], v[1]))
        if L < 1e-6:
            return center + d * n_dir
        theta = wrap_pi(math.atan2(v[1], v[0]) - math.atan2(n_dir[1], n_dir[0]))
        if abs(theta) < math.radians(3.0) or abs(theta) > math.radians(85.0):
            return center + d * n_dir
        tilt = min(abs(theta), d * math.sin(abs(theta)) / L)
        return center + d * _rot(n_dir, math.copysign(tilt, theta))

    def _corner(center, n_dir, d, target, r_max=CORNER_R_M):
        """Sharp junction (chord angle >= 45 deg, e.g. g6 exit south with
        the g7 clearance point due west): one tilted stub cannot express
        a corner, so lay a circular arc of radius r from the gate, tangent
        to n_dir, turning until it faces the target, and return
        (stub_on_that_arc, arc_end). The straight run to the target starts
        at arc_end. Falls back to _stub_on_arc when the corner will not
        fit before the target."""
        v = np.array([target[0] - center[0], target[1] - center[1]])
        L = float(np.hypot(v[0], v[1]))
        if L < 1e-6:
            return center + d * n_dir, None
        theta = wrap_pi(math.atan2(v[1], v[0]) - math.atan2(n_dir[1], n_dir[0]))
        if abs(theta) < math.radians(45.0) or abs(theta) > math.radians(120.0):
            return _stub_on_arc(center, n_dir, d, target), None
        sgn = math.copysign(1.0, theta)
        perp = np.array([-n_dir[1] * sgn, n_dir[0] * sgn, 0.0])
        r = min(r_max, 0.45 * L)
        end = (center + r * math.sin(abs(theta)) * n_dir
               + r * (1.0 - math.cos(theta)) * perp)
        rest = np.array([target[0] - end[0], target[1] - end[1]])
        w = v / L
        if float(rest[0] * w[0] + rest[1] * w[1]) < 1.0:
            return _stub_on_arc(center, n_dir, d, target), None
        ang = min(abs(theta), d / (2.0 * r)) if CORNER_STUB_TILT else 0.0
        stub = center + d * _rot(n_dir, sgn * ang)
        return stub, end

    # Diagonal climb-out toward the first gate, not a vertical elevator to
    # cruise altitude over the spawn. A single (0,0,takeoff_alt) anchor made
    # the spline go straight up and only pitch forward near g0. Two low
    # forward anchors carry the path toward the first opening as it climbs,
    # so the drone leans into the course off the deck.
    e0 = events[0]
    first_xy = np.array([e0.x, e0.y])
    climb = np.array([first_xy[0] * 0.30, first_xy[1] * 0.30,
                      0.45 * p.takeoff_alt_m])
    blend = np.array([first_xy[0] * 0.60, first_xy[1] * 0.60,
                      0.80 * p.takeoff_alt_m])
    # CROSSING DIRECTION per gate (the tangent of the line AT the gate).
    # Lane gates: the line through the neighbouring crossings (a gate off
    # the run's chord, g1, otherwise got a straight exit from g0, a jog,
    # and a straight entry - the drone lagged the jog 0.36 m and hit the
    # post, race_004 2026-09-09). Other gates: tilt toward the bisector of
    # the incoming and outgoing chords, capped at POSE_ANGLE_MAX_DEG, so the
    # drone apexes THROUGH the gate instead of crossing square and only
    # then starting the corner (g6 -> g7: the turn began at g7's edge).
    # Reversal gates, the stacked pair and the run ends keep their normal.
    cross_dir: List[np.ndarray] = []
    for k, c in enumerate(events):
        n = np.array([math.cos(c.heading_rad), math.sin(c.heading_rad), 0.0])
        d = n
        if lane_dir[k] is not None and not reversal[k]:
            same = lambda j: (0 <= j < len(events) and lane_dir[j] is not None
                              and np.allclose(lane_dir[j], lane_dir[k]))
            a_pt = lane[k - 1] if same(k - 1) else lane[k]
            b_pt = lane[k + 1] if same(k + 1) else lane[k]
            tv = b_pt[:2] - a_pt[:2]
            if np.hypot(tv[0], tv[1]) > 1e-6:
                dev = wrap_pi(math.atan2(tv[1], tv[0]) - c.heading_rad)
                dev = max(-_LANE_ALIGN_RAD, min(_LANE_ALIGN_RAD, dev))
                d = _rot(n, dev)
        elif reversal[k] and k + 1 < len(events) and REVERSAL_CROSS_MAX_DEG > 0:
            dout = lane[k + 1][:2] - lane[k][:2]
            if np.hypot(dout[0], dout[1]) > 1.0:
                dev = wrap_pi(math.atan2(dout[1], dout[0]) - c.heading_rad)
                cap = math.radians(REVERSAL_CROSS_MAX_DEG)
                d = _rot(n, max(-cap, min(cap, dev)))
        elif (not reversal[k] and 0 < k < len(events) - 1
              and POSE_ANGLE_MAX_DEG > 0):
            din = lane[k][:2] - lane[k - 1][:2]
            dout = lane[k + 1][:2] - lane[k][:2]
            li, lo = float(np.hypot(*din)), float(np.hypot(*dout))
            if li > 1.0 and lo > 1.0:
                bis = din / li + dout / lo
                if np.hypot(bis[0], bis[1]) > 1e-6:
                    dev = wrap_pi(math.atan2(bis[1], bis[0]) - c.heading_rad)
                    cap = math.radians(POSE_ANGLE_MAX_DEG)
                    d = _rot(n, max(-cap, min(cap, dev)))
        if c.label in POSE_TILT_OVERRIDE_DEG and not reversal[k]:
            d = _rot(n, math.radians(POSE_TILT_OVERRIDE_DEG[c.label]))
        cross_dir.append(d)

    anchors: List[np.ndarray] = [np.array([0.0, 0.0, 0.2]), climb, blend]
    center_idx: List[int] = []
    for k, c in enumerate(events):
        n = np.array([math.cos(c.heading_rad), math.sin(c.heading_rad), 0.0])
        ctr = lane[k]
        # In an aligned run, the approach/exit anchors follow the LANE,
        # not the gate normal (see _lane_points). Reversal gates keep the
        # normal - their clearance routing depends on it.
        n_anchor = cross_dir[k]
        # Lay each stub ON its junction arc (see junc[] above): the pre
        # stub of a gate with an incoming arc tilts backward along it,
        # the post stub of a gate with an outgoing arc tilts forward.
        pre_dir = post_dir = n_anchor
        if k > 0 and junc[k - 1] is not None:
            r_in, s_in = junc[k - 1]
            pre_dir = _rot(n, -s_in * pre_d[k] / (2.0 * r_in))
        if junc[k] is not None:
            r_out, s_out = junc[k]
            post_dir = _rot(n, s_out * post_d[k] / (2.0 * r_out))
        pre = ctr - pre_d[k] * pre_dir
        post = ctr + post_d[k] * post_dir
        climb_in = (k > 0 and abs(float(ctr[2] - lane[k - 1][2])) > 1.0
                    and math.hypot(ctr[0] - lane[k - 1][0],
                                   ctr[1] - lane[k - 1][1]) > 1.0)
        if climb_in and CLIMB_RUNIN_M > 0:
            pre = ctr - (pre_d[k] + CLIMB_RUNIN_M) * n_anchor
        # Anchor crowding rule (global): when the previous exit anchor and
        # this entry anchor are closer than the base standoff they fight each
        # other and the spline S-wiggles - merge them into their midpoint.
        # The stacked pair is unaffected (its post/pre are 2.7 m apart in z).
        # Reversal junction: the incoming leg otherwise crosses this gate's
        # PLANE just outside the opening - through the frame post (measured:
        # the g7 hairpin clipped the post at every standoff length). Route it
        # around the OUTER frame: a clearance anchor on the arrival side,
        # 1.2 m beyond the frame edge, slightly before the gate plane.
        # Fully derived from geometry; works at any reversal on any course.
        # TURN APEX (non-reversal bends): the pre/center/post triplets
        # alone connect bent junctions with near-straight legs and
        # compress the whole turn into ~2 m at each gate - the g3->g4->
        # g5 top ran as a polygon at 3.4 m/s where one wide arc fits
        # (tangent-chord angles at g3->g4: 44.8 vs 43.2 deg, i.e. an
        # r~7 m circle passes through BOTH gates with the right
        # headings, worth ~5.4 m/s at the current lateral budget). Add the tangent arc's apex between the two
        # crossings so the spline bows into the sweep. Pure geometry,
        # every junction, no per-gate anything.
        if (k > 0 and not reversal[k]
                and math.hypot(ctr[0] - anchors[-1][0],
                               ctr[1] - anchors[-1][1]) > 1.0):
            prev_e = events[k - 1]
            a2, b2 = lane[k - 1][:2], ctr[:2]
            cvec = b2 - a2
            L = float(np.hypot(cvec[0], cvec[1]))
            turn = abs(wrap_pi(c.heading_rad - prev_e.heading_rad))
            if L > 3.0 and math.radians(25.0) < turn < math.radians(140.0):
                chord_ang = math.atan2(cvec[1], cvec[0])
                phi_a = wrap_pi(chord_ang - prev_e.heading_rad)
                phi_b = wrap_pi(c.heading_rad - chord_ang)
                phi = 0.5 * (abs(phi_a) + abs(phi_b))
                asym = abs(abs(phi_a) - abs(phi_b))
                if (phi > math.radians(10.0)
                        and asym > math.radians(ASYM_APEX_MAX_DEG)):
                    # no single arc fits (g9->g10-top): instead of a bow,
                    # start each turn AT its gate - previous post toward
                    # this pre, this pre back toward that post
                    n_prev = cross_dir[k - 1]
                    if CORNER_ARC:
                        stub, end = _corner(lane[k - 1], n_prev, post_d[k - 1], pre)
                    else:
                        stub, end = _stub_on_arc(lane[k - 1], n_prev, post_d[k - 1], pre), None
                    anchors[-1] = stub
                    if end is not None and np.linalg.norm(end - anchors[-1]) > _DEDUP_M:
                        anchors.append(end)
                    if not climb_in:
                        pre = _stub_on_arc(ctr, -n_anchor, pre_d[k], anchors[-1])
                if (phi > math.radians(10.0)
                        and asym <= math.radians(ASYM_APEX_MAX_DEG)):
                    r = L / (2.0 * math.sin(phi))
                    sag = min(3.0, r * (1.0 - math.cos(phi)))
                    ta = np.array([math.cos(prev_e.heading_rad),
                                   math.sin(prev_e.heading_rad)])
                    tb = np.array([math.cos(c.heading_rad),
                                   math.sin(c.heading_rad)])
                    cross = ta[0] * tb[1] - ta[1] * tb[0]
                    out = (np.array([-cvec[1], cvec[0]]) / L
                           * (-1.0 if cross < 0 else 1.0) * -1.0)
                    mid = 0.5 * (a2 + b2)
                    apex = np.array([mid[0] + out[0] * sag,
                                     mid[1] + out[1] * sag,
                                     0.5 * (lane[k - 1][2] + ctr[2])])
                    if BIARC and ARC_SAMPLE_M > 0.0:
                        p0 = anchors[-1]
                        u = p0[:2] - lane[k - 1][:2]
                        for q in _biarc_points(p0, u, pre, n_anchor,
                                               p0[2], pre[2], ARC_SAMPLE_M):
                            if np.linalg.norm(q - anchors[-1]) > _DEDUP_M:
                                anchors.append(q)
                    elif ARC_SAMPLE_M > 0.0:
                        # Sample the WHOLE junction circle uniformly from
                        # the previous post stub to this pre stub instead
                        # of one apex anchor: with anchors 2 / 4.5 / 4.7 /
                        # 2 m apart the spline rippled to r 3.1 m at the
                        # g5 pre stub on an r 8.4 m arc (v 6.2 instead of
                        # 10 m/s). Both stubs already lie on this circle.
                        # circle through the previous post stub (p0, with
                        # its own tangent u) AND this pre stub (p1): the
                        # nominal r = L/(2 sin phi) circle misses g5 by
                        # 1.3 m when the two chord angles differ.
                        p0 = anchors[-1][:2]
                        p1 = pre[:2]
                        u = p0 - lane[k - 1][:2]
                        u = u / max(float(np.hypot(u[0], u[1])), 1e-6)
                        if ARC_SPLIT_MISMATCH:
                            chord = p1 - p0
                            ch = math.atan2(chord[1], chord[0])
                            al0 = wrap_pi(ch - math.atan2(u[1], u[0]))
                            al1 = wrap_pi(math.atan2(n_anchor[1], n_anchor[0]) - ch)
                            al = 0.5 * (al0 + al1)
                            u = np.array([math.cos(ch - al), math.sin(ch - al)])
                        nrm = np.array([-u[1], u[0]])
                        dvec = p1 - p0
                        den = 2.0 * float(np.dot(dvec, nrm))
                        if abs(den) < 1e-6:
                            den = 1e-6
                        r_fit = float(np.dot(dvec, dvec)) / den   # signed: +left
                        sgn = 1.0 if r_fit > 0 else -1.0
                        r = abs(r_fit)
                        Cc = p0 + r_fit * nrm
                        th0 = math.atan2(p0[1] - Cc[1], p0[0] - Cc[0])
                        th1 = math.atan2(p1[1] - Cc[1], p1[0] - Cc[0])
                        dth = wrap_pi(th1 - th0)
                        if sgn * dth < 0:
                            dth += sgn * 2.0 * math.pi
                        n_pts = max(1, int(abs(dth) * r / ARC_SAMPLE_M))
                        z0, z1 = anchors[-1][2], pre[2]
                        for i in range(1, n_pts + 1):
                            f = i / (n_pts + 1)
                            th = th0 + f * dth
                            q = np.array([Cc[0] + r * math.cos(th),
                                          Cc[1] + r * math.sin(th),
                                          z0 + f * (z1 - z0)])
                            if np.linalg.norm(q - anchors[-1]) > _DEDUP_M:
                                anchors.append(q)
                    elif np.linalg.norm(apex - anchors[-1]) > _DEDUP_M:
                        anchors.append(apex)
        if (DOGLEG_S and k > 0 and not reversal[k]
                and math.hypot(ctr[0] - anchors[-1][0],
                               ctr[1] - anchors[-1][1]) > 1.0):
            prev_e = events[k - 1]
            a2, b2 = lane[k - 1][:2], ctr[:2]
            cvec = b2 - a2
            L = float(np.hypot(cvec[0], cvec[1]))
            turn = abs(wrap_pi(c.heading_rad - prev_e.heading_rad))
            chord_ang = math.atan2(cvec[1], cvec[0])
            psi = wrap_pi(chord_ang - prev_e.heading_rad)
            if (L > 4.0 and turn < math.radians(25.0)
                    and abs(psi) > math.radians(DOGLEG_MIN_DEG)):
                # two equal arcs, each turning alpha = 2*psi, radius
                # r = L / (4 sin psi); each half-chord bows by
                # r (1 - cos psi) toward its own turn side
                r = L / (4.0 * math.sin(abs(psi)))
                sag = DOGLEG_SAG_SCALE * r * (1.0 - math.cos(psi))
                u = cvec / L
                left = np.array([-u[1], u[0]])
                # the first arc turns toward the chord (left if psi > 0)
                # and an arc bulges to the OUTSIDE of its turn, i.e. to
                # the right of its own half-chord for a left turn
                side = -1.0 if psi > 0 else 1.0
                sgn = 1.0 if psi > 0 else -1.0
                # Lay BOTH stubs on their arcs, the same chord-to-tangent
                # tilt d/(2r) the junction code uses. A straight stub
                # means "exit north for 2 m, then start turning" and a
                # last-second bend into the far gate (Brian, 2026-09-09:
                # "exiting g10-low 3 m before the arc, turning last minute
                # into g0"). The previous gate's post anchor is the last
                # one appended, so it is re-laid here.
                n_prev = cross_dir[k - 1]
                d_post = post_d[k - 1]
                anchors[-1] = (lane[k - 1]
                               + d_post * _rot(n_prev, sgn * d_post / (2.0 * r)))
                pre = ctr - pre_d[k] * _rot(n_anchor, sgn * pre_d[k] / (2.0 * r))
                q1 = a2 + 0.25 * L * u + side * sag * left
                q2 = a2 + 0.75 * L * u - side * sag * left
                z1 = 0.75 * lane[k - 1][2] + 0.25 * ctr[2]
                z2 = 0.25 * lane[k - 1][2] + 0.75 * ctr[2]
                for q, zq in ((q1, z1), (q2, z2)):
                    qa = np.array([q[0], q[1], zq])
                    if np.linalg.norm(qa - anchors[-1]) > _DEDUP_M:
                        anchors.append(qa)
        if k > 0 and reversal[k]:
            bar = np.array([-math.sin(c.heading_rad),
                            math.cos(c.heading_rad), 0.0])
            side = math.copysign(1.0, float(np.dot(bar, anchors[-1] - ctr)))
            if REVERSAL_SIDE_FLIP:
                side = -side
            # Route around the OUTER frame on the arrival side. BACK is
            # what decides whether the path curves into the opening or jogs
            # into it - swept on the g7 reversal (Rmin / slowest point on the
            # leg): 0.5 -> 0.33 m, 0.90 m/s; 1.5 -> 0.41 m, 1.00 m/s;
            # 2.5 -> 0.43 m, 1.04 m/s; 3.5 -> 0.38 m, 0.91 m/s.
            # An explicit multi-point U-turn arc was also tried here and did
            # NOT beat this single anchor (Rmin 0.40 m at best, and a longer
            # path), so the simpler form is kept.
            clr = (ctr + bar * side * CLEARANCE_LATERAL_M
                   - n * CLEARANCE_BACK_M)
            # 3D reversal (wingover): lift the clearance apex by
            # reversal_climb_m so the path arcs UP and over the hairpin
            # instead of a flat tight U. The entry (g_prev post) and exit
            # (g_k pre/center) stay at gate altitude, so the spline climbs
            # into the apex and descends out - the turn happens slow at the
            # top where a tight radius is free, and gravity does the brake/
            # accel. climb=0 recovers the old flat clearance anchor.
            clr[2] = anchors[-1][2] + p.reversal_climb_m
            loop_pts = []
            swing_done = False
            if REVERSAL_OVERTOP:
                # OVER THE TOP (Brian, feature/rip): treat g7 like g10-low.
                # Climb on the way in from the previous gate to a point
                # OVERTOP_H_M above the gate's own approach point (standoff
                # OVERTOP_STANDOFF_M on the approach side), heading AWAY
                # from the crossing direction, then a vertical half-ellipse
                # (horizontal semi-axis OVERTOP_B_M) reverses the heading
                # while dropping to the gate's lead-in, and the drone dives
                # through heading the right way. Same physics and code
                # shape as STACK_U; replaces the 18 m horizontal swing.
                d_x = cross_dir[k]
                dxy = np.array([d_x[0], d_x[1], 0.0])
                dxy = dxy / max(float(np.linalg.norm(dxy)), 1e-9)
                pre_pt = ctr - dxy * OVERTOP_STANDOFF_M
                z_lo = float(ctr[2]); z_hi = z_lo + OVERTOP_H_M
                z_mid = 0.5 * (z_hi + z_lo); a_v = 0.5 * (z_hi - z_lo)
                hi_pt = np.array([pre_pt[0], pre_pt[1], z_hi])
                loop_pts.append(hi_pt)
                for j in range(1, STACK_U_SAMPLES):
                    th = math.pi * j / STACK_U_SAMPLES
                    q_xy = pre_pt[:2] - dxy[:2] * (OVERTOP_B_M * math.sin(th))
                    loop_pts.append(np.array([q_xy[0], q_xy[1], z_mid + a_v * math.cos(th)]))
                clr = loop_pts[0].copy()
                swing_done = True
            if REVERSAL_SWING and REVERSAL_SWING_R_M > 0.0 and not swing_done:
                R = REVERSAL_SWING_R_M
                d_x = cross_dir[k]
                # centre on the FAR side, the circle TANGENT to the crossing
                # direction at the straight lead-in point (not at the gate:
                # ending the arc 1.5 m early left a 25 deg kink, r 1.1 m)
                lead_pt = ctr - d_x * REVERSAL_LEADIN_M
                C = lead_pt + R * _rot(d_x, -side * math.pi / 2.0)
                phi_g = math.atan2(lead_pt[1] - C[1], lead_pt[0] - C[0])
                P = anchors[-1]
                dv = P[:2] - C[:2]
                dist = float(np.hypot(dv[0], dv[1]))
                if dist > R + 0.2:
                    ang = math.atan2(dv[1], dv[0])
                    alpha = math.acos(R / dist)
                    best = None
                    for sg in (1.0, -1.0):
                        th_t = ang + sg * alpha
                        # arc from the tangent point forward to the gate,
                        # measured backwards from the gate as +side*back
                        sweep = (side * (th_t - phi_g)) % (2.0 * math.pi)
                        if math.radians(120.0) <= sweep <= math.radians(340.0):
                            if best is None or sweep < best[1]:
                                best = (th_t, sweep)
                    if best is not None:
                        th_t, sweep = best
                        back_end = 0.0          # last sample IS the lead-in point (uniform spacing; a 0.6 m gap rippled to r 1.6)
                        n_pts = max(2, int((sweep - back_end) * R / SWING_SAMPLE_M))
                        for i in range(n_pts + 1):
                            back = sweep - i * (sweep - back_end) / n_pts
                            th = phi_g + side * back
                            q = C + R * np.array([math.cos(th), math.sin(th), 0.0])
                            q[2] = ctr[2]
                            loop_pts.append(q)
                        clr = loop_pts[0].copy()
                        swing_done = True
            if REVERSAL_LOOP_R_M > 0.0 and not swing_done:
                R = REVERSAL_LOOP_R_M
                d_x = cross_dir[k]
                C = ctr + R * _rot(d_x, side * math.pi / 2.0)
                phi_g = math.atan2(ctr[1] - C[1], ctr[0] - C[0])
                z_a = anchors[-1][2] + p.reversal_climb_m
                z_b = ctr[2]
                for back in REVERSAL_LOOP_ANGLES_BACK:
                    th = phi_g - side * math.radians(back)
                    q = C + R * np.array([math.cos(th), math.sin(th), 0.0])
                    # altitude follows the arc: same height for a flat
                    # reversal (g7), a continuous descent for the stack
                    f_arc = 1.0 - back / 180.0
                    q[2] = z_a + f_arc * (z_b - z_a)
                    loop_pts.append(q)
                clr = loop_pts[0].copy()
            # the previous gate's exit stub turns toward the clearance
            # point from the gate itself (g6 -> g7: it left g6 straight
            # south for 2 m before bending west)
            prev_e = events[k - 1]
            if (not loop_pts) and math.hypot(lane[k - 1][0] - anchors[-1][0],
                          lane[k - 1][1] - anchors[-1][1]) < 1.5 * post_d[k - 1]:
                # (with an explicit loop the previous gate's stub keeps its
                # capped crossing tilt; re-aiming it at the loop pushed the
                # g6 crossing to -50 deg and its clearance to 0.2 m)
                n_prev = cross_dir[k - 1]
                if CORNER_ARC:
                    stub, end = _corner(lane[k - 1], n_prev, post_d[k - 1], clr)
                else:
                    stub, end = _stub_on_arc(lane[k - 1], n_prev, post_d[k - 1], clr), None
                anchors[-1] = stub
                if end is not None and np.linalg.norm(end - anchors[-1]) > _DEDUP_M:
                    anchors.append(end)
            if loop_pts:
                for q in loop_pts:
                    if np.linalg.norm(q - anchors[-1]) > _DEDUP_M:
                        anchors.append(q)
                # short straight lead-in along the tilted crossing so the
                # path is STRAIGHT through the opening (curving through it
                # put samples 0.66 m off-centre 0.67 m before the plane)
                pre = ctr - cross_dir[k] * REVERSAL_LEADIN_M
                pre[2] = ctr[2]
            elif np.linalg.norm(clr - anchors[-1]) > _DEDUP_M:
                anchors.append(clr)
        stacked_in = (STACK_U and k > 0
                      and math.hypot(ctr[0] - lane[k - 1][0],
                                     ctr[1] - lane[k - 1][1]) < 1.0
                      and abs(float(ctr[2] - lane[k - 1][2])) > 1.0)
        if stacked_in:
            # Half-ellipse in the vertical plane from the previous (top)
            # gate's post stub down to this (low) gate's pre stub: horizontal
            # semi-axis STACK_U_R_M out along the top gate's exit direction,
            # vertical semi-axis half the height difference, tangent to the
            # horizontal stubs at both ends. Sampled so the spline rounds it.
            # The U lies in the LOW gate's approach plane: out along the
            # reverse of its entry direction and back. The top gate's post
            # anchor is re-laid onto that same line (its own crossing may
            # be tilted, g10-top -30 deg: the U built along that tilt ended
            # 1.2 m west of the low gate's pre stub and the spline jogged
            # east through an r 0.85 m fold at the bottom).
            u_dir = -pre_dir / max(float(np.linalg.norm(pre_dir[:2])), 1e-9)
            u_dir = np.array([u_dir[0], u_dir[1], 0.0])
            d0 = pre_d[k]
            z_top = float(anchors[-1][2]); z_low = float(pre[2])
            anchors[-1] = np.array([lane[k - 1][0] + u_dir[0] * d0,
                                    lane[k - 1][1] + u_dir[1] * d0, z_top])
            n_prev = u_dir
            base = lane[k - 1][:2] + n_prev[:2] * d0
            z_mid = 0.5 * (z_top + z_low); a_v = 0.5 * (z_top - z_low)
            for j in range(1, STACK_U_SAMPLES):
                th = math.pi * j / STACK_U_SAMPLES
                q_xy = base + n_prev[:2] * (STACK_U_R_M * math.sin(th))
                q = np.array([q_xy[0], q_xy[1], z_mid + a_v * math.cos(th)])
                if np.linalg.norm(q - anchors[-1]) > _DEDUP_M:
                    anchors.append(q)
        gap = float(np.linalg.norm(pre - anchors[-1]))
        if k > 0 and gap < p.anchor_standoff_m and not stacked_in:
            anchors[-1] = 0.5 * (anchors[-1] + pre)
        elif gap > _DEDUP_M:
            anchors.append(pre)
        center_idx.append(len(anchors))
        anchors.append(ctr)
        anchors.append(post)   # centers/posts always kept: post defines exit
    # Run-out: descend from the final post anchor to the park altitude.
    # Two reshapings were tried here and both reverted: a straight carry-on
    # along the exit normal, and a quarter-arc bending back toward the start
    # gate. Neither earned its keep, and the run-out is after the last scored
    # crossing, so it is left as the simplest thing that works. The reason the
    # drone no longer lurches at the end is the "run-out" ceiling in
    # _speed_profile(), which is a SPEED rule, not a geometry one.
    # Park CLEAR of the final gate's frame, then descend. Parking directly
    # above the last post put the drone down onto g10-low's frame base
    # 2.5 s AFTER a clean 24/24 (crash at t=80.4 vs finish 77.9, run
    # INVALID; ). Push the park point along the final
    # gate's exit heading past the frame half-width (2.7/2) plus margin,
    # so the descent lands in open floor. Post-finish geometry, no scored
    # crossing affected.
    last_e = events[-1]
    exit_n = np.array([math.cos(last_e.heading_rad),
                       math.sin(last_e.heading_rad), 0.0])
    last = anchors[-1]
    park = last + exit_n * (2.7 / 2.0 + 0.9)
    # ...and to the SIDE of the line, away from the next gate on the loop:
    # the 2-lap finish is g0 with g1 only 4.5 m ahead, and the straight
    # run-out descended through g1's frame plane at z 0.8 (referee-geometry
    # scan, 2026-09-09: the only sample within 0.20 m of any frame on the
    # whole plan). A post-finish contact still voids the run.
    bar = np.array([-exit_n[1], exit_n[0], 0.0])
    nxt = events[1] if len(events) > 1 else None
    if nxt is not None:
        to_next = np.array([nxt.x - last_e.x, nxt.y - last_e.y, 0.0])
        side = -1.0 if float(np.dot(bar, to_next)) >= 0 else 1.0
        park = park + bar * side * (2.7 / 2.0 + 0.9)
    park[2] = PARK_ALT_M
    anchors.append(park)

    # EARLY CLIMB: on a leg with a big altitude change, put the interior
    # anchors at the DESTINATION altitude so the climb happens right after
    # the previous gate's exit stub, not in the last 3 m before the gate.
    # race_026: the drone climbed 2.7 m at 2.6 m/s with the throttle pinned
    # while turning onto g10-top and crossed 0.9 m wide of it.
    if EARLY_CLIMB:
        for k in range(1, len(center_idx)):
            z_from = anchors[center_idx[k - 1]][2]
            z_to = anchors[center_idx[k]][2]
            a0, a1 = anchors[center_idx[k - 1]], anchors[center_idx[k]]
            if STACK_U and math.hypot(a1[0] - a0[0], a1[1] - a0[1]) < 1.0:
                continue        # the stacked pair's U keeps its own altitudes
            if abs(z_to - z_from) > 1.0:
                lo, hi = center_idx[k - 1] + 2, center_idx[k]
                if CLIMB_RAMP:
                    lo = center_idx[k - 1] + 1     # the ramp starts AT the gate: the exit stub climbs too
                if CLIMB_RAMP and hi - lo >= 1:
                    # STRAIGHT climb (Brian): height ramps linearly with anchor
                    # distance from the previous gate's exit stub to this
                    # gate's entry stub. The step version snapped every
                    # interior anchor to the destination height, so the spline
                    # ran flat, ramped between two anchors and ran flat again
                    # (the S-shaped climb into g10-top).
                    pts = [anchors[lo - 1]] + [anchors[i] for i in range(lo, hi)]
                    d = np.cumsum([0.0] + [float(np.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1])) for j in range(1, len(pts))])
                    # the entry stub (last interior anchor) sits at the gate height
                    total = d[-1] if d[-1] > 1e-6 else 1.0
                    for j, i in enumerate(range(lo, hi), start=1):
                        anchors[i][2] = z_from + (z_to - z_from) * min(1.0, d[j] / total)
                else:
                    for i in range(lo, hi):
                        anchors[i][2] = z_to
    global _LAST_REVERSAL, _LAST_LABELS
    _LAST_REVERSAL = [bool(r) for r in reversal]
    _LAST_LABELS = [c.label for c in events]
    return np.array(anchors), center_idx


# ---------------------------------------------------------------------------
# Centripetal Catmull-Rom
# ---------------------------------------------------------------------------

def _cr_segment(p0, p1, p2, p3, n: int, alpha: float = 0.5) -> np.ndarray:
    """Barry-Goldman evaluation of one centripetal CR segment p1->p2,
    n points including p1, excluding p2."""
    def knot(ti, pa, pb):
        return ti + max(float(np.linalg.norm(pb - pa)), 1e-6) ** alpha
    t0 = 0.0
    t1 = knot(t0, p0, p1)
    t2 = knot(t1, p1, p2)
    t3 = knot(t2, p2, p3)
    out = np.empty((n, 3))
    for j, t in enumerate(np.linspace(t1, t2, n, endpoint=False)):
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out[j] = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2
    return out


def sample_spline(anchors: np.ndarray, dense_ds: float = _DENSE_DS):
    """Dense-sample the spline through all anchors.

    Returns (dense[N,3], s[N], anchor_dense_idx) where anchor_dense_idx[i]
    is the dense-sample index of anchor i (used to pin event arc positions).
    dense_ds can be coarsened by the line optimizer's inner loop.
    """
    ext = np.vstack([2 * anchors[0] - anchors[1], anchors,
                     2 * anchors[-1] - anchors[-2]])
    dense: List[np.ndarray] = []
    anchor_dense_idx: List[int] = []
    for i in range(1, len(ext) - 2):
        anchor_dense_idx.append(len(dense))
        chord = float(np.linalg.norm(ext[i + 1] - ext[i]))
        n = max(8, int(chord / dense_ds))
        dense.extend(_cr_segment(ext[i - 1], ext[i], ext[i + 1], ext[i + 2], n))
    dense.append(ext[-2])
    anchor_dense_idx.append(len(dense) - 1)
    dense_arr = np.array(dense)
    seg = np.linalg.norm(np.diff(dense_arr, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return dense_arr, s, anchor_dense_idx


def _menger_curvature(P: np.ndarray) -> np.ndarray:
    """kappa = 4*Area/(|a||b||c|) = 2|a x b|/(|a||b||c|) per interior point."""
    k = np.zeros(len(P))
    a = P[1:-1] - P[:-2]
    b = P[2:] - P[1:-1]
    c = P[2:] - P[:-2]
    cross = np.cross(a, b)
    num = 2.0 * np.linalg.norm(cross, axis=1)
    den = (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
           * np.linalg.norm(c, axis=1) + 1e-12)
    k[1:-1] = num / den
    k[0], k[-1] = k[1], k[-2]
    return k


# ---------------------------------------------------------------------------
# Plan
# ---------------------------------------------------------------------------

@dataclass
class Plan:
    s: np.ndarray          # [N] arc length
    pos: np.ndarray        # [N,3]
    vel: np.ndarray        # [N,3] v * tangent
    acc: np.ndarray        # [N,3] feedforward
    t: np.ndarray          # [N] timestamps
    v: np.ndarray          # [N] speed
    v_lim: np.ndarray      # [N] pointwise ceiling (diagnostic)
    tangent: np.ndarray    # [N,3]
    kappa: np.ndarray      # [N] 3D curvature (diagnostic)
    dpsi_ds: np.ndarray    # [N] heading rate per meter (diagnostic)
    dkappa_ds: np.ndarray  # [N] curvature sharpness (diagnostic)
    binding: np.ndarray    # [N] name of the ceiling that set v_lim there
    events: List[dict]     # per crossing: event, lap, label, s, t, v, x/y/z
    meta: dict = field(default_factory=dict)
    yaw: Optional[np.ndarray] = None   # [N] nose heading (rad), see _yaw_profile
    yaw_hold: Optional[np.ndarray] = None

    @property
    def total_s(self) -> float:
        return float(self.t[-1])

    def predicted_lap_times(self, crossings_per_lap: int) -> List[float]:
        """Cumulative time at each lap's final crossing (model prediction)."""
        out = []
        for i in range(crossings_per_lap - 1, len(self.events),
                       crossings_per_lap):
            out.append(self.events[i]["t"])
        return out

    def to_json_dict(self) -> dict:
        return {
            "version": PLAN_VERSION,
            "frame": "sim ENU (MapToSim of course_map)",
            **self.meta,
            "predicted": {
                "total_s": round(self.total_s, 2),
                "note": "model prediction, unverified",
            },
            "events": [
                {**e, "s": round(e["s"], 3), "t": round(e["t"], 3),
                 "v": round(e["v"], 3)}
                for e in self.events
            ],
            "samples": np.column_stack(
                [self.t, self.pos, self.vel, self.acc]).round(4).tolist(),
            # nose heading per sample; the follower flies it when present
            # (older plans without it fall back to the path tangent)
            "yaw": (None if self.yaw is None
                    else np.asarray(self.yaw).round(4).tolist()),
            "yaw_hold": (None if self.yaw_hold is None
                         else [int(b) for b in np.asarray(self.yaw_hold)]),
        }


YAW_CUSP_DEG = 120.0     # tangent turning more than this within YAW_CUSP_WIN_M
YAW_CUSP_WIN_M = 3.0     # of arc is a cusp: hold the nose, do not chase it
YAW_HOLD_AFTER_M = 5.0   # keep the nose held this far past the cusp (the
                         # exit gate of the stacked pair and its stub are
                         # flown backwards), then
YAW_BLEND_M = 5.0        # unwind to the tangent over this arc
YAW_PREFLIP_M = 6.0      # start the 180 flip this far BEFORE the cusp (pre-yaw)
YAW_CUSP_FLIP = True     # Brian: flip the nose 180 deg AT the cusp instead of
                         # holding it (drift backwards through the fold, then
                         # dive nose-first through the low gate); the follower
                         # yaws only while there is throttle to yaw with
_LAST_YAW: list = []     # [yaw, hold] of the last _speed_profile() call


def _yaw_profile(T: np.ndarray, s: np.ndarray, s_min: float = 0.0):
    """Nose heading per sample. Tangent-following everywhere except across
    a CUSP, where the path folds back on itself faster than any nose can
    follow (the stacked pair: through g10-top heading south, stop, back
    north through g10-low). There the nose is HELD on the entry heading -
    the drone flies the exit gate backwards - for YAW_HOLD_AFTER_M past
    the fold, then blended to the tangent over YAW_BLEND_M. One geometric
    rule, no gate names: a cusp is wherever the XY tangent turns more than
    YAW_CUSP_DEG within YAW_CUSP_WIN_M of arc (dot product of the two
    unit tangents, so unwrap artefacts and the near-vertical takeoff
    spiral cannot fake one). A loop (g7: 270 deg over 15 m) never
    qualifies. Returns (yaw [rad, wrapped], hold mask)."""
    n = len(s)
    ds = float(s[1] - s[0]) if n > 1 else 1.0
    w = max(1, int(round(0.5 * YAW_CUSP_WIN_M / ds)))
    txy = np.hypot(T[:, 0], T[:, 1])
    psi = np.arctan2(T[:, 1], T[:, 0])
    # heading is undefined where the path is near-vertical: carry the last
    # defined tangent heading through those samples
    for i in range(1, n):
        if txy[i] <= 0.3:
            psi[i] = psi[i - 1]
    ux, uy = np.cos(psi), np.sin(psi)
    dot = np.ones(n)
    dot[w:n - w] = ux[2 * w:] * ux[:n - 2 * w] + uy[2 * w:] * uy[:n - 2 * w]
    defined = np.zeros(n, dtype=bool)
    defined[w:n - w] = (txy[2 * w:] > 0.3) & (txy[:n - 2 * w] > 0.3)
    cusp = defined & (dot < math.cos(math.radians(YAW_CUSP_DEG)))
    cusp[s < s_min] = False      # the takeoff spiral folds too; not a cusp
    hold = np.zeros(n, dtype=bool)
    yaw = psi.copy()
    i = 0
    while i < n:
        if not cusp[i]:
            i += 1
            continue
        j = i
        while j < n and cusp[j]:
            j += 1
        held = psi[max(i - 1, 0)]
        if YAW_CUSP_FLIP:
            held = held + math.pi
            # PRE-YAW (Brian): the nose is already reversed when the drop
            # starts - the flip is done on the approach where there is
            # throttle to yaw with, not in the fall.
            k0 = i
            while k0 > 0 and s[i] - s[k0 - 1] < YAW_PREFLIP_M:
                k0 -= 1
            for kk in range(k0, i):
                yaw[kk] = held
                hold[kk] = True
        k = i
        while k < n and s[k] - s[j - 1] < YAW_HOLD_AFTER_M:
            yaw[k] = held
            hold[k] = True
            k += 1
        j2 = k
        while k < n and s[k] - s[j2] < YAW_BLEND_M:
            f = (s[k] - s[j2]) / YAW_BLEND_M
            d = (psi[k] - held + math.pi) % (2 * math.pi) - math.pi
            yaw[k] = held + f * d
            k += 1
        i = k
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return yaw, hold


THRUST_SHARE = 1.0       # L5 flew 29.65 clean at 1.0 (2026-09-10 batch 6); CALIBRATED (race_044): 1.0 left the altitude loop nothing; was 0.85 for the 42 s run. share of the max horizontal thrust vector the PLAN may
               # use (drag + cornering together); the rest is headroom for the
               # altitude loop and attitude corrections (race_035, see
               # _speed_profile). Terminal speed in the plan drops 9.7 -> 9.0.
BRAKE_DRAG_SHARE = 0.5   # back to the flown value (1.0 tried on feature/rip, the calibrated replay could not follow it). share of a_drag(v) the brake pass may count as
               # free deceleration. Physically all of it is (level off at
               # 9.7 m/s and drag brakes at 25 m/s^2), but collecting it
               # means swinging the thrust vector ~60 deg, and the
               # attitude cannot do that in the 0.15 s the full-share
               # profile allowed: race_031 planned 9.7 -> 7.3 in 1.5 m
               # after g3, the tilt eased 66 -> 62 deg, speed fell at
               # 3 m/s^2 instead of 13, and the drone ran 3 m wide of g4.
BIARC = False  # junction = two tangent-continuous arcs matching BOTH crossing
               # headings exactly (_biarc_points). Measured WORSE than the
               # one-circle junction (+0.2 s, g3->g4 r 2.7 vs 5.5): the
               # spline absorbs a ~10 deg tangent mismatch at a stub for
               # free, and forcing the exact tangent spends radius on it.
               # Kept for a course where a junction needs it.
ARC_SPLIT_MISMATCH = True   # one-circle junction: share the exit/entry
               # tangent mismatch EQUALLY between both stubs instead of
               # matching the exit exactly and squaring up at the entry
               # (g4->g5, 17 deg apart: the entry stub kinked at r 2.9 m /
               # 4.1 m/s two metres before g5; split, each end is 8.5 deg
               # off, which the spline rounds without a visible kink)


def _arc_through(p, t, q, ds):
    """Points on the circle tangent to unit t at p and passing through q,
    from p toward q, every ~ds m, endpoints excluded. Straight if q lies
    on the tangent line."""
    p = np.asarray(p, float); q = np.asarray(q, float); t = np.asarray(t, float)
    nrm = np.array([-t[1], t[0]])
    d = q - p
    den = 2.0 * float(np.dot(d, nrm))
    L = float(np.hypot(d[0], d[1]))
    if abs(den) < 1e-6 * max(L, 1.0):
        n_pts = max(0, int(L / ds))
        return [p + d * (i / (n_pts + 1)) for i in range(1, n_pts + 1)]
    r_s = float(np.dot(d, d)) / den          # signed radius, + = left turn
    sgn = 1.0 if r_s > 0 else -1.0
    r = abs(r_s)
    C = p + r_s * nrm
    th0 = math.atan2(p[1] - C[1], p[0] - C[0])
    th1 = math.atan2(q[1] - C[1], q[0] - C[0])
    dth = wrap_pi(th1 - th0)
    if sgn * dth < 0:
        dth += sgn * 2.0 * math.pi
    n_pts = max(0, int(abs(dth) * r / ds))
    return [np.array([C[0] + r * math.cos(th0 + dth * i / (n_pts + 1)),
                      C[1] + r * math.sin(th0 + dth * i / (n_pts + 1))])
            for i in range(1, n_pts + 1)]


def _biarc_points(p0, t0, p1, t1, z0, z1, ds):
    """BIARC junction: two circular arcs, tangent-continuous at their joint,
    leaving p0 along unit t0 and arriving at p1 along unit t1 EXACTLY.
    Why: a single circle through both stubs can only honour ONE tangent;
    with the g4->g5 crossing angles 17 deg apart it arrived at the g5
    stub 8 deg off and the spline squared up in the last 2 m at r 2.9 m
    (4.1 m/s where the sweep runs 8). Construction: A = p0 + d0 t0,
    B = p1 - d1 t1, joint J = (A+B)/2 with |A-B| = d0 + d1, which makes
    the two arcs meet with a common tangent at J. The split d0:d1 is the
    free parameter; the EQUAL-tangent split dumps all the asymmetry into
    one arc (g4->g5: 16.9 m then 4.5 m), so the split is searched for the
    largest MINIMUM radius - the corner speed is set by the tighter arc.
    Returns the interior XYZ points (z linear in arc fraction), endpoints
    excluded."""
    p0 = np.asarray(p0, float)[:2]; p1 = np.asarray(p1, float)[:2]
    t0 = np.asarray(t0, float)[:2]; t1 = np.asarray(t1, float)[:2]
    t0 = t0 / max(float(np.hypot(*t0)), 1e-9)
    t1 = t1 / max(float(np.hypot(*t1)), 1e-9)
    v = p1 - p0

    def radius(p, t, q):
        """signed: + = left turn, - = right, inf = straight"""
        nrm = np.array([-t[1], t[0]])
        d = q - p
        den = 2.0 * float(np.dot(d, nrm))
        return float("inf") if abs(den) < 1e-9 else float(np.dot(d, d)) / den

    def solve(lam):
        # d0 = lam*D, d1 = (1-lam)*D, |v - D*(lam t0 + (1-lam) t1)| = D
        w = lam * t0 + (1.0 - lam) * t1
        a = float(np.dot(w, w)) - 1.0
        b = -2.0 * float(np.dot(v, w))
        c = float(np.dot(v, v))
        if abs(a) < 1e-9:
            D = c / max(-b, 1e-9)
        else:
            disc = b * b - 4.0 * a * c
            if disc < 0:
                return None
            roots = [(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)]
            pos = [r for r in roots if r > 1e-6]
            if not pos:
                return None
            D = min(pos)
        A = p0 + lam * D * t0
        B = p1 - (1.0 - lam) * D * t1
        J = 0.5 * (A + B)
        r1 = radius(p0, t0, J)
        r2 = -radius(p1, -t1, J)      # second arc traversed J -> p1
        c_shape = (r1 * r2 > 0) or math.isinf(r1) or math.isinf(r2)
        return J, min(abs(r1), abs(r2)), c_shape

    # Prefer C-shaped pairs (both arcs turn the same way): an S-shaped
    # pair can post a larger minimum radius yet be a longer, wigglier
    # path (measured: max-min over all shapes cost +1.8 s on the lap).
    best = None
    for lam in np.linspace(0.05, 0.95, 37):
        r = solve(float(lam))
        if r is None:
            continue
        key = (r[2], r[1])
        if best is None or key > (best[2], best[1]):
            best = r
    if best is None:
        return []
    J = best[0]
    # dense trace of both arcs, then UNIFORM resampling over the whole
    # junction: per-arc sampling left uneven gaps (1.7 / 1.7 / 2.2 m) and
    # the spline rippled through them to r 2.4 m on an r 5.5 m corner
    fine = 0.1
    seg1 = _arc_through(p0, t0, J, fine)
    seg2 = _arc_through(p1, -t1, J, fine)[::-1]     # built backwards from p1
    chain = np.array([p0] + seg1 + [J] + seg2 + [p1])
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(chain, axis=0).T))])
    tot = max(float(cum[-1]), 1e-9)
    n_pts = max(1, int(tot / ds))
    out = []
    for i in range(1, n_pts + 1):
        f = i / (n_pts + 1)
        x = float(np.interp(f * tot, cum, chain[:, 0]))
        y = float(np.interp(f * tot, cum, chain[:, 1]))
        out.append(np.array([x, y, z0 + f * (z1 - z0)]))
    return out


def _speed_profile(P: np.ndarray, s: np.ndarray, cfg: VehicleConfig,
                   centers: np.ndarray):
    lim = cfg.limits
    ds = float(s[1] - s[0])
    n = len(P)

    T = np.gradient(P, s, axis=0)
    T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
    kappa = _menger_curvature(P)
    txy = np.hypot(T[:, 0], T[:, 1])
    psi = np.unwrap(np.arctan2(T[:, 1], T[:, 0]))
    dpsi_ds = np.abs(np.gradient(psi, s))

    # Curvature SHARPNESS |dkappa/ds| on a ~1 m smoothed kappa (Menger on
    # 0.25 m samples is too noisy to differentiate raw).
    win = max(1, int(round(1.0 / ds)) | 1)
    kern = np.ones(win) / win
    kappa_s = np.convolve(kappa, kern, mode="same")
    dkappa_ds = np.abs(np.gradient(kappa_s, s))

    a_lat = cfg.a_lat_planner()
    # Lateral budget shrinks in TIGHT arcs: the follower commanded 20 m/s^2
    # mean in the 2 m g7 loop and the airframe delivered 6.7 (race_010,
    # 2026-09-09) - it went wide and missed g7. Full budget at r >= 4 m,
    # A_LAT_TIGHT at r <= 2 m, linear between. Wide arcs (g0-g6) keep
    # their speed.
    # Curvature NORMAL (3D unit vector toward the centre of curvature). The
    # tight-loop budget below is keyed on the HORIZONTAL curvature only:
    # it exists for the follower running wide on flat loops, and a vertical
    # U through the stacked pair is not that.
    dT = np.gradient(T, s, axis=0)
    n_len = np.linalg.norm(dT, axis=1)
    n_vec = dT / np.maximum(n_len, 1e-9)[:, None]
    n_vec[n_len < 1e-6] = 0.0
    # The ceiling below uses a ~0.75 m smoothed curvature: Menger on 0.25 m
    # samples ripples through the junction arcs and the profile rippled
    # +-0.7 m/s along g4->g5 with it (the follower then brakes and
    # accelerates every metre - Brian: "weird oscillation after g4").
    win_c = max(1, int(round(0.75 / ds)) | 1)
    kappa_c = np.convolve(kappa, np.ones(win_c) / win_c, mode="same")
    kappa_h = kappa_c * np.hypot(n_vec[:, 0], n_vec[:, 1])
    r_here = 1.0 / np.maximum(kappa_h, 1e-6)
    frac_r = np.clip((r_here - R_TIGHT_M) / (R_FULL_M - R_TIGHT_M), 0.0, 1.0)
    # ...and only on LEVEL paths: it exists for the follower running wide
    # on flat loops; in the stack's vertical U "wide" is a harmless extra
    # half metre of travel, and the real limits there are thrust and
    # gravity (below).
    flat_turn = (np.abs(T[:, 2]) < 0.5) & (np.abs(n_vec[:, 2]) < 0.5)
    a_lat_eff = np.where(flat_turn, A_LAT_TIGHT + frac_r * (a_lat - A_LAT_TIGHT), a_lat)
    # DRAG SHARES THE TILT (race_030, 2026-09-09): the ONE horizontal thrust
    # vector, |a| <= a_lat_full at max tilt, has to supply the drag along
    # the path AND the corner accel across it. At 9 m/s drag alone is 21 of
    # the 24 m/s^2, so a 5.5 m corner that this ceiling priced at 9.1 m/s
    # (lateral 15) was physically impossible: the follower pinned at 68 deg
    # from g3 onward and ran 3 m wide of g4. The lateral capacity at speed
    # v is therefore sqrt(a_full^2 - a_drag(v)^2), and the plan may use
    # a_lat_eff/a_full of THAT (the margin stays a share of what is left
    # for cornering, so it still buys tracking room). Solved per sample for
    # the largest v with v^2*kappa <= share * sqrt(a_full^2 - a_drag(v)^2)
    # by bisection (a_drag is the config's general lin+quad law). On a
    # straight this reduces to the terminal speed by itself.
    # THRUST HEADROOM (race_035): the plan is written against the absolute
    # thrust ceiling, and the drone flew the g4->g5 arc with m_max pinned
    # at 1.00, throttle 1600-1740, altitude sagging 0.3 m and 0.6-0.8 m
    # wide until it clipped g5's post. Holding altitude at 67 deg already
    # takes 70% of the motors; the attitude loop's corrections need the
    # rest. So the plan may use THRUST_SHARE of the maximum horizontal
    # thrust vector - drag AND cornering both fit inside that - and the
    # remainder is the altitude loop's and the mixer's.
    # 3D THRUST-VECTOR CEILING (Brian, 2026-09-09 night, the stack): the
    # thrust the motors must supply at speed v is
    #     th = v^2*kappa*n + a_drag(v)*T + g*zhat
    # and it is feasible when (1) th_z >= 0 (a quad cannot push DOWN: over
    # the top of a vertical U gravity alone bends the path, so v^2/r <= g
    # there), (2) |th| <= the thrust ceiling times THRUST_SHARE (pulling
    # out at the bottom of a U: v^2/r <= T - g), and (3) the HORIZONTAL
    # part fits inside the tilt clamp with the cornering margin,
    # |th_xy| <= share * THRUST_SHARE * th_z * tan(max_tilt). On a level
    # path th_z = g and (3) is exactly the old joint drag+cornering
    # ceiling; the old form priced a vertical U as a 0.5 m horizontal loop
    # (1.7 m/s, the 3 s stack), this prices it by thrust and gravity.
    tan_tilt = math.tan(min(cfg.tilt_rad(), math.radians(89.0)))   # 90 deg = no tilt bound; the thrust cap (cond 2) rules
    t_max = float(max(cfg.thrust.curve_acc)) * THRUST_SHARE
    share = a_lat_eff / cfg.a_lat_full()

    def feasible(v_arr):
        ad = np.array([cfg.a_drag(float(x)) for x in v_arr])
        a_curv = (v_arr * v_arr * kappa_c)[:, None] * n_vec
        a_drag_v = ad[:, None] * T
        th = a_curv + a_drag_v
        th[:, 2] += G
        th_z = th[:, 2]
        ok1 = th_z >= 0.0
        ok2 = np.linalg.norm(th, axis=1) <= t_max
        # horizontal budget at this th_z, drag takes its share first, the
        # cornering MARGIN applies to what is left (as before on level paths)
        h_cap = THRUST_SHARE * np.maximum(th_z, 0.0) * tan_tilt
        drag_xy = np.hypot(a_drag_v[:, 0], a_drag_v[:, 1])
        room = np.sqrt(np.maximum(h_cap * h_cap - drag_xy * drag_xy, 0.0))
        curv_xy = np.hypot(a_curv[:, 0], a_curv[:, 1])
        ok3 = (drag_xy <= h_cap) & (curv_xy <= share * room)
        return ok1 & ok2 & ok3

    lo = np.zeros(n)
    hi = np.full(n, float(lim.v_max_mps))
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        ok = feasible(mid)
        lo = np.where(ok, mid, lo)
        hi = np.where(ok, hi, mid)
    # Moving MINIMUM over V_TILT_SMOOTH_M: the junction arcs meet the stubs
    # at slightly different curvature and the raw ceiling ripples +-0.7 m/s
    # every metre through g4->g5; a minimum never raises the ceiling, it
    # just stops the follower braking and accelerating on every ripple.
    v_tilt = lo
    wm = max(1, int(round(V_TILT_SMOOTH_M / ds)) | 1)
    if wm > 1:
        pad = wm // 2
        vp = np.pad(v_tilt, pad, mode="edge")
        v_tilt = np.array([vp[i:i + wm].min() for i in range(n)])
    a_full = cfg.a_lat_full() * THRUST_SHARE   # for a_avail below
    # Named pointwise ceilings; v_lim = elementwise min, and the argmin NAME
    # is kept per sample so reports say WHAT binds, not a guess.
    ceilings = {"v_max": np.full(n, float(lim.v_max_mps))}
    ceilings["tilt/curvature"] = v_tilt

    # Attitude-slew ceiling: a_lat = v^2*kappa, so at steady speed
    # d(a_lat)/dt ~ v^3 * dkappa/ds. Heavy low-pitch builds (8" Archer) are
    # limited by how fast they can ROTATE to a tilt, not by the tilt itself -
    # without this cap, corner ENTRIES are geometrically fine but physically
    # unreachable, and it presents as tracking error that looks like bad
    # gains.
    slew_v = np.cbrt(lim.a_lat_rate_max / np.maximum(dkappa_ds, 1e-6))
    # Slew only binds where the path actually BENDS. On a near-straight
    # lane (kappa < 0.15, r > ~7 m) the spline's millimetric curvature
    # ripple gives dkappa ~0.1 and this ceiling priced phantom corner
    # entries at 3.5-4.3 m/s across the straightened g0-g2 lane
    # ("slowing inside every gate for no reason", ;
    # same guard existed in the pre-baseline planner for the same
    # reason). Real corners (kappa >= 0.3 here) are untouched.
    slew_v[kappa_s < 0.15] = np.inf
    ceilings["accel-slew"] = slew_v

    # Yaw-rate ceiling: the nose tracks the path tangent, dpsi/dt = v*dpsi/ds.
    # Only where the heading is defined (path not near-vertical), and NOT
    # where the yaw profile holds the nose across a cusp (see _yaw_profile:
    # the stacked pair is flown as a stop-and-reverse with the nose kept on
    # the entry heading, so the tangent's 180 deg flip there is not a yaw
    # the drone has to perform - it was pricing the cusp at 1.2 m/s, and in
    # flight the spin it demanded churned the motors and floated the drone
    # 1.5 s at 4.5 m, race_029).
    # first passage of the first crossing (lap 2 passes it again, farther on)
    _d0 = np.linalg.norm(P - centers[0][None, :], axis=1)
    _near0 = np.flatnonzero(_d0 < 1.0)
    s_first = (float(s[_near0[0]]) if len(_near0) else 0.0) + YAW_CUSP_WIN_M
    yaw_prof, yaw_hold = _yaw_profile(T, s, s_min=s_first)
    v_yaw = np.full(n, np.inf)
    yaw_ok = (txy > 0.2) & ~yaw_hold
    v_yaw[yaw_ok] = lim.max_yaw_rate_rps / np.maximum(dpsi_ds[yaw_ok], 1e-6)
    ceilings["yaw-rate"] = v_yaw

    tz = T[:, 2]
    v_slope = np.full(n, np.inf)
    up = tz > 1e-3
    v_slope[up] = lim.vz_up_max / tz[up]
    dn = tz < -1e-3
    v_slope[dn] = np.minimum(v_slope[dn], lim.vz_down_max / (-tz[dn]))
    ceilings["climb/descent"] = v_slope

    # Uniform crossing-window cap (ONE global rule for every gate).
    dmin = np.min(np.linalg.norm(P[:, None, :] - centers[None, :, :], axis=2),
                  axis=1)
    v_gate = np.full(n, np.inf)
    v_gate[dmin < lim.gate_window_m] = lim.v_gate_mps
    ceilings["gate-window"] = v_gate
    # Reversal gates: hold the loop speed THROUGH the gate. Accelerating out
    # of the loop before the plane is what put the follower wide at g7.
    if _LAST_REVERSAL and len(_LAST_REVERSAL) == len(centers):
        v_rev = np.full(n, np.inf)
        for k, is_rev in enumerate(_LAST_REVERSAL):
            if is_rev:
                dk = np.linalg.norm(P[:, :2] - centers[k, :2], axis=1)
                v_rev[dk < REVERSAL_WINDOW_M] = (V_REVERSAL_SWING_MPS
                                                 if REVERSAL_SWING
                                                 else V_REVERSAL_MPS)
        for k, lab in enumerate(_LAST_LABELS):
            if lab in GATE_SPEED_CAP:
                dk = np.linalg.norm(P[:, :2] - centers[k, :2], axis=1)
                v_rev[dk < REVERSAL_WINDOW_M] = np.minimum(v_rev[dk < REVERSAL_WINDOW_M], GATE_SPEED_CAP[lab])
        ceilings["reversal-gate"] = v_rev

    # RUN-OUT: past the FINAL crossing the race is already scored, so there is
    # nothing to gain by accelerating again - and the time-optimal profile
    # otherwise does exactly that, sprinting back up to v_max on the open path
    # before braking hard into the park point (measured: exit the last gate at
    # 1.30 m/s, climb to 3.5, dip to 0.32 at the corner, push back to 1.91,
    # then stop - the drone lurching instead of settling). Cap the ceiling at
    # a braking parabola, v = sqrt(2 * a_brake * distance still to run, so the
    # limit is one the brake pass can actually follow. A LINEAR taper was used
    # here first; it works while the last gate is crossed slowly, but once the
    # crossing sped up to 3.7 m/s a straight line to zero is not a shape any
    # real deceleration can hold, and the profile broke back up again.
    # LAST passage near the final center, not the global argmin: with
    # laps=2 the final crossing (lap-2 g10-low) shares its XYZ with the
    # lap-1 passage, and argmin picked the lap-1 one - so the run-out
    # taper plus the may-only-fall accumulate below governed the ENTIRE
    # second lap (planned lap1 66.6 s vs lap0 32.4 s, flat 1.40 m/s
    # tail, 2026-09-08). Same bug class as the g7 fold-jump: nearest-by-
    # distance on a lap-overlapping path. Written/tested at laps=1.
    _d_last = np.linalg.norm(P - centers[-1][None, :], axis=1)
    i_last = int(np.flatnonzero(_d_last < float(_d_last.min()) + 0.5)[-1])
    runout = np.full(n, np.inf)
    if i_last < n - 1:
        v_at_gate = float(np.min(np.vstack(
            [ceilings[k] for k in ceilings])[:, i_last]))
        runout[i_last:] = np.minimum(
            v_at_gate,
            np.sqrt(2.0 * lim.a_brake_max * np.maximum(s[-1] - s[i_last:], 0.0)))
    ceilings["run-out"] = runout

    names = list(ceilings)
    stack = np.vstack([ceilings[k] for k in names])
    v_lim = stack.min(axis=0)
    binding = np.array(names, dtype=object)[stack.argmin(axis=0)]
    floored = v_lim < cfg.planner.v_floor_mps
    binding[floored] = "v_floor(" + binding[floored] + ")"
    v_lim = np.maximum(v_lim, cfg.planner.v_floor_mps)
    # ... except on the run-out. The floor exists so the drone never stalls
    # mid-course, but after the last gate it is exactly wrong: it held the
    # profile at 0.30 m/s and then cut straight to a dead stop. Re-apply the
    # taper on top of the floor so the speed decays continuously to zero.
    v_lim = np.minimum(v_lim, runout)

    # Two-pass with friction circle.
    def a_avail(budget: float, vi: float, ki: float) -> float:
        # friction circle against the lateral capacity LEFT at this speed
        ad = cfg.a_drag(vi)
        cap = max(0.1, a_lat / cfg.a_lat_full()
                  * math.sqrt(max(a_full * a_full - ad * ad, 0.0)))
        frac = min(1.0, (vi * vi * ki) / cap)
        return max(0.1, budget * math.sqrt(max(0.0, 1.0 - frac * frac)))

    # Gravity along the path: the +T tangent component of gravity is -g*Tz
    # (Tz>0 climbing, <0 descending). Climbing, gravity fights the motor
    # (less accel) but aids braking; descending, it adds free acceleration
    # and eats braking. This is what makes a wingover pay: bleed speed into
    # the climb for free, get it back on the descent - without it the
    # profile sees a climb as only a longer path (measured: wingover made
    # the lap SLOWER until this term was added).
    # Drag (measured, [vehicle] drag_*): the plant loses a_drag(v) against
    # the velocity. Accelerating, the forward budget is the SMALLER of the
    # net-accel cap and what max-tilt thrust has left after drag - the
    # profile then saturates at the terminal speed by itself (measured
    # 9.7 m/s at 68 deg; without this term the plan asked for 15 m/s on
    # the g1-g3 straight and the follower pinned at max tilt, race_029).
    # Braking, drag is free deceleration on top of the brake budget.
    a_thrust = cfg.a_lat_full() * THRUST_SHARE
    Tz = T[:, 2]
    v = v_lim.copy()
    v[0] = 0.0
    for i in range(n - 1):
        budget = max(0.1, min(lim.a_accel_max, a_thrust - cfg.a_drag(v[i])))
        aa = a_avail(budget, v[i], kappa[i]) - G * Tz[i]
        aa = max(0.1, aa)
        v[i + 1] = min(v_lim[i + 1], math.sqrt(v[i] * v[i] + 2 * aa * ds))
    v[-1] = 0.0
    for i in range(n - 1, 0, -1):
        budget = lim.a_brake_max
        if i >= i_last:
            budget *= RUNOUT_BRAKE_FRAC     # settle, do not slam (see above)
        ab = (a_avail(budget, v[i], kappa[i])
              + BRAKE_DRAG_SHARE * cfg.a_drag(v[i]) + G * Tz[i - 1])
        ab = max(0.1, ab)
        v[i - 1] = min(v[i - 1], math.sqrt(v[i] * v[i] + 2 * ab * ds))

    # Belt and braces on the run-out: whatever the ceilings and the two passes
    # worked out, speed after the final crossing may only ever fall. A running
    # minimum can only lower samples, so it can never demand more braking than
    # the pass above already allowed - it just removes any residual bump left
    # by a tight corner on the way to the park point.
    if i_last < n - 1:
        v[i_last:] = np.minimum.accumulate(v[i_last:])

    t = np.zeros(n)
    floor = cfg.planner.v_floor_mps
    for i in range(n - 1):
        t[i + 1] = t[i] + 2 * ds / max(v[i] + v[i + 1], floor)

    vel = v[:, None] * T
    acc = np.gradient(vel, t, axis=0)
    _LAST_YAW[:] = [yaw_prof, yaw_hold]
    return T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel, acc


A_LAT_TIGHT = 36.6    # UNCAPPED (feature/uncapped, 2026-09-10): was 7.0 - ramp off (>= a_lat_full at 75 deg). m/s^2 lateral budget in arcs of radius <= R_TIGHT_M
R_TIGHT_M = 2.0
R_FULL_M = 4.0        # full a_lat_planner() budget from this radius up
ARC_SAMPLE_M = 2.5    # junction-arc anchor spacing (0 = single apex anchor)
SMOOTH_WIN_M = 1.25   # moving-average window on the dense path (0 = off)
SMOOTH_PIN_M = 1.6    # keep the crossing stubs (1.5 m) untouched so the tilt at the gate stays as built


def _smooth_path(P: np.ndarray, sg: np.ndarray, centers: np.ndarray):
    """Moving-average smoothing of the dense path AWAY from the gates.
    The anchor spline has curvature spikes at every stub joint (the
    profile hit 5.3 m/s at g5 on a leg whose arcs are 7 m); a 1.25 m
    window removes the spikes without changing the line. Samples near a
    gate centre are pinned so every crossing stays exactly where the
    anchors put it. The path is re-parametrised to uniform arc length."""
    if SMOOTH_WIN_M <= 0.0 or len(P) < 8:
        return P, sg
    ds = float(sg[1] - sg[0])
    size = max(3, int(round(SMOOTH_WIN_M / ds)) | 1)
    ker = np.ones(size) / size
    pad = size // 2
    Pp = np.pad(P, ((pad, pad), (0, 0)), mode="edge")
    Pm = np.column_stack([np.convolve(Pp[:, k], ker, mode="valid")
                          for k in range(3)])
    d = np.min(np.linalg.norm(P[:, None, :2] - centers[None, :, :2], axis=2),
               axis=1)
    w = np.clip((d - SMOOTH_PIN_M) / 0.5, 0.0, 1.0)
    Q = P + w[:, None] * (Pm - P)
    s2 = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0),
                                                         axis=1))])
    n = len(sg)
    sg2 = np.linspace(0.0, float(s2[-1]), n)
    Q2 = np.column_stack([np.interp(sg2, s2, Q[:, k]) for k in range(3)])
    return Q2, sg2


def plan(cfg: VehicleConfig, course=None) -> Plan:
    if course is None:
        course = course_bridge.load_course(laps=cfg.planner.laps)

    anchors, center_idx = build_anchors(course, cfg)
    dense, s_dense, anchor_dense_idx = sample_spline(anchors)

    total = float(s_dense[-1])
    n = max(2, int(total / cfg.planner.sample_ds_m) + 1)
    sg = np.linspace(0.0, total, n)
    P = np.column_stack([np.interp(sg, s_dense, dense[:, k])
                         for k in range(3)])

    events_list = [course.event(i) for i in range(course.total_events)]
    centers = np.array([[c.x, c.y, c.z] for c in events_list])
    P, sg = _smooth_path(P, sg, centers)

    (T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel,
     acc) = _speed_profile(P, sg, cfg, centers)

    events = []
    for k, c in enumerate(events_list):
        # arc position of the crossing on the FINAL path (smoothing and the
        # arc resampling re-parametrise it; the raw spline index was up to
        # 0.6 m late, which the follower and the replay both consume)
        c_xy = anchors[center_idx[k]][:2]
        s_raw = float(s_dense[anchor_dense_idx[center_idx[k]]])
        win = np.flatnonzero(np.abs(sg - s_raw) < 4.0)   # same lap only
        j_near = int(win[np.argmin(np.linalg.norm(P[win, :2] - c_xy, axis=1))])
        s_ev = float(sg[j_near])
        events.append({
            "event": k,
            "lap": course.lap_of(k),
            "label": c.label,
            "s": s_ev,
            "t": float(np.interp(s_ev, sg, t)),
            "v": float(np.interp(s_ev, sg, v)),
            "x": round(c.x, 3), "y": round(c.y, 3), "z": round(c.z, 3),
            "heading_rad": round(c.heading_rad, 4),
        })

    map_p = course_bridge.map_path()
    meta = {
        "map_source": course.source,
        "map_sha1": hashlib.sha1(map_p.read_bytes()).hexdigest()
        if map_p.exists() else None,
        "config_path": str(cfg.path).replace("\\", "/"),
        "config_sha1": cfg.sha1,
        "params": cfg.raw,
        "laps": course.laps,
        "crossings_per_lap": len(course.crossings),
        "path_length_m": round(float(sg[-1]), 2),
        "frame_violations": frame_violations(P, course),
    }
    yaw_prof, yaw_hold = (_LAST_YAW if _LAST_YAW else (None, None))
    return Plan(s=sg, pos=P, vel=vel, acc=acc, t=t, v=v, v_lim=v_lim,
                tangent=T, kappa=kappa, dpsi_ds=dpsi_ds,
                dkappa_ds=dkappa_ds, binding=binding, events=events,
                meta=meta, yaw=yaw_prof, yaw_hold=yaw_hold)


def frame_violations(P: np.ndarray, course) -> int:
    """Samples of the path that touch any gate's physical frame (the sim
    referee now crashes the run on contact; a plan must be contact-clean)."""
    pq = course_bridge.pq_course()
    n = 0
    for pos in P:
        for g in course.gates:
            if pq.gate_frame_hit(g, pos):
                n += 1
                break
    return n


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def next_numbered(path_pattern: str) -> Path:
    """out/plans/plan_XXX.json-style auto-numbering ('XXX' -> 000, 001...)."""
    i = 0
    while True:
        p = Path(path_pattern.replace("XXX", f"{i:03d}"))
        if not p.exists():
            return p
        i += 1


def write_plan(p: Plan, path: os.PathLike) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(p.to_json_dict(), f)
    return path


def load_plan(path: os.PathLike) -> dict:
    """Load a plan JSON into arrays: keys t, pos, vel, acc, s (recomputed),
    events, meta-ish fields kept as-is."""
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    samples = np.asarray(d["samples"], dtype=float)
    d["t_arr"] = samples[:, 0]
    d["pos"] = samples[:, 1:4]
    d["vel"] = samples[:, 4:7]
    d["acc"] = samples[:, 7:10]
    h = d.get("yaw_hold")
    d["hold_arr"] = None if h is None else np.asarray(h, dtype=bool)
    y = d.get("yaw")
    d["yaw_arr"] = (np.asarray(y, dtype=float)
                    if y is not None and len(y) == len(samples) else None)
    seg = np.linalg.norm(np.diff(d["pos"], axis=0), axis=1)
    d["s_arr"] = np.concatenate([[0.0], np.cumsum(seg)])
    return d


# ---------------------------------------------------------------------------
# Checkpoint report (the step-2 -> step-3 gate)
# ---------------------------------------------------------------------------

def report(p: Plan, baseline_s: Optional[float] = 225.3) -> str:
    per_lap = p.meta["crossings_per_lap"]
    lines = []
    lines.append(f"PLAN  {len(p.events)} events, "
                 f"{p.meta['path_length_m']:.0f} m path, "
                 f"config {p.meta['config_sha1'][:8]}")
    laps = p.predicted_lap_times(per_lap)
    lap_str = "  ".join(
        f"lap{i} {t - (laps[i-1] if i else 0.0):.1f}s" for i, t in enumerate(laps))
    lines.append(f"      predicts total {p.total_s:.1f} s ({lap_str}) "
                 f"- model prediction, unverified"
                 + (f"; baseline {baseline_s:.1f} s" if baseline_s else ""))

    # Speed-profile minimum BETWEEN first and last crossing (excludes the
    # takeoff ramp and final park where v -> 0 by construction).
    s0, s1 = p.events[0]["s"], p.events[-1]["s"]
    mask = (p.s >= s0) & (p.s <= s1)
    idx = np.flatnonzero(mask)
    imin = idx[np.argmin(p.v[idx])]
    s_min, v_min = float(p.s[imin]), float(p.v[imin])
    near = min(p.events, key=lambda e: abs(e["s"] - s_min))
    lines.append(f"CHECK frame contacts: {p.meta.get('frame_violations', '?')} samples "
                 "(MUST be 0 - the referee crashes the run on contact)")
    lines.append(f"CHECK speed-profile minimum: {v_min:.2f} m/s at "
                 f"s={s_min:.1f} m (nearest event: {near['label']}, "
                 f"{s_min - near['s']:+.1f} m along-path)")
    i = imin
    what = ("accel/brake passes" if p.v_lim[i] > p.v[i] + 0.05
            else str(p.binding[i]))
    lines.append(f"      binding constraint there: {what}; "
                 f"kappa={p.kappa[i]:.2f} 1/m, dkappa/ds={p.dkappa_ds[i]:.2f}"
                 f" 1/m^2, dpsi/ds={p.dpsi_ds[i]:.2f} rad/m, "
                 f"v_lim={p.v_lim[i]:.2f} m/s")
    # Where each ceiling class rules the profile (samples between the first
    # and last crossing) - one line to see WHAT this vehicle config is
    # limited by overall.
    from collections import Counter
    counts = Counter(str(b) for b in p.binding[idx])
    total_n = sum(counts.values())
    top = ", ".join(f"{k} {100*c//total_n}%" for k, c in
                    counts.most_common(4))
    lines.append(f"      profile ruled by: {top}")

    lines.append("      per-event crossing speeds (m/s):")
    for e in p.events:
        if e["event"] % per_lap == 0 and e["event"]:
            lines.append("      --- lap boundary ---")
        lines.append(f"        {e['event']:2d} {e['label']:8s} "
                     f"t={e['t']:6.1f}  v={e['v']:4.2f}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=None, help="vehicle.toml path")
    ap.add_argument("--out", default=None,
                    help="plan JSON path (default out/plans/plan_XXX.json)")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    p = plan(cfg)
    print(report(p))

    out = Path(args.out) if args.out else next_numbered(
        str(AIGP_REPO / "out" / "plans" / "plan_XXX.json"))
    write_plan(p, out)
    print(f"FILES plan -> {out}")
    if not args.no_render:
        try:
            from raceline import render_plan
            png = out.with_suffix(".png")
            render_plan.render(p, png)
            print(f"      render -> {png}")
        except Exception as e:   # render is eyeball-only, never blocks a plan
            print(f"      render skipped: {e}")
    return p, out


if __name__ == "__main__":
    main()
