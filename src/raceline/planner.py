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
STACKED_STANDOFF_M = 2.0
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
# radius) minus measured tracking error on a straightened lane (<= 0.16 m,
# flight 2026-09-03). The g0-g3 stagger is ~1 m; crossing at the hole
# EDGES instead of the centers turns the r~1.5-4 m interpolation wiggle
# into r~15+ m arcs, which is what lets those gates run near v_max
# (Brian, 2026-09-08: "we can go probably full speed at least g0-g5;
# we are slowing when crossing every gate").
_LANE_USE_M = 0.40
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

    # DISABLED (Brian, 2026-09-08): shifting crossings toward a straight
    # lane trades the centered ±0.75 margin for straightness, and the
    # follower's attitude-lag OVERSHOOT on the g1->g2 swing-back then ate
    # the margin and clipped g2's +0.41 post (flown +0.53 vs planned
    # +0.06). Megan's centred crossings pass this leg verified 12/12.
    # Get a COMPLETE 2-lap on the board with proven geometry first;
    # re-earn g0-g3 speed later WITH a follower that doesn't overshoot
    # (needs the tune sprint / a lookahead fix, not a planner nudge).
    # The apex-arc sweep in build_anchors is independent and stays on.
    return pts, [None] * n

    def _joins(a, b):
        # aligned headings AND travel along them: the lap-1 g10-low ->
        # lap-2 g0 leg matches headings (both ~north) but travels ~34
        # deg off them - a dogleg, not a lane. Grouping it dragged the
        # lap-2 g0-g3 diagonal 8 m east and crawled lap-2 g0/g1 at
        # 3.05-3.11 vs lap-1's 5.2 (Brian, 2026-09-08).
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
            # swing east that overshot g2's +0.41 post and crashed;
            # Brian, 2026-09-08: "g0-g3 is essentially straight, the left
            # turn after g0 is wrong"). Endpoints are pinned so the chord
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
    # Brian, 2026-09-08). The lane runs within _LANE_ALIGN_RAD of each
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

    # Junction arc parameters (k -> k+1): (radius, turn sign) for bent
    # non-reversal junctions, else None. Used twice: the apex anchor
    # mid-leg, and ROTATING each gate's pre/post stub onto the arc - a
    # straight 2 m stub along the gate heading forces the spline to
    # re-bend hard right after it (the residual dips after g3 / around
    # g4; Brian, 2026-09-08: "do we really need to be slowing there?").
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
        phi = 0.5 * (abs(wrap_pi(chord_ang - events[k].heading_rad))
                     + abs(wrap_pi(events[k + 1].heading_rad - chord_ang)))
        if phi <= math.radians(10.0):
            continue
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

    anchors: List[np.ndarray] = [np.array([0.0, 0.0, p.takeoff_alt_m])]
    center_idx: List[int] = []
    for k, c in enumerate(events):
        n = np.array([math.cos(c.heading_rad), math.sin(c.heading_rad), 0.0])
        ctr = lane[k]
        # In an aligned run, the approach/exit anchors follow the LANE,
        # not the gate normal (see _lane_points). Reversal gates keep the
        # normal - their clearance routing depends on it.
        if lane_dir[k] is not None and not reversal[k]:
            n_anchor = np.array([lane_dir[k][0], lane_dir[k][1], 0.0])
        else:
            n_anchor = n
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
        # headings, worth ~5.4 m/s at the current lateral budget;
        # Brian, 2026-09-08: "why are the g3-g4 and g4-g5 turns so
        # straight?"). Add the tangent arc's apex between the two
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
                if phi > math.radians(10.0):
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
                    if np.linalg.norm(apex - anchors[-1]) > _DEDUP_M:
                        anchors.append(apex)
        if k > 0 and reversal[k]:
            bar = np.array([-math.sin(c.heading_rad),
                            math.cos(c.heading_rad), 0.0])
            side = math.copysign(1.0, float(np.dot(bar, anchors[-1] - ctr)))
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
            clr[2] = anchors[-1][2]
            if np.linalg.norm(clr - anchors[-1]) > _DEDUP_M:
                anchors.append(clr)
        gap = float(np.linalg.norm(pre - anchors[-1]))
        if k > 0 and gap < p.anchor_standoff_m:
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
    # INVALID; Brian, 2026-09-08). Push the park point along the final
    # gate's exit heading past the frame half-width (2.7/2) plus margin,
    # so the descent lands in open floor. Post-finish geometry, no scored
    # crossing affected.
    last_e = events[-1]
    exit_n = np.array([math.cos(last_e.heading_rad),
                       math.sin(last_e.heading_rad), 0.0])
    last = anchors[-1]
    park = last + exit_n * (2.7 / 2.0 + 0.9)
    park[2] = PARK_ALT_M
    anchors.append(park)

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
        }


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
    # Named pointwise ceilings; v_lim = elementwise min, and the argmin NAME
    # is kept per sample so reports say WHAT binds, not a guess.
    ceilings = {"v_max": np.full(n, float(lim.v_max_mps))}
    ceilings["tilt/curvature"] = np.sqrt(a_lat / np.maximum(kappa, 1e-6))

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
    # ("slowing inside every gate for no reason", Brian 2026-09-08;
    # same guard existed in the pre-baseline planner for the same
    # reason). Real corners (kappa >= 0.3 here) are untouched.
    slew_v[kappa_s < 0.15] = np.inf
    ceilings["accel-slew"] = slew_v

    # Yaw-rate ceiling: the nose tracks the path tangent, dpsi/dt = v*dpsi/ds.
    # Only where the heading is defined (path not near-vertical).
    v_yaw = np.full(n, np.inf)
    yaw_ok = txy > 0.2
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
        frac = min(1.0, (vi * vi * ki) / a_lat)
        return max(0.1, budget * math.sqrt(max(0.0, 1.0 - frac * frac)))

    v = v_lim.copy()
    v[0] = 0.0
    for i in range(n - 1):
        aa = a_avail(lim.a_accel_max, v[i], kappa[i])
        v[i + 1] = min(v_lim[i + 1], math.sqrt(v[i] * v[i] + 2 * aa * ds))
    v[-1] = 0.0
    for i in range(n - 1, 0, -1):
        budget = lim.a_brake_max
        if i >= i_last:
            budget *= RUNOUT_BRAKE_FRAC     # settle, do not slam (see above)
        ab = a_avail(budget, v[i], kappa[i])
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
    return T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel, acc


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

    (T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel,
     acc) = _speed_profile(P, sg, cfg, centers)

    events = []
    for k, c in enumerate(events_list):
        s_ev = float(s_dense[anchor_dense_idx[center_idx[k]]])
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
        "config_path": str(cfg.path),
        "config_sha1": cfg.sha1,
        "params": cfg.raw,
        "laps": course.laps,
        "crossings_per_lap": len(course.crossings),
        "path_length_m": round(float(sg[-1]), 2),
        "frame_violations": frame_violations(P, course),
    }
    return Plan(s=sg, pos=P, vel=vel, acc=acc, t=t, v=v, v_lim=v_lim,
                tangent=T, kappa=kappa, dpsi_ds=dpsi_ds,
                dkappa_ds=dkappa_ds, binding=binding, events=events,
                meta=meta)


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
