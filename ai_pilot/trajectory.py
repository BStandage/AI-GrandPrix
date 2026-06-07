#
# Racing-line trajectory: a smooth spline through the gate centres, plus a
# carrot-lookahead lookup for path following.
#
# WHY: the pursuit pilot points straight at the active gate and slows at every
# corner -> it clears the course but is slow (34 s). A drone that follows a smooth
# pre-planned LINE through the gates can carry speed THROUGH the corners. This module
# builds that line (Catmull-Rom through the gate centres, which passes exactly through
# every gate) and, given the drone's position, returns a "carrot" point a fixed
# distance ahead along the line to steer toward.
#
# This is the geometric (constant-arc) racing line - the foundation. The time-optimal
# version (CPC, arXiv:2108.04537) replaces the *reference* with a dynamically-optimal
# trajectory but is tracked the same way.
#
# Frame: everything is NED, the same frame as the gates (position_ned) and the
# drone's odometry - they share an origin (verified in flight).
#

import numpy as np


def _catmull_rom(knots, samples_per_seg=24):
    """Catmull-Rom spline (passes through every knot). Returns sampled points."""
    knots = np.asarray(knots, dtype=float)
    # phantom endpoints (reflect the first/last segment) so the spline reaches the ends
    pts = np.vstack([2 * knots[0] - knots[1], knots, 2 * knots[-1] - knots[-2]])
    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        for s in range(samples_per_seg):
            t = s / samples_per_seg
            t2, t3 = t * t, t * t * t
            out.append(0.5 * (
                (2 * p1)
                + (-p0 + p2) * t
                + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
            ))
    out.append(np.asarray(knots[-1], dtype=float))
    return np.array(out)


class Trajectory:
    """Smooth racing line through the gates, with a carrot-lookahead query."""

    def __init__(self, gates, start=(0.0, 0.0, 0.0), runout=10.0):
        # knots: spawn, then every gate centre in order, then a short run-out past
        # the last gate so the carrot always has line ahead of it to chase.
        ordered = sorted(gates, key=lambda g: g.get("gate_id", 0))
        knots = [np.asarray(start, dtype=float)]
        knots += [np.asarray(g["position_ned"], dtype=float) for g in ordered]
        last, prev = knots[-1], knots[-2]
        d = last - prev
        n = float(np.linalg.norm(d)) or 1.0
        knots.append(last + d / n * runout)

        self.pts = _catmull_rom(knots)
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])   # cumulative arc length
        self.length = float(self.s[-1])

    def carrot(self, pos, lookahead):
        """Return (carrot_point_ned, progress_frac, cross_track_m).

        carrot_point : a point `lookahead` metres further along the line than the
                       drone's nearest projection - the thing to steer toward.
        progress_frac: 0..1 how far along the whole line we are (for finish/▒readout).
        cross_track  : metres the drone is off the line right now.
        """
        pos = np.asarray(pos, dtype=float)
        d = np.linalg.norm(self.pts - pos, axis=1)
        i = int(np.argmin(d))
        target_s = self.s[i] + lookahead
        j = int(np.searchsorted(self.s, target_s))
        j = min(j, len(self.pts) - 1)
        return self.pts[j], self.s[i] / max(self.length, 1e-6), float(d[i])
