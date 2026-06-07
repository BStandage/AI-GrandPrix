#
# Racing line through the gates: apex-clipped path + a speed profile (brake before
# corners, accelerate through), tracked like an F1 line - NOT flown gate-centre to
# gate-centre.
#
# Three pieces:
#   1) APEX waypoints - each gate is a 2.72 m window, so we don't fly the centre; we
#      shift the waypoint toward the INSIDE of its corner (toward the chord of the
#      previous->next gate), capped inside the opening. Cutting the apex straightens
#      the line and lets us carry speed.
#   2) SPEED PROFILE - curvature kappa at every point -> max corner speed sqrt(a_lat/kappa),
#      then a forward pass (accel limit) and backward pass (brake limit) so speed RAMPS
#      DOWN before a corner and back up after it. This is the "brake before, accelerate
#      through" baked into the reference.
#   3) carrot() - given the drone's position, returns the look-ahead point, the line's
#      tangent direction (yaw to this - it's smooth, no apex sign-flip), the cross-track
#      error, and the TARGET SPEED there.
#
# Everything is NED, the same frame as the gates + odometry (within one run).
#

import math

import numpy as np


def _catmull_rom(knots, samples_per_seg=24):
    knots = np.asarray(knots, dtype=float)
    pts = np.vstack([2 * knots[0] - knots[1], knots, 2 * knots[-1] - knots[-2]])
    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        for s in range(samples_per_seg):
            t = s / samples_per_seg
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                              + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(np.asarray(knots[-1], dtype=float))
    return np.array(out)


def _apex_waypoints(centers, apex_max):
    """Shift each interior waypoint toward the chord of its neighbours (corner-cut),
    capped at apex_max m so it stays inside the gate opening. Endpoints unchanged."""
    wp = [np.asarray(centers[0], dtype=float)]
    for i in range(1, len(centers) - 1):
        a, b, c = (np.asarray(centers[i - 1], float),
                   np.asarray(centers[i], float),
                   np.asarray(centers[i + 1], float))
        chord_mid = 0.5 * (a + c)
        toward = chord_mid - b                 # points to the inside of the corner
        toward[2] = 0.0                         # HORIZONTAL cut only - a vertical shift would
                                               # move the line toward the gate's top/bottom bar
                                               # (it clipped the BOTTOM of gate 0, which sits at
                                               # spawn height while gate 1 is 5 m lower)
        dist = float(np.linalg.norm(toward))
        if dist > 1e-6:
            wp.append(b + toward / dist * min(apex_max, dist))   # b keeps its true altitude
        else:
            wp.append(b)
    wp.append(np.asarray(centers[-1], dtype=float))
    return wp


def _curvature(pts):
    """Menger curvature (1/R) at each sample from 3 consecutive points (3D)."""
    n = len(pts)
    k = np.zeros(n)
    for i in range(1, n - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            continue
        area = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0))
        k[i] = 4.0 * area / (a * b * c)
    k[0], k[-1] = k[1], k[-2]
    # light smoothing
    return np.convolve(k, np.ones(5) / 5.0, mode="same")


def _speed_profile(s, kappa, v_max, a_lat, a_accel, a_brake):
    """Corner-speed limit then forward (accel) + backward (brake) passes."""
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(kappa, 1e-4)))
    ds = np.diff(s)
    v[0] = min(v[0], 1.5)                       # start from near standstill
    for i in range(1, len(v)):                  # forward: limit acceleration
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_accel * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):         # backward: limit braking (brake EARLY)
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_brake * ds[i]))
    return v


class Trajectory:
    def __init__(self, gates, start=(0.0, 0.0, 0.0), runout=12.0,
                 apex_max=1.0, v_max=7.0, a_lat=9.0, a_accel=6.0, a_brake=7.0):
        ordered = sorted(gates, key=lambda g: g.get("gate_id", 0))
        centers = [np.asarray(start, float)] + [np.asarray(g["position_ned"], float) for g in ordered]
        last, prev = centers[-1], centers[-2]
        d = (last - prev).copy()
        d[2] = 0.0   # runout stays LEVEL at the last gate's altitude. The course descends into the
                     # final gates and the floor is right below them; a runout that kept the descent
                     # dove the line BELOW the last gate into the obstacle floor (session 151125).
        centers.append(last + d / (np.linalg.norm(d) or 1.0) * runout)

        self.gate_centers = [np.asarray(g["position_ned"], float) for g in ordered]
        wp = _apex_waypoints(centers, apex_max)
        self.pts = _catmull_rom(wp)
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.s[-1])
        self.kappa = _curvature(self.pts)
        self.a_lat = a_lat
        self.vprof = _speed_profile(self.s, self.kappa, v_max, a_lat, a_accel, a_brake)

    def carrot(self, pos, lookahead):
        """Return (carrot_ned, progress_frac, cross_track_m, tangent_ned, v_target)."""
        pos = np.asarray(pos, dtype=float)
        n = len(self.pts)
        # nearest point by HORIZONTAL distance, NOT 3D. On a descending line the closest
        # 3D point to a drone flying below it is one further AHEAD and lower down the slope,
        # so near_z ~= drone_z and the altitude error reads ~0 -> it never climbs back and
        # rides below every gate. Horizontal-nearest gives the line's altitude directly
        # above/below us, so the vertical error is real.
        dh = np.linalg.norm(self.pts[:, :2] - pos[:2], axis=1)
        i = int(np.argmin(dh))
        j = min(int(np.searchsorted(self.s, self.s[i] + lookahead)), n - 1)

        tan3 = self.pts[min(j + 1, n - 1)] - self.pts[max(j - 1, 0)]
        tn = float(np.linalg.norm(tan3))
        tangent = tan3 / tn if tn > 1e-6 else np.array([1.0, 0.0, 0.0])
        # target speed: the MIN of the profile from here to the carrot, so we are
        # already slowing for a corner that begins within the look-ahead.
        v_target = float(np.min(self.vprof[i:j + 1])) if j >= i else float(self.vprof[i])
        # NEAREST-point altitude + local slope, for VERTICAL tracking. Using the nearest
        # point (where we actually are), NOT the look-ahead carrot, so we don't descend
        # early/below the line on the drops (that read as "too low").
        ntan = self.pts[min(i + 1, n - 1)] - self.pts[max(i - 1, 0)]
        nn = float(np.linalg.norm(ntan))
        near_z = float(self.pts[i][2])
        near_slope_down = float(ntan[2] / nn) if nn > 1e-6 else 0.0
        # tangent at the NEAREST point (where the drone IS). The velocity reference must use
        # THIS, not the look-ahead carrot tangent: on a corner the carrot is already past the
        # apex and its tangent points into the next straight, which steers the drone inside
        # EARLY and cuts the corner into the inner gate post (gate 1's V). Nearest tangent keeps
        # it heading along the line at its current spot until it actually reaches the apex.
        near_tan = (ntan / nn) if nn > 1e-6 else np.array([1.0, 0.0, 0.0])
        # CENTRIPETAL acceleration feedforward: a = v^2 * d2r/ds2 at the nearest point. A purely
        # reactive tracker only banks once cross-track error has built up, so it LAGS the line on
        # a fast curve (rode ~0.8 m inside the gate-3 chicane and clipped the apex). Feeding the
        # turn's own acceleration forward makes it bank INTO the corner. Zero on straights.
        # SYMMETRIC stencil only (k points each side). Near the ends k shrinks to 0 -> FF=0,
        # instead of a lopsided il==i stencil that degenerates into a first difference and spat
        # out a bogus ~6 m/s2 spike at the spawn that slammed the roll +-35 deg (session 144653).
        k = min(2, i, n - 1 - i)                # +-k points (~k*2 m); 0 at the very start/end
        if k >= 1:
            ds_c = 0.5 * float(self.s[i + k] - self.s[i - k])
            d2r = ((self.pts[i + k] - 2.0 * self.pts[i] + self.pts[i - k]) / (ds_c * ds_c)
                   if ds_c > 1e-6 else np.zeros(3))
        else:
            d2r = np.zeros(3)
        accel_ff = float(self.vprof[i]) ** 2 * d2r
        # clamp to the line's design lateral accel (a_lat) so a discrete-curvature overshoot
        # can't command more bank than the airframe / the line was planned for.
        aff_h = math.hypot(accel_ff[0], accel_ff[1])
        if aff_h > self.a_lat:
            accel_ff = accel_ff * (self.a_lat / aff_h)
        xtrack = float(np.linalg.norm(self.pts[i] - pos))   # 3D off-line distance (incl. vertical)
        return (self.pts[j], self.s[i] / max(self.length, 1e-6), xtrack,
                tangent, v_target, near_z, near_slope_down, self.pts[i], near_tan, accel_ff)
