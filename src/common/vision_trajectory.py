"""
VisionTrajectory: incremental Catmull-Rom racing line built from camera-estimated gate positions.

Unlike Trajectory (which requires a complete gate list at init), VisionTrajectory accumulates
world-frame gate position estimates produced by gate_detector.estimate_gate_world_position()
and maintains a rolling spline through them.  Once at least two distinct gates are registered
it exposes the same carrot() / track_point() interface as Trajectory, allowing vision_pilot
to switch from pixel-servo to world-frame acceleration tracking (ported from oracle_pilot).

Gate deduplication: detections within GATE_MERGE_DIST m (horizontal) are merged via running
mean so per-frame PnP noise never adds spurious duplicate waypoints to the spline.  The start
knot is always the drone's current position (supplied on register_gate calls), so the carrot
stays ahead.  Spline rebuilds on every new gate and every 5th refinement of an existing one.
"""

import math

import numpy as np

try:
    from common.trajectory import _catmull_rom, _curvature, _speed_profile
except ImportError:
    from trajectory import _catmull_rom, _curvature, _speed_profile

# Gate deduplication and buffer management
GATE_MERGE_DIST = 2.0   # m horizontal: closer than this = same gate, running-mean update
MAX_GATES = 8           # rolling buffer cap; oldest gates pruned from front once exceeded

# Default speed profile — more conservative than oracle_pilot (camera range is uncertain)
VT_V_MAX = 6.0          # m/s line speed cap
VT_A_LAT = 9.0          # m/s² design lateral accel (corner speed limit in _speed_profile)
VT_A_ACCEL = 5.0        # m/s² forward accel limit
VT_A_BRAKE = 6.0        # m/s² braking limit


class VisionTrajectory:
    """Incremental Catmull-Rom spline, built on-the-fly from vision-estimated gate positions.

    Usage (inside update_vision_control):
        vtraj = data.setdefault("_vision_traj", VisionTrajectory())
        # each tick when a gate detection is smoothed:
        vtraj.register_gate(world_gate_xyz, drone_pos=pos)
        # once ready:
        result = vtraj.get_tracking_commands(pos, lookahead=5.5)
        if result:
            carrot, prog, xtrack, tangent, v_target,
            near_z, near_slope_down, near_pt, near_tan, accel_ff = result
    """

    def __init__(self, v_max=VT_V_MAX, a_lat=VT_A_LAT,
                 a_accel=VT_A_ACCEL, a_brake=VT_A_BRAKE):
        self.v_max = v_max
        self.a_lat = a_lat
        self.a_accel = a_accel
        self.a_brake = a_brake
        self._gate_pts: list = []       # list of np.array(3,) NED world estimates
        self._gate_weights: list = []   # observation count per gate (running-mean denominator)
        # Spline state — None until first build
        self.pts = None     # (N, 3) Catmull-Rom sample array
        self.s = None       # (N,)  cumulative arc-length
        self.vprof = None   # (N,)  speed profile
        self.kappa = None   # (N,)  curvature
        self.length = 0.0   # total arc length (m)

    # ------------------------------------------------------------------
    # Public API — mirrors Trajectory.carrot() / track_point() interface
    # ------------------------------------------------------------------

    def ready(self) -> bool:
        """True when >= 2 distinct gates are registered and the spline exists."""
        return len(self._gate_pts) >= 2 and self.pts is not None

    def register_gate(self, world_pos, drone_pos=None) -> bool:
        """Add or refine a gate estimate; rebuild spline on new gates.

        Args:
            world_pos: NED position estimate for the detected gate  (array-like (3,))
            drone_pos: current drone NED position used as spline start knot.
                       Pass None if unavailable; spline deferred to next call with pos.
        Returns:
            True  when the spline was (re)built this call.
            False when this was a same-gate refinement with no rebuild.
        """
        world_pos = np.asarray(world_pos, dtype=float)
        # Merge with an existing estimate that's within GATE_MERGE_DIST (same physical gate)
        for i, gp in enumerate(self._gate_pts):
            if np.linalg.norm(gp[:2] - world_pos[:2]) < GATE_MERGE_DIST:
                n = self._gate_weights[i]
                self._gate_pts[i] = (gp * n + world_pos) / (n + 1)
                self._gate_weights[i] = n + 1
                # Periodic rebuild (every 5 refinements) so improving estimates propagate to path
                if (n + 1) % 5 == 0 and drone_pos is not None:
                    self._build(drone_pos)
                    return True
                return False
        # New gate — append, prune oldest if over cap
        self._gate_pts.append(world_pos.copy())
        self._gate_weights.append(1)
        if len(self._gate_pts) > MAX_GATES:
            self._gate_pts.pop(0)
            self._gate_weights.pop(0)
        if drone_pos is not None and len(self._gate_pts) >= 2:
            self._build(drone_pos)
            return True
        return False

    def track_point(self, pos, lookahead: float):
        """Point on the line `lookahead` arc-metres ahead of the nearest horizontal point.
        Mirrors Trajectory.track_point() for pure-pursuit look-ahead in vision_pilot.
        """
        if self.pts is None:
            return np.asarray(pos, dtype=float)
        pos = np.asarray(pos, dtype=float)
        n = len(self.pts)
        dh = np.linalg.norm(self.pts[:, :2] - pos[:2], axis=1)
        i = int(np.argmin(dh))
        j = min(int(np.searchsorted(self.s, self.s[i] + lookahead)), n - 1)
        return self.pts[j]

    def get_tracking_commands(self, pos, lookahead: float = 5.5):
        """Mirror Trajectory.carrot() for drop-in use in oracle-style world-frame control.

        Returns tuple matching oracle_pilot.py's carrot() unpacking exactly:
            (carrot, prog, xtrack, tangent, v_target,
             near_z, near_slope_down, near_pt, near_tan, accel_ff)
        or None if the spline is not yet ready (< 2 gates registered).
        """
        if not self.ready():
            return None
        pos = np.asarray(pos, dtype=float)
        n = len(self.pts)
        # Horizontal-nearest: avoids altitude aliasing on descending sections
        # (same design rationale as Trajectory.carrot)
        dh = np.linalg.norm(self.pts[:, :2] - pos[:2], axis=1)
        i = int(np.argmin(dh))
        j = min(int(np.searchsorted(self.s, self.s[i] + lookahead)), n - 1)

        # Carrot tangent (at the look-ahead point, for yaw direction)
        tan3 = self.pts[min(j + 1, n - 1)] - self.pts[max(j - 1, 0)]
        tn = float(np.linalg.norm(tan3))
        tangent = tan3 / tn if tn > 1e-6 else np.array([1.0, 0.0, 0.0])

        # Speed: min of the profile from nearest to carrot → already braking for upcoming corners
        v_target = float(np.min(self.vprof[i:j + 1])) if j >= i else float(self.vprof[i])

        # Nearest-point altitude + local slope (vertical tracking uses nearest, NOT carrot, to
        # avoid premature descent into drops that begin within the look-ahead)
        ntan = self.pts[min(i + 1, n - 1)] - self.pts[max(i - 1, 0)]
        nn = float(np.linalg.norm(ntan))
        near_z = float(self.pts[i][2])
        near_slope_down = float(ntan[2] / nn) if nn > 1e-6 else 0.0
        near_tan = ntan / nn if nn > 1e-6 else np.array([1.0, 0.0, 0.0])

        # Centripetal feedforward: symmetric stencil only (k=0 at ends → FF=0 to avoid
        # lopsided first-difference spikes, same guard as Trajectory.carrot)
        k = min(2, i, n - 1 - i)
        if k >= 1:
            ds_c = 0.5 * float(self.s[i + k] - self.s[i - k])
            d2r = ((self.pts[i + k] - 2.0 * self.pts[i] + self.pts[i - k]) / (ds_c * ds_c)
                   if ds_c > 1e-6 else np.zeros(3))
        else:
            d2r = np.zeros(3)
        accel_ff = float(self.vprof[i]) ** 2 * d2r
        # Clamp to design lateral accel so curvature discretisation spikes can't over-bank
        aff_h = math.hypot(float(accel_ff[0]), float(accel_ff[1]))
        if aff_h > self.a_lat:
            accel_ff = accel_ff * (self.a_lat / aff_h)

        xtrack = float(np.linalg.norm(self.pts[i] - pos))
        prog = float(self.s[i]) / max(self.length, 1e-6)

        return (self.pts[j], prog, xtrack, tangent, v_target,
                near_z, near_slope_down, self.pts[i], near_tan, accel_ff)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _build(self, drone_pos):
        """(Re)build the Catmull-Rom spline from drone_pos through all registered gate estimates."""
        drone_pos = np.asarray(drone_pos, dtype=float)
        knots = [drone_pos] + list(self._gate_pts)
        pts = _catmull_rom(knots, samples_per_seg=20)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s_arr = np.concatenate([[0.0], np.cumsum(seg)])
        kappa = _curvature(pts)
        vprof = _speed_profile(s_arr, kappa, self.v_max, self.a_lat, self.a_accel, self.a_brake)
        self.pts = pts
        self.s = s_arr
        self.kappa = kappa
        self.vprof = vprof
        self.length = float(s_arr[-1])
        print(f"[vision_traj] spline rebuilt: {self.length:.0f} m through "
              f"{len(self._gate_pts)} gates", flush=True)