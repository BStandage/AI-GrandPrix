"""
Estimate the racing line from perception alone, with no ground-truth gate positions and no absolute
position telemetry (spec VADR-TS-002 sec 3.3).

Think of it like the racing line an F1 car takes. We want the best path through the gates, not to
naively clip the center of each one.

This module is used by the offline validator (analysis/vision_map.py), not by the live pilot. The
pilot does its own dead reckoning and gate tracking inline, so changes here do not affect flight.

Both pieces work in a coordinate frame (a position reference, not a camera image frame) and do not
care which one. The validator uses that to run them two ways on one logged session: fed the true
drone pose, and fed the dead-reckoned pose. If it fails with the true pose, the bug is in perception.
If it works with the true pose but fails with dead reckoning, the bug is in the dead reckoning.
Splitting those two error sources apart is the whole point.

The two pieces:

  LocalFrame   integrates body-frame velocity (rotated to the world by the orientation quat, both
               allowed telemetry) into a position anchored at race start. This is dead reckoning.
               Drift is bounded over a ~20 s lap, and the gates are mapped in the SAME frame, so the
               drone-to-line geometry the follower needs stays consistent even as the frame drifts.

  GateMapper   rotates each detection's PnP body pose into that frame to get a world point for the
               gate, associates it to a running map (nearest cluster), and fuses each cluster by a
               component-wise median. The median is immune to the single-frame PnP range blow-ups the
               data shows when a gate is very close or far. The map is the racing line: ordered gate
               positions to spline through.
"""

import math

from common.gate_geometry import quat_to_rotmat


def rotate_body_to_world(quat, v):
    """Rotate a 3-vector from body coordinates to world coordinates using the orientation quaternion.
    The drone reports velocity and PnP poses in body coordinates, so this puts them in the map frame."""
    R = quat_to_rotmat(quat)
    return [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]]


class LocalFrame:
    """Dead-reckoned local position from body velocity + orientation. No absolute position used."""

    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]

    def update(self, quat, v_body, dt):
        """Advance the position by one step. Rotates body velocity into the world, adds velocity * dt,
        and returns the updated position."""
        vw = rotate_body_to_world(quat, v_body)
        self.pos[0] += vw[0] * dt
        self.pos[1] += vw[1] * dt
        self.pos[2] += vw[2] * dt
        return self.pos


def _median(xs):
    """Median of a list of numbers (0.0 if the list is empty)."""
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


class GateMapper:
    """Accumulate per-detection world points into a small set of gate position estimates."""

    def __init__(self, assoc_radius=6.0, min_obs=4, trust_min=6.0, trust_max=32.0):
        self.assoc_radius = assoc_radius      # m: an observation within this of a cluster is the same gate
        self.min_obs = min_obs                # ignore clusters with fewer observations (noise)
        self.trust_min = trust_min            # only fuse detections whose range is in the trusted band
        self.trust_max = trust_max            # (outside it the pinhole/PnP range is a known blow-up)
        self.clusters = []                    # each: {"obs": [ [x,y,z], ... ]}

    def observe(self, pose_pos, quat, pnp_body, range_m=None):
        """Add one camera detection (its PnP body pose) given the drone pose in the map frame."""
        if range_m is None:
            range_m = math.sqrt(sum(c * c for c in pnp_body))
        if not (self.trust_min <= range_m <= self.trust_max):
            return
        world_v = rotate_body_to_world(quat, pnp_body)
        gate_world = [pose_pos[i] + world_v[i] for i in range(3)]
        # associate to the nearest existing cluster (by its current median), else start a new one
        best, bd = None, self.assoc_radius
        for cl in self.clusters:
            d = math.dist(self._cluster_median(cl), gate_world)
            if d < bd:
                bd, best = d, cl
        if best is None:
            self.clusters.append({"obs": [gate_world]})
        else:
            best["obs"].append(gate_world)

    @staticmethod
    def _cluster_median(cl):
        """The cluster's gate position: the component-wise median over its observed world points."""
        obs = cl["obs"]
        return [_median([o[i] for o in obs]) for i in range(3)]

    def estimate(self, drone_pos=None):
        """Ordered list of gate position estimates (the racing line). Each: {pos, n}. Ordered by
        distance from drone_pos if given (forward order along the course), else by observation count."""
        gates = [{"pos": self._cluster_median(cl), "n": len(cl["obs"])}
                 for cl in self.clusters if len(cl["obs"]) >= self.min_obs]
        if drone_pos is not None:
            gates.sort(key=lambda g: math.dist(g["pos"], drone_pos))
        else:
            gates.sort(key=lambda g: -g["n"])
        return gates
