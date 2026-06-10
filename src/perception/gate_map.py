"""
Estimate the racing line from PERCEPTION alone - no ground-truth gate positions, no absolute
position telemetry (spec VADR-TS-002 sec 3.3).

Two pieces, both frame-agnostic so the same code validates offline (fed true pose) and flies live
(fed the integrated LOCAL pose):

  LocalFrame  - integrates body-frame linear velocity (rotated to the world by the orientation quat,
                both ALLOWED telemetry) into a position in a local frame anchored at race start. This
                is dead-reckoning / the "L" in VIO; drift is bounded over a ~20 s lap and, crucially,
                the gates are mapped in the SAME frame, so the drone-to-line geometry the follower
                needs stays consistent even as the whole frame drifts.

  GateMapper  - each camera detection's PnP body pose is rotated+translated into that frame to give a
                world point for the gate, then associated to a running map (nearest cluster) and
                fused robustly (component-wise median over observations -> immune to the single-frame
                PnP range blow-ups the data shows when a gate is very close or far). The map IS the
                racing line: ordered gate positions to spline through.
"""

import math

from common.gate_geometry import quat_to_rotmat


def rotate_body_to_world(quat, v):
    R = quat_to_rotmat(quat)
    return [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]]


class LocalFrame:
    """Dead-reckoned local position from body velocity + orientation. No absolute position used."""

    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]

    def update(self, quat, v_body, dt):
        vw = rotate_body_to_world(quat, v_body)
        self.pos[0] += vw[0] * dt
        self.pos[1] += vw[1] * dt
        self.pos[2] += vw[2] * dt
        return self.pos


def _median(xs):
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

    def observe(self, pose_pos, quat, pnp_body, rng=None):
        """Add one camera detection (its PnP body pose) given the drone pose in the map frame."""
        if rng is None:
            rng = math.sqrt(sum(c * c for c in pnp_body))
        if not (self.trust_min <= rng <= self.trust_max):
            return
        gw = [pose_pos[i] + rotate_body_to_world(quat, pnp_body)[i] for i in range(3)]
        # associate to the nearest existing cluster (by its current median), else start a new one
        best, bd = None, self.assoc_radius
        for cl in self.clusters:
            m = self._med(cl)
            d = math.dist(m, gw)
            if d < bd:
                bd, best = d, cl
        if best is None:
            self.clusters.append({"obs": [gw]})
        else:
            best["obs"].append(gw)

    @staticmethod
    def _med(cl):
        obs = cl["obs"]
        return [_median([o[i] for o in obs]) for i in range(3)]

    def estimate(self, drone_pos=None):
        """Ordered list of gate position estimates (the racing line). Each: {pos, n}. Ordered by
        distance from drone_pos if given (forward order along the course), else by observation count."""
        gates = [{"pos": self._med(cl), "n": len(cl["obs"])}
                 for cl in self.clusters if len(cl["obs"]) >= self.min_obs]
        if drone_pos is not None:
            gates.sort(key=lambda g: math.dist(g["pos"], drone_pos))
        else:
            gates.sort(key=lambda g: -g["n"])
        return gates
