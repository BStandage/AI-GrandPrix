"""
Camera/perception model for the offline sim. Projects the true gates into the image with the EXACT
spec camera model (perception.vision_pose.project_body_to_offset), producing the same detection
dicts the real detector emits. Optional noise/dropout is CALIBRATED from real flight logs
(sim.perception_noise) so the sim's camera is as unreliable as the real one - the whole point is to
test the pilot against realistic perception, not a perfect oracle.
"""

import math

from common.camera import FX, GATE_OUTER_M, WIDTH, HEIGHT
from common.gate_geometry import relative_gate
from perception.gate_detection import GateDetection
from perception.vision_pose import project_body_to_offset

DETECT_MAX_RANGE = 45.0      # m: gates beyond this are too small to detect (matches the real logs)
MIN_FWD = 0.5                # m: a gate must be in front of the camera


def render(pos, quat, gates, noise=None, rng=None):
    """Return the detector's view of `gates` from pose (pos, quat): a list of GateDetection,
    nearest-first, the same contract the real detector emits. `noise` is an optional PerceptionNoise
    model, `rng` an optional random.Random for reproducibility."""
    dets = []
    for g in gates:
        rel = relative_gate(pos, quat, g)
        if rel["forward"] <= MIN_FWD or rel["distance"] > DETECT_MAX_RANGE:
            continue
        proj = project_body_to_offset(rel["forward"], rel["right"], rel["down"])
        if proj is None:
            continue
        ox, oy = proj
        dist = rel["distance"]
        px = FX * GATE_OUTER_M / max(dist, 0.5)        # pixel height of the 2.7 m outer ring
        det = GateDetection(offset_x=ox, offset_y=oy, area=px * px, distance_m=dist,
                              has_opening=dist < 22.0, gate_id=g.get("gate_id"))
        if noise is not None:
            det = noise.apply(det, dist, rng)
            if det is None:                              # dropped this frame
                continue
        # reject anything that fell outside the frame (after any noise)
        if abs(det.offset_x) > 1.0 or abs(det.offset_y) > 1.0:
            continue
        det.area_frac = det.area / float(WIDTH * HEIGHT)
        det.bbox = (0, 0, int(math.sqrt(det.area)), int(math.sqrt(det.area)))
        dets.append(det)
    dets.sort(key=lambda d: d.area, reverse=True)
    return dets
