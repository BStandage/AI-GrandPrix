"""
Perception-noise model CALIBRATED from real flight logs (session vision_frames.jsonl, measured via
the per-detection-vs-truth analysis). The sim camera applies this so the pilot is tested against
perception as unreliable as the real thing - that's the whole point of the harness.

Measured (analysis of ~3600 real detections vs ground truth):
  * detection rate per frame is only 0.15-0.6 depending on range (gates are missed most frames),
  * image-bearing noise std ~0.05-0.08 in offset units,
  * pinhole RANGE is garbage: median ~40% short, ~50% of detections off by >50% (blow-ups).
"""

# detection probability vs range (m), from the measured per-range detect rate (smoothed/capped)
_DETECT_P = [(5, 0.15), (10, 0.30), (15, 0.40), (20, 0.55), (25, 0.45), (30, 0.25), (45, 0.20)]

OFFX_STD = 0.05       # image-bearing noise (offset units); a bit below the raw measured 0.13, which
OFFY_STD = 0.05       # is inflated by truth-association; the real cross-track world error was ~1 m.
RANGE_MED = 0.62      # pinhole/true range ratio, median (reads ~40% short)
RANGE_STD = 0.35      # spread of the range ratio
BLOWUP_P = 0.18       # fraction of detections whose range additionally blows up
BLOWUP_MULT = (1.6, 3.2)


def _detect_p(dist):
    pts = _DETECT_P
    if dist <= pts[0][0]:
        return pts[0][1]
    if dist >= pts[-1][0]:
        return pts[-1][1]
    for (d0, p0), (d1, p1) in zip(pts, pts[1:]):
        if d0 <= dist <= d1:
            return p0 + (p1 - p0) * (dist - d0) / (d1 - d0)
    return pts[-1][1]


class PerceptionNoise:
    def apply(self, det, dist, rng):
        """Corrupt a clean detection like the real detector does, or return None (dropout)."""
        rng = rng or __import__("random")
        if rng.random() > _detect_p(dist):
            return None                                   # missed this frame
        det.offset_x += rng.gauss(0, OFFX_STD)
        det.offset_y += rng.gauss(0, OFFY_STD)
        ratio = max(0.15, rng.gauss(RANGE_MED, RANGE_STD))
        if rng.random() < BLOWUP_P:
            ratio *= rng.uniform(*BLOWUP_MULT)
        det.distance_m = dist * ratio                     # range is the unreliable axis
        return det
