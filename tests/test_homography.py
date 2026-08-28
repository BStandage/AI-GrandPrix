"""Homography math against synthetic input with known ground truth."""
import math
import unittest

import numpy as np

import _paths

ex = _paths.load_extractor()


def make_true_H(tilt=0.15, scale=9.0, tx=40.0, ty=520.0):
    """A metric->pixel projective map with mild perspective, y flipped
    (image rows grow downward). Returns metric->px 3x3."""
    Hmp = np.array([
        [scale, 0.35 * scale, tx],
        [0.10 * scale, -scale, ty],
        [1e-4, tilt * 1e-3, 1.0],
    ])
    return Hmp


def project(Hmp, pts):
    pts = np.asarray(pts, float)
    ones = np.ones((len(pts), 1))
    q = (Hmp @ np.hstack([pts, ones]).T).T
    return q[:, :2] / q[:, 2:3]


class TestHomography(unittest.TestCase):
    def test_recovers_known_map(self):
        Hmp = make_true_H()
        corners_m = [(0, 0), (20, 0), (20, 50), (0, 50)]
        corners_px = project(Hmp, corners_m)
        H = ex.compute_homography(corners_px, corners_m)
        rng = np.random.default_rng(7)
        test_m = rng.uniform([0, 0], [20, 50], (40, 2))
        test_px = project(Hmp, test_m)
        back = ex.px_to_m(H, test_px)
        err = np.linalg.norm(back - test_m, axis=1)
        self.assertLess(err.max(), 1e-5)

    def test_roundtrip_inverse(self):
        Hmp = make_true_H()
        corners_m = [(0, 0), (20, 0), (20, 50), (0, 50)]
        H = ex.compute_homography(project(Hmp, corners_m), corners_m)
        pts = np.array([[3.3, 7.7], [12.0, 41.0]])
        self.assertLess(
            np.abs(ex.px_to_m(H, ex.m_to_px(H, pts)) - pts).max(), 1e-8)

    def test_degenerate_points_raise(self):
        with self.assertRaises(ValueError):
            ex.compute_homography(
                [(0, 0), (1, 1), (2, 2), (3, 3)],
                [(0, 0), (1, 0), (1, 1), (0, 1)])

    def test_grid_verifier_accepts_true_grid_rejects_bad_scale(self):
        Hmp = make_true_H(tilt=0.0)
        img = np.full((560, 260), 235, np.uint8)
        for gx in range(0, 21, 5):
            p = project(Hmp, [(gx, 0), (gx, 50)]).astype(int)
            import cv2
            cv2.line(img, tuple(p[0]), tuple(p[1]), 180, 1)
        for gy in range(0, 51, 5):
            import cv2
            p = project(Hmp, [(0, gy), (20, gy)]).astype(int)
            cv2.line(img, tuple(p[0]), tuple(p[1]), 180, 1)
        fg = np.zeros_like(img)
        corners_m = [(0, 0), (20, 0), (20, 50), (0, 50)]
        H = ex.compute_homography(project(Hmp, corners_m), corners_m)
        res = ex.verify_grid(img, fg, H)
        self.assertTrue(res["ok"], res)
        self.assertLess(res["rms_err_m"], 0.15)

        # 12% scale error => ticks land off-pitch => must NOT verify
        bad_m = [(x * 1.12, y * 1.12) for x, y in corners_m]
        Hbad = ex.compute_homography(project(Hmp, corners_m), bad_m)
        res_bad = ex.verify_grid(img, fg, Hbad)
        self.assertFalse(res_bad["ok"], res_bad)


if __name__ == "__main__":
    unittest.main()
