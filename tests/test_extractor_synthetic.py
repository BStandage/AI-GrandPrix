"""End-to-end extraction on a synthetic render with known ground truth."""
import math
import unittest

import cv2
import numpy as np

import _paths

ex = _paths.load_extractor()

# ground truth: (x, y, yaw_deg, stacked). Entry order = list order.
TRUE_GATES = [
    (10.0, 5.0, 0.0, True),     # start/finish, stacked (crossed render)
    (16.0, 15.0, 30.0, False),
    (10.0, 28.0, 90.0, False),
    (4.0, 38.0, -20.0, False),
    (12.0, 45.0, 0.0, False),
]
SCALE = 9.0  # px per meter in the synthetic image


def m2px(x, y):
    return int(round(40 + x * SCALE)), int(round(520 - y * SCALE))


def render_course():
    img = np.full((560, 260, 3), (238, 236, 234), np.uint8)
    for gx in range(0, 21, 5):
        cv2.line(img, m2px(gx, 0), m2px(gx, 52), (210, 208, 206), 1)
    for gy in range(0, 53, 5):
        cv2.line(img, m2px(0, gy), m2px(20, gy), (210, 208, 206), 1)

    # racing line through the gates, perpendicular crossings
    wp = []
    for x, y, yaw_deg, _ in TRUE_GATES:
        n = math.radians(yaw_deg + 90)
        wp.append((x - 2.5 * math.cos(n), y - 2.5 * math.sin(n)))
        wp.append((x, y))
        wp.append((x + 2.5 * math.cos(n), y + 2.5 * math.sin(n)))
    pts = np.array([m2px(*p) for p in wp], np.int32)
    cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (200, 120, 40), 2,
                  cv2.LINE_AA)

    for x, y, yaw_deg, stacked in TRUE_GATES:
        yaws = (yaw_deg, yaw_deg + 90) if stacked else (yaw_deg,)
        for yw in yaws:
            a = math.radians(yw)
            h = 1.35
            p0 = m2px(x - h * math.cos(a), y - h * math.sin(a))
            p1 = m2px(x + h * math.cos(a), y + h * math.sin(a))
            cv2.line(img, p0, p1, (30, 140, 240), 6)

    for cx, cy in [(2, 8), (18, 8), (2, 25), (18, 40)]:
        cv2.circle(img, m2px(cx, cy), 9, (40, 40, 230), -1)
    return img


class TestSyntheticExtraction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.img = render_course()
        corners_m = [(0, 0), (20, 0), (20, 50), (0, 50)]
        corners_px = [m2px(*c) for c in corners_m]
        cls.H = ex.compute_homography(corners_px, corners_m)
        cls.shapes, cls.line_mask, cls.fg = ex.segment(cls.img, cls.H)
        cls.gates = ex.group_gates(cls.shapes)
        start = m2px(TRUE_GATES[0][0], TRUE_GATES[0][1] - 2.5)
        cls.path, cls.tmeta = ex.trace_line(cls.line_mask, cls.H,
                                            start_px=start)
        ex.assign_order(cls.gates, cls.path)

    def test_grid_verifies(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        res = ex.verify_grid(gray, self.fg, self.H)
        self.assertTrue(res["ok"], res)
        self.assertLessEqual(res["rms_err_m"], 0.2)

    def test_gate_count_and_stacked(self):
        self.assertEqual(len(self.gates), len(TRUE_GATES))
        stacked = [g for g in self.gates if g["type"] == "stacked"]
        self.assertEqual(len(stacked), 1)
        self.assertLess(
            math.dist(stacked[0]["xy"], TRUE_GATES[0][:2]), 0.8)

    def test_positions_and_yaw(self):
        for tx, ty, tyaw, stacked in TRUE_GATES:
            g = min(self.gates, key=lambda g: math.dist(g["xy"], (tx, ty)))
            derr = math.dist(g["xy"], (tx, ty))
            self.assertLess(derr, 0.5,
                            f"gate at ({tx},{ty}) recovered {derr:.2f} m off")
            if not stacked:
                yerr = abs((g["members"][0]["yaw"] - math.radians(tyaw)
                            + math.pi / 2) % math.pi - math.pi / 2)
                self.assertLess(math.degrees(yerr), 8.0)

    def test_traversal_order_and_heading(self):
        self.assertIsNotNone(self.path, self.tmeta)
        for want_order, (tx, ty, tyaw, _st) in enumerate(TRUE_GATES):
            g = min(self.gates, key=lambda g: math.dist(g["xy"], (tx, ty)))
            self.assertEqual(g["order"], want_order)
            n = math.radians(tyaw + 90)
            err = abs(ex.wrap_pi(g["entry_heading_rad"] - n))
            self.assertLess(math.degrees(err), 25.0)

    def test_cones_excluded(self):
        blobs = [s for s in self.shapes if s["kind"] == "blob"]
        self.assertGreaterEqual(len(blobs), 4)
        for b in blobs:
            for g in self.gates:
                self.assertGreater(math.dist(b["xy"], g["xy"]), 1.5)

    def test_build_map_schema(self):
        mapd = ex.build_map(self.gates,
                            {"ok": True, "rms_err_m": 0.05, "max_err_m": 0.1,
                             "status": "verified"},
                            self.tmeta, "synthetic.png", [])
        self.assertEqual(mapd["source"], "estimated_from_overhead_image")
        geo = mapd["gate_geometry"]
        self.assertAlmostEqual(geo["opening_center_height_m"], 1.35)
        st = [g for g in mapd["gates"] if g["type"] == "stacked"][0]
        self.assertEqual([o["z"] for o in st["openings"]], [4.05, 1.35])
        # out-and-back: top entry heading is the reverse of the bottom's
        top, bot = st["openings"]
        self.assertAlmostEqual(
            abs(ex.wrap_pi(top["entry_heading_rad"]
                           - bot["entry_heading_rad"])), math.pi, places=2)
        for g in mapd["gates"]:
            for o in g["openings"]:
                self.assertIsNotNone(o["z"])


if __name__ == "__main__":
    unittest.main()
