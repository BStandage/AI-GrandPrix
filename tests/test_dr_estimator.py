"""Dead-reckoning estimator: integration, gate fixes, own gate counting."""
import math
import unittest

import _paths  # noqa: F401
import numpy as np

from seeker.brain import Detection
from seeker.dr_estimator import DeadReckonSource, G
from seeker import synthetic_camera as cam


def R_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TestIntegration(unittest.TestCase):
    def test_constant_accel_integrates(self):
        src = DeadReckonSource(v_decay_s=0.0)
        R = np.eye(3)
        t = 0.0
        for _ in range(100):                       # 1 s at 100 Hz, 1 m/s^2 forward
            src.integrate(t, R, [1.0, 0.0, G], 1.0, 0.0)
            t += 0.01
        self.assertAlmostEqual(src.v[0], 1.0, places=1)
        self.assertAlmostEqual(src.p[0], 0.5, places=1)
        self.assertEqual(src.p[2], 1.0)

    def test_gravity_is_removed_in_body_tilt(self):
        # nose-down 10 deg, hovering: specific force is along body z, no horizontal accel in world
        src = DeadReckonSource(v_decay_s=0.0)
        pitch = math.radians(10)
        R = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        a_body = R.T @ np.array([0.0, 0.0, G])
        for i in range(50):
            src.integrate(i * 0.01, R, a_body, 1.0, 0.0)
        self.assertLess(abs(src.v[0]), 1e-6)


class TestFixes(unittest.TestCase):
    def test_fix_pulls_position_to_the_gate_geometry(self):
        src = DeadReckonSource(fix_gain=1.0, vel_gain=0.0)
        src.R = R_yaw(math.pi / 2)                 # facing north
        src.p[:] = [0.0, 0.0, 1.35]
        src.integrate(0.0, src.R, [0, 0, G], 1.35, 0.0)
        # gate straight ahead at (0, 8): a sighting dead centre, 8 m away
        det = Detection(offset_x=0.0, offset_y=-math.tan(math.radians(20)) / cam.HALF_TAN_Y, area_frac=0.02, t=0.0, range_m=8.0)
        r = src.apply_fix(det, (0.0, 8.0, 1.35))
        self.assertLess(abs(src.p[0]), 0.05)
        self.assertLess(abs(src.p[1]), 0.05)
        self.assertLess(r, 0.05)
        # now the estimate had drifted 1 m east: the same sighting corrects it
        src.p[:] = [1.0, 0.0, 1.35]
        src.apply_fix(det, (0.0, 8.0, 1.35))
        self.assertLess(abs(src.p[0]), 0.05)


class TestGateCounting(unittest.TestCase):
    def test_counts_crossings_in_order_only(self):
        src = DeadReckonSource(v_decay_s=0.0)
        src.set_events([(0.0, 5.0, 1.35, math.pi / 2), (0.0, 15.0, 1.35, math.pi / 2), (5.0, 20.0, 1.35, 0.0)])
        R = np.eye(3)
        src.v[:] = [0.0, 2.0, 0.0]
        t = 0.0
        for _ in range(1200):                      # 12 s north at 2 m/s -> y = 24
            src.integrate(t, R, [0, 0, G], 1.35, 0.0); t += 0.01
        self.assertEqual(src.next_event, 2)       # g0 and g1 crossed, g2 (eastbound at x=5) not
        # a crossing off to the side does not count
        src2 = DeadReckonSource(v_decay_s=0.0)
        src2.set_events([(0.0, 5.0, 1.35, math.pi / 2)])
        src2.p[:] = [3.0, 0.0, 1.35]; src2.v[:] = [0.0, 2.0, 0.0]
        for i in range(500):
            src2.integrate(i * 0.01, R, [0, 0, G], 1.35, 0.0)
        self.assertEqual(src2.next_event, 0)


if __name__ == "__main__":
    unittest.main()
