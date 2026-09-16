"""Dead-reckoning estimator: integration, altitude filter, gate association, fixes, own gate counting."""
import math
import unittest

import _paths  # noqa: F401
import numpy as np

from seeker.brain import Detection
from seeker.dr_estimator import DeadReckonSource, VerticalFilter, G
from seeker import synthetic_camera as cam


def R_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def sighting(src, gate_xyz, rng_err=1.0):
    """The Detection a perfect camera at src's TRUE pose would return for the
    gate (the test's own pose, not the estimator's)."""
    return sighting_from(src.R, src.p, gate_xyz, rng_err)


def sighting_from(R, p, gate_xyz, rng_err=1.0):
    d = np.array(gate_xyz, dtype=float) - np.asarray(p, dtype=float)
    d_b = R.T @ d
    fwd = d_b[0] * math.cos(cam.CAM_TILT_RAD) + d_b[2] * math.sin(cam.CAM_TILT_RAD)
    up = -d_b[0] * math.sin(cam.CAM_TILT_RAD) + d_b[2] * math.cos(cam.CAM_TILT_RAD)
    x_img = -d_b[1] / fwd
    y_img = -up / fwd
    return Detection(offset_x=x_img / cam.HALF_TAN_X, offset_y=y_img / cam.HALF_TAN_Y, area_frac=0.02, t=0.0,
                     range_m=float(np.hypot(d[0], d[1])) * rng_err)


class TestIntegration(unittest.TestCase):
    def test_constant_accel_integrates(self):
        src = DeadReckonSource(v_decay_s=0.0)
        R = np.eye(3)
        t = 0.0
        for _ in range(100):                       # 1 s at 100 Hz, 1 m/s^2 forward
            src.integrate(t, R, [1.0, 0.0, G], 1.0)
            t += 0.01
        self.assertAlmostEqual(src.v[0], 1.0, places=1)
        self.assertAlmostEqual(src.p[0], 0.5, places=1)
        self.assertAlmostEqual(src.p[2], 1.0, places=3)

    def test_gravity_is_removed_in_body_tilt(self):
        # nose-down 10 deg, hovering: specific force is along body z, no horizontal accel in world
        src = DeadReckonSource(v_decay_s=0.0)
        pitch = math.radians(10)
        R = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        a_body = R.T @ np.array([0.0, 0.0, G])
        for i in range(50):
            src.integrate(i * 0.01, R, a_body, 1.0)
        self.assertLess(abs(src.v[0]), 1e-6)


class TestVerticalFilter(unittest.TestCase):
    def test_ground_accel_is_ignored_until_lift(self):
        vf = VerticalFilter()
        for i in range(60):                        # 0.6 s settling on the deck: the sim's IMU reads free fall
            z, vz = vf.update(0.01, -G, 0.0 + (0.02 if i % 2 else -0.02), True)
        self.assertFalse(vf.airborne)
        self.assertEqual(vz, 0.0)
        self.assertLess(abs(z), 0.05)
        for i in range(40):                        # lift-off: baro climbs to 0.4 m
            vf.update(0.01, 1.0, 0.01 * i, True)
        self.assertTrue(vf.airborne)


    def test_tracks_a_climb_through_noisy_baro_with_accel_bias(self):
        rng = np.random.default_rng(1)
        vf = VerticalFilter()
        vf.airborne = True
        dt = 0.01
        z_true, vz_true = 0.0, 0.0
        errs = []
        for i in range(800):                       # 8 s: 2 s still, 3 s climb at 0.8 m/s, 3 s hold
            t = i * dt
            az = 0.8 / 0.5 if 2.0 <= t < 2.5 else (-0.8 / 0.5 if 5.0 <= t < 5.5 else 0.0)
            vz_true += az * dt
            z_true += vz_true * dt
            z_meas = z_true + rng.normal(0.0, 0.1)     # the sim's baro noise
            z, vz = vf.update(dt, az + 0.3, z_meas, True)   # 0.3 m/s^2 accel bias (3 % scale error)
            if t > 6.5:
                errs.append((abs(z - z_true), abs(vz - vz_true)))
        ez = max(e[0] for e in errs)
        evz = max(e[1] for e in errs)
        self.assertLess(ez, 0.15, f"altitude error {ez:.2f} m")
        self.assertLess(evz, 0.3, f"vario error {evz:.2f} m/s")


LANDMARKS = [(0.0, 8.0, 1.35, math.pi / 2), (0.0, 8.0, 4.05, math.pi / 2),    # a stacked pair
             (6.0, 20.0, 1.35, 0.0), (-10.0, 3.0, 1.35, math.pi)]


class TestAssociation(unittest.TestCase):
    def setUp(self):
        self.src = DeadReckonSource()
        self.src.R = R_yaw(math.pi / 2)            # facing north, on the y axis
        self.src.p[:] = [0.0, 0.0, 1.35]
        self.src.integrate(0.0, self.src.R, [0, 0, G], 1.35)

    def test_picks_the_gate_it_is_looking_at_including_the_stacked_pair(self):
        for i in (0, 1, 2):
            det = sighting(self.src, LANDMARKS[i][:3])
            self.assertEqual(self.src.associate(det, LANDMARKS), i, f"landmark {i}")

    def test_gate_behind_the_camera_is_not_a_candidate_and_junk_is_unmatched(self):
        det = Detection(offset_x=0.9, offset_y=0.9, area_frac=0.01, t=0.0, range_m=12.0)   # a false positive
        self.assertIsNone(self.src.associate(det, LANDMARKS))
        idx, r = self.src.observe(det, LANDMARKS)
        self.assertIsNone(idx)
        self.assertEqual(self.src.unmatched, 1)
        self.assertEqual(self.src.fixes, 0)

    def test_association_survives_a_drifted_estimate(self):
        # the truth is 1.2 m east of the estimate: the sighting still matches gate 0
        det = sighting_from(self.src.R, [1.2, 0.0, 1.35], LANDMARKS[0][:3])
        self.assertEqual(self.src.associate(det, LANDMARKS), 0)
        idx, r = self.src.observe(det, LANDMARKS)
        self.assertEqual(idx, 0)
        self.assertGreater(self.src.p[0], 0.3)     # pulled toward the truth (gain 0.35)

    def test_range_only_moves_the_estimate_half_along_the_line_of_sight(self):
        det = sighting(self.src, LANDMARKS[0][:3], rng_err=0.8)   # range read 20 % short
        self.src.observe(det, LANDMARKS)
        # a 1.6 m short range would put us 1.6 m north; along-weight 0.5 and gain 0.35 -> 0.28 m
        self.assertLess(abs(self.src.p[1]), 0.35)
        self.assertLess(abs(self.src.p[0]), 0.02)

    def test_implausible_fix_is_rejected_and_undone(self):
        self.src.reject_m = 0.5
        det = sighting_from(self.src.R, [3.0, 0.0, 1.35], LANDMARKS[0][:3])   # truth 3 m east
        self.src.assoc_sigma_m = 5.0                                          # let it associate
        idx, r = self.src.observe(det, LANDMARKS)
        self.assertIsNone(idx)
        self.assertEqual(self.src.rejected, 1)
        self.assertEqual(self.src.fixes, 0)
        self.assertEqual(self.src.p[0], 0.0)


class TestSeveralBlobs(unittest.TestCase):
    def test_the_matching_blob_wins_over_a_bigger_false_positive(self):
        src = DeadReckonSource()
        src.R = R_yaw(math.pi / 2)
        src.p[:] = [0.0, 0.0, 1.35]
        src.integrate(0.0, src.R, [0, 0, G], 1.35)
        junk = Detection(offset_x=0.8, offset_y=0.8, area_frac=0.05, t=0.0, range_m=3.0)   # biggest, matches nothing
        real = sighting_from(src.R, [0.3, 0.0, 1.35], LANDMARKS[0][:3])                     # gate 0, truth 0.3 m east
        idx, r = src.observe_any([junk, real], LANDMARKS)
        self.assertEqual(idx, 0)
        self.assertGreater(src.p[0], 0.1)
        self.assertEqual(src.unmatched, 0)
        idx, r = src.observe_any([junk], LANDMARKS)
        self.assertIsNone(idx)
        self.assertEqual(src.unmatched, 1)


class TestFixes(unittest.TestCase):
    def test_fix_pulls_position_to_the_gate_geometry(self):
        src = DeadReckonSource(fix_gain=1.0, vel_gain=0.0, along_weight=1.0)
        src.R = R_yaw(math.pi / 2)                 # facing north
        src.p[:] = [0.0, 0.0, 1.35]
        src.integrate(0.0, src.R, [0, 0, G], 1.35)
        det = Detection(offset_x=0.0, offset_y=-math.tan(math.radians(20)) / cam.HALF_TAN_Y, area_frac=0.02, t=0.0, range_m=8.0)
        r = src.apply_fix(det, (0.0, 8.0, 1.35))
        self.assertLess(abs(src.p[0]), 0.05)
        self.assertLess(abs(src.p[1]), 0.05)
        self.assertLess(r, 0.05)
        # now the estimate had drifted 1 m east: repeated sightings correct it
        # (a single 8 m sighting is weighted 0.69; the history must see the new p)
        src.p[:] = [1.0, 0.0, 1.35]
        for i in range(4):
            src.integrate(0.01 * (i + 1), src.R, [0, 0, G], 1.35)
            det.t = 0.01 * (i + 1)
            src.apply_fix(det, (0.0, 8.0, 1.35))
        self.assertLess(abs(src.p[0]), 0.05)

    def test_bearing_only_fix_moves_across_the_line_of_sight_only(self):
        src = DeadReckonSource(fix_gain=1.0, vel_gain=0.0)
        src.R = R_yaw(math.pi / 2)
        src.p[:] = [1.0, 2.0, 1.35]
        src.integrate(0.0, src.R, [0, 0, G], 1.35)
        det = sighting_from(src.R, [0.0, 0.0, 1.35], (0.0, 8.0, 1.35))
        det.range_m = None
        for i in range(4):
            src.integrate(0.01 * (i + 1), src.R, [0, 0, G], 1.35)
            det.t = 0.01 * (i + 1)
            src.apply_fix(det, (0.0, 8.0, 1.35))
        self.assertLess(abs(src.p[0]), 0.05)       # lateral error gone
        self.assertGreater(src.p[1], 1.5)          # along-track error untouched


class TestGateCounting(unittest.TestCase):
    def test_counts_crossings_in_order_only(self):
        src = DeadReckonSource(v_decay_s=0.0)
        src.set_events([(0.0, 5.0, 1.35, math.pi / 2), (0.0, 15.0, 1.35, math.pi / 2), (5.0, 20.0, 1.35, 0.0)])
        R = np.eye(3)
        src.v[:] = [0.0, 2.0, 0.0]
        t = 0.0
        for _ in range(1200):                      # 12 s north at 2 m/s -> y = 24
            src.integrate(t, R, [0, 0, G], 1.35); t += 0.01
        self.assertEqual(src.next_event, 2)       # g0 and g1 crossed, g2 (eastbound at x=5) not
        src2 = DeadReckonSource(v_decay_s=0.0)
        src2.set_events([(0.0, 5.0, 1.35, math.pi / 2)])
        src2.p[:] = [6.0, 0.0, 1.35]; src2.v[:] = [0.0, 2.0, 0.0]
        for i in range(500):
            src2.integrate(i * 0.01, R, [0, 0, G], 1.35)
        self.assertEqual(src2.next_event, 0)


class TestCrossingByFix(unittest.TestCase):
    def test_a_fix_that_jumps_the_estimate_over_the_plane_counts(self):
        src = DeadReckonSource(fix_gain=1.0, vel_gain=0.0, along_weight=1.0)
        src.set_events([(0.0, 8.0, 1.35, math.pi / 2)])
        src.R = R_yaw(math.pi / 2)
        src.p[:] = [0.0, 7.7, 1.35]                # 0.3 m short of the plane
        src.integrate(0.0, src.R, [0, 0, G], 1.35)
        det = sighting_from(src.R, [0.0, 8.4, 1.35], (0.0, 8.0, 1.35))   # truth: 0.4 m past it, gate behind
        det.range_m = 0.4
        det.offset_x = 0.0
        # the sighting says the gate is 0.4 m away along the observed bearing: the
        # estimator cannot see behind, so model it as a fix from a 2nd landmark ahead
        src.apply_fix(Detection(offset_x=0.0, offset_y=-math.tan(math.radians(20)) / cam.HALF_TAN_Y,
                                area_frac=0.02, t=0.0, range_m=7.6), (0.0, 16.0, 1.35))
        self.assertGreater(src.p[1], 8.0)
        self.assertEqual(src.next_event, 1)

    def test_a_wide_miss_still_advances_the_count(self):
        src = DeadReckonSource(v_decay_s=0.0)
        src.set_events([(0.0, 5.0, 1.35, math.pi / 2), (0.0, 15.0, 1.35, math.pi / 2)])
        src.p[:] = [3.0, 0.0, 1.35]; src.v[:] = [0.0, 2.0, 0.0]      # 3 m off centre: a miss, but past it
        for i in range(400):
            src.integrate(i * 0.01, np.eye(3), [0, 0, G], 1.35)
        self.assertEqual(src.next_event, 1)


if __name__ == "__main__":
    unittest.main()
