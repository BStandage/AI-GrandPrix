"""ANGLE-mode stick mapping and the FC state source frame conversions."""
import math
import unittest

import _paths  # noqa: F401
import numpy as np

from raceline.config import load_config, DEFAULT_CONFIG_PATH
from raceline.rc_backend import StateEstimate, angle_sticks
from hardware.state import FcStateSource, rot_zyx


def est(yaw=0.0):
    return StateEstimate(p=np.zeros(3), v=np.zeros(3), R=rot_zyx(0, 0, yaw), yaw=yaw,
                         omega=np.zeros(3))


class TestAngleSticks(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(DEFAULT_CONFIG_PATH)
        self.cfg.follower.angle_limit_deg = 80.0

    def test_level_is_centred(self):
        roll, pitch, _ = angle_sticks(self.cfg, est(), (0.0, 0.0))
        self.assertEqual((roll, pitch), (1500, 1500))

    def test_forward_accel_is_nose_down_positive_pitch(self):
        roll, pitch, ang = angle_sticks(self.cfg, est(), (9.81, 0.0))   # 45 deg forward
        self.assertEqual(roll, 1500)
        self.assertAlmostEqual(ang[1], 45.0, places=3)
        self.assertEqual(pitch, round(1500 + 500 * 45 / 80))

    def test_right_accel_is_positive_roll(self):
        roll, pitch, ang = angle_sticks(self.cfg, est(), (0.0, -9.81))  # -y = right
        self.assertEqual(pitch, 1500)
        self.assertAlmostEqual(ang[0], 45.0, places=3)
        self.assertGreater(roll, 1500)

    def test_yaw_rotates_the_frame(self):
        # nose pointing +y (yaw 90): a world +y accel is FORWARD -> pitch only
        roll, pitch, ang = angle_sticks(self.cfg, est(math.pi / 2), (0.0, 5.0))
        self.assertEqual(roll, 1500)
        self.assertGreater(pitch, 1500)

    def test_limit_clamps_to_full_stick(self):
        roll, pitch, ang = angle_sticks(self.cfg, est(), (100.0, 0.0))
        self.assertEqual(pitch, 2000)
        self.assertAlmostEqual(ang[1], 80.0)


class FakeBridge:
    def __init__(self, roll=0.0, pitch=0.0, heading=0.0, alt=1.0, gyro=(0, 0, 0)):
        from hardware import msp
        from hardware.bridge import FcState
        self.s = FcState(t=1.0, attitude=msp.Attitude(roll, pitch, heading),
                         altitude=msp.Altitude(alt, 0.1), imu=msp.RawImu((0, 0, 512), gyro, (0, 0, 0)))

    def state(self):
        return self.s


class TestFcStateSource(unittest.TestCase):
    def test_heading_to_world_yaw_with_map_north(self):
        src = FcStateSource(FakeBridge(), map_north_heading_deg=30.0)
        # facing the map's north (+y): world yaw = +90 deg
        self.assertAlmostEqual(math.degrees(src.heading_to_world_yaw(30.0)), 90.0)
        # facing the map's east (+x): heading 120 -> yaw 0
        self.assertAlmostEqual(math.degrees(src.heading_to_world_yaw(120.0)), 0.0)

    def test_level_estimate(self):
        src = FcStateSource(FakeBridge(heading=90.0, alt=2.5), map_north_heading_deg=0.0)
        src.alt_offset_m = 1.0
        e = src.estimate()
        self.assertAlmostEqual(e.p[2], 1.5)
        self.assertAlmostEqual(e.yaw, 0.0)                     # heading east = world +x
        np.testing.assert_allclose(e.R, np.eye(3), atol=1e-9)

    def test_nose_up_tilts_body_z_backward(self):
        src = FcStateSource(FakeBridge(pitch=30.0, heading=90.0))
        e = src.estimate()
        zb = e.R[:, 2]                                         # body z in world
        self.assertLess(zb[0], 0.0)                            # nose up -> thrust points backward (-x)

    def test_gyro_frd_to_flu(self):
        src = FcStateSource(FakeBridge(gyro=(10.0, 20.0, 30.0)))
        e = src.estimate()
        np.testing.assert_allclose(np.degrees(e.omega), [10.0, -20.0, -30.0])


class ClimbBridge:
    """A flight controller whose barometer works and whose vario does not.

    That is d45, measured 2026-09-20: lifting the aircraft 0.76 m moved
    `alt` cleanly every time and `vario` stayed at exactly 0.00 m/s. The
    barometer also only resolves about 0.076 m (1 Pa), so the height arrives
    in steps, which is the other half of what the filter has to cope with."""

    LSB = 0.076

    def __init__(self):
        from hardware import msp
        from hardware.bridge import FcState
        self.msp = msp
        self.t = 0.0
        self.z = 0.0
        self.s = FcState(t=0.0, attitude=msp.Attitude(0.0, 0.0, 0.0),
                         altitude=msp.Altitude(0.0, 0.0),
                         imu=msp.RawImu((0, 0, 512), (0, 0, 0), (0, 0, 0)))

    def step(self, dt, vz):
        self.t += dt
        self.z += vz * dt
        self.s.t = self.t
        # a NEW Altitude object each tick, quantised, vario dead
        q = round(self.z / self.LSB) * self.LSB
        self.s.altitude = self.msp.Altitude(q, 0.0)

    def state(self):
        return self.s


class TestVerticalSpeedWithoutFcVario(unittest.TestCase):
    """The FC's vario is not trusted; we derive vertical speed ourselves.

    Regression for the bug that would have grounded the whole race: the
    airborne latch in `hardware.runtime` waits for vz > 0.5 m/s before it lets
    dead reckoning integrate. Reading vz from the FC meant it was always 0.00,
    the latch never fired, and the follower would have flown the entire course
    believing it was still parked on the start line."""

    def _climb(self, rate=1.0, seconds=2.0, dt=0.02):
        br = ClimbBridge()
        src = FcStateSource(br)
        vzs = []
        for _ in range(int(seconds / dt)):
            br.step(dt, rate)
            e = src.estimate()
            vzs.append(float(e.v[2]))
        return src, vzs

    def test_fc_vario_is_zero_but_we_still_see_the_climb(self):
        src, vzs = self._climb(rate=1.0, seconds=2.0)
        self.assertFalse(src.fc_vario_alive)      # the FC never reported one
        self.assertGreater(vzs[-1], 0.5)          # and we found the climb anyway

    def test_the_runtime_airborne_latch_would_fire(self):
        # exactly the test hardware.runtime applies: z > 0.30 AND vz > 0.5
        br = ClimbBridge()
        src = FcStateSource(br)
        latched = False
        for _ in range(150):
            br.step(0.02, 1.5)
            e = src.estimate()
            if float(e.p[2]) > 0.30 and float(e.v[2]) > 0.5:
                latched = True
                break
        self.assertTrue(latched, "the airborne latch never fired on a real climb")

    def test_sitting_still_reports_no_vertical_speed(self):
        # the other half: baro quantisation must not manufacture a climb that
        # trips the latch while the aircraft waits to be armed
        _, vzs = self._climb(rate=0.0, seconds=3.0)
        self.assertLess(max(abs(v) for v in vzs), 0.5)


if __name__ == "__main__":
    unittest.main()
