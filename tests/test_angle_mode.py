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

    def test_betaflight_pitch_is_positive_nose_down(self):
        """A positive MSP_ATTITUDE pitch means NOSE DOWN on this firmware.

        Measured on d45, 2026-09-20, with hardware.tiltcheck: held still and
        pitched 31 degrees, the vertical acceleration with gravity removed
        read -5.24 m/s^2 where it must read zero. An inverted sign predicts
        g * (cos 2t - 1) = -5.20 at that angle. Held level it read 0.00,
        which is why every at-rest check we had passed.

        The same rotation carries the accelerometer into the world frame for
        the dead reckoning, so this was not only an altitude problem.
        """
        src = FcStateSource(FakeBridge(pitch=30.0, heading=90.0))
        e = src.estimate()
        zb = e.R[:, 2]                                         # body z in world
        self.assertGreater(zb[0], 0.0)     # nose DOWN -> thrust tips forward (+x)

    def test_negative_pitch_is_nose_up(self):
        src = FcStateSource(FakeBridge(pitch=-30.0, heading=90.0))
        zb = src.estimate().R[:, 2]
        self.assertLess(zb[0], 0.0)        # nose up -> thrust tips backward (-x)

    def test_gravity_is_removed_at_any_attitude(self):
        """The check hardware.tiltcheck runs on the aircraft, run here on a
        perfect one: a stationary aircraft has zero vertical acceleration
        whatever way it is pointing."""
        import numpy as _np
        for pitch in (-30.0, -10.0, 0.0, 10.0, 30.0):
            for roll in (-30.0, 0.0, 30.0):
                br = FakeBridge(roll=roll, pitch=pitch)
                src = FcStateSource(br)
                src.acc_lsb_per_g = 512.0
                # what a still aircraft at that attitude really measures:
                # gravity expressed in its own body frame
                e = src.estimate()
                g_body = e.R.T @ _np.array([0.0, 0.0, 9.80665])
                az = float((e.R @ g_body)[2]) - 9.80665
                self.assertAlmostEqual(az, 0.0, places=6,
                                       msg=f"roll {roll} pitch {pitch}: az={az:+.3f}")

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


class BaroBridge:
    """A flight controller whose barometer is being blown about by prop wash.

    The exact sequence d45 logged on its first props-on flight, 2026-09-20:
    -0.10, -3.86, -0.49, +1.00 m within half a second, sitting at about 0.4 m.
    """

    SEQ = [-0.10, -0.10, -3.86, -0.49, -0.49, 1.00]

    def __init__(self):
        from hardware import msp
        from hardware.bridge import FcState
        self.msp = msp
        self.i = 0
        self.t = 0.0
        self.s = FcState(t=0.0, attitude=msp.Attitude(0.0, 0.0, 0.0),
                         altitude=msp.Altitude(self.SEQ[0], 0.0),
                         imu=msp.RawImu((0, 0, 512), (0, 0, 0), (0, 0, 0)))

    def step(self):
        self.i += 1
        self.t += 0.1
        self.s.t = self.t
        self.s.altitude = self.msp.Altitude(self.SEQ[min(self.i, len(self.SEQ) - 1)], 0.0)

    def state(self):
        return self.s


class TestBarometerOutliers(unittest.TestCase):
    """A 3.8 m step in 100 ms is 37 m/s. That is not an altitude.

    d45's first props-on flight: the altitude loop believed the -3.86 m sample,
    decided it was 3.9 m low, and commanded 1837 PWM - near full throttle. A
    0.4 m hover reached 2 m. The aircraft responded correctly to fiction.
    """

    def test_impossible_jumps_do_not_survive(self):
        br = BaroBridge()
        src = FcStateSource(br)
        seen = [float(src.estimate().p[2])]
        for _ in range(len(BaroBridge.SEQ) - 1):
            br.step()
            seen.append(float(src.estimate().p[2]))
        worst = max(abs(b - a) for a, b in zip(seen, seen[1:]))
        self.assertLess(worst, 1.3,
                        f"a {worst:.2f} m step in 100 ms survived: {seen}")

    def test_the_rate_limit_still_catches_what_the_median_cannot(self):
        """A median of three is beaten by two consecutive bad samples. The
        rate limit is the layer behind it, and it has to still be there."""
        br = BaroBridge()
        br.SEQ = [0.0, 0.0, 0.0, -4.0, -4.1, -4.0, 0.0]
        src = FcStateSource(br)
        vals = [float(src.estimate().p[2])]
        for _ in range(len(br.SEQ) - 1):
            br.step()
            vals.append(float(src.estimate().p[2]))
        self.assertGreater(src.alt_rejected, 0,
                           f"the rate limit never fired: {vals}")

    def test_a_real_climb_is_not_rejected(self):
        # 3 m/s is a hard but entirely real climb: every sample must survive
        br = BaroBridge()
        br.SEQ = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]
        src = FcStateSource(br)
        vals = [float(src.estimate().p[2])]
        for _ in range(5):
            br.step()
            vals.append(float(src.estimate().p[2]))
        self.assertEqual(src.alt_rejected, 0, f"a real climb was rejected: {vals}")
        # a median of three costs exactly one sample of lag and nothing else,
        # so a 0.3 m/sample ramp ends one step behind
        self.assertAlmostEqual(vals[-1], 1.2, places=2)


class TestBarometerSpikeRejection(unittest.TestCase):
    """A single bad sample must not reach anything.

    Both aircraft produced exactly this shape with props running: d45 read
    -3.86 m while sitting at 0.3 on 2026-09-20, d44 read +1.76 m while at
    about 0.9 on 2026-09-21. A rate limit lets the first of a pair through; a
    median of three cannot be moved by one sample at all.
    """

    def _run(self, seq):
        br = BaroBridge()
        br.SEQ = seq
        src = FcStateSource(br)
        out = [float(src.estimate().p[2])]
        for _ in range(len(seq) - 1):
            br.step()
            out.append(float(src.estimate().p[2]))
        return out

    def test_d45_negative_spike_never_appears(self):
        out = self._run([0.30, 0.30, 0.30, -3.86, 0.31, 0.30])
        self.assertGreater(min(out), -0.5, f"the -3.86 spike survived: {out}")

    def test_d44_positive_spike_never_appears(self):
        out = self._run([0.90, 0.90, 0.90, 1.76, 0.91, 0.90])
        self.assertLess(max(out), 1.2, f"the +1.76 spike survived: {out}")

    def test_a_real_climb_still_gets_through(self):
        # a median of three costs one sample of lag and nothing else
        out = self._run([0.0, 0.3, 0.6, 0.9, 1.2, 1.5])
        self.assertGreater(out[-1], 1.1, f"a real climb was flattened: {out}")


class TestThrottleGating(unittest.TestCase):
    """A barometer reading taken while the throttle was moving is not evidence.

    The spikes are throttle-coupled: d44 logged a clean trace under --no-baro
    at a constant ~1227 PWM, then spiked to 1.76 m minutes later with the
    altitude loop chasing. Same aircraft, same current, minutes apart. The
    difference was whether the loop was disturbing its own sensor.
    """

    def test_a_steady_throttle_is_accepted(self):
        src = FcStateSource(BaroBridge())
        for i in range(20):
            src.note_throttle(1228, t=i * 0.02)
        self.assertTrue(src.throttle_is_steady(0.4))

    def test_a_moving_throttle_is_not(self):
        src = FcStateSource(BaroBridge())
        for i in range(20):
            src.note_throttle(1000 + i * 30, t=i * 0.02)   # 1500 PWM/s
        self.assertFalse(src.throttle_is_steady(0.4))

    def test_it_does_not_gate_before_it_has_evidence(self):
        # with no throttle history at all, gating would silence the barometer
        # entirely - which is worse than the problem
        src = FcStateSource(BaroBridge())
        self.assertTrue(src.throttle_is_steady(0.0))


class TestBaroTrustMonitor(unittest.TestCase):
    """The aircraft should notice its own barometer lying.

    Five flights chased an altitude the sensor was not delivering, and the one
    mode that worked never asked for a height at all. Throwing that switch was
    a human decision each time.
    """

    def test_it_trusts_a_sane_barometer(self):
        br = BaroBridge()
        br.SEQ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        src = FcStateSource(br)
        for _ in range(len(br.SEQ)):
            src.estimate()
            br.step()
        self.assertTrue(src.baro_trusted)

    def test_the_monitor_starts_trusting(self):
        src = FcStateSource(BaroBridge())
        self.assertTrue(src.baro_trusted)
