"""The seeker brain against a toy world: a point-mass drone that obeys the
brain's accel/altitude/yaw commands, and a camera model that reports a
gate when it is inside the field of view. No position ever reaches the
brain; the test checks it still threads the gates in order, including a
stacked out-and-back, and lands."""
import math
import unittest

import _paths  # noqa: F401

from seeker.brain import Crossing, Detection, SeekerBrain, SeekerConfig, wrap_pi


class ToyWorld:
    """Point mass with drag, first-order yaw and altitude, camera with
    90 deg horizontal / 59 deg vertical field of view and 20 deg up-tilt."""

    def __init__(self, crossings, dt=0.02):
        self.c = crossings
        self.dt = dt
        self.t = 0.0
        self.x = self.y = 0.0
        self.vx = self.vy = 0.0
        self.z = 0.0
        self.vz = 0.0
        self.yaw = math.pi / 2
        self.k_drag = 0.255
        self.crossed = []          # labels in order
        self._sides = {}

    def step(self, cmd):
        ax, ay = cmd.a_des
        v = math.hypot(self.vx, self.vy)
        ax -= self.k_drag * v * self.vx
        ay -= self.k_drag * v * self.vy
        self.vx += ax * self.dt; self.vy += ay * self.dt
        px, py = self.x, self.y
        self.x += self.vx * self.dt; self.y += self.vy * self.dt
        # altitude: first-order toward target, 1.5 m/s max
        dz = cmd.z_target - self.z
        self.vz = max(-1.5, min(1.5, 3.0 * dz))
        self.z += self.vz * self.dt
        # yaw: first-order, 2 rad/s max
        e = wrap_pi(cmd.yaw_target - self.yaw)
        self.yaw = wrap_pi(self.yaw + max(-2.0, min(2.0, 4.0 * e)) * self.dt)
        self.t += self.dt
        self._check_crossings(px, py)

    def _check_crossings(self, px, py):
        for i, c in enumerate(self.c):
            nx, ny = math.cos(c.heading_rad), math.sin(c.heading_rad)
            s0 = (px - c.x) * nx + (py - c.y) * ny
            s1 = (self.x - c.x) * nx + (self.y - c.y) * ny
            if s0 < 0 <= s1:
                lat = -(self.x - c.x) * ny + (self.y - c.y) * nx
                if abs(lat) < 0.75 and abs(self.z - c.z) < 0.75:
                    self.crossed.append(c.label)

    def detect(self, target):
        """Detection of `target` if it is in front, inside the FOV, and not
        too close; area_frac from the apparent size of a 2.7 m ring."""
        dx, dy, dz = target.x - self.x, target.y - self.y, target.z - self.z
        dist = math.hypot(dx, dy)
        if dist < 0.3:
            return None
        bearing = wrap_pi(math.atan2(dy, dx) - self.yaw)
        if abs(bearing) > math.radians(45):
            return None
        elev = math.atan2(dz, dist) - math.radians(20)
        if abs(elev) > math.radians(29):
            return None
        # only gates roughly facing us are recognisable (within 70 deg of head-on)
        if abs(wrap_pi(target.heading_rad - self.yaw)) > math.radians(70):
            return None
        app = 2.7 / max(dist, 0.3)                 # apparent size (rad-ish)
        area_frac = min(1.0, (app / 2.0) ** 2 * 0.35)
        # image convention: +offset_x = gate to the RIGHT; bearing is CCW (left) positive
        return Detection(offset_x=-math.tan(bearing) / 1.0, offset_y=-math.tan(elev) / 0.5625,
                         area_frac=area_frac, t=self.t)


def course():
    # start at (0,0), g0 7 m north, g1 north-east, g2 stacked out-and-back, g3 back west
    return [
        Crossing("g0", 0.0, 7.0, 1.35, math.pi / 2),
        Crossing("g1", 6.0, 16.0, 1.35, math.pi / 2),
        Crossing("g2-top", 6.0, 26.0, 4.05, math.pi / 2),
        Crossing("g2-low", 6.0, 26.0, 1.35, -math.pi / 2),
        Crossing("g3", -4.0, 16.0, 1.35, -math.pi / 2),
    ]


class TestSeekerBrain(unittest.TestCase):
    def run_course(self, laps=1, max_t=240.0):
        cs = course()
        # the lap closes on g0 again; g0 is northbound, reached from g3 heading back
        w = ToyWorld(cs)
        brain = SeekerBrain(cs, SeekerConfig(), start_xy=(0.0, 0.0), laps=laps)
        phases = []
        while w.t < max_t and not brain.done:
            k = min(brain.k, len(brain.seq) - 1)
            det = w.detect(brain.seq[k])
            cmd = brain.step(w.t, w.yaw, w.z, w.vz, det)
            if not phases or phases[-1][0] != cmd.phase:
                phases.append((cmd.phase, round(w.t, 1), cmd.crossing))
            w.step(cmd)
        return w, brain, phases

    def test_threads_every_gate_in_order_without_position(self):
        w, brain, phases = self.run_course()
        expected = [c.label for c in brain.seq]
        self.assertEqual(w.crossed[:len(expected)], expected,
                         f"crossed {w.crossed}; phases {phases}")
        self.assertTrue(brain.done, f"did not land; phases {phases}")
        self.assertLess(w.z, 0.2)

    def test_never_uses_position(self):
        import inspect
        from seeker import brain as b
        brain = b.SeekerBrain(course(), b.SeekerConfig(), laps=1)
        self.assertFalse(hasattr(brain, "x") or hasattr(brain, "y") or hasattr(brain, "pos"))
        self.assertEqual(list(inspect.signature(b.SeekerBrain.step).parameters),
                         ["self", "t", "yaw", "z", "vz", "det"])

    def test_lost_gate_falls_back_to_seek_then_hold(self):
        cs = course()
        brain = SeekerBrain(cs, SeekerConfig(), laps=1)
        t = 0.0
        # takeoff done
        for _ in range(100):
            brain.step(t, math.pi / 2, 1.35, 0.0, None); t += 0.02
        self.assertEqual(brain.phase, "SEEK")
        brain.step(t, math.pi / 2, 1.35, 0.0, Detection(0.1, 0.0, 0.01, t))
        self.assertEqual(brain.phase, "TRACK")
        for _ in range(40):
            brain.step(t, math.pi / 2, 1.35, 0.0, None); t += 0.02
        self.assertEqual(brain.phase, "SEEK")
        for _ in range(2000):
            brain.step(t, math.pi / 2, 1.35, 0.0, None); t += 0.02
        self.assertEqual(brain.phase, "HOLD")


if __name__ == "__main__":
    unittest.main()
