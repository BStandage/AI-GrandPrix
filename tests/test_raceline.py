"""Racing-line stack: config strictness, planner geometry/limits, and the
plan-validity oracle - replaying the plan through the sim's RaceTracker."""
import math
import os
import tempfile
import unittest

import _paths  # noqa: F401  (sys.path shim)

import numpy as np

from raceline.config import load_config, ConfigError, G
from raceline import course as course_bridge
from raceline import planner

CFG = load_config()
COURSE = course_bridge.load_course()
PLAN = planner.plan(CFG, COURSE)


class TestConfigStrict(unittest.TestCase):
    def _write(self, text):
        f = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False,
                                        encoding="utf-8")
        f.write(text)
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_loads_repo_config(self):
        self.assertGreater(CFG.limits.v_max_mps, 0)
        self.assertEqual(len(CFG.sha1), 40)

    def test_unknown_key_rejected(self):
        base = CFG.path.read_text()
        bad = base.replace("v_max_mps", "v_max_typo")
        with self.assertRaises(ConfigError):
            load_config(self._write(bad))

    def test_missing_key_rejected(self):
        base = CFG.path.read_text()
        bad = "\n".join(l for l in base.splitlines()
                        if not l.startswith("kp_pos"))
        with self.assertRaises(ConfigError):
            load_config(self._write(bad))


class TestStandoffRule(unittest.TestCase):
    def test_travel_reversal_triggers_turn_standoff(self):
        # crossing headings 90 deg apart, but the leg to gate 1 opposes its
        # entry heading (planar switchback) -> pre[1] must widen
        xy = [(0.0, 0.0), (-10.0, -2.0)]
        h = [-math.pi / 2, 0.0]     # exit south, enter east
        pre, post = planner.standoffs(xy, h, 2.0, 3.5, math.radians(100))
        self.assertEqual(pre[1], 3.5)

    def test_degenerate_leg_uses_heading_comparison(self):
        xy = [(5.0, 5.0), (5.0, 5.0)]           # stacked pair: same XY
        h = [-math.pi / 2, math.pi / 2]
        pre, post = planner.standoffs(xy, h, 2.0, 3.5, math.radians(100))
        self.assertEqual((post[0], pre[1]), (3.5, 3.5))

    def test_straight_line_keeps_base(self):
        xy = [(0.0, 0.0), (10.0, 0.0)]
        h = [0.0, 0.0]
        pre, post = planner.standoffs(xy, h, 2.0, 3.5, math.radians(100))
        self.assertEqual((post[0], pre[1]), (2.0, 2.0))


class TestPlanGeometry(unittest.TestCase):
    def test_tracker_replay_completes_all_events(self):
        """THE validity oracle: the plan's own samples, fed to the sim's
        ordered tracker, must cross all 24 openings in order/direction."""
        pq = course_bridge.pq_course()
        tracker = pq.RaceTracker(COURSE)
        for t, pos in zip(PLAN.t, PLAN.pos):
            tracker.update(float(t), pos)
        self.assertTrue(tracker.complete,
                        f"plan only passes {tracker.events_passed}/"
                        f"{COURSE.total_events} events")

    def test_path_hits_crossing_centers(self):
        for e in PLAN.events:
            p = np.array([np.interp(e["s"], PLAN.s, PLAN.pos[:, k])
                          for k in range(3)])
            d = np.linalg.norm(p - np.array([e["x"], e["y"], e["z"]]))
            self.assertLess(d, 0.15, f"{e['label']} center miss {d:.2f} m")

    def test_crossing_direction(self):
        for k, e in enumerate(PLAN.events):
            c = COURSE.event(k)
            tx = np.interp(e["s"], PLAN.s, PLAN.tangent[:, 0])
            ty = np.interp(e["s"], PLAN.s, PLAN.tangent[:, 1])
            dot = tx * math.cos(c.heading_rad) + ty * math.sin(c.heading_rad)
            self.assertGreater(dot, 0.8,
                               f"{e['label']} crossed off-normal (dot={dot:.2f})")

    def test_speed_caps(self):
        lim = CFG.limits
        self.assertLessEqual(float(PLAN.v.max()), lim.v_max_mps + 1e-6)
        for e in PLAN.events:
            self.assertLessEqual(e["v"], lim.v_gate_mps + 0.05,
                                 f"{e['label']} crossing speed {e['v']:.2f}")

    def test_lateral_accel_within_planner_budget(self):
        a_lat = PLAN.v ** 2 * PLAN.kappa
        # small tolerance: kappa is a discrete estimate
        self.assertLessEqual(float(a_lat.max()), CFG.a_lat_planner() * 1.15)

    def test_yaw_rate_ceiling(self):
        txy = np.hypot(PLAN.tangent[:, 0], PLAN.tangent[:, 1])
        mask = txy > 0.3
        yaw_rate = PLAN.v[mask] * PLAN.dpsi_ds[mask]
        # the v_floor may slightly override the yaw bound at a hairpin cusp;
        # allow floor * dpsi_ds there
        bound = np.maximum(CFG.limits.max_yaw_rate_rps,
                           CFG.planner.v_floor_mps * PLAN.dpsi_ds[mask])
        self.assertTrue(np.all(yaw_rate <= bound * 1.1 + 1e-6))

    def test_accel_slew_ceiling(self):
        # d(a_lat)/dt ~ v^3 * dkappa/ds must respect a_lat_rate_max wherever
        # that ceiling binds (v_floor may override at hairpin cusps).
        rate = PLAN.v ** 3 * PLAN.dkappa_ds
        bound = np.maximum(CFG.limits.a_lat_rate_max,
                           CFG.planner.v_floor_mps ** 3 * PLAN.dkappa_ds)
        self.assertTrue(np.all(rate <= bound * 1.15 + 1e-6))

    def test_binding_attribution_present(self):
        self.assertEqual(len(PLAN.binding), len(PLAN.v))
        self.assertTrue(all(isinstance(str(b), str) for b in PLAN.binding))

    def test_archer_estimate_config_plans(self):
        cfg = load_config(CFG.path.parent / "archer_block2.toml")
        p = planner.plan(cfg, COURSE)
        pq = course_bridge.pq_course()
        tracker = pq.RaceTracker(COURSE)
        for t, pos in zip(p.t, p.pos):
            tracker.update(float(t), pos)
        self.assertTrue(tracker.complete)

    def test_time_monotone(self):
        self.assertTrue(np.all(np.diff(PLAN.t) > 0))

    def test_plan_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "plan.json")
            planner.write_plan(PLAN, path)
            loaded = planner.load_plan(path)
        self.assertEqual(loaded["version"], planner.PLAN_VERSION)
        self.assertEqual(len(loaded["events"]), COURSE.total_events)
        self.assertEqual(loaded["pos"].shape, PLAN.pos.shape)
        np.testing.assert_allclose(loaded["pos"], PLAN.pos, atol=1e-3)
        self.assertEqual(loaded["config_sha1"], CFG.sha1)


if __name__ == "__main__":
    unittest.main()
