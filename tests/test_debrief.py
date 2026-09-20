"""Debrief cam: the two things the drone can measure about itself with no
ground truth.

The first class drives the REAL estimator through a synthetic flight and
checks the numbers it records; the rest feed logs to the tool. If the tool
cannot read back an error that was put in on purpose, it cannot be trusted to
read back a real one. Nothing here uses a truth column, because the drone
will not have one.
"""
import csv
import math
import tempfile
import unittest
from pathlib import Path

import _paths  # noqa: F401
import numpy as np

from raceline import debrief
from raceline import planner as plan_io
from seeker.brain import Detection
from seeker.dr_estimator import DeadReckonSource
from seeker import synthetic_camera as cam

PLAN = Path(_paths.REPO) / "config" / "ladder" / "plans" / "plan_LADDER_k050_cam35_120.json"


def R_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def sighting(R, p_true, gate_xyz, rng_scale=1.0):
    """What a perfect camera at the TRUE pose sees. rng_scale != 1 is a range
    calibration error, the thing the along innovation is supposed to expose."""
    d = np.array(gate_xyz, dtype=float) - np.asarray(p_true, dtype=float)
    d_b = R.T @ d
    fwd = d_b[0] * math.cos(cam.CAM_TILT_RAD) + d_b[2] * math.sin(cam.CAM_TILT_RAD)
    up = -d_b[0] * math.sin(cam.CAM_TILT_RAD) + d_b[2] * math.cos(cam.CAM_TILT_RAD)
    return Detection(offset_x=(-d_b[1] / fwd) / cam.HALF_TAN_X,
                     offset_y=(-up / fwd) / cam.HALF_TAN_Y,
                     area_frac=0.02, t=0.0,
                     range_m=float(np.hypot(d[0], d[1])) * rng_scale)


class TestEstimatorRecords(unittest.TestCase):
    """The estimator's own bookkeeping, driven through the real code."""

    def test_crossing_offset_is_what_it_believed(self):
        """Cross a gate 0.4 m to the left of its centre: cross_lat must read
        +0.4, and the event index must be the gate. The crossing heading is
        +y, so travelling north, and left of that is -x."""
        src = DeadReckonSource()
        src.set_events([(0.0, 10.0, 1.35, math.pi / 2)])   # crossing heading +y
        src.p[:] = [-0.4, 9.0, 1.35]
        src._count_crossings(np.array([-0.4, 9.0, 1.35]), np.array([-0.4, 11.0, 1.35]))
        self.assertEqual(src.next_event, 1)
        self.assertEqual(src.cross_ev, 0)
        self.assertAlmostEqual(src.cross_lat, 0.4, places=3)
        self.assertAlmostEqual(src.cross_dz, 0.0, places=3)

    def test_crossing_offset_sign_is_left_positive(self):
        """The mirror of the above: right of centre has to come out negative,
        or every direction the debrief reports is backwards."""
        src = DeadReckonSource()
        src.set_events([(0.0, 10.0, 1.35, math.pi / 2)])
        src._count_crossings(np.array([0.3, 9.0, 1.35]), np.array([0.3, 11.0, 1.35]))
        self.assertLess(src.cross_lat, 0.0)

    def test_crossing_height_offset(self):
        """+ above the opening centre."""
        src = DeadReckonSource()
        src.set_events([(0.0, 10.0, 1.35, math.pi / 2)])
        src._count_crossings(np.array([0.0, 9.0, 1.75]), np.array([0.0, 11.0, 1.75]))
        self.assertAlmostEqual(src.cross_dz, 0.40, places=3)

    def test_a_range_error_shows_in_the_along_innovation(self):
        """The camera reads 10 % short. The along innovation must come out
        positive (the camera says we are nearer the gate than DR thought) and
        the cross innovation must stay near zero."""
        src = DeadReckonSource()
        gate = (0.0, 10.0, 1.35)
        src.R = R_yaw(math.pi / 2)
        src.p[:] = [0.0, 0.0, 1.35]
        src.t_prev = 0.0
        src.observe(sighting(src.R, src.p, gate, rng_scale=0.9), [gate])
        self.assertGreater(src.fix_along_sum, 0.5)
        self.assertLess(abs(src.fix_cross_sum), 0.05)

    def test_a_bearing_error_shows_in_the_cross_innovation(self):
        """A camera that points 3 deg off to one side: the disagreement lands
        in cross, not along."""
        src = DeadReckonSource()
        gate = (0.0, 10.0, 1.35)
        src.R = R_yaw(math.pi / 2)
        src.p[:] = [0.0, 0.0, 1.35]
        src.t_prev = 0.0
        det = sighting(src.R, src.p, gate)
        det = Detection(offset_x=det.offset_x + math.radians(3.0) / cam.HALF_TAN_X,
                        offset_y=det.offset_y, area_frac=det.area_frac, t=det.t,
                        range_m=det.range_m)
        src.observe(det, [gate])
        self.assertGreater(abs(src.fix_cross_sum), 0.2)
        self.assertLess(abs(src.fix_along_sum), 0.1)

    def test_a_rejected_fix_does_not_enter_the_sums(self):
        """A wrong-gate fix is undone by observe(); the statistic must not
        keep it, or one bad match poisons the calibration number."""
        src = DeadReckonSource()
        gate = (0.0, 10.0, 1.35)
        src.R = R_yaw(math.pi / 2)
        src.p[:] = [0.0, 0.0, 1.35]
        src.t_prev = 0.0
        # the sighting associates (2.9 deg off the prediction) but moves the
        # estimate far more than this reject threshold allows, so observe()
        # undoes it: the statistic must be undone with it
        src.reject_m = 0.01
        det = sighting(src.R, np.array([0.5, 0.0, 1.35]), gate)
        i, _ = src.observe(det, [gate])
        self.assertIsNone(i)
        self.assertEqual(src.rejected, 1)
        self.assertEqual(src.fixes, 0)
        self.assertEqual(src.fix_cross_sum, 0.0)
        self.assertEqual(src.fix_along_sum, 0.0)


def fake_log(path, plan, believed_offsets, fix_cross=0.0, fix_along=0.0,
             blind_legs=(), hardware=False, rate=5):
    """A flight log in either format, carrying only columns the real drone
    writes: no truth anywhere."""
    pos = plan["pos"]
    ev_s = np.array([e["s"] for e in plan["events"]], dtype=float)
    leg = np.searchsorted(ev_s, plan["s_arr"], side="left")
    fixes = rej = unm = 0
    cx_sum = al_sum = 0.0
    cross_ev, cross_lat, cross_dz = -1, 0.0, 0.0
    rows = []
    for i in range(len(pos)):
        k = int(min(leg[i], len(ev_s) - 1))
        if k not in blind_legs:
            fixes += 1
            cx_sum += fix_cross
            al_sum += fix_along
        if i and leg[i] != leg[i - 1]:                 # a crossing just happened
            cross_ev = int(leg[i - 1])
            cross_lat = believed_offsets.get(cross_ev, 0.0)
            cross_dz = 0.0
        rows.append([f"{plan['t_arr'][i]:.2f}", k, f"{pos[i, 0]:.3f}", f"{pos[i, 1]:.3f}",
                     f"{pos[i, 2]:.3f}", fixes, rej, unm, cross_ev,
                     f"{cross_lat:.3f}", f"{cross_dz:.3f}", f"{cx_sum:.3f}", f"{al_sum:.3f}"])
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        if hardware:
            w.writerow(["t", "phase_or_event", "x", "y", "z", "vx", "vy", "vz", "yaw_deg",
                        "det_x", "det_y", "det_area", "det_range", "fixes", "fix_res",
                        "rej", "unm", "cross_ev", "cross_lat", "cross_dz",
                        "fix_cx_sum", "fix_al_sum", "throttle", "roll", "pitch", "yaw",
                        "arm", "att_hz", "rtt_ms", "timeouts", "cam_fps", "vbat"])
            for r in rows[::rate]:
                t, k, x, y, z, f, rj, un, cev, cl, cz, cxs, als = r
                w.writerow([t, f"ev{k}", x, y, z, 0, 0, 0, 0, "", "", "", "",
                            f, 0.1, rj, un, cev, cl, cz, cxs, als,
                            1500, 1500, 1500, 1500, 1800, 50, 3, 0, 30, 15.8])
        else:
            w.writerow(["t", "ev", "x", "y", "z", "tx", "ty", "tz", "vx", "vy", "tvx", "tvy",
                        "fixes", "rej", "unm", "lm", "res", "det_area", "det_range",
                        "cross_ev", "cross_lat", "cross_dz", "fix_cx_sum", "fix_al_sum", "reason"])
            for r in rows:
                t, k, x, y, z, f, rj, un, cev, cl, cz, cxs, als = r
                # truth columns deliberately filled with nonsense: the tool
                # must not read them
                w.writerow([t, k, x, y, z, 999, 999, 999, 0, 0, 0, 0,
                            f, rj, un, "", 0.0, "", "", cev, cl, cz, cxs, als, ""])
    return path


class TestReadsTheLog(unittest.TestCase):
    def setUp(self):
        self.plan = plan_io.load_plan(PLAN)
        self.plan["_path"] = str(PLAN)
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_believed_offsets_come_back_per_gate(self):
        p = fake_log(self.dir / "dr_001.csv", self.plan, {5: 0.42, 12: -0.30})
        legs = {r["leg"]: r for r in debrief.leg_rows(debrief.load_trace(p), self.plan)}
        self.assertAlmostEqual(legs[5]["cross_lat"], 0.42, places=2)
        self.assertAlmostEqual(legs[12]["cross_lat"], -0.30, places=2)
        self.assertAlmostEqual(legs[6]["cross_lat"], 0.0, places=2)

    def test_truth_columns_are_ignored(self):
        """The sim trace's tx/ty/tz are filled with 999 and nothing changes."""
        p = fake_log(self.dir / "dr_002.csv", self.plan, {5: 0.42})
        tr = debrief.load_trace(p)
        self.assertNotIn("truth", tr)
        legs = {r["leg"]: r for r in debrief.leg_rows(tr, self.plan)}
        self.assertAlmostEqual(legs[5]["cross_lat"], 0.42, places=2)

    def test_outside_the_opening_is_proven_error(self):
        """It cannot have been 1.0 m off centre in a gate it flew through."""
        p = fake_log(self.dir / "dr_003.csv", self.plan, {5: 1.00})
        legs = {r["leg"]: r for r in debrief.leg_rows(debrief.load_trace(p), self.plan)}
        self.assertAlmostEqual(legs[5]["proven_err"], 0.25, places=2)
        self.assertEqual(legs[6]["proven_err"], 0.0)

    def test_leg_innovation_is_exact_at_a_slow_log_rate(self):
        """The hardware log writes every 5th tick. Differencing the estimator's
        running sums must still give the exact mean."""
        p = fake_log(self.dir / "hw_x.csv", self.plan, {}, fix_cross=0.12,
                     fix_along=-0.07, hardware=True, rate=5)
        legs = [r for r in debrief.leg_rows(debrief.load_trace(p), self.plan)]
        for r in legs[1:-1]:
            self.assertAlmostEqual(r["fix_cross"], 0.12, places=2)
            self.assertAlmostEqual(r["fix_along"], -0.07, places=2)

    def test_blind_legs_are_flagged_from_the_fix_count(self):
        p = fake_log(self.dir / "dr_004.csv", self.plan, {}, blind_legs=(6, 7))
        legs = {r["leg"]: r for r in debrief.leg_rows(debrief.load_trace(p), self.plan)}
        self.assertTrue(legs[6]["blind"])
        self.assertFalse(legs[5]["blind"])

    def test_a_log_without_the_columns_still_draws_the_path(self):
        """An old log: no crossing or innovation columns. The picture must
        still come out rather than the tool falling over."""
        p = self.dir / "hw_old.csv"
        with open(p, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "phase_or_event", "x", "y", "z", "fixes"])
            for i in range(200):
                w.writerow([f"{i * 0.02:.2f}", f"ev{i // 40}",
                            f"{self.plan['pos'][i, 0]:.3f}",
                            f"{self.plan['pos'][i, 1]:.3f}", "1.35", i // 4])
        tr = debrief.load_trace(p)
        self.assertFalse(tr["has_debrief_cols"])
        legs = debrief.leg_rows(tr, self.plan)
        self.assertTrue(all(r["cross_lat"] is None for r in legs))
        self.assertTrue(Path(debrief.render(tr, self.plan, legs, self.dir / "old.png")).is_file())

    def test_png_and_log_are_written(self):
        p = fake_log(self.dir / "dr_005.csv", self.plan, {5: 0.3})
        r = debrief.debrief_one(p, PLAN, self.dir, self.dir / "log.csv")
        self.assertTrue(Path(r["png"]).is_file())
        self.assertGreater(Path(r["png"]).stat().st_size, 10_000)
        self.assertGreater(r["added"], 0)

    def test_a_run_is_logged_once(self):
        p = fake_log(self.dir / "dr_006.csv", self.plan, {5: 0.3})
        log = self.dir / "log.csv"
        self.assertGreater(debrief.debrief_one(p, PLAN, self.dir, log)["added"], 0)
        self.assertEqual(debrief.debrief_one(p, PLAN, self.dir, log)["added"], 0)


class TestAggregate(unittest.TestCase):
    def setUp(self):
        self.plan = plan_io.load_plan(PLAN)
        self.plan["_path"] = str(PLAN)
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.log = self.dir / "log.csv"

    def tearDown(self):
        self.tmp.cleanup()

    def _runs(self, offsets_per_run, **kw):
        for i, off in enumerate(offsets_per_run):
            p = fake_log(self.dir / f"dr_{i:03d}.csv", self.plan, off, **kw)
            debrief.debrief_one(p, PLAN, self.dir, self.log)
        return {s["leg"]: s for s in debrief.aggregate(self.log)}

    def test_a_repeatable_offset_is_called_consistent(self):
        s = self._runs([{5: 0.34}, {5: 0.36}, {5: 0.35}, {5: 0.33}])
        self.assertTrue(s[5]["consistent"])
        self.assertAlmostEqual(s[5]["cross_lat"], 0.345, places=2)
        self.assertFalse(s[4]["consistent"])

    def test_an_offset_that_flips_sign_is_not_consistent(self):
        s = self._runs([{5: 0.4}, {5: -0.4}, {5: 0.4}, {5: -0.4}])
        self.assertFalse(s[5]["consistent"])
        self.assertGreater(s[5]["cross_lat_sd"], 0.3)

    def test_two_runs_are_never_enough(self):
        s = self._runs([{5: 0.5}, {5: 0.5}])
        self.assertFalse(s[5]["consistent"], "two runs cannot establish a bias")

    def test_a_whole_course_innovation_reads_as_calibration(self):
        """Every leg disagrees the same way: that is the camera, not drift."""
        for i in range(3):
            p = fake_log(self.dir / f"dr_{i:03d}.csv", self.plan, {},
                         fix_cross=0.0, fix_along=0.31)
            debrief.debrief_one(p, PLAN, self.dir, self.log)
        summary = debrief.aggregate(self.log)
        cal = debrief.calibration(summary)
        self.assertAlmostEqual(cal["fix_along"], 0.31, places=2)
        self.assertLess(abs(cal["fix_cross"]), 0.01)
        txt = debrief.print_aggregate(summary)
        self.assertIn("range scale", txt)
        self.assertIn("camcal", txt)

    def test_report_names_the_gate_and_the_direction(self):
        self._runs([{5: 0.34}, {5: 0.36}, {5: 0.35}])
        txt = debrief.print_aggregate(debrief.aggregate(self.log))
        self.assertIn("left", txt)
        self.assertIn("CONSISTENT", txt)

    def test_report_says_nothing_to_chase_when_clean(self):
        """Three clean runs must read as clean, not as a finding."""
        self._runs([{}, {}, {}])
        self.assertIn("Nothing to chase yet",
                      debrief.print_aggregate(debrief.aggregate(self.log)))

    def test_aggregate_png(self):
        self._runs([{5: 0.34}, {5: 0.36}, {5: 0.35}])
        png = self.dir / "agg.png"
        debrief.aggregate(self.log, png)
        self.assertTrue(png.is_file())


if __name__ == "__main__":
    unittest.main()
