"""Loader: provenance validation, z never null, rigid transform math."""
import json
import math
import os
import tempfile
import unittest

import _paths  # noqa: F401  (sys.path shim)

from common import course_map


def minimal_map(source="published", order=0):
    return {
        "frame": "ENU, origin at SW corner of footprint, +X east, +Y north",
        "source": source,
        "footprint_m": [60.0, 21.0],
        "gate_geometry": {"outer_m": 2.7, "opening_m": 1.5, "depth_m": 0.26,
                          "opening_center_height_m": 1.35},
        "gates": [
            {"id": 0, "order": order, "xy": [5.0, 10.0], "yaw_rad": 0.0,
             "entry_heading_rad": 1.5708, "type": "single",
             "openings": [{"z": 1.35}], "confidence": "med"},
            {"id": 1, "order": None if order is None else order + 1,
             "xy": [15.0, 30.0], "yaw_rad": 0.5,
             "entry_heading_rad": None, "type": "stacked",
             "openings": [{"z": 4.05}, {"z": 1.35}], "confidence": "low"},
        ],
        "meta": {},
    }


class TestLoader(unittest.TestCase):
    def _load(self, d):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "m.json")
            with open(p, "w") as f:
                json.dump(d, f)
            return course_map.load(p)

    def test_accepts_all_three_provenances(self):
        for src in course_map.PROVENANCES:
            cm = self._load(minimal_map(source=src))
            self.assertEqual(cm.source, src)
            self.assertEqual(len(cm.gates), 2)

    def test_rejects_unknown_provenance(self):
        for bad in ("hand_tuned", "", None, "Published"):
            with self.assertRaises(ValueError):
                self._load(minimal_map(source=bad))

    def test_rejects_null_opening_z(self):
        d = minimal_map()
        d["gates"][0]["openings"] = [{"z": None}]
        with self.assertRaises(ValueError):
            self._load(d)

    def test_stacked_needs_two_openings(self):
        d = minimal_map()
        d["gates"][1]["openings"] = [{"z": 1.35}]
        with self.assertRaises(ValueError):
            self._load(d)

    def test_ordered_gates_raises_without_order(self):
        cm = self._load(minimal_map(order=None))
        d = minimal_map(order=None)
        d["gates"][0]["order"] = None
        cm = self._load(d)
        with self.assertRaises(ValueError):
            cm.ordered_gates()

    def test_rigid_transform_roundtrip(self):
        cm = self._load(minimal_map())
        dyaw, tx, ty = 0.7, -3.0, 12.5
        t = cm.transformed(dyaw, tx, ty)
        g0, t0 = cm.gates[0], t.gates[0]
        c, s = math.cos(dyaw), math.sin(dyaw)
        self.assertAlmostEqual(t0.xy[0], c * g0.xy[0] - s * g0.xy[1] + tx)
        self.assertAlmostEqual(t0.xy[1], s * g0.xy[0] + c * g0.xy[1] + ty)
        self.assertAlmostEqual(t0.yaw_rad, g0.yaw_rad + dyaw)
        back = t.transformed(-dyaw, *course_map.fit_rigid_transform(
            [t0.xy, t.gates[1].xy], [g0.xy, cm.gates[1].xy])[1:])
        self.assertAlmostEqual(back.gates[0].xy[0], g0.xy[0], places=6)

    def test_fit_rigid_transform_recovers_known(self):
        import random
        rnd = random.Random(3)
        src = [(rnd.uniform(0, 60), rnd.uniform(0, 21)) for _ in range(6)]
        dyaw, tx, ty = 0.31, 4.2, -7.7
        c, s = math.cos(dyaw), math.sin(dyaw)
        dst = [(c * x - s * y + tx, s * x + c * y + ty) for x, y in src]
        fy, fx, fty = course_map.fit_rigid_transform(src, dst)
        self.assertAlmostEqual(fy, dyaw, places=9)
        self.assertAlmostEqual(fx, tx, places=8)
        self.assertAlmostEqual(fty, ty, places=8)

    def test_align_by_id(self):
        cm = self._load(minimal_map())
        ref = cm.transformed(0.2, 1.0, -2.0, source="published")
        aligned = course_map.align(cm, ref)
        self.assertEqual(aligned.source, "published")
        for a, r in zip(aligned.gates, ref.gates):
            self.assertAlmostEqual(a.xy[0], r.xy[0], places=6)
            self.assertAlmostEqual(a.xy[1], r.xy[1], places=6)


if __name__ == "__main__":
    unittest.main()
