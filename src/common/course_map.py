"""
Course map loader — the ONE way downstream code gets gate positions.

    from common import course_map
    cm = course_map.load("data/course_map.json")
    for g in cm.ordered_gates():
        g.xy, g.yaw_rad, g.openings ...

Nothing downstream may hardcode gate positions. A map file carries a
`source` provenance, one of PROVENANCES:

    estimated_from_overhead_image   scale-accurate at best; plan margins
    published                       official organizer coordinates
    survey_refined                  refined from on-site flight data

Swapping estimated -> published/survey coordinates is a data change only:
same schema, same loader. `fit_rigid_transform` + `CourseMap.transformed`
align an estimated map onto published coordinates when those land.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

PROVENANCES = (
    "estimated_from_overhead_image",
    "published",
    "survey_refined",
)


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Gate:
    id: int
    order: int | None
    xy: tuple[float, float]
    yaw_rad: float
    entry_heading_rad: float | None
    type: str                       # "single" | "stacked"
    openings: list[dict]            # [{"z": ...}] — stacked: top first
    confidence: str                 # "low" | "med" | "high"
    extras: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, d):
        core = {"id", "order", "xy", "yaw_rad", "entry_heading_rad",
                "type", "openings", "confidence"}
        if d.get("type") not in ("single", "stacked"):
            raise ValueError(f"gate {d.get('id')}: bad type {d.get('type')!r}")
        openings = d.get("openings") or []
        if not openings or any(o.get("z") is None for o in openings):
            raise ValueError(f"gate {d.get('id')}: opening z is required, "
                             "never null")
        if d["type"] == "stacked" and len(openings) != 2:
            raise ValueError(f"gate {d.get('id')}: stacked needs 2 openings")
        return cls(
            id=int(d["id"]),
            order=None if d.get("order") is None else int(d["order"]),
            xy=(float(d["xy"][0]), float(d["xy"][1])),
            yaw_rad=float(d["yaw_rad"]),
            entry_heading_rad=(None if d.get("entry_heading_rad") is None
                               else float(d["entry_heading_rad"])),
            type=d["type"],
            openings=openings,
            confidence=d.get("confidence", "low"),
            extras={k: v for k, v in d.items() if k not in core},
        )

    def to_dict(self):
        return {
            "id": self.id, "order": self.order,
            "xy": [self.xy[0], self.xy[1]],
            "yaw_rad": self.yaw_rad,
            "entry_heading_rad": self.entry_heading_rad,
            "type": self.type, "openings": self.openings,
            "confidence": self.confidence, **self.extras,
        }


@dataclass
class CourseMap:
    frame: str
    source: str
    footprint_m: tuple[float, float]
    gate_geometry: dict
    gates: list[Gate]
    meta: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, d):
        src = d.get("source")
        if src not in PROVENANCES:
            raise ValueError(
                f"unrecognized map provenance {src!r}; expected one of "
                f"{PROVENANCES}")
        gates = [Gate.from_dict(g) for g in d.get("gates", [])]
        ids = [g.id for g in gates]
        if len(set(ids)) != len(ids):
            raise ValueError("duplicate gate ids")
        return cls(
            frame=d.get("frame", ""),
            source=src,
            footprint_m=tuple(d.get("footprint_m", (0.0, 0.0))),
            gate_geometry=d.get("gate_geometry", {}),
            gates=gates,
            meta=d.get("meta", {}),
        )

    def to_dict(self):
        return {
            "frame": self.frame, "source": self.source,
            "footprint_m": list(self.footprint_m),
            "gate_geometry": self.gate_geometry,
            "gates": [g.to_dict() for g in self.gates],
            "meta": self.meta,
        }

    def gate_by_id(self, gid):
        for g in self.gates:
            if g.id == gid:
                return g
        raise KeyError(gid)

    def ordered_gates(self):
        """Gates with a traversal order, in that order. Raises if the map
        has no verified order — callers must not silently assume one."""
        witho = [g for g in self.gates if g.order is not None]
        if len(witho) != len(self.gates):
            raise ValueError("map has gates without traversal order "
                             "(line trace failed at extraction)")
        return sorted(witho, key=lambda g: g.order)

    def transformed(self, dyaw_rad, tx, ty, source=None):
        """Rigid transform: rotate by dyaw about origin, then translate.
        Returns a new CourseMap; yaw/heading rotate with the frame."""
        c, s = math.cos(dyaw_rad), math.sin(dyaw_rad)
        out = []
        for g in self.gates:
            x, y = g.xy
            ng = Gate(
                id=g.id, order=g.order,
                xy=(c * x - s * y + tx, s * x + c * y + ty),
                yaw_rad=_wrap_pi(g.yaw_rad + dyaw_rad),
                entry_heading_rad=(None if g.entry_heading_rad is None
                                   else _wrap_pi(g.entry_heading_rad + dyaw_rad)),
                type=g.type, openings=[dict(o) for o in g.openings],
                confidence=g.confidence, extras=dict(g.extras),
            )
            out.append(ng)
        meta = dict(self.meta)
        meta["transformed_by"] = {"dyaw_rad": dyaw_rad, "tx": tx, "ty": ty}
        return CourseMap(frame=self.frame, source=source or self.source,
                         footprint_m=self.footprint_m,
                         gate_geometry=dict(self.gate_geometry),
                         gates=out, meta=meta)


def fit_rigid_transform(src_xy, dst_xy):
    """Least-squares 2D rigid transform (no scale) mapping src -> dst.

    Returns (dyaw_rad, tx, ty) for use with CourseMap.transformed().
    src_xy/dst_xy: sequences of (x, y) pairs, matched by index.
    """
    if len(src_xy) != len(dst_xy) or len(src_xy) < 2:
        raise ValueError("need >=2 matched point pairs")
    n = len(src_xy)
    sx = sum(p[0] for p in src_xy) / n
    sy = sum(p[1] for p in src_xy) / n
    dx = sum(p[0] for p in dst_xy) / n
    dy = sum(p[1] for p in dst_xy) / n
    num = den = 0.0
    for (ax, ay), (bx, by) in zip(src_xy, dst_xy):
        ax, ay, bx, by = ax - sx, ay - sy, bx - dx, by - dy
        num += ax * by - ay * bx
        den += ax * bx + ay * by
    dyaw = math.atan2(num, den)
    c, s = math.cos(dyaw), math.sin(dyaw)
    tx = dx - (c * sx - s * sy)
    ty = dy - (s * sx + c * sy)
    return dyaw, tx, ty


def align(est_map, ref_map, source="published"):
    """Rigid-align est_map onto ref_map using gates matched by id."""
    pairs = [(e.xy, ref_map.gate_by_id(e.id).xy) for e in est_map.gates
             if any(r.id == e.id for r in ref_map.gates)]
    if len(pairs) < 2:
        raise ValueError("fewer than 2 shared gate ids to align on")
    dyaw, tx, ty = fit_rigid_transform([p[0] for p in pairs],
                                       [p[1] for p in pairs])
    return est_map.transformed(dyaw, tx, ty, source=source)


def load(path):
    with open(path, encoding="utf-8") as f:
        return CourseMap.from_dict(json.load(f))


def save(cm, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cm.to_dict(), f, indent=1)
