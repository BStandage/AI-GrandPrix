"""Strict loader for config/vehicle.toml — the stack's single tuning surface.

Strict both ways: an unknown key (typo) and a missing key both fail loudly,
so an intern's edit can't silently do nothing. Access is attribute-style:
cfg.limits.v_max_mps. The file's sha1 is carried into every plan for
provenance.
"""

from __future__ import annotations

import hashlib
import math
import tomllib
from pathlib import Path
from types import SimpleNamespace

G = 9.81

AIGP_REPO = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = AIGP_REPO / "config" / "vehicle.toml"

# section -> {key: required type}. int is accepted where float is expected.
_SCHEMA = {
    "meta": {"name": str, "updated": str},
    "vehicle": {"mass_kg": float, "drag_lin": float, "drag_quad": float},
    "thrust": {"hover_pwm": int, "pwm_min": int, "pwm_max": int,
               "curve_pwm": list, "curve_acc": list},
    "limits": {"max_tilt_deg": float, "max_yaw_rate_rps": float,
               "a_lat_rate_max": float,
               "v_max_mps": float, "v_gate_mps": float, "gate_window_m": float,
               "vz_up_max": float, "vz_down_max": float,
               "a_accel_max": float, "a_brake_max": float},
    "planner": {"anchor_standoff_m": float, "anchor_standoff_turn_m": float,
                "turn_angle_deg": float, "sample_ds_m": float,
                "a_lat_margin": float, "takeoff_alt_m": float,
                "v_floor_mps": float},
    "follower": {"kp_pos": float, "kd_pos": float, "lookahead_m": float,
                 "ka_att": float, "stick_clamp": int,
                 "kp_z": float, "kd_z": float, "ki_z": float,
                 "kyaw": float, "yaw_clamp": int, "yaw_lookahead_m": float,
                 "takeoff_pwm": int, "min_alt_translation_m": float},
}


class ConfigError(ValueError):
    pass


def _check(cond: bool, msg: str) -> None:
    if not cond:
        raise ConfigError(msg)


class VehicleConfig:
    def __init__(self, raw: dict, path: Path, sha1: str):
        self.raw = raw
        self.path = path
        self.sha1 = sha1
        for section, keys in _SCHEMA.items():
            setattr(self, section, SimpleNamespace(**{k: raw[section][k]
                                                      for k in keys}))

    # Derived quantities — defined ONCE here so planner and follower agree.
    def tilt_rad(self) -> float:
        return math.radians(self.limits.max_tilt_deg)

    def a_lat_full(self) -> float:
        """Lateral accel at max tilt (follower clamp)."""
        return G * math.tan(self.tilt_rad())

    def a_lat_planner(self) -> float:
        """Lateral accel the PLAN may use (margin leaves tilt authority for
        tracking error)."""
        return self.planner.a_lat_margin * self.a_lat_full()


def load_config(path=None) -> VehicleConfig:
    path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    data = path.read_bytes()
    raw = tomllib.loads(data.decode("utf-8"))

    _check(set(raw) == set(_SCHEMA),
           f"{path.name}: sections {sorted(set(raw) ^ set(_SCHEMA))} "
           f"unknown or missing")
    for section, keys in _SCHEMA.items():
        got = set(raw[section])
        _check(got == set(keys),
               f"{path.name} [{section}]: keys "
               f"{sorted(got ^ set(keys))} unknown or missing")
        for k, typ in keys.items():
            v = raw[section][k]
            ok = isinstance(v, typ) or (typ is float and isinstance(v, int))
            _check(ok and not isinstance(v, bool),
                   f"{path.name} [{section}].{k}: expected {typ.__name__}, "
                   f"got {type(v).__name__}")

    t = raw["thrust"]
    _check(len(t["curve_pwm"]) == len(t["curve_acc"]) >= 2,
           f"{path.name} [thrust]: curve_pwm/curve_acc must be equal-length "
           "lists of >= 2 points")
    _check(0 < raw["limits"]["max_tilt_deg"] < 60,
           f"{path.name}: max_tilt_deg out of sane range (0, 60)")
    _check(raw["planner"]["sample_ds_m"] > 0.01,
           f"{path.name}: sample_ds_m too small")

    return VehicleConfig(raw, path, hashlib.sha1(data).hexdigest())
