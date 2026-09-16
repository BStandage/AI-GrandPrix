"""Strict loader for config/vehicle.toml - the stack's single tuning surface.

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

import numpy as np

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
    "planner": {"laps": int,
                "anchor_standoff_m": float, "anchor_standoff_turn_m": float,
                "turn_angle_deg": float, "sample_ds_m": float,
                "a_lat_margin": float, "takeoff_alt_m": float,
                "v_floor_mps": float, "reversal_climb_m": float},
    "optimizer": {"apex_max_m": float},
    "follower": {"kp_pos": float, "kd_pos": float, "lookahead_m": float,
                 "lookahead_t": float,
                 "ka_att": float, "kw_att": float, "stick_clamp": int,
                 "kp_z": float, "kd_z": float, "ki_z": float,
                 "kyaw": float, "yaw_clamp": int, "yaw_lookahead_m": float,
                 "takeoff_pwm": int, "min_alt_translation_m": float},
}


# section -> {key: default}. Accepted when present, filled in when absent, so
# older tomls (archer_block2, dev_fast) keep loading.
_OPTIONAL = {
    "vehicle": {"drag_quad_z": None},   # None -> same as drag_quad (isotropic)
    "follower": {"angle_limit_deg": 80.0},   # Betaflight `angle_limit` when flying ANGLE mode (AIGP_ANGLE_MODE=1)
    # LOOK WINDOW: within look_window_m of arc length before each crossing the
    # planner stops accelerating (pitch capped near look_tilt_deg) so the
    # camera can hold the gate; the sprint happens right after the previous
    # gate instead. 0 = off.
    "limits": {"look_window_m": 0.0, "look_tilt_deg": 12.0,
               # BLIND TURNS: a crossing whose heading differs from the previous
               # crossing's by more than blind_turn_deg is approached with the
               # camera off the gate, so the estimate runs blind into it. Cap
               # speed within blind_turn_window_m before such a crossing at
               # v_blind_turn_mps (0 = off). One rule for every gate.
               "blind_turn_deg": 90.0, "blind_turn_window_m": 6.0, "v_blind_turn_mps": 0.0},
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
            vals = {k: raw[section][k] for k in keys}
            for k, default in _OPTIONAL.get(section, {}).items():
                vals[k] = raw[section].get(k, default)
            setattr(self, section, SimpleNamespace(**vals))

    # Derived quantities - defined ONCE here so planner and follower agree.
    def tilt_rad(self) -> float:
        return math.radians(self.limits.max_tilt_deg)

    def a_lat_full(self) -> float:
        """Horizontal accel available while holding altitude: g*tan(tilt),
        bounded by the thrust ceiling (sqrt(T_max^2 - g^2)). With the tilt
        clamp at 90 the thrust bound is the one that counts (36 m/s^2);
        an unbounded tan() broke every ratio built on this number."""
        g = 9.81
        by_tilt = g * math.tan(min(self.tilt_rad(), math.radians(89.0)))
        t_max = float(max(self.thrust.curve_acc))
        by_thrust = math.sqrt(max(t_max * t_max - g * g, 0.0))
        return min(by_tilt, by_thrust)

    def a_lat_planner(self) -> float:
        """Lateral accel the PLAN may use (margin leaves tilt authority for
        tracking error)."""
        return self.planner.a_lat_margin * self.a_lat_full()

    def drag_k_xyz(self):
        """Quadratic drag per WORLD axis as [kx, ky, kz] in 1/m: the plant
        applies F = -k |v| v with a larger k on the vertical (physics.py
        linear_drag [0.2, 0.2, 0.4]); flown climbs measured kz 0.45-0.5
        against 0.256 horizontal (2026-09-10). Used by the trajectory
        optimizer and the MPC; the planner's a_drag stays horizontal."""
        vh = self.vehicle
        kh = vh.drag_quad / vh.mass_kg
        kz = (vh.drag_quad if vh.drag_quad_z is None else vh.drag_quad_z) / vh.mass_kg
        return [kh, kh, kz]

    def a_drag(self, v_mps: float) -> float:
        """Speed-dependent loss the plant applies against the velocity,
        as an accel (m/s^2): (drag_lin*v + drag_quad*v^2) / mass. The
        planner subtracts it from thrust when accelerating and adds it
        when braking; the follower feeds it forward so the position loop
        does not have to carry it as steady-state velocity error."""
        vh = self.vehicle
        return (vh.drag_lin * v_mps + vh.drag_quad * v_mps * v_mps) / vh.mass_kg

    def pwm_for_thrust(self, thrust_mps2: float) -> float:
        """Inverse of the measured [thrust] curve: specific thrust (m/s^2 of
        accel the motors give a LEVEL drone, hover = G) -> throttle PWM.
        Linear between the measured points, clamped to [pwm_min, pwm_max]."""
        th = self.thrust
        pwm = float(np.interp(thrust_mps2, th.curve_acc, th.curve_pwm))
        return min(max(pwm, float(th.pwm_min)), float(th.pwm_max))


def load_config(path=None) -> VehicleConfig:
    # an empty string (an env var declared but unset in the container) means the default
    path = Path(path) if path else DEFAULT_CONFIG_PATH
    data = path.read_bytes()
    raw = tomllib.loads(data.decode("utf-8"))

    _check(set(raw) == set(_SCHEMA),
           f"{path.name}: sections {sorted(set(raw) ^ set(_SCHEMA))} "
           f"unknown or missing")
    for section, keys in _SCHEMA.items():
        got = set(raw[section]) - set(_OPTIONAL.get(section, {}))
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
    _check(all(a < b for a, b in zip(t["curve_acc"], t["curve_acc"][1:]))
           and all(a < b for a, b in zip(t["curve_pwm"], t["curve_pwm"][1:])),
           f"{path.name} [thrust]: curve_pwm/curve_acc must be strictly "
           "increasing (the follower inverts the curve)")
    _check(t["curve_pwm"][0] <= t["hover_pwm"] <= t["curve_pwm"][-1],
           f"{path.name} [thrust]: hover_pwm must lie inside curve_pwm")
    _check(0 < raw["limits"]["max_tilt_deg"] <= 89.5,
           f"{path.name}: max_tilt_deg out of sane range (0, 89.5]")
    _check(raw["planner"]["sample_ds_m"] > 0.01,
           f"{path.name}: sample_ds_m too small")

    return VehicleConfig(raw, path, hashlib.sha1(data).hexdigest())
