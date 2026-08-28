"""Runnable solvers (pilots) for the elodin sim.

Every module here exposes `autopilot(SensorUpdate) -> RCCommand` and is
selected by name:  RACE_SOLVER=solvers.<module>  (race.py sets this for
the follower automatically).

Start from _template.py. Shared control loops live in raceline.rc_backend;
every tunable number lives in config/vehicle.toml - never hardcode one.
"""
