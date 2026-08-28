"""Config-driven racing-line stack for the elodin sim.

config/vehicle.toml -> planner (offline timed trajectory through the gate
openings) -> follower (RACE_SOLVER=raceline.follower) -> race.py one-command
loop. All tuning is GLOBAL; per-gate anything is banned (RESTRICTIONS.md).
"""
