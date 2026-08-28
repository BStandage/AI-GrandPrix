# Solvers

Every flyable pilot is one file in this folder with one function:

```python
def autopilot(update: SensorUpdate) -> RCCommand:
```

| Solver | What it does |
|---|---|
| `_template.py` | Copy me. Arms, hovers in front of g0, every layer labeled. |
| `follower.py` | THE race solver: flies a planned trajectory (arc-length carrot). Needs `AIGP_TRAJ` - use `race.py`, which handles it. |
| `sysid_slew.py` | Measurement flight: attitude-slew steps + analyzer. |
| `vision_probe.py` | Collects FPV frames of the gates, then runs the HSV detector offline. |

## Add a solver

```
cd src/solvers
cp _template.py my_solver.py        # edit the YOUR BRAIN GOES HERE section
```

Run it (WSL, from the sim repo):

```
RACE_SOLVER=solvers.my_solver AIGP_SIM_TIME=30 uv run elodin run sim/main.py
```

## Tune a solver

Edit `config/vehicle.toml`. Never hardcode a number in solver code.

| I want to change... | Edit |
|---|---|
| top speed / speed through gates | `[limits] v_max_mps`, `v_gate_mps` |
| how hard it can tilt (= accelerate/corner) | `[limits] max_tilt_deg` |
| pitch/roll stick authority | `[follower] stick_clamp`, `ka_att` |
| yaw speed | `[follower] kyaw`, `yaw_clamp` |
| altitude response | `[follower] kp_z`, `kd_z`, `ki_z` |
| the planned racing line itself | `[planner]` + `[limits]`, then check the plan report |

Experiment on a copy (`config/dev_you.toml`, run with `--config`), promote
to `vehicle.toml` when it wins at 24/24. Per-gate tuning of anything is
banned (`RESTRICTIONS.md`).

## What's off the shelf

- `raceline.rc_backend` - proven throttle / tilt / yaw loops. Use them.
- `raceline.planner` - map -> timed trajectory (what `follower` flies).
- `raceline.course` / `sim.pq_course` - gate positions, crossing sequence.
- `update.frame_rgba` - the FPV camera, for vision solvers.

Full walkthrough: `docs/GETTING_STARTED_RACING_LINE.md`.
