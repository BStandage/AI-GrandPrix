# AI Pilot notes

Quick orientation for the autonomous drone pilot, and a record of the known-good state.

## What works right now

The default `trajectory` pilot clears the full 6-gate course (set a session record).
It flies a smooth racing line through the gate centres using a world-frame acceleration
controller.

## IMPORTANT: this is a ground-truth (oracle) proof-of-concept, not competition-ready

This branch flies on data the *current sim build leaks* but the *scored competition does not
provide* (see the technical spec, VADR-TS-002):

- We read the drone's absolute pose/velocity from the `ODOMETRY` message. The scored interface
  has no `ODOMETRY` and explicitly does not expose absolute position. You must estimate your
  own state from the camera + IMU (VIO/SLAM).
- We read exact gate positions from a track broadcast. The scored interface has no such
  broadcast. Gates are found through the camera stream (30 Hz JPEG, UDP 5600).

So this branch proves the *control/physics* layer, not the competition pipeline.

What transfers to a competition build:
- The command interface (`SET_ATTITUDE_TARGET`), the measured dynamics in `dynamics.py`, and
  the world-frame trajectory tracker. Given a gate pose and a state estimate, that math flies.

What is missing (the actual Round 1 work):
- State estimation (replace ground-truth pose).
- Gate perception: detect the gate in the camera + solve its pose (PnP), feeding the same
  controller. Scaffolded in gate_detector.py / pnp_offline.py.

Spec facts to remember (not all reflected in the code yet):
- The passable opening is the gate's INNER square, 1.5 m x 1.5 m (the 2.7 m is the outer
  frame). With a 280 mm drone that is ~0.6 m clearance per side, so precision matters.
- Camera: tilted 20 deg up; pinhole 640x360, fx=fy=320, cx,cy=320,180, VFoV 90 deg.
- Command rate must be < 100 Hz. This code still runs CONTROL_HZ=250 (out of spec, left as-is
  for now since it's a sim POC).
- Round 1 just requires navigating the course; max run 8 minutes.

## Module map

The control code is split by responsibility:

- `dynamics.py` - the measured drone model (thrust/climb and speed/lean tables, signs,
  limits) and the low-level send. The "physics + actuator" layer. Every other module
  builds on this.
- `race.py` - race state: countdown gating (`should_fly`, `seconds_to_go`) and how the
  gate list is populated (`load_cached_gates`, disabled by default).
- `trajectory_pilot.py` - the default pilot. Tracks a racing line in the world frame.
- `pursuit_pilot.py` - the fallback pilot. Chases each gate in turn (slower but proven).
- `dev_modes.py` - non-pilot tools: `keyboard` (manual data collection) and
  `characterize` (scripted physics measurement -> CSV).
- `controller.py` - picks the mode each tick, runs it, prints the status readout. This is
  what `main.py` / `setup.py` import. Switch pilots with `CONTROL_MODE`.
- `trajectory.py` - the racing-line geometry (spline through the gates + speed profile +
  the `carrot` query). Not a pilot; the trajectory pilot consumes it.
- `gate_geometry.py` - frame math (quaternion rotation, gate-relative body coordinates).

## How the trajectory pilot flies (the short version)

1. Build a line through the gate centres once (rebuilt if the gate list changes).
2. Each tick, ask the line for the nearest point, the local direction, the target speed,
   and the line's own turning (centripetal) acceleration.
3. Rotate the body-frame odometry velocity into the world frame (the sim reports velocity
   in body frame, which is the single most important gotcha here).
4. Command a world acceleration that pulls onto the line, matches its velocity, holds speed
   against drag, and feeds the turn forward.
5. Convert that acceleration into roll/pitch tilt via the heading. Yaw only points the nose
   and does not affect tracking, so the drone does not crab.
6. Hold altitude to the line separately, via the measured thrust/climb curve.

## Known-good tuning (current values)

Vertical and yaw limits live in `dynamics.py`. Pilot gains live in each pilot file.
The main knobs on the trajectory pilot (`trajectory_pilot.py`):

- `TRAJ_V_MAX = 7.0` - target speed. Raise for time once the course is clean; the sharp
  gate-3 V overshoots if this is too high.
- `TRAJ_APEX_MAX = 0.0` - thread gate centres (no apex cut). A small apex can be re-added
  for speed once it reliably clears.
- `TRAJ_VERT_BIAS = 0.3` - vertical aim above gate centres. By-eye knob: higher if it clips
  bottoms, lower if it clips tops.
- `TRAJ_KP_POS = 1.5`, `TRAJ_KD_VEL = 2.5` - horizontal tracking (roughly critically damped).

## System-identification campaign (sysid_*)

An automated, crash-tolerant flight campaign that maps the drone's true dynamic envelope so the
planner can fly at the airframe's real limits (not the conservative `characterize` numbers).
Standalone harness - reuses the MAVLink connection, the MAVLinkRX telemetry thread, and
`dynamics.py`; it is NOT a `CONTROL_MODE` because it owns a multi-trial reset/arm/climb loop.

- `sysid_campaign.py` - entry point. `python sysid_campaign.py --all` (or `--rotational`,
  `--drag`, `--recovery`, `--feasibility`). `--report-only [dir]` rebuilds the report with no
  sim. ESC aborts. Output: `datasets/sysid_<ts>/tab[1-4]_*.csv` + `SYSID_REPORT.md`.
- `sysid_runner.py` - the `TrialRunner` state machine (reset -> arm -> climb -> run maneuver ->
  capture per tick -> abort on collision/floor/timeout). `snapshot()` rotates body velocity to
  world and exposes `up_align` (= R[2][2]: +1 upright, 0 horizon, -1 inverted; singularity-free).
- `sysid_maneuvers.py` - setpoint generators (rate steps, attitude holds, drag runs, inverted
  dive, the Recovery flip, the inverted-feasibility probe, lateral step).
- `sysid_batteries.py` - the four test matrices.
- `sysid_report.py` - parses the tabs, fits curves, writes the Markdown report (No-Go-Zone
  `Δz_recovery(vz)`, efficiency audit, saturation warnings, master summary). Smoke-testable alone.

Interface reality (the campaign is re-framed onto these, NOT the blueprint's motor model):
- We command body rates + collective thrust, never the motor mixer. `motor_1..4` are observed
  normalized outputs (saturation proxy), not RPM.
- No vehicle mass is exposed, so everything is in acceleration (m/s²); no absolute Newtons.
- Inverted/Split-S flight is gated by an in-harness feasibility probe (Tier 4); if the sim
  refuses, the recovery grid is skipped and the report documents the limit.

## Next steps

- Push `TRAJ_V_MAX` up now that the course is clean.
- Run the sysid campaign against the sim and fold the measured maxima back into `dynamics.py`.
- Swap the front-end to vision/PnP feeding this same controller (replace ground-truth gates
  with detector + pose).
