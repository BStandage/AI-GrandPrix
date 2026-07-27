"""
The four sysid test batteries. Each drives the TrialRunner through a test matrix and writes one
tab CSV into the campaign's output directory.

Granularity differs by tab (matching the source blueprint):
  Tab 1 rotational  - one row per control tick (the rate response is a time series)
  Tab 2 drag        - one row per control tick (the velocity sweep is a time series)
  Tab 3 recovery    - one row per trial (each is a single phase-plane point)
  Tab 4 feasibility - one row per trial (each is one entry-speed/bank point)

Re-framed from the blueprint onto our real interface: we command body rates + collective, not a
motor mixer; motor_1..4 are observed normalized outputs (saturation proxy, not RPM); all force
quantities are accelerations (m/s^2), since the sim exposes no vehicle mass.
"""

import csv
import math
import os

from common.dynamics import CONTROL_HZ
from common.dynamics import HOVER_THRUST
from flight_test.sysid_attitude_maneuvers import AttitudeScript
from flight_test.sysid_maneuvers import (DragRun, InvertedDive, InvertedProbe, LateralStep, RateStep,
                             Recovery)
from flight_test.sysid_maneuvers_tab5 import PointApproach, StationHold
from flight_test.sysid_runner import Trial

MOTOR_SAT = 0.98        # a motor output at/above this counts as saturated


def _f2(x):
    """Format a possibly-None metric to 2 dp for the console readouts."""
    return f"{x:.2f}" if isinstance(x, (int, float)) else "n/a"


def _writer(out_dir, filename, header):
    f = open(os.path.join(out_dir, filename), "w", newline="")
    w = csv.DictWriter(f, fieldnames=header, extrasaction="ignore")
    w.writeheader()
    return f, w


def _add_alpha(rows, omega_key):
    """Per-tick angular acceleration alpha = d(omega)/dt from consecutive samples."""
    if rows:
        rows[0]["alpha"] = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b["t"] - a["t"]
        b["alpha"] = (b[omega_key] - a[omega_key]) / dt if dt > 0 else 0.0


def _add_drag_accel(rows):
    """Per-tick drag deceleration during the level coast: a = -d|v_h|/dt. Also the parametric
    coefficient a/v^2 (only meaningful while actually coasting and moving)."""
    if rows:
        rows[0]["drag_accel"] = 0.0
        rows[0]["drag_coeff"] = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b["t"] - a["t"]
        dec = -(b["vh"] - a["vh"]) / dt if dt > 0 else 0.0
        b["drag_accel"] = dec
        b["drag_coeff"] = dec / (b["vh"] ** 2) if b["vh"] > 1.0 else 0.0


def _max_saturation_ms(rows):
    """Longest run of consecutive ticks with a motor pinned at saturation, in milliseconds."""
    longest = run = 0
    for r in rows:
        if r.get("motor_max", 0.0) >= MOTOR_SAT:
            run += 1
            longest = max(longest, run)
        else:
            run = 0
    return longest * 1000.0 / CONTROL_HZ


# ===========================================================================================
# Tab 1 - Rotational dynamics & actuator saturation sweep (Task 1.1)
# ===========================================================================================
ROT_AXES = ["roll", "pitch", "yaw"]
# Resolution is concentrated at the LOW end: the airframe saturates ~29 rad/s by a commanded ~6,
# so dense steps 1..7 resolve the response knee, then a few coarse points confirm the ceiling.
# Capped at 12: saturation is ~29 (roll/pitch) / ~18 (yaw) by a commanded ~6, so 9 and 12
# already confirm the plateau. Commanding 20/30 only spun the drone through several flips into
# the floor (and below the world), which destabilised the whole campaign - no extra data.
ROT_RATES = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 9.0, 12.0]   # rad/s commanded
ROT_MAX_CMD = max(ROT_RATES)
TAB1_HEADER = ["trial", "t", "timestamp_ms", "axis", "commanded_rate", "input_pct", "seg",
               "alt", "climb_up",
               "motor_1", "motor_2", "motor_3", "motor_4", "motor_max",
               "measured_omega", "alpha", "cross_axis_drift",
               "rollspeed", "pitchspeed", "yawspeed"]
_OMEGA_KEY = {"roll": "rollspeed", "pitch": "pitchspeed", "yaw": "yawspeed"}
_CROSS_KEYS = {"roll": ("pitchspeed", "yawspeed"), "pitch": ("rollspeed", "yawspeed"),
               "yaw": ("rollspeed", "pitchspeed")}


def run_rotational(runner, out_dir, setup_alt=45.0):
    print("\n=== Tab 1: Rotational dynamics sweep ===", flush=True)
    f, w = _writer(out_dir, "tab1_rotational.csv", TAB1_HEADER)
    try:
        for axis in ROT_AXES:
            okey = _OMEGA_KEY[axis]
            c0, c1 = _CROSS_KEYS[axis]
            for rate in ROT_RATES:
                name = f"rot_{axis}_{rate:g}"
                print(f"  {name}", flush=True)
                trial = Trial(name, RateStep(axis, rate, hold_s=0.5, recover_s=0.5),
                              setup_alt=setup_alt, timeout_s=1.5,
                              params={"axis": axis, "commanded_rate": rate})
                res = runner.run(trial)
                if res.outcome == "aborted":
                    print("  aborted by user.", flush=True)
                    return
                _add_alpha(res.rows, okey)
                for r in res.rows:
                    r.update({
                        "trial": name, "axis": axis, "commanded_rate": rate,
                        "input_pct": 100.0 * rate / ROT_MAX_CMD,
                        "timestamp_ms": (r["t_usec"] / 1000.0) if r.get("t_usec") else r["t"] * 1000.0,
                        "measured_omega": r[okey],
                        "cross_axis_drift": math.hypot(r[c0], r[c1]),
                    })
                    w.writerow(r)
                f.flush()
    finally:
        f.close()
    print("  -> tab1_rotational.csv", flush=True)


# ===========================================================================================
# Tab 2 - Aerodynamic drag envelope (Task 1.2)
# ===========================================================================================
TAB2_HEADER = ["trial", "t", "phase", "alt", "vx_w", "vy_w", "vz_w", "airspeed", "vh", "climb_up",
               "roll", "pitch", "yaw", "cmd_thrust", "motor_max", "drag_accel", "drag_coeff"]


def run_drag(runner, out_dir, setup_alt=45.0, dive_alt=110.0):
    print("\n=== Tab 2: Aerodynamic drag envelope ===", flush=True)
    f, w = _writer(out_dir, "tab2_drag.csv", TAB2_HEADER)
    trials = [
        ("drag_fwd", DragRun("forward", 0.52, 9.0), setup_alt, 8.0),
        ("drag_lat", DragRun("lateral", 0.52, 8.0), setup_alt, 8.0),
        # inverted powered dive from high up -> terminal downward velocity (needs vertical room)
        ("drag_inverted_dive", InvertedDive(thrust=1.0), dive_alt, 10.0),
    ]
    try:
        for name, man, alt, timeout in trials:
            print(f"  {name}", flush=True)
            trial = Trial(name, man, setup_alt=alt, timeout_s=timeout, floor_alt=3.0,
                          params={"kind": name})
            res = runner.run(trial)
            if res.outcome == "aborted":
                print("  aborted by user.", flush=True)
                return
            _add_drag_accel(res.rows)
            for r in res.rows:
                r["trial"] = name
                r["airspeed"] = math.sqrt(r["vx_w"] ** 2 + r["vy_w"] ** 2 + r["vz_w"] ** 2)
                if "phase" not in r:
                    r["phase"] = "dive" if "dive" in name else "?"
                w.writerow(r)
            f.flush()
            print(f"    outcome={res.outcome}, {len(res.rows)} samples", flush=True)
    finally:
        f.close()
    print("  -> tab2_drag.csv", flush=True)


# ===========================================================================================
# Tab 3 - Recovery phase-plane matrix (Task 1.3), gated by a Tier-4 feasibility probe
# ===========================================================================================
# Kept short (6 x 2 x 2 = 24 trials) so the whole grid finishes before the sim's physics degrade
# over a long session. 40 m/s is dropped (the powered dive tops out ~37 m/s from 120 m), and the
# oblique axis is dropped (roll and pitch bracket it). Re-add cells once the grid is proven.
REC_ENTRY_VZ = [10.0, 15.0, 20.0, 25.0, 30.0, 35.0]   # target downward speed (m/s)
REC_STRATEGIES = ["continuous", "snap"]
REC_AXES = ["roll", "pitch"]
TAB3_HEADER = ["trial", "entry_vz_target", "entry_vz_actual", "strategy", "axis",
               "throttle_cut_time", "horizon_crossing_time", "peak_down_vz", "trigger_alt",
               "min_floor_clearance", "delta_z_loss", "max_actuator_saturation_ms",
               "arrested", "outcome"]


def _feasibility_probe(runner, setup_alt):
    """Run the inverted probe once. Returns its metrics dict (or None if aborted)."""
    print("  [gate] inverted-flight feasibility probe...", flush=True)
    probe = InvertedProbe()
    trial = Trial("recovery_probe", probe, setup_alt=setup_alt, timeout_s=4.0, floor_alt=2.0)
    res = runner.run(trial)
    if res.outcome == "aborted":
        return None
    m = probe.metrics
    print(f"  [gate] reached_inverted={m['reached_inverted']} "
          f"min_up_align={m['min_up_align']:.2f} "
          f"rotated_at_zero_thrust={m['rotated_at_zero_thrust']} "
          f"(max rate {m['rate_at_zero_thrust']:.1f} rad/s)", flush=True)
    return m


def run_recovery(runner, out_dir, setup_alt=120.0):
    print("\n=== Tab 3: Recovery phase-plane matrix ===", flush=True)
    f, w = _writer(out_dir, "tab3_recovery.csv", TAB3_HEADER)
    try:
        probe = _feasibility_probe(runner, setup_alt)
        if probe is None:
            print("  aborted by user.", flush=True)
            return
        if not probe["reached_inverted"]:
            print("  [gate] sim did NOT support inverted flight - skipping the recovery grid.\n"
                  "         (documented in the report; this is the Tier-4 limit.)", flush=True)
            w.writerow({"trial": "GATE_FAILED", "outcome": "inverted_unsupported",
                        "entry_vz_target": probe["min_up_align"]})
            f.flush()
            return

        for vz in REC_ENTRY_VZ:
            for strat in REC_STRATEGIES:
                for axis in REC_AXES:
                    name = f"rec_{vz:g}_{strat}_{axis}"
                    print(f"  {name}", flush=True)
                    man = Recovery(vz, strat, axis)
                    trial = Trial(name, man, setup_alt=setup_alt, timeout_s=15.0, floor_alt=1.0,
                                  params={"entry_vz_target": vz, "strategy": strat, "axis": axis})
                    res = runner.run(trial)
                    if res.outcome == "aborted":
                        print("  aborted by user.", flush=True)
                        return
                    m = man.metrics
                    trig = m["trigger_alt"]
                    minc = m["min_alt"]
                    w.writerow({
                        "trial": name,
                        "entry_vz_target": vz,
                        "entry_vz_actual": m["entry_vz"],
                        "strategy": strat, "axis": axis,
                        "throttle_cut_time": m["throttle_cut_time"],
                        "horizon_crossing_time": m["horizon_crossing_time"],
                        "peak_down_vz": m["peak_down_vz"],
                        "trigger_alt": trig,
                        "min_floor_clearance": minc,
                        "delta_z_loss": (trig - minc) if (trig is not None and minc is not None) else None,
                        "max_actuator_saturation_ms": _max_saturation_ms(res.rows),
                        "arrested": m["arrested"],
                        "outcome": res.outcome,
                    })
                    f.flush()
                    print(f"    entry_vz={m['entry_vz']}, dz_loss="
                          f"{(trig - minc) if (trig and minc is not None) else '?'}, "
                          f"outcome={res.outcome}", flush=True)
    finally:
        f.close()
    print("  -> tab3_recovery.csv", flush=True)


# ===========================================================================================
# Tab 4 - Kinematic feasibility cone (Task 1.4): flown validation. The analytical cone is
# derived from Tab 1/2 numbers in the report.
# ===========================================================================================
# NB: achievable speeds for THIS airframe are ~9 m/s at 30deg lean (SPEED_LEAN_TABLE), far below
# the blueprint's 15-35 m/s. We sweep realistic entry speeds and the report flags the gap.
FEAS_ENTRY_SPEEDS = [4.0, 7.0, 9.0]
FEAS_BANK = 0.5     # rad (~29 deg, the pilots' max strafe bank)
FEAS_FWD_DIST = 15.0
TAB4_HEADER = ["trial", "entry_speed_target", "entry_speed_actual", "bank_rad",
               "forward_distance", "lateral_offset", "longitudinal_offset", "outcome"]


def run_feasibility(runner, out_dir, setup_alt=45.0):
    print("\n=== Tab 4: Kinematic feasibility cone (flown validation) ===", flush=True)
    f, w = _writer(out_dir, "tab4_feasibility.csv", TAB4_HEADER)
    try:
        for v in FEAS_ENTRY_SPEEDS:
            name = f"feas_v{v:g}_bank{FEAS_BANK:g}"
            print(f"  {name}", flush=True)
            man = LateralStep(v, FEAS_BANK, forward_distance=FEAS_FWD_DIST)
            trial = Trial(name, man, setup_alt=setup_alt, timeout_s=14.0, floor_alt=2.0,
                          params={"entry_speed": v})
            res = runner.run(trial)
            if res.outcome == "aborted":
                print("  aborted by user.", flush=True)
                return
            bank_rows = [r for r in res.rows if r.get("phase") == "bank"]
            entry_actual = lateral = longitudinal = None
            if bank_rows:
                start = bank_rows[0]
                end = bank_rows[-1]
                entry_actual = start["vh"]
                # displacement in the world frame, split into the initial-heading axis and its
                # perpendicular using the heading (yaw) at bank start
                dx = end["x"] - start["x"]
                dy = end["y"] - start["y"]
                psi = start["yaw"]
                longitudinal = dx * math.cos(psi) + dy * math.sin(psi)
                lateral = -dx * math.sin(psi) + dy * math.cos(psi)
            w.writerow({
                "trial": name, "entry_speed_target": v, "entry_speed_actual": entry_actual,
                "bank_rad": FEAS_BANK, "forward_distance": FEAS_FWD_DIST,
                "lateral_offset": abs(lateral) if lateral is not None else None,
                "longitudinal_offset": longitudinal, "outcome": res.outcome,
            })
            f.flush()
            print(f"    lateral_offset={abs(lateral) if lateral is not None else '?'}, "
                  f"outcome={res.outcome}", flush=True)
    finally:
        f.close()
    print("  -> tab4_feasibility.csv", flush=True)


# ===========================================================================================
# Tab 5 - Closed-loop point tracking & station hold (Round-1 ODOMETRY only, no vision/gates)
# ===========================================================================================
# The missing closed-loop piece: accelerate to an entry speed, then close a world-frame position
# loop and stop dead at a known point. Conservative starting gains respect the 96 ms rotational
# lag and the weak passive drag (active nose-up does the braking); they will need tuning.
PT_TARGET_DISTS = [5.0, 10.0, 20.0]                 # m forward from spawn
PT_ENTRY_SPEEDS = [2.0, 4.0, 6.0, 8.0]              # m/s to spin up to before closing the loop
# Assumed active braking authority (~30 deg nose-up, g*tan(30) ~ 5.7 m/s^2, derated). Used only
# for the feasibility skip below - drops cells where the drone physically cannot stop in distance.
PT_BRAKE_ACCEL = 5.0                                # m/s^2
PT_KP_FWD, PT_KD_FWD, PT_KP_LAT, PT_KP_ALT = 0.3, 0.8, 0.3, 0.5
TAB5_HEADER = ["trial", "t", "phase", "x", "y", "alt", "vx_w", "vy_w", "vz_w", "vh",
               "roll", "pitch", "err_fwd", "err_lat", "err_alt",
               "cmd_roll_rate", "cmd_pitch_rate", "cmd_thrust"]


def run_point_tracking(runner, out_dir, setup_alt=45.0):
    print("\n=== Tab 5: Closed-loop point tracking & station hold ===", flush=True)
    f, w = _writer(out_dir, "tab5_point_tracking.csv", TAB5_HEADER)
    try:
        # ---- baseline: how stable is the closed-loop hover before we ask for any motion? ----
        name = "station_hold"
        print(f"  {name}", flush=True)
        man = StationHold(10.0, kp_fwd=PT_KP_FWD, kp_lat=PT_KP_LAT, kp_alt=PT_KP_ALT)
        trial = Trial(name, man, setup_alt=setup_alt, timeout_s=12.0, floor_alt=3.0,
                      params={"kind": name})
        res = runner.run(trial)
        if res.outcome == "aborted":
            print("  aborted by user.", flush=True)
            return
        for r in res.rows:
            r["trial"] = name
            w.writerow(r)
        f.flush()
        m = man.metrics
        print(f"    max_drift={_f2(m['max_drift_m'])} m, rms={_f2(m['rms_error_m'])} m, "
              f"alt_drift={_f2(m['altitude_drift_m'])} m, outcome={res.outcome}", flush=True)

        # ---- point approaches: dist x entry-speed matrix, skipping the un-stoppable corners ----
        for dist in PT_TARGET_DISTS:
            for v in PT_ENTRY_SPEEDS:
                stop_time = v / PT_BRAKE_ACCEL          # time to bleed v at the assumed braking accel
                if v * stop_time > 2.0 * dist:          # stopping distance > target -> can't stop in time
                    print(f"  skip pt_d{dist:g}_v{v:g} (entry speed too high to stop in {dist:g} m)",
                          flush=True)
                    continue
                name = f"pt_d{dist:g}_v{v:g}"
                print(f"  {name}", flush=True)
                man = PointApproach(dist, v, PT_KP_FWD, PT_KD_FWD, PT_KP_LAT, PT_KP_ALT)
                trial = Trial(name, man, setup_alt=setup_alt, timeout_s=20.0, floor_alt=3.0,
                              params={"target_dist": dist, "entry_speed": v})
                res = runner.run(trial)
                if res.outcome == "aborted":
                    print("  aborted by user.", flush=True)
                    return
                for r in res.rows:
                    r["trial"] = name
                    w.writerow(r)
                f.flush()
                m = man.metrics
                print(f"    entry_v={_f2(m['entry_speed_actual'])} m/s, "
                      f"overshoot={_f2(m['peak_overshoot_m'])} m, "
                      f"settle={_f2(m['settling_time_s'])} s, "
                      f"final_err={_f2(m['final_error_m'])} m, outcome={res.outcome}", flush=True)
    finally:
        f.close()
    print("  -> tab5_point_tracking.csv", flush=True)


# ===========================================================================================
# Tab 6 - Attitude-setpoint interface (send_attitude_setpoint sign/gain/lag characterisation)
# ===========================================================================================
# The rate tabs above characterise send_rate_attitude. This one characterises the ABSOLUTE-attitude
# interface the vision/hover pilots fly, whose signs were never measured. Each trial commands ONE
# fixed attitude (mode="attitude") and we log the ground-truth response.
ATT_ANGLES_DEG = [5, 10, 15, 20]                          # tilt magnitudes swept, both signs
ATT_THRUSTS = [0.15, 0.20, 0.25, 0.299, 0.35, 0.45, 0.60]  # level-hold thrust sweep
ATT_YAWS_DEG = [0, 45, -45, 90, 0]                        # absolute-yaw probe sequence
TAB6_HEADER = ["trial", "group", "seg", "t", "timestamp_ms",
               "cmd_roll_deg", "cmd_pitch_deg", "cmd_yaw_deg", "cmd_thr",
               "alt", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed",
               "vx_w", "vy_w", "vz_w", "vh", "climb_up", "up_align", "motor_max"]


def _write_att_rows(w, name, group, rows):
    for r in rows:
        w.writerow({
            # per-row group comes from the maneuver's extra (pitch/roll/yaw/thrust/level); fall back
            # to the battery-level label only if a row didn't tag one.
            "trial": name, "group": r.get("group", group), "seg": r.get("seg", ""),
            "t": r["t"],
            "timestamp_ms": (r["t_usec"] / 1000.0) if r.get("t_usec") else r["t"] * 1000.0,
            "cmd_roll_deg": math.degrees(r.get("cmd_roll", 0.0)),
            "cmd_pitch_deg": math.degrees(r.get("cmd_pitch", 0.0)),
            "cmd_yaw_deg": math.degrees(r.get("cmd_yaw", 0.0)),
            "cmd_thr": r.get("cmd_thr", 0.0),
            "alt": r["alt"], "roll": r["roll"], "pitch": r["pitch"], "yaw": r["yaw"],
            "rollspeed": r["rollspeed"], "pitchspeed": r["pitchspeed"], "yawspeed": r["yawspeed"],
            "vx_w": r["vx_w"], "vy_w": r["vy_w"], "vz_w": r["vz_w"],
            "vh": r["vh"], "climb_up": r["climb_up"], "up_align": r["up_align"],
            "motor_max": r.get("motor_max", 0.0),
        })


SETTLE_S = 2.0          # level-out between measurement holds (bleed off the accumulated velocity)
HOLD_S = 3.0            # pitch/roll hold time (reach terminal speed)
YAW_STEP_S = 2.5       # per yaw setpoint
THR_HOLD_S = 2.5       # per thrust level


def _attitude_phases():
    """The full sweep as one continuous script: pitch +/-, roll +/-, yaw steps, thrust steps, with a
    level settle before each. +/- magnitudes are interleaved so forward/back (and L/R) drift cancels."""
    settle = {"group": "level", "seg": "level", "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
              "thrust": None, "dur": SETTLE_S}
    ph = []
    for axis in ("pitch", "roll"):
        for deg in ATT_ANGLES_DEG:
            for sign in (+1, -1):
                ang = math.radians(sign * deg)
                ph.append(dict(settle))
                ph.append({"group": axis, "seg": f"{axis}{sign * deg:+d}",
                           "roll": ang if axis == "roll" else 0.0,
                           "pitch": ang if axis == "pitch" else 0.0,
                           "yaw": 0.0, "thrust": None, "dur": HOLD_S})
    ph.append(dict(settle))
    for i, d in enumerate(ATT_YAWS_DEG):
        ph.append({"group": "yaw", "seg": f"yaw{d:+d}_{i}", "roll": 0.0, "pitch": 0.0,
                   "yaw": math.radians(d), "thrust": HOVER_THRUST, "dur": YAW_STEP_S})
    for thr in ATT_THRUSTS:
        ph.append(dict(settle))
        ph.append({"group": "thrust", "seg": f"thr{thr:.3f}", "roll": 0.0, "pitch": 0.0,
                   "yaw": 0.0, "thrust": thr, "dur": THR_HOLD_S})
    return ph


def run_attitude(runner, out_dir, setup_alt=45.0):
    print("\n=== Tab 6: Attitude-setpoint interface (single continuous flight) ===", flush=True)
    phases = _attitude_phases()
    total = sum(p["dur"] for p in phases)
    print(f"  {len(phases)} phases, ~{total:.0f}s of flight - ONE reset/GO/climb, no per-trial resets",
          flush=True)
    f, w = _writer(out_dir, "tab6_attitude.csv", TAB6_HEADER)
    try:
        trial = Trial("att_script", AttitudeScript(phases), setup_alt=setup_alt,
                      timeout_s=total + 2.0, mode="attitude", floor_alt=2.0,
                      params={"group": "attitude"})
        res = runner.run(trial)
        if res.outcome == "aborted":
            print("  aborted by user.", flush=True)
        else:
            print(f"  flight outcome: {res.outcome} ({len(res.rows)} rows)", flush=True)
        _write_att_rows(w, "att_script", "attitude", res.rows)
    finally:
        f.close()
    print("  -> tab6_attitude.csv", flush=True)
