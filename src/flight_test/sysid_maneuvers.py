"""
Maneuver generators for the sysid campaign.

Each maneuver is a small object with `step(t, st)` that the TrialRunner calls once per control
tick. It returns either:
  (roll_rate, pitch_rate, yaw_rate, thrust)                  - command this, keep going
  (roll_rate, pitch_rate, yaw_rate, thrust, extra_dict)      - ... and tag the captured row
  None                                                       - maneuver finished (graceful end)

Two command styles are used here, both legitimate against the rate interface:
  - OPEN-LOOP body-rate: command a raw body rate directly (no sign mapping). This is how the
    existing characterize probes measure max roll/yaw rate, and how the flips are flown.
  - CLOSED-LOOP attitude: a P loop on measured attitude -> body rate, using the MEASURED signs
    in dynamics.py (ROLL_SIGN/PITCH_SIGN/YAW_SIGN). This is how the pilots hold a lean.

Inversion is tracked with st["up_align"] (= R[2][2]): +1 upright, 0 at the horizon, -1 inverted.
It is singularity-free, unlike Euler roll/pitch which blow up near vertical.
"""

import math

from common.dynamics import (G_ACC, HOVER_THRUST, KP_ATT, MAX_RATE, PITCH_SIGN, ROLL_SIGN, YAW_SIGN,
                      clamp, thrust_for_climb)

# A strong default flip rate for the aerobatic maneuvers. Deliberately above the pilots'
# MAX_RATE=6 clamp so we actually exercise the airframe's rotational authority.
FLIP_RATE = 12.0


def ang_err(target, current):
    """Shortest signed angular error target-current, wrapped to [-pi, pi]."""
    e = (target - current + math.pi) % (2 * math.pi) - math.pi
    return e


def axis_rates(axis, rate):
    """Body-rate triple for a named axis ('roll'/'pitch'/'yaw'/'oblique')."""
    if axis == "roll":
        return (rate, 0.0, 0.0)
    if axis == "pitch":
        return (0.0, rate, 0.0)
    if axis == "yaw":
        return (0.0, 0.0, rate)
    if axis == "oblique":   # simultaneous roll+pitch (diagonal flip)
        return (rate, rate, 0.0)
    raise ValueError(f"unknown axis {axis!r}")


class RateStep:
    """Task 1.1: an instantaneous body-rate step on one axis, held briefly to measure the
    response, then a CLOSED-LOOP re-level to stop the tumble before the next reset.

    The measurement (omega, alpha, lag, cross-axis drift) all happens in the first ~150 ms of the
    step, so a short hold is plenty. The earlier design held for 0.5 s then commanded an inverse
    open-loop step, which spun the drone through several flips and sent it crashing below the
    world - after which sim-reset could not recover it. Re-levelling instead keeps the post-trial
    state sane so reset reliably returns to spawn."""

    def __init__(self, axis, rate, thrust=HOVER_THRUST, hold_s=0.35, recover_s=0.65):
        self.axis = axis
        self.rate = rate
        self.thrust = thrust
        self.hold_s = hold_s
        self.recover_s = recover_s

    def step(self, t, st):
        if t < self.hold_s:
            rr, pr, yr = axis_rates(self.axis, self.rate)
            return (rr, pr, yr, self.thrust, {"seg": "step"})
        if t < self.hold_s + self.recover_s:
            # closed-loop drive back to level (shortest-angle), to arrest the rotation
            rr = clamp(ROLL_SIGN * KP_ATT * ang_err(0.0, st["roll"]), -MAX_RATE, MAX_RATE)
            pr = clamp(PITCH_SIGN * KP_ATT * ang_err(0.0, st["pitch"]), -MAX_RATE, MAX_RATE)
            yr = clamp(YAW_SIGN * KP_ATT * ang_err(0.0, st["yaw"]), -MAX_RATE, MAX_RATE)
            return (rr, pr, yr, self.thrust, {"seg": "recover"})
        return None


class HoldAttitude:
    """Closed-loop hold of a target roll/pitch (and heading) for a fixed duration.

    Uses the measured attitude signs, so this matches how the pilots fly. Capable of driving to
    a non-zero target (e.g. a fixed lean for a drag run, or 180 deg for an inverted hold)."""

    def __init__(self, des_roll, des_pitch, thrust, duration, des_yaw=0.0):
        self.des_roll = des_roll
        self.des_pitch = des_pitch
        self.des_yaw = des_yaw
        self.thrust = thrust
        self.duration = duration

    def step(self, t, st):
        if t > self.duration:
            return None
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(self.des_roll, st["roll"]), -MAX_RATE, MAX_RATE)
        pr = clamp(PITCH_SIGN * KP_ATT * ang_err(self.des_pitch, st["pitch"]), -MAX_RATE, MAX_RATE)
        yr = clamp(YAW_SIGN * KP_ATT * ang_err(self.des_yaw, st["yaw"]), -MAX_RATE, MAX_RATE)
        return (rr, pr, yr, self.thrust)


class DragRun:
    """Task 1.2 (translational): accelerate to a target speed on one lean axis, then CUT the
    lean to level and coast. During the level coast the only horizontal force is aerodynamic
    drag, so the measured horizontal deceleration IS the drag acceleration a(v)."""

    def __init__(self, direction, lean_rad, target_speed, hover_thrust=HOVER_THRUST,
                 accel_timeout=6.0, coast_timeout=6.0):
        self.direction = direction            # 'forward' (pitch) or 'lateral' (roll)
        self.lean = lean_rad
        self.target_speed = target_speed
        self.hover = hover_thrust
        self.accel_timeout = accel_timeout
        self.coast_timeout = coast_timeout
        self.phase = "accel"
        self._coast_t0 = None

    def step(self, t, st):
        des_roll = self.lean if self.direction == "lateral" else 0.0
        des_pitch = self.lean if self.direction == "forward" else 0.0

        if self.phase == "accel":
            # tilt-compensate thrust so altitude holds while leaning
            thrust = clamp(self.hover / max(math.cos(self.lean), 0.5), 0.0, 1.0)
            if st["vh"] >= self.target_speed or t > self.accel_timeout:
                self.phase = "coast"
                self._coast_t0 = t
            else:
                rr = clamp(ROLL_SIGN * KP_ATT * ang_err(des_roll, st["roll"]), -MAX_RATE, MAX_RATE)
                pr = clamp(PITCH_SIGN * KP_ATT * ang_err(des_pitch, st["pitch"]), -MAX_RATE, MAX_RATE)
                return (rr, pr, 0.0, thrust, {"phase": "accel"})

        # coast: level attitude, hover thrust; pure-drag deceleration
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(0.0, st["roll"]), -MAX_RATE, MAX_RATE)
        pr = clamp(PITCH_SIGN * KP_ATT * ang_err(0.0, st["pitch"]), -MAX_RATE, MAX_RATE)
        if st["vh"] < 1.0 or (t - self._coast_t0) > self.coast_timeout:
            return None
        return (rr, pr, 0.0, self.hover, {"phase": "coast"})


class InvertedDive:
    """Task 1.2 (inverted vertical drag): roll inverted and pin full thrust toward the floor.
    Terminal vz is where downward motor thrust + gravity balance airframe drag. Runs until a
    floor abort or timeout; the battery reads the steady terminal velocity from the rows."""

    def __init__(self, thrust=1.0, flip_rate=FLIP_RATE):
        self.thrust = thrust
        self.flip_rate = flip_rate
        self.phase = "invert"

    def step(self, t, st):
        if self.phase == "invert":
            if st["up_align"] < -0.85:          # inverted enough
                self.phase = "dive"
            else:
                # flip via roll, no thrust while crossing the horizon
                return (self.flip_rate, 0.0, 0.0, 0.0, {"phase": "invert"})
        # dive: hold inverted (small roll correction toward 180), full thrust -> accelerate down
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(math.pi, st["roll"]), -MAX_RATE, MAX_RATE)
        return (rr, 0.0, 0.0, self.thrust, {"phase": "dive"})


class Recovery:
    """Task 1.3: the full inverted-recovery maneuver, recording the phase-plane metrics.

    Phases: invert -> drive (accelerate down to a target entry velocity) -> recover (rotate
    back, either keeping 100% thrust = 'continuous' or chopping to 0% = 'snap') -> catch (full
    thrust up to arrest the descent). Exposes self.metrics for the battery."""

    def __init__(self, target_entry_vz, strategy, axis, dive_thrust=1.0, flip_rate=FLIP_RATE,
                 min_drive_alt=8.0):
        self.target_entry_vz = target_entry_vz      # desired downward speed (m/s, positive)
        self.strategy = strategy                    # 'continuous' or 'snap'
        self.axis = axis                            # 'roll' / 'pitch' / 'oblique'
        self.dive_thrust = dive_thrust
        self.flip_rate = flip_rate
        self.min_drive_alt = min_drive_alt          # bail out of the dive above this altitude
        self.phase = "invert"
        self.metrics = {
            "entry_vz": None, "trigger_alt": None, "throttle_cut_time": None,
            "horizon_crossing_time": None, "peak_down_vz": 0.0, "min_alt": None, "arrested": False,
        }

    def _track(self, st):
        down = -st["climb_up"]
        if down > self.metrics["peak_down_vz"]:
            self.metrics["peak_down_vz"] = down
        if self.metrics["min_alt"] is None or st["alt"] < self.metrics["min_alt"]:
            self.metrics["min_alt"] = st["alt"]

    def step(self, t, st):
        self._track(st)
        down = -st["climb_up"]

        if self.phase == "invert":
            if st["up_align"] < -0.85:
                self.phase = "drive"
            else:
                rr, pr, yr = axis_rates(self.axis, self.flip_rate)
                return (rr, pr, yr, 0.0, {"phase": "invert"})

        if self.phase == "drive":
            # accelerate downward until we hit the target entry speed (or run low on altitude)
            if down >= self.target_entry_vz or st["alt"] < self.min_drive_alt:
                self.metrics["entry_vz"] = down
                self.metrics["trigger_alt"] = st["alt"]
                self.phase = "recover"
                if self.strategy == "snap":
                    self.metrics["throttle_cut_time"] = t
            else:
                rr = clamp(ROLL_SIGN * KP_ATT * ang_err(math.pi, st["roll"]), -MAX_RATE, MAX_RATE)
                return (rr, 0.0, 0.0, self.dive_thrust, {"phase": "drive"})

        if self.phase == "recover":
            thrust = 0.0 if self.strategy == "snap" else self.dive_thrust
            # crossed back above the horizon?
            if st["up_align"] > 0.0:
                if self.metrics["horizon_crossing_time"] is None:
                    self.metrics["horizon_crossing_time"] = t
                self.phase = "catch"
            else:
                rr, pr, yr = axis_rates(self.axis, self.flip_rate)
                return (rr, pr, yr, thrust, {"phase": "recover"})

        # catch: full thrust up, drive back to level, until the descent is arrested
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(0.0, st["roll"]), -MAX_RATE, MAX_RATE)
        pr = clamp(PITCH_SIGN * KP_ATT * ang_err(0.0, st["pitch"]), -MAX_RATE, MAX_RATE)
        if st["climb_up"] > -0.5:               # downward velocity essentially arrested
            self.metrics["arrested"] = True
            return None
        return (rr, pr, 0.0, 1.0, {"phase": "catch"})


class InvertedProbe:
    """Tier-4 feasibility gate (run before the recovery grid). Tries to (a) reach inverted and
    (b) keep rotating at zero collective, recording what actually happened. The battery reads
    self.metrics to decide whether the inverted regime is supported at all."""

    def __init__(self, flip_rate=FLIP_RATE, hold_s=2.0):
        self.flip_rate = flip_rate
        self.hold_s = hold_s
        self.metrics = {"reached_inverted": False, "min_up_align": 1.0,
                        "rotated_at_zero_thrust": False, "rate_at_zero_thrust": 0.0}
        self._t_inverted = None

    def step(self, t, st):
        self.metrics["min_up_align"] = min(self.metrics["min_up_align"], st["up_align"])
        if st["up_align"] < -0.85:
            self.metrics["reached_inverted"] = True
        # always command a roll rate at ZERO thrust: does the airframe keep rotating with no
        # collective? (real quads lose authority at idle; the sim may differ - that's the test)
        if abs(st["rollspeed"]) > 1.0:
            self.metrics["rotated_at_zero_thrust"] = True
        self.metrics["rate_at_zero_thrust"] = max(self.metrics["rate_at_zero_thrust"],
                                                  abs(st["rollspeed"]))
        if t > self.hold_s:
            return None
        return (self.flip_rate, 0.0, 0.0, 0.0, {"phase": "probe"})


class LateralStep:
    """Task 1.4 flown validation: reach a forward entry speed, then hold a fixed bank while
    continuing forward, so the battery can measure the lateral displacement achieved over a
    given forward distance and compare it to the analytical feasibility cone."""

    def __init__(self, entry_speed, bank_rad, forward_distance=15.0, hover_thrust=HOVER_THRUST,
                 spinup_timeout=6.0, bank_timeout=6.0):
        self.entry_speed = entry_speed
        self.bank = bank_rad
        self.forward_distance = forward_distance
        self.hover = hover_thrust
        self.spinup_timeout = spinup_timeout
        self.bank_timeout = bank_timeout
        self.phase = "spinup"
        self._start = None          # (x, y) at the moment banking begins
        self._t_bank0 = None
        self.lean = math.atan2(self.entry_speed_accel(), G_ACC)

    def entry_speed_accel(self):
        # a modest forward lean to reach the entry speed; the exact value isn't critical
        return min(self.entry_speed, 9.0)

    def step(self, t, st):
        if self.phase == "spinup":
            thrust = clamp(self.hover / max(math.cos(self.lean), 0.5), 0.0, 1.0)
            if st["vh"] >= self.entry_speed or t > self.spinup_timeout:
                self.phase = "bank"
                self._start = (st["x"], st["y"])
                self._t_bank0 = t
            else:
                pr = clamp(PITCH_SIGN * KP_ATT * ang_err(self.lean, st["pitch"]), -MAX_RATE, MAX_RATE)
                return (0.0, pr, 0.0, thrust, {"phase": "spinup"})

        # bank: hold roll + keep the forward lean; stop after travelling forward_distance
        travelled = math.hypot(st["x"] - self._start[0], st["y"] - self._start[1])
        if travelled >= self.forward_distance or (t - self._t_bank0) > self.bank_timeout:
            return None
        thrust = clamp(self.hover / max(math.cos(self.lean) * math.cos(self.bank), 0.4), 0.0, 1.0)
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(self.bank, st["roll"]), -MAX_RATE, MAX_RATE)
        pr = clamp(PITCH_SIGN * KP_ATT * ang_err(self.lean, st["pitch"]), -MAX_RATE, MAX_RATE)
        return (rr, pr, 0.0, thrust, {"phase": "bank"})
