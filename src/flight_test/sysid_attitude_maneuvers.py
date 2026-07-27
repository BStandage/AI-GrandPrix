"""
Attitude-setpoint interface characterisation - ONE CONTINUOUS FLIGHT (Tab 6).

The other batteries reset the sim between every trial. In a scored race each reset drops the race
out of "live", so a reset-per-trial battery just sits at the start gate forever. This one instead
flies the WHOLE sweep in a single flight after one GO: it steps through a scripted list of attitude
holds (pitch +/-, roll +/-, yaw steps, thrust steps), levelling briefly between each, and never
resets. The runner sends each returned (roll, pitch, yaw, thrust) as an ABSOLUTE attitude setpoint
(Trial mode="attitude") and logs the ground-truth response every tick.
"""

import math

from common.dynamics import HOVER_THRUST, clamp


def _tilt_thrust(roll, pitch, hover=HOVER_THRUST):
    """Collective that keeps the VERTICAL thrust component at hover while tilted (floor cos at 0.4)."""
    return clamp(hover / max(math.cos(roll) * math.cos(pitch), 0.4), 0.0, 1.0)


class AttitudeScript:
    """One continuous scripted flight. `phases` is a list of dicts:
        {group, seg, roll, pitch, yaw, thrust (or None for tilt-comp hover), dur}
    step(t) walks the phases by cumulative time and streams that phase's absolute attitude setpoint,
    tagging each row with its group/seg/commanded values. Returns None when the script is done."""

    def __init__(self, phases):
        self.phases = phases
        self.total = sum(p["dur"] for p in phases)

    def step(self, t, st):
        acc = 0.0
        for ph in self.phases:
            if t < acc + ph["dur"]:
                thr = ph["thrust"] if ph["thrust"] is not None else _tilt_thrust(ph["roll"], ph["pitch"])
                return (ph["roll"], ph["pitch"], ph["yaw"], thr,
                        {"group": ph["group"], "seg": ph["seg"], "cmd_roll": ph["roll"],
                         "cmd_pitch": ph["pitch"], "cmd_yaw": ph["yaw"], "cmd_thr": thr})
            acc += ph["dur"]
        return None
