"""
The seeker pilot: brain + the proven loops -> RC sticks, in ANGLE mode.

Shared by the sim adapter (solvers.seeker) and the Orin runtime
(hardware.runtime), so the drone flies the code the sim validated.

    pilot = SeekerPilot(cfg, crossings, start_xy=(0, 0), laps=2)
    out = pilot.tick(t, est, det, baro_fresh)   # -> Sticks

`est` is a raceline.rc_backend.StateEstimate built from SENSORS ONLY:
p = (0, 0, altitude), v = (0, 0, vertical speed), R and yaw from the
attitude. The horizontal position is never read.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import os

from raceline.rc_backend import AltitudeLoop, StateEstimate, YawLoop, angle_sticks, attitude_sticks
from seeker.brain import Command, Crossing, Detection, SeekerBrain, SeekerConfig

T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.55
# Output stage. ANGLE (AIGP_ANGLE_MODE=1): tilt-angle sticks, the FC levels
# itself - the hardware control shape. ACRO (default): the proven
# thrust-vector attitude loop on est.R (rate sticks) - what the sim's SITL
# flies stably; the sim's ANGLE mode still has an unresolved attitude-feed
# problem (2026-09-16). The brain and every loop above this line are
# identical in both.
ANGLE_MODE = os.environ.get("AIGP_ANGLE_MODE", "0") == "1"
AUX2_ANGLE = 1800 if ANGLE_MODE else 1500


@dataclass
class Sticks:
    throttle: int = 1000
    roll: int = 1500
    pitch: int = 1500
    yaw: int = 1500
    arm: int = 1000
    aux2: int = AUX2_ANGLE
    phase: str = "INIT"
    crossing: int = 0
    done: bool = False
    z_target: float = 0.0
    yaw_target: float = 0.0
    a_des: tuple = (0.0, 0.0)
    tilt_deg: tuple = (0.0, 0.0)


class SeekerPilot:
    def __init__(self, cfg, crossings: Sequence[Crossing], start_xy=(0.0, 0.0), laps: int = 2,
                 seeker_cfg: Optional[SeekerConfig] = None, t0: float = 0.0):
        self.cfg = cfg
        self.brain = SeekerBrain(crossings, seeker_cfg or SeekerConfig(), start_xy=start_xy, laps=laps)
        self.alt = AltitudeLoop(cfg)
        self.yaw = YawLoop(cfg)
        self.t0 = t0
        self.last: Optional[Command] = None
        self.zt = None                 # slew-limited altitude target
        self.t_zt = None
        self.zt_rate_mps = 1.2         # a step target (takeoff, the stacked gate) would overshoot on a lagging vz

    @property
    def done(self) -> bool:
        return self.brain.done

    def tick(self, t: float, est: Optional[StateEstimate], det: Optional[Detection],
             baro_fresh: bool = True) -> Sticks:
        tr = t - self.t0
        if tr < T_DISARMED_END or est is None:
            return Sticks(arm=1000, throttle=1000, phase="INIT")
        if tr < T_ARM_IDLE_END:
            return Sticks(arm=1800, throttle=1000, phase="ARM")
        cmd = self.brain.step(t, est.yaw, float(est.p[2]), float(est.v[2]), det)
        self.last = cmd
        if cmd.done:
            return Sticks(arm=1000, throttle=1000, phase=cmd.phase, crossing=cmd.crossing, done=True)
        airborne = float(est.p[2]) >= self.cfg.follower.min_alt_translation_m
        if self.zt is None:
            self.zt, self.t_zt = float(est.p[2]), t
        dz_max = self.zt_rate_mps * max(0.0, min(0.1, t - self.t_zt))
        self.t_zt = t
        self.zt += max(-dz_max, min(dz_max, cmd.z_target - self.zt))
        throttle = self.alt.throttle(t, est, self.zt, 0.0, airborne, baro_fresh, 0.0)
        if ANGLE_MODE:
            roll, pitch, ang = angle_sticks(self.cfg, est, cmd.a_des, self.alt.a_cmd)
        else:
            roll, pitch, eb = attitude_sticks(self.cfg, est, cmd.a_des, self.alt.a_cmd)
            ang = (float(eb[0]), float(eb[1]))
        yaw_stick = self.yaw.stick(est, cmd.yaw_target)
        return Sticks(throttle=int(throttle), roll=roll, pitch=pitch, yaw=yaw_stick,
                      arm=1800 if cmd.arm else 1000, phase=cmd.phase, crossing=cmd.crossing,
                      z_target=self.zt, yaw_target=cmd.yaw_target, a_des=cmd.a_des,
                      tilt_deg=ang)


def crossings_from_course(course) -> list[Crossing]:
    """sim.pq_course.RaceCourse -> the brain's per-lap crossing list (sim frame)."""
    return [Crossing(c.label, c.x, c.y, c.z, c.heading_rad) for c in course.crossings]
