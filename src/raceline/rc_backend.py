"""Shared RC-stick backend: state estimate + the flight-proven control
loops that turn desired accel / altitude / yaw into Betaflight sticks.

Used by the trajectory follower AND the sysid diagnostics, so both fly the
identical plant interface. This module is also the sim->real boundary: on
hardware, GroundTruthSource is replaced by an estimator and the RCCommand
goes to a UART writer - these loops and their toml gains carry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from raceline.config import G


@dataclass
class StateEstimate:
    p: np.ndarray      # world position [x, y, z]
    v: np.ndarray      # world velocity [vx, vy, vz]
    R: np.ndarray      # body->world rotation, 3x3
    yaw: float         # world yaw (CCW from +x)
    omega: np.ndarray = None   # BODY angular rate [wx, wy, wz] (rad/s)


def rot_from_quat(q) -> np.ndarray:
    """3x3 body->world rotation from Elodin scalar-last quat [qx,qy,qz,qw]."""
    qx, qy, qz, qw = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


class GroundTruthSource:
    """Sim-only state source: ground-truth pose/velocity from the update."""

    def estimate(self, u) -> StateEstimate:
        R = rot_from_quat(u.world_pos[0:4])
        # world_vel is a spatial twist [angular xyz, linear xyz]; the body
        # angular rate is R^T * world angular velocity (used for attitude
        # rate damping so the fast plant does not overshoot/hunt).
        omega_w = np.asarray(u.world_vel[0:3], dtype=float)
        return StateEstimate(
            p=np.asarray(u.world_pos[4:7], dtype=float),
            v=np.asarray(u.world_vel[3:6], dtype=float),
            R=R,
            yaw=math.atan2(R[1, 0], R[0, 0]),
            omega=R.T @ omega_w,
        )


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class AltitudeLoop:
    """Throttle loop in ACCELERATION units, closed through the measured
    thrust curve and the actual tilt.

        a_cmd = kp_z*(z_t - z) + kd_z*(vz_ff - vz) + I        [m/s^2]
        T     = (G + a_cmd) / cos(tilt)                        [m/s^2]
        pwm   = curve^-1(T)                                    [thrust] table

    Why (2026-09-09, fitted to thrust_002 open-loop steps, unverified in a
    race): the previous loop summed PWM around hover_pwm with NO tilt term.
    A 64 deg bank keeps only cos(64) = 0.44 of thrust vertical, so the P
    term alone had to find ~260 PWM of lift and the drone sagged ~1 m per
    hard bank, then ballooned when the bank relaxed at the gate (the
    +-0.5 m altitude ring hitting gate tops; the 2026-09-08 fall-out-of-
    bank at g2). The one-line "full tilt comp" tried before scaled PWM
    linearly from pwm_min, but the real curve is 40 percent steeper than
    that above hover, so it over-delivered ~1 m/s^2 and floated every
    gate high. Inverting the measured table makes both the compensation
    and the loop gain exact across the throttle range.

    The integral only runs on small in-flight errors: winding up during the
    takeoff climb measurably biased altitude +0.4 m for ~30 s.
    """

    I_CLAMP = 3.0        # m/s^2 of integral authority (was +-60 PWM ~ 3.8)
    BALLOON_ACC = -2.5   # m/s^2: well above target, command at least this
                         # much descent (was hover-60 PWM ~ -3.7 m/s^2)
    COS_TILT_MIN = 0.25  # cap the compensation at ~75 deg of tilt
    MAX_THRUST_G = 2.0   # HARD CEILING ON THE ACTUATOR, not on any estimate.
                         # The only previous bound here was pwm_max, which on
                         # the measured curve is 5.98 g - six times the
                         # aircraft's weight, available on every tick of every
                         # flight. d45's hover loop used it: it asked for 1837
                         # PWM, 4.8 g, to reach a target 0.76 m off the ground,
                         # because a bad barometer sample said it was four
                         # metres low and nothing downstream said that was
                         # absurd.
                         #
                         # Every other guard in this file reasons about whether
                         # a NUMBER is trustworthy. This one asks what the
                         # aircraft is being told to DO. The k 0.10 plan's
                         # steepest climb between gates is 1.30 m/s, which with
                         # tilt compensation needs about 1.4 g, so 2.0 leaves
                         # half again as much headroom as the course asks for
                         # and still refuses to send six.
    CLIMB_ACC_MAX = 12.0 # m/s^2: most climb any altitude error may buy. The
                         # plans' steepest is under 10 at the g9 climb, so this
                         # never binds in normal flight - it exists so that one
                         # bad altitude sample cannot ask for full throttle.

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.i_term = 0.0
        self.last_t = 0.0
        self.a_cmd = 0.0     # last commanded vertical accel (for logging)
        self.thrust = 0.0    # last commanded specific thrust (for logging)

    def throttle(self, t: float, est: StateEstimate, z_target: float,
                 vz_ff: float, airborne: bool, integrate: bool,
                 az_ff: float = 0.0) -> int:
        f, th = self.cfg.follower, self.cfg.thrust
        dt = max(1e-3, t - self.last_t)
        self.last_t = t
        err = z_target - est.p[2]
        if integrate and airborne and abs(err) < 0.5:
            self.i_term = clamp(self.i_term + err * dt * f.ki_z,
                                -self.I_CLAMP, self.I_CLAMP)
        if not airborne and est.v[2] < 0.7:
            self.a_cmd, self.thrust = 0.0, 0.0
            return int(f.takeoff_pwm)
        # az_ff: the plan's vertical acceleration at the carrot, so the loop
        # LEADS a climb instead of chasing it (race_053: 0.3-0.5 m behind
        # the plan through the g9 climb with 500 PWM of throttle unused)
        a_cmd = f.kp_z * err + f.kd_z * (vz_ff - est.v[2]) + self.i_term + az_ff
        # no-balloon guard: aggressive braking transients lifted the drone
        # 1.4 m above its line and into a gate's top bar. Well above target
        # the loop may not command a climb.
        if err < -0.4:
            a_cmd = min(a_cmd, self.BALLOON_ACC)
        # CLIMB AUTHORITY CAP. A large altitude error otherwise asks for a
        # proportionally large acceleration with nothing between it and the
        # motors. d45, 2026-09-20: one prop-wash barometer sample read -3.86 m
        # against a 0.15 m target, the loop concluded it was 4 m low, and asked
        # for +32.9 m/s^2 - 4.35 g, near full throttle - on the strength of a
        # single reading. The aircraft obeyed and a 0.4 m hover reached 2 m.
        #
        # The sensor is fixed at the source (FcStateSource rejects impossible
        # jumps) and physically with foam over the port, but no altitude error,
        # however genuine, justifies more climb than this. A plan's steepest
        # climb asks for a few m/s^2; the ceiling at g9 is under 10.
        a_cmd = min(a_cmd, self.CLIMB_ACC_MAX)
        # exact tilt compensation: only cos(tilt) of the thrust is vertical
        cos_tilt = max(float(est.R[2, 2]), self.COS_TILT_MIN)
        thrust = max(0.0, (G + a_cmd) / cos_tilt)
        thrust = min(thrust, self.MAX_THRUST_G * G)
        self.a_cmd, self.thrust = a_cmd, thrust
        pwm = self.cfg.pwm_for_thrust(thrust)
        return int(round(clamp(pwm, th.pwm_min, th.pwm_max)))


def attitude_sticks(cfg, est: StateEstimate, a_des, a_z: float = 0.0) -> tuple:
    """World-frame desired accel -> roll/pitch sticks via the tilt-vector
    error expressed in body frame (the proven acro loop).

    a_z is the altitude loop's vertical acceleration demand: the thrust
    vector to point along is (ax, ay, G + a_z). Sized against G alone
    (race_045) an 18 m/s^2 climb demand plus an 18 m/s^2 horizontal one
    became a 62 deg tilt, the altitude loop then scaled thrust by
    1/cos(62) to keep its vertical, and the total ran past the motors
    into g10-top's frame."""
    f = cfg.follower
    ax, ay = float(a_des[0]), float(a_des[1])
    # The vertical demand enters the mapping only when CLIMBING. Descending,
    # the throttle does the descent and the tilt serves the horizontal
    # demand against hover thrust: with a_z of -20 in the stack's drop even
    # a 0.5 g floor gave 77-84 deg of tilt, and the pull-out throttle then
    # shoved the drone 1 m sideways into g10-low's post (race_046, race_051).
    # a_z may be negative when the caller runs a coherent thrust vector
    # (solvers.follower THRUST_VECTOR_MODE clips it at -0.85 g itself);
    # other callers pass 0 or a climb demand
    gz = max(G + float(a_z), 0.15 * G)
    n = math.sqrt(ax * ax + ay * ay + gz * gz)
    zd = np.array([ax / n, ay / n, gz / n])
    zb = est.R[:, 2]
    e = np.cross(zb, zd)
    eb = est.R.T @ e
    # Rate damping: oppose the current body roll/pitch rate as the attitude
    # approaches target, so the proportional loop stops overshooting on the
    # fast aerobatic plant (pitch fwd/back hunting). Body omega x=roll rate,
    # y=pitch rate. kw_att is the damping gain.
    wx = float(est.omega[0]) if est.omega is not None else 0.0
    wy = float(est.omega[1]) if est.omega is not None else 0.0
    roll = int(round(clamp(1500.0 + f.ka_att * eb[0] - f.kw_att * wx,
                           1500 - f.stick_clamp, 1500 + f.stick_clamp)))
    pitch = int(round(clamp(1500.0 + f.ka_att * eb[1] - f.kw_att * wy,
                            1500 - f.stick_clamp, 1500 + f.stick_clamp)))
    return roll, pitch, eb


def angle_sticks(cfg, est: StateEstimate, a_des, a_z: float = 0.0) -> tuple:
    """World-frame desired accel -> roll/pitch sticks for Betaflight ANGLE
    mode, where the stick commands a TILT ANGLE (full deflection =
    `angle_limit`, linear) and the FC closes the attitude loop itself.

    Same thrust-vector target as attitude_sticks: point the body z axis at
    (ax, ay, G + a_z). Expressed in the yaw-aligned frame (forward, left):
    pitch = asin(forward component), roll = -asin(left component / cos
    pitch). Signs follow the measured stick conventions: +pitch stick =
    nose down = accelerate forward, +roll stick = roll right = accelerate
    toward -y (right). Yaw uses est.yaw only - no attitude feedback is
    needed here because the FC does the levelling; that is the point.
    Returns (roll, pitch, (roll_deg, pitch_deg))."""
    f = cfg.follower
    limit = float(getattr(f, "angle_limit_deg", 80.0))
    ax, ay = float(a_des[0]), float(a_des[1])
    gz = max(G + float(a_z), 0.15 * G)
    n = math.sqrt(ax * ax + ay * ay + gz * gz)
    zx, zy = ax / n, ay / n                      # desired body-z, world xy
    c, s = math.cos(est.yaw), math.sin(est.yaw)
    fwd = c * zx + s * zy                        # along the nose
    left = -s * zx + c * zy                      # along body +y (FLU)
    pitch_rad = math.asin(clamp(fwd, -1.0, 1.0))
    cp = max(math.cos(pitch_rad), 1e-3)
    roll_rad = -math.asin(clamp(left / cp, -1.0, 1.0))
    lim = math.radians(limit)
    pitch_rad = clamp(pitch_rad, -lim, lim)
    roll_rad = clamp(roll_rad, -lim, lim)
    roll = int(round(1500.0 + 500.0 * roll_rad / lim))
    pitch = int(round(1500.0 + 500.0 * pitch_rad / lim))
    return roll, pitch, (math.degrees(roll_rad), math.degrees(pitch_rad))


class YawLoop:
    """Yaw stick toward a target heading, holding the last valid target."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.hold = None

    def stick(self, est: StateEstimate, yaw_des) -> int:
        f = self.cfg.follower
        if yaw_des is not None:
            self.hold = yaw_des
        if self.hold is None:
            return 1500
        yerr = (self.hold - est.yaw + math.pi) % (2 * math.pi) - math.pi
        # +stick = yaw right = world yaw DECREASES (measured), hence the minus
        return int(round(clamp(1500.0 - f.kyaw * yerr,
                               1500 - f.yaw_clamp, 1500 + f.yaw_clamp)))
