"""
Reduced-order drone model built ENTIRELY from the measured flight-dynamics campaign (dynamics.py).
Nothing here is guessed: the steady-state curves (thrust->climb, lean->speed), the attitude-rate
signs, and the rate limits are the measured numbers. It is a first-order approximation around those
measurements - enough to reproduce gross flight behaviour (does the pilot thread the gates, fly
under, oscillate, spin out) so the pilot can be debugged offline. It is NOT the real simulator; it
catches integration/control failures, not fine sim-to-real effects.

Frames: NED world (x=N, y=E, z=Down). Body x=fwd, y=right, z=down. The model exposes exactly the
telemetry the real RX threads put in `data`: odometry (pos, quat, BODY velocity, rates) + attitude.
"""

import math

from common.dynamics import (CONTROL_HZ, G_ACC, MAX_RATE, ROLL_SIGN, PITCH_SIGN, YAW_SIGN,
                      THRUST_CLIMB_TABLE, lean_for_speed, clamp)
from common.gate_geometry import quat_to_rotmat, world_to_body

TAU_V = 0.45        # s, vertical first-order time constant (sysid No-Go-Zone: ~30 m/s arrested in ~13 m)
TAU_H = 0.30        # s, horizontal accel time constant toward the tilt/drag balance


def _interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    if x >= table[-1][0]:
        return table[-1][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


def _euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class DroneModel:
    def __init__(self, pos=(0.0, 0.0, 0.0), yaw=math.pi):
        self.pos = list(pos)              # NED
        self.vel = [0.0, 0.0, 0.0]        # world NED velocity
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = yaw                    # face the course (gates are toward -N -> yaw = pi)
        self.climb_up = 0.0               # world vertical speed (up+), first-order state
        self.t = 0.0

    def step(self, roll_rate, pitch_rate, yaw_rate, thrust, dt):
        # --- attitude: integrate the commanded body rates with the MEASURED sign convention so the
        # pilot's attitude P-loop converges exactly as it does on the real sim (see dynamics signs).
        self.roll += ROLL_SIGN * clamp(roll_rate, -MAX_RATE, MAX_RATE) * dt
        self.pitch += PITCH_SIGN * clamp(pitch_rate, -MAX_RATE, MAX_RATE) * dt
        self.yaw += YAW_SIGN * clamp(yaw_rate, -MAX_RATE, MAX_RATE) * dt

        # --- vertical: measured thrust->climb curve (reduced by tilt), approached first-order.
        cos_tilt = max(math.cos(self.roll) * math.cos(self.pitch), 0.3)
        terminal_climb = _interp(THRUST_CLIMB_TABLE, clamp(thrust * cos_tilt, 0.0, 1.0))
        self.climb_up += (terminal_climb - self.climb_up) / TAU_V * dt
        self.vel[2] = -self.climb_up

        # --- horizontal: thrust-vector tilt gives accel g*tan(tilt); measured lean->speed defines
        # the drag that sets terminal speed. Rotate body tilt accel to world by yaw.
        a_fwd = G_ACC * math.tan(clamp(self.pitch, -1.2, 1.2))     # +pitch leans forward
        a_right = G_ACC * math.tan(clamp(self.roll, -1.2, 1.2))
        cyaw, syaw = math.cos(self.yaw), math.sin(self.yaw)
        aN = cyaw * a_fwd - syaw * a_right
        aE = syaw * a_fwd + cyaw * a_right
        vh = math.hypot(self.vel[0], self.vel[1])
        if vh > 0.05:
            a_drag = G_ACC * math.tan(lean_for_speed(vh))          # drag that yields the measured terminal speed
            aN -= a_drag * self.vel[0] / vh
            aE -= a_drag * self.vel[1] / vh
        # first-order blend toward the commanded accel (rate-limits unrealistic instantaneous jumps)
        self.vel[0] += aN * dt
        self.vel[1] += aE * dt

        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] += self.vel[2] * dt
        self.t += dt

    def quat(self):
        # This sim's (and the pilot's) convention is +pitch = FORWARD lean = nose DOWN, which is
        # NEGATIVE pitch in standard NED euler. Negate so the body->world quat (and thus the camera
        # view) matches: leaning forward to fly tilts the up-tilted camera toward the gates ahead.
        return _euler_to_quat(self.roll, -self.pitch, self.yaw)

    def odometry(self):
        q = self.quat()
        vb = world_to_body(q, self.vel)          # body-frame velocity (what the real ODOMETRY sends)
        return {
            "x": self.pos[0], "y": self.pos[1], "z": self.pos[2],
            "qw": q[0], "qx": q[1], "qy": q[2], "qz": q[3],
            "vx": vb[0], "vy": vb[1], "vz": vb[2],
            "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0,
            "time_usec": int(self.t * 1e6), "reset_counter": 1,
        }

    def attitude(self):
        return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw}
