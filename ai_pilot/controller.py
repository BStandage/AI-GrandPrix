"""
Top-level control loop: pick a flight mode each tick, run it, print a status readout.

Modes (set CONTROL_MODE):
  "trajectory"   - follow a racing line through the gates (fast; the default)   [trajectory_pilot]
  "pursuit"      - chase each gate in turn (proven ~34 s fallback)              [pursuit_pilot]
  "keyboard"     - manual rate-mode flight for data collection                 [dev_modes]
  "characterize" - fly a scripted profile and log the physics envelope         [dev_modes]
"""

import math
import time

from pymavlink import mavutil

from dynamics import CONTROL_HZ, MAVLINK_CMD_SIM_RESET
from gate_geometry import active_gate_relative
from dev_modes import char_phase_at, update_characterize_control, update_keyboard_rate_control
from pursuit_pilot import update_pursuit_control
from trajectory_pilot import update_trajectory_control

CONTROL_MODE = "trajectory"
GATE_READOUT_PERIOD_S = 0.5   # print a status line this often


class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms
        self.tick = 0

    def update(self):
        if CONTROL_MODE == "trajectory":
            update_trajectory_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "pursuit":
            update_pursuit_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "keyboard":
            update_keyboard_rate_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "characterize":
            update_characterize_control(self.sim_conn, self.system_boot_ms, self.data)
        else:
            raise ValueError(f"unknown CONTROL_MODE: {CONTROL_MODE!r}")

        self._readout()
        time.sleep(1.0 / CONTROL_HZ)

    def _readout(self):
        self.tick += 1
        if self.tick % int(CONTROL_HZ * GATE_READOUT_PERIOD_S) != 0:
            return

        odo = self.data.get("odometry") or self.data.get("local_position_ned") or {}
        vz = odo.get("vz")   # NED: + is downward
        z = odo.get("z")
        climb_str = f"{-vz:+.2f}" if vz is not None else " n/a"

        if CONTROL_MODE == "characterize":
            t = time.time() - self.data.get("char_t0", time.time())
            label, _ = char_phase_at(t)
            vx, vy = odo.get("vx", 0.0), odo.get("vy", 0.0)
            print(
                f"char[{label}] t={t:5.1f}s  climb={climb_str}m/s  "
                f"speed_h={math.hypot(vx, vy):5.2f}m/s  alt={-(z or 0.0):+6.2f}m",
                flush=True,
            )
            return

        if CONTROL_MODE == "keyboard":
            thr = self.data.get("manual_thrust", 0.0)
            print(f"manual: thr={thr:.3f}  climb={climb_str}m/s  alt={-(z or 0.0):+6.2f}m", flush=True)
            return

        if CONTROL_MODE == "trajectory":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("traj_regime", "?")
            rel = active_gate_relative(self.data)   # for reference: nearest real gate
            gatestr = f"gate {rel['gate_id']} dist={rel['distance']:4.1f}m" if rel else "no gate"
            print(
                f"traj[{regime}] xtrack={self.data.get('traj_xtrack', 0.0):4.1f}m "
                f"yaw={self.data.get('traj_yawrate', 0.0):+4.1f} "
                f"v={self.data.get('traj_vcur', 0.0):4.1f}/{self.data.get('traj_vtgt', 0.0):3.1f}m/s "
                f"climb={climb_str} alt={-(z or 0.0):+6.1f}m | {gatestr} thr={thr:.3f}",
                flush=True,
            )
            return

        # pursuit readout: show the regime + the active gate it's chasing
        thr = self.data.get("oracle_thrust", 0.0)
        regime = self.data.get("pursuit_regime", "?")
        rs = self.data.get("race_status") or {}
        rel = active_gate_relative(self.data)
        if rel is not None:
            dc = self.data.get("pursuit_desired_climb", 0.0)
            print(
                f"pursuit[{regime}] gate {rel['gate_id']} dist={rel['distance']:5.1f}m "
                f"fwd={rel['forward']:+6.1f} right={rel['right']:+6.1f} down={rel['down']:+6.1f} "
                f"az={rel['azimuth_deg']:+4.0f} | thr={thr:.3f} climb={climb_str} want={dc:+.2f}m/s",
                flush=True,
            )
        else:
            print(
                f"pursuit[{regime}] armed={self.data.get('armed')} "
                f"gates={len(self.data.get('gates') or [])} "
                f"pose={'ok' if self.data.get('odometry') is not None else 'MISSING'} "
                f"active_gate={rs.get('active_gate_index')} thr={thr:.3f}",
                flush=True,
            )

    def arm(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

    def send_sim_reset_command(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            MAVLINK_CMD_SIM_RESET,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0
        )
