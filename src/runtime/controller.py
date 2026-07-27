"""
Top-level control loop: pick a flight mode each tick, run it, print a status readout.

Modes (set CONTROL_MODE):
  "vision"       - fly from the CAMERA alone, no ground truth (Round-1 goal)  [pilots.vision_pilot]
  "trajectory"   - follow a racing line through the known gates (fast oracle) [pilots.oracle_pilot]
  "keyboard"     - manual rate-mode flight for data collection               [pilots.dev_modes]
  "characterize" - fly a scripted profile and log the physics envelope       [pilots.dev_modes]

(The retired "pursuit" pilot lives in archive/pursuit_pilot.py and is no longer wired in.)
"""

import math
import time

from common.dynamics import CONTROL_HZ, send_arm, send_sim_reset
from common.gate_geometry import active_gate_relative
from pilots.dev_modes import char_phase_at, update_characterize_control, update_keyboard_rate_control
from pilots.oracle_pilot import update_trajectory_control
from pilots.phase1_pilot.phase1_pilot import update_phase1_control
from pilots.attack_pilot import update_attack_control
from pilots.vision_pilot import update_vision_control

CONTROL_MODE = "attack"
GATE_READOUT_PERIOD_S = 0.5   # print a status line this often


class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms
        self.tick = 0
        data["_control_mode"] = CONTROL_MODE   # so the vision data collector records which pilot flew

    def update(self):
        if CONTROL_MODE == "vision":
            update_vision_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "trajectory":
            update_trajectory_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "phase1":
            update_phase1_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "attack":
            update_attack_control(self.sim_conn, self.system_boot_ms, self.data)
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

        if CONTROL_MODE == "vision":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("vision_regime", "?")
            dbg = self.data.get("vis_dbg")
            tgt = self.data.get("vision_target")
            seen = f"gate offx={dbg[0]:+.2f} offy={dbg[1]:+.2f} dist~{dbg[2]:4.1f}m dclimb={dbg[3]:+.1f}" \
                if dbg else ("acquiring" if tgt is None else "?")
            # ground-truth nearest gate shown ONLY as a reference check (the pilot does not use it)
            rel = active_gate_relative(self.data)
            gtchk = f" | GTchk g{rel['gate_id']} d={rel['distance']:4.1f}m" if rel else ""
            print(f"vis[{regime}] {seen} climb={climb_str} alt={-(z or 0.0):+6.1f}m thr={thr:.3f}{gtchk}",
                  flush=True)
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

        if CONTROL_MODE == "attack":
            regime = self.data.get("attack_regime", "?")
            gc = self.data.get("_at_gate_count", 0)
            dr = self.data.get("_at_des_roll", 0.0)
            dp = self.data.get("_at_des_pitch", 0.0)
            az = self.data.get("_at_az", 0.0)
            el = self.data.get("_at_el", 0.0)
            thr = self.data.get("_at_thrust", 0.0)
            dets = self.data.get("_at_ndets", 0)
            area = self.data.get("_at_area", 0.0)
            print(
                f"atk[{regime}] gates={gc} roll*={math.degrees(dr):+.0f} pitch*={math.degrees(dp):+.0f} "
                f"az={math.degrees(az):+.0f} el={math.degrees(el):+.0f} thr={thr:.2f} "
                f"climb={self.data.get('_at_climb', 0.0):+.1f} dets={dets} area={area:.2f}",
                flush=True,
            )
            return

        if CONTROL_MODE == "phase1":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("traj_regime", "?")
            rel = active_gate_relative(self.data)   # for reference: nearest real gate
            gatestr = f"gate {rel['gate_id']} dist={rel['distance']:4.1f}m" if rel else "no gate"
            print(
                f"phase1[{regime}] xtrack={self.data.get('traj_xtrack', 0.0):4.1f}m "
                f"yaw={self.data.get('traj_yawrate', 0.0):+4.1f} "
                f"v={self.data.get('traj_vcur', 0.0):4.1f}/{self.data.get('traj_vtgt', 0.0):3.1f}m/s "
                f"climb={climb_str} alt={-(z or 0.0):+6.1f}m | {gatestr} thr={thr:.3f}",
                flush=True,
            )
            return

    def arm(self):
        send_arm(self.sim_conn, arm=True)

    def send_sim_reset_command(self):
        send_sim_reset(self.sim_conn)
