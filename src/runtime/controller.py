"""Top-level control loop: select a pilot each tick and print a status readout.

Physical Qualifier (unknown course):
  "steady" - DEFAULT. Pure vision; maps / races the real layout.
  "ace" / "giga" - open-loop tape only AFTER a trusted map exists (VQ2 archive
                   tapes will not match PQ geometry).

Other modes stay importable for archaeology.
"""

import math

from common.dynamics import CONTROL_HZ, send_arm, send_sim_reset
from common.gate_geometry import active_gate_relative

CONTROL_MODE = "vml"
GATE_READOUT_PERIOD_S = 1

_PQ_MODES = ("steady", "ace", "giga", "cl", "fair", "vins", "vml", "probe")


class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms
        self.tick = 0
        data["_control_mode"] = CONTROL_MODE
        if CONTROL_MODE not in _PQ_MODES:
            print(
                f"[controller] WARNING: CONTROL_MODE={CONTROL_MODE!r} is not a PQ "
                f"day-to-day mode {_PQ_MODES}",
                flush=True,
            )

    def update(self):
        if CONTROL_MODE == "vins":
            from pilots.vins_pilot import update_vins_control
            update_vins_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "vml":
            from pilots.vml_pilot import update_vml_control
            update_vml_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "probe":
            from pilots.vml_pilot import update_probe_control
            update_probe_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "fair":
            from pilots.fair_pilot import update_fair_control
            update_fair_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "cl":
            from pilots.cl_pilot.cl_pilot import update_cl_control
            update_cl_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "ace":
            from pilots.ace_pilot import update_ace_control
            update_ace_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "giga":
            from pilots.giga_pilot import update_giga_control
            update_giga_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "steady":
            from pilots.steady_pilot import update_steady_control
            update_steady_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "vision":
            from pilots.vision_pilot import update_vision_control
            update_vision_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "trajectory":
            from pilots.oracle_pilot import update_trajectory_control
            update_trajectory_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "phase1":
            from pilots.phase1_pilot.phase1_pilot import update_phase1_control
            update_phase1_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "sprint":
            from pilots.sprint_pilot import update_sprint_control
            update_sprint_control(self.sim_conn, self.system_boot_ms, self.data)
        else:
            raise ValueError(f"unknown CONTROL_MODE: {CONTROL_MODE!r}")

        self._readout()
        # Pace in main.py (deadline scheduler).

    def _readout(self):
        self.tick += 1
        if self.tick % int(CONTROL_HZ * GATE_READOUT_PERIOD_S) != 0:
            return

        odo = self.data.get("odometry") or self.data.get("local_position_ned") or {}
        vz = odo.get("vz")
        z = odo.get("z")
        climb_str = f"{-vz:+.2f}" if vz is not None else " n/a"

        if CONTROL_MODE == "steady":
            regime = self.data.get("steady_regime", "?")
            gc = self.data.get("_sp_gate_count", 0)
            dr = self.data.get("_sp_des_roll", 0.0)
            dp = self.data.get("_sp_des_pitch", 0.0)
            az = self.data.get("_sp_az", 0.0)
            el = self.data.get("_sp_el", 0.0)
            thr = self.data.get("_sp_thrust", 0.0)
            dets = self.data.get("_sp_ndets", 0)
            area = self.data.get("_sp_area", 0.0)
            print(
                f"steady[{regime}] gates={gc} roll*={math.degrees(dr):+.0f} pitch*={math.degrees(dp):+.0f} "
                f"az={math.degrees(az):+.0f} el={math.degrees(el):+.0f} thr={thr:.2f} "
                f"climb={self.data.get('_sp_climb', 0.0):+.1f} dets={dets} area={area:.2f}",
                flush=True,
            )
            return

        if CONTROL_MODE in ("vml", "probe"):
            key = "vml_regime" if CONTROL_MODE == "vml" else "probe_regime"
            print(f"{CONTROL_MODE}[{self.data.get(key, '?')}] climb={climb_str} "
                  f"alt={-(z or 0.0):+6.1f}m", flush=True)
            return

        if CONTROL_MODE in ("fair", "cl"):
            key = "fair_regime" if CONTROL_MODE == "fair" else "cl_regime"
            print(f"{CONTROL_MODE}[{self.data.get(key, '?')}] climb={climb_str} "
                  f"alt={-(z or 0.0):+6.1f}m", flush=True)
            return

        if CONTROL_MODE in ("ace", "giga"):
            rs = self.data.get("race_status") or {}
            gi = rs.get("active_gate_index")
            if gi is not None and gi != self.data.get("_readout_last_gate"):
                self.data["_readout_last_gate"] = gi
                key = "ace_regime" if CONTROL_MODE == "ace" else "giga_regime"
                print(f"{CONTROL_MODE}: race_gate -> {gi}  [{self.data.get(key, '?')}]",
                      flush=True)
            return

        if CONTROL_MODE == "vision":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("vision_regime", "?")
            dbg = self.data.get("vis_dbg")
            tgt = self.data.get("vision_target")
            seen = f"gate offx={dbg[0]:+.2f} offy={dbg[1]:+.2f} dist~{dbg[2]:4.1f}m dclimb={dbg[3]:+.1f}" \
                if dbg else ("acquiring" if tgt is None else "?")
            rel = active_gate_relative(self.data)
            gtchk = f" | GTchk g{rel['gate_id']} d={rel['distance']:4.1f}m" if rel else ""
            print(f"vis[{regime}] {seen} climb={climb_str} alt={-(z or 0.0):+6.1f}m thr={thr:.3f}{gtchk}",
                  flush=True)
            return

        if CONTROL_MODE == "trajectory":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("traj_regime", "?")
            rel = active_gate_relative(self.data)
            gatestr = f"gate {rel['gate_id']} dist={rel['distance']:4.1f}m" if rel else "no gate"
            print(
                f"traj[{regime}] xtrack={self.data.get('traj_xtrack', 0.0):4.1f}m "
                f"yaw={self.data.get('traj_yawrate', 0.0):+4.1f} "
                f"v={self.data.get('traj_vcur', 0.0):4.1f}/{self.data.get('traj_vtgt', 0.0):3.1f}m/s "
                f"climb={climb_str} alt={-(z or 0.0):+6.1f}m | {gatestr} thr={thr:.3f}",
                flush=True,
            )
            return

        if CONTROL_MODE == "sprint":
            regime = self.data.get("sprint_regime", "?")
            gc = self.data.get("_sp_gate_count", 0)
            dp = self.data.get("_sp_des_pitch", 0.0)
            az = self.data.get("_sp_az", 0.0)
            thr = self.data.get("_sp_thrust", 0.0)
            area = self.data.get("_sp_area", 0.0)
            print(
                f"sprint[{regime}] gates={gc} v_est={self.data.get('_sp_v_est', 0.0):+.1f}m/s "
                f"pitch*={math.degrees(dp):+.0f} az={math.degrees(az):+.0f} thr={thr:.2f} "
                f"climb={self.data.get('_sp_climb', 0.0):+.1f} area={area:.2f}",
                flush=True,
            )
            return

        if CONTROL_MODE == "phase1":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("traj_regime", "?")
            rel = active_gate_relative(self.data)
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
