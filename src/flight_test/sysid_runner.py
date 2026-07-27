"""
TrialRunner: the automation core of the system-identification campaign.

A flight campaign is hundreds of short, often crash-ending trials. The racing controller's
single-tick `update()` model can't express "reset -> arm -> climb -> run maneuver -> detect
crash -> repeat", so this is a standalone state machine that owns its own loop and reuses the
proven plumbing: the MAVLink connection, the MAVLinkRX telemetry thread (fills shared_data),
and dynamics.py for the command sends and the measured climb curve.

A Trial bundles a maneuver (a per-tick setpoint generator) with its setup altitude and abort
limits. `TrialRunner.run(trial)` returns a TrialResult: the captured per-tick rows plus the
outcome. The batteries turn those into the tab CSVs.

Frames (MAVLink): NED world (x=North, y=East, z=Down); body (x=fwd, y=right, z=down). Altitude
up = -z. The sim reports ODOMETRY velocity in BODY frame, so we rotate it to world for every
trial (the single most important gotcha in this codebase - see trajectory_pilot.py).
"""

import math
import time

from common.dynamics import (CONTROL_HZ, KP_ATT, MAX_RATE, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      send_arm, send_attitude_setpoint, send_rate_attitude, send_sim_reset,
                      thrust_for_climb)
from common.gate_geometry import quat_to_rotmat
from common.race import race_is_live, seconds_to_go

# Environment collision id (ground/buildings); 1001 is a gate (mavlink_rx.on_collision).
COLLISION_ENVIRONMENT = 1002

# Setup-phase tolerances for "we are stabilized at the test altitude and ready to start".
SETUP_ALT_TOL = 1.5        # m
SETUP_RATE_TOL = 0.5       # rad/s on each body axis
SETUP_SETTLE_S = 0.4       # must stay within tolerance this long
SETUP_TIMEOUT_S = 25.0     # give up climbing after this
RESET_TIMEOUT_S = 6.0      # wait for the sim-reset to take effect
RESET_RETRIES = 3          # re-issue the reset this many times if the drone isn't back at spawn
SPAWN_ALT_LO = -10.0       # a sane post-reset altitude band (m); outside this = broken state
SPAWN_ALT_HI = 15.0
ARM_TIMEOUT_S = 6.0
# Closed-loop climb-rate gain for the setup climb. Pure feedforward off dynamics.py's thrust
# curve overshoots badly (the real airframe climbs far harder than the measured table), so we
# feed back on the MEASURED climb rate. Stronger than the trajectory pilot's trim gain because
# here it carries the whole load, not just a correction on top of an accurate FF.
SETUP_KP_CLIMB = 0.06


class Trial:
    """One test: a maneuver plus its setup altitude and abort limits.

    maneuver: object with step(t, st) -> (roll_rate, pitch_rate, yaw_rate, thrust) or None (done).
              t is seconds since the maneuver started; st is a state snapshot (see snapshot()).
    """

    def __init__(self, name, maneuver, setup_alt=45.0, timeout_s=8.0, floor_alt=1.0,
                 abort_on_collision=True, params=None, mode="rate"):
        self.name = name
        self.maneuver = maneuver
        self.setup_alt = setup_alt
        self.timeout_s = timeout_s
        self.floor_alt = floor_alt          # below this altitude (m) we abort as a floor breach
        self.abort_on_collision = abort_on_collision
        self.params = params or {}          # metadata echoed into the trial's summary row
        # "rate": maneuver returns body RATES -> send_rate_attitude (the sysid default).
        # "attitude": maneuver returns ABSOLUTE angles -> send_attitude_setpoint (Tab 6, the
        # interface the vision/hover pilots fly). Only the maneuver phase differs; setup/climb
        # is always rate-mode self-levelling.
        self.mode = mode


class TrialResult:
    def __init__(self, name, outcome, rows, params, collision=None, setup_alt=None):
        self.name = name
        self.outcome = outcome              # completed | collision | floor | timeout | aborted | setup_failed | nan
        self.rows = rows                    # list of per-tick state dicts (see _capture)
        self.params = params
        self.collision = collision
        self.setup_alt = setup_alt

    @property
    def ok(self):
        return self.outcome in ("completed", "collision", "floor", "timeout")


def snapshot(data):
    """A flat snapshot of the current vehicle state, with body velocity rotated to world.

    Returns None until odometry has arrived (we need pose + quaternion to do anything)."""
    odo = data.get("odometry")
    if odo is None:
        return None
    att = data.get("attitude") or {}
    quat = (odo.get("qw", 1.0), odo.get("qx", 0.0), odo.get("qy", 0.0), odo.get("qz", 0.0))
    R = quat_to_rotmat(quat)
    vb = (odo.get("vx", 0.0), odo.get("vy", 0.0), odo.get("vz", 0.0))
    vw = (R[0][0] * vb[0] + R[0][1] * vb[1] + R[0][2] * vb[2],
          R[1][0] * vb[0] + R[1][1] * vb[1] + R[1][2] * vb[2],
          R[2][0] * vb[0] + R[2][1] * vb[1] + R[2][2] * vb[2])
    z = odo.get("z", 0.0)
    imu = data.get("highres_imu") or {}
    mot = data.get("actuator_output_status") or {}
    return {
        "t_usec": odo.get("time_usec"),
        "x": odo.get("x", 0.0), "y": odo.get("y", 0.0), "z": z,
        "alt": -z,                                  # altitude up+ (floor at alt 0)
        "quat": quat,
        "roll": att.get("roll", 0.0), "pitch": att.get("pitch", 0.0), "yaw": att.get("yaw", 0.0),
        # body angular rates (prefer odometry; fall back to attitude message)
        "rollspeed": odo.get("rollspeed", att.get("rollspeed", 0.0)),
        "pitchspeed": odo.get("pitchspeed", att.get("pitchspeed", 0.0)),
        "yawspeed": odo.get("yawspeed", att.get("yawspeed", 0.0)),
        "vx_w": vw[0], "vy_w": vw[1], "vz_w": vw[2],
        "climb_up": -vw[2],                          # true world vertical speed (up+)
        "vh": math.hypot(vw[0], vw[1]),              # world horizontal speed
        # up_align = world-up component of the body thrust axis = R[2][2].
        #   +1 fully upright, 0 at the horizon, -1 fully inverted. Singularity-free, unlike Euler.
        "up_align": R[2][2],
        "xgyro": imu.get("xgyro", 0.0), "ygyro": imu.get("ygyro", 0.0), "zgyro": imu.get("zgyro", 0.0),
        "xacc": imu.get("xacc", 0.0), "yacc": imu.get("yacc", 0.0), "zacc": imu.get("zacc", 0.0),
        "motor": (mot.get("motor_front_left", 0.0), mot.get("motor_front_right", 0.0),
                  mot.get("motor_back_left", 0.0), mot.get("motor_back_right", 0.0)),
        "reset_counter": odo.get("reset_counter"),
        "armed": bool(data.get("armed")),
    }


class TrialRunner:
    def __init__(self, conn, data, system_boot_ms):
        self.conn = conn
        self.data = data
        self.boot_ms = system_boot_ms

    # -- low-level helpers ------------------------------------------------------------------
    def _send(self, rr, pr, yr, thrust):
        send_rate_attitude(self.conn, self.boot_ms, rr, pr, yr, thrust)

    def _tick(self):
        time.sleep(1.0 / CONTROL_HZ)

    def _esc(self):
        # Abort is driven by the running flag only (set by main()'s Ctrl-C / KeyboardInterrupt
        # handler). We deliberately do NOT poll the `keyboard` library here: it reads global key
        # state and intermittently false-reported ESC as pressed, aborting healthy runs after a
        # trial or two. Ctrl-C is reliable and interrupts the loops immediately.
        return not self.data.get("running", True)

    def _hold_throttle_down(self, seconds):
        """Stream zero-thrust, level setpoints for a beat. The sim refuses to arm while the
        collective is up ('throttle down please'), so we hold it down before/through arming."""
        t0 = time.time()
        while time.time() - t0 < seconds:
            if self._esc():
                return False
            self._send(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.02)
        return True

    def wait_for_telemetry(self, timeout=10.0):
        """Block until the first odometry snapshot is available."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if snapshot(self.data) is not None:
                return True
            time.sleep(0.05)
        return False

    def _wait_for_race_go(self, timeout=60.0):
        """Hold at spawn, THROTTLE DOWN, until the race goes live - so we never move during the
        "3..2..1..GO" countdown (moving early = DQ). We stream only zero-thrust level setpoints
        while disarmed, which is no movement. Each trial calls this after its reset, so it also
        covers a countdown that restarts on reset. If no race ever goes live within `timeout`
        (a free-practice sim with no countdown), we proceed anyway rather than hang."""
        if race_is_live(self.data):
            return True
        t0 = time.time()
        last_note = -1e9
        while time.time() - t0 < timeout:
            if self._esc():
                return False
            if race_is_live(self.data):
                return True
            if time.time() - last_note > 5.0:      # re-print every ~5 s so a long wait isn't silent
                last_note = time.time()
                tg = seconds_to_go(self.data)
                if tg is not None and tg <= 0:
                    # started in the past but not live = race FINISHED. Waiting won't help; the sim
                    # needs the race RESTARTED (with this client running) to arm a fresh countdown.
                    print("  [runner] race is FINISHED - RESTART THE RACE in the sim to continue "
                          "(holding at spawn, throttle down) ...", flush=True)
                else:
                    print("  [runner] waiting for race GO"
                          + (f" (~{tg:.0f}s)" if (tg and tg > 0) else "") + " - holding at spawn ...",
                          flush=True)
            self._send(0.0, 0.0, 0.0, 0.0)   # throttle DOWN, disarmed = no movement, no early start
            time.sleep(0.05)
        print("  [runner] no race-go within timeout - proceeding (practice sim?)", flush=True)
        return True

    # -- phases -----------------------------------------------------------------------------
    def _reset(self):
        """Reset the sim and wait for the drone to return to a SANE spawn altitude.

        A violent tumble can punch the drone through the floor (alt ~ -100 m); the old code
        accepted any reset_counter bump and handed that broken state to the climb, which then
        chased a phantom altitude. Here we re-issue the reset until altitude is back in a sane
        band, and give up (skip the trial) rather than fly from underground."""
        for attempt in range(RESET_RETRIES):
            send_sim_reset(self.conn)
            t0 = time.time()
            while time.time() - t0 < RESET_TIMEOUT_S:
                if self._esc():
                    return False
                self._send(0.0, 0.0, 0.0, 0.0)
                st = snapshot(self.data)
                if st is not None and SPAWN_ALT_LO < st["alt"] < SPAWN_ALT_HI:
                    self._hold_throttle_down(0.3)   # settle, throttle DOWN, so the next arm is accepted
                    return True
                time.sleep(0.05)
            cur = snapshot(self.data)
            altstr = f"{cur['alt']:.0f} m" if cur else "?"
            print(f"  [runner] reset attempt {attempt + 1}/{RESET_RETRIES}: alt={altstr}, "
                  f"not at spawn - retrying", flush=True)
        print("  [runner] WARNING: reset failed to return to spawn - skipping this trial", flush=True)
        return False

    def _arm(self):
        # Hold the throttle down FIRST: a non-zero collective left over from a previous trial/run
        # makes the sim reject the arm with "throttle down please".
        if not self._hold_throttle_down(0.8):
            return False
        send_arm(self.conn, arm=True)
        t0 = time.time()
        while time.time() - t0 < ARM_TIMEOUT_S:
            if self._esc():
                return False
            if self.data.get("armed"):
                return True
            self._send(0.0, 0.0, 0.0, 0.0)   # keep throttle down while waiting
            send_arm(self.conn, arm=True)    # resend; the first command can land before arming is allowed
            time.sleep(0.1)
        print("  [runner] WARNING: arm not confirmed (sim still says throttle up?) - proceeding", flush=True)
        return True

    def _climb_to(self, target_alt):
        """Climb to target altitude and self-level, reusing the measured thrust/climb curve and
        the same hold-altitude P loop as the characterize mode (dev_modes.update_characterize).

        Prints a throttled altitude readout so the climb is visible (and a stalled climb - e.g.
        an arena ceiling - is obvious: altitude stops rising below the target)."""
        t0 = time.time()
        settled_since = None
        n = 0
        last_alt = None
        while time.time() - t0 < SETUP_TIMEOUT_S:
            if self._esc():
                return False
            st = snapshot(self.data)
            if st is None:
                time.sleep(0.05)
                continue
            alt = st["alt"]
            last_alt = alt
            desired_climb = clamp(0.8 * (target_alt - alt), -4.0, 8.0)
            # feedforward (measured curve) + feedback on actual climb rate so a stale/too-hot
            # thrust model can't run the altitude away
            thrust = clamp(thrust_for_climb(desired_climb)
                           + SETUP_KP_CLIMB * (desired_climb - st["climb_up"]), 0.0, 1.0)
            rr = clamp(ROLL_SIGN * KP_ATT * (0.0 - st["roll"]), -MAX_RATE, MAX_RATE)
            pr = clamp(PITCH_SIGN * KP_ATT * (0.0 - st["pitch"]), -MAX_RATE, MAX_RATE)
            yr = clamp(YAW_SIGN * KP_ATT * (0.0 - st["yaw"]), -MAX_RATE, MAX_RATE)
            self._send(rr, pr, yr, thrust)

            n += 1
            if n % CONTROL_HZ == 0:     # ~once per second
                print(f"    climbing -> alt={alt:6.1f}/{target_alt:.0f} m  climb={st['climb_up']:+5.1f} m/s",
                      flush=True)

            level = (abs(st["rollspeed"]) < SETUP_RATE_TOL and abs(st["pitchspeed"]) < SETUP_RATE_TOL
                     and abs(st["yawspeed"]) < SETUP_RATE_TOL)
            if abs(alt - target_alt) < SETUP_ALT_TOL and abs(st["climb_up"]) < 1.0 and level:
                settled_since = settled_since or time.time()
                if time.time() - settled_since > SETUP_SETTLE_S:
                    print(f"    settled at alt={alt:.1f} m", flush=True)
                    return True
            else:
                settled_since = None
            self._tick()
        # Timed out. Report the altitude actually reached - if it's well below target, that is
        # the arena ceiling (or a too-aggressive target) and downstream batteries should cap to it.
        print(f"  [runner] WARNING: did not settle at {target_alt:.0f} m within {SETUP_TIMEOUT_S:.0f}s "
              f"(reached alt={last_alt:.1f} m)", flush=True)
        return True   # proceed with whatever altitude we reached; the trial records actual state

    # -- public -----------------------------------------------------------------------------
    def run(self, trial):
        """Reset -> arm -> climb -> run the maneuver, capturing one row per control tick."""
        if self._esc():
            return TrialResult(trial.name, "aborted", [], trial.params)
        # Setup can fail two ways: the user hit ESC (running=False -> abort the whole campaign),
        # or the sim wouldn't return to a flyable state (skip just this trial, keep going).
        # Reset FIRST (back to spawn, throttle down), THEN wait out the race countdown before any
        # movement - arming/climbing during the countdown is an early start and gets us DQ'd.
        if not (self._reset() and self._wait_for_race_go()
                and self._arm() and self._climb_to(trial.setup_alt)):
            outcome = "aborted" if not self.data.get("running", True) else "setup_failed"
            return TrialResult(trial.name, outcome, [], trial.params)

        self.data["collision"] = None   # clear any stale collision from a previous trial
        rows = []
        outcome = "completed"
        t0 = time.time()
        while True:
            if self._esc():
                outcome = "aborted"
                break
            st = snapshot(self.data)
            if st is None:
                outcome = "nan"
                break
            t = time.time() - t0
            cmd = trial.maneuver.step(t, st)
            if cmd is None:
                outcome = "completed"
                break
            # step() returns (rr, pr, yr, thrust) or (rr, pr, yr, thrust, extra_fields_dict).
            extra = {}
            if len(cmd) == 5:
                rr, pr, yr, thrust, extra = cmd
            else:
                rr, pr, yr, thrust = cmd
            if trial.mode == "attitude":
                # (rr, pr, yr) are ABSOLUTE roll/pitch/yaw angles here, not rates.
                send_attitude_setpoint(self.conn, self.boot_ms, rr, pr, yr, thrust)
            else:
                self._send(rr, pr, yr, thrust)
            rows.append(self._capture(t, st, (rr, pr, yr, thrust), extra))

            if trial.abort_on_collision:
                col = self.data.get("collision")
                if col is not None:
                    outcome = "collision"
                    break
            if not math.isfinite(st["alt"]):
                outcome = "nan"
                break
            if st["alt"] < trial.floor_alt and st["climb_up"] < 0:
                outcome = "floor"
                break
            if t > trial.timeout_s:
                outcome = "timeout"
                break
            self._tick()

        # leave the throttle DOWN between trials/at exit - a non-zero collective left here makes
        # the next arm (this run's next trial, or a fresh re-run) fail with "throttle down please"
        self._send(0.0, 0.0, 0.0, 0.0)
        return TrialResult(trial.name, outcome, rows, trial.params,
                           collision=self.data.get("collision"), setup_alt=trial.setup_alt)

    @staticmethod
    def _capture(t, st, cmd, extra=None):
        rr, pr, yr, thrust = cmd
        row = dict(st)
        row.pop("quat", None)
        m = st["motor"]
        row.pop("motor", None)
        row.update({
            "t": t,
            "cmd_roll_rate": rr, "cmd_pitch_rate": pr, "cmd_yaw_rate": yr, "cmd_thrust": thrust,
            "motor_1": m[0], "motor_2": m[1], "motor_3": m[2], "motor_4": m[3],
            "motor_max": max(m),
        })
        if extra:
            row.update(extra)
        return row
