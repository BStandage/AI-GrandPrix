"""
POSITION-CONTROL PROBE - does the sim's stabilised controller accept
SET_POSITION_TARGET_LOCAL_NED in VQ2? (TS-003 4.3 lists it as a supported control
input; 9.3 blocks only telemetry OUT. If this works, the whole open-loop-attitude
fragility is optional.)

Run in a TRAINING flight (sim up, client during countdown):
    python -m runtime.pos_probe

Script: hover at 2 m -> hold -> step 5 m forward-ish -> step sideways+up -> hold.
Everything is logged to datasets/pos_probe_*.csv (commands + IMU). Watch the drone:
  - tracks the steps smoothly  -> position control WORKS: we build pos_pilot
  - sits/drifts/ignores        -> attitude tape remains the only mode

Touches NOTHING of ace_pilot: separate module, separate log, no shared state.
"""

import csv
import os
import time

from pymavlink import mavutil

from common.paths import DATASETS_DIR
from runtime.setup import setup_components

SIM_IP, SIM_PORT = "127.0.0.1", 14550
YAW0 = 1.6947    # spawn heading (+97.1 deg), same convention as the tape

# v2: CARROT RAMPS - the sim's controller bang-bangs on step targets (5.8 g bursts
# to the ceiling on a 2 m step). The target now GLIDES between keyframes; the
# tracking error stays centimetres and the controller stays gentle.
# keyframes: (t_end, x, y, z, yaw) - target interpolates linearly between them
# v4: THE Z-SIGN TEST, alone. One glided 0.5 m vertical command. Outcomes:
#   rises ~0.5 m  -> true NED (z down): position control usable as specced
#   presses floor -> z-up quirk: flip the sign and retest
#   chaos at 0.5 m glided -> the controller itself is untamed at ANY error
KEYS = [
    (3.0,  0.0, 0.0,  0.0, YAW0),   # sit at the origin target
    (7.0,  0.0, 0.0, -0.5, YAW0),   # glide to z=-0.5 (NED: 0.5 m UP) over 4 s
    (15.0, 0.0, 0.0, -0.5, YAW0),   # hold 8 s - long, uncontaminated observation
]
def carrot(t):
    prev_t, px, py, pz, pyaw = 0.0, KEYS[0][1], KEYS[0][2], KEYS[0][3], KEYS[0][4]
    for (te, x, y, z, yw) in KEYS:
        if t <= te:
            f = (t - prev_t) / max(te - prev_t, 1e-6)
            return (px + (x - px) * f, py + (y - py) * f, pz + (z - pz) * f, yw)
        prev_t, px, py, pz, pyaw = te, x, y, z, yw
    return KEYS[-1][1], KEYS[-1][2], KEYS[-1][3], KEYS[-1][4]

VEL_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

POS_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def send_position_target(conn, boot_ms, x, y, z, yaw):
    now_ms = int(time.time() * 1000)
    conn.mav.set_position_target_local_ned_send(
        now_ms - boot_ms,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POS_ONLY_MASK,
        x, y, z,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        yaw, 0.0,
    )


def send_velocity_target(conn, boot_ms, vx, vy, vz, yaw):
    now_ms = int(time.time() * 1000)
    conn.mav.set_position_target_local_ned_send(
        now_ms - boot_ms,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VEL_ONLY_MASK,
        0.0, 0.0, 0.0,
        vx, vy, vz,
        0.0, 0.0, 0.0,
        yaw, 0.0,
    )


def main():
    shared = {"running": True}
    boot_ms = int(time.time() * 1000)
    comps = setup_components(shared, boot_ms, SIM_IP, SIM_PORT)
    conn = comps["sim_conn"]
    comps["controller"].arm()

    path = os.path.join(DATASETS_DIR, time.strftime("pos_probe_%Y%m%d_%H%M%S.csv"))
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "phase", "tgt_x", "tgt_y", "tgt_z", "xacc", "yacc", "zacc", "imu_us"])
    print(f"pos probe -> {path}", flush=True)

    print("waiting for race GO (use a TRAINING flight) ...", flush=True)
    while True:
        rs = shared.get("race_status") or {}
        start = rs.get("race_start_boot_time_ms", -1)
        now = rs.get("sim_boot_time_ms", 0)
        if start is not None and start >= 0 and now >= start:
            break
        send_position_target(conn, boot_ms, 0.0, 0.0, 0.0, YAW0)
        time.sleep(0.05)

    print("GO - v4: single z-sign test, 0.5 m glided", flush=True)
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            if t > KEYS[-1][0]:
                break
            x, y, z, yaw = carrot(t)
            send_position_target(conn, boot_ms, x, y, z, yaw)
            imu = shared.get("highres_imu") or {}
            w.writerow([f"{t:.3f}", "zsign", f"{x:.2f}", f"{y:.2f}", f"{z:.2f}",
                        imu.get("xacc", ""), imu.get("yacc", ""), imu.get("zacc", ""),
                        imu.get("time_usec", "")])
            time.sleep(1 / 50)
    finally:
        f.close()
        shared["running"] = False
    print("probe complete - did the drone track the steps? (eyes + the CSV decide)", flush=True)


if __name__ == "__main__":
    main()
