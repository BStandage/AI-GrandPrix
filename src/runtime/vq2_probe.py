"""
VQ2 VERTICAL/BRAKE CALIBRATION PROBE - the dataset the tape's z errors demand.

VQ2 has no odometry, so truth comes from SHORT-HORIZON IMU integration: vertical accel drift
only matters over long horizons; every measurement segment here is 1.2-2.0 s, integrated from
a fresh zero each segment. The script stays in the start corridor (fore/aft pulses cancel).

Measures, on the REAL race sim:
  1. hover collective (zero-climb thrust) - bracket around 0.24-0.31
  2. thrust->climb response at race collectives (0.30/0.35/0.40/0.45) - the thrust CURVE, not
     just the hover point (if it is super-linear above hover, tilt-comp overthrusts and every
     hard brake balloons: Brian's "pitch back -> climb" observation)
  3. BRAKE COUPLING: pitch-back pulses (-15/-25 deg, 1.2 s) at tilt-comp thrust - directly
     measures the climb a real brake causes with our current thrust law
  4. same for forward lean (accel coupling)

Run (VQ2 sim up, client during countdown):  python -m runtime.vq2_probe
Output: datasets/vq2_probe_YYYYMMDD_HHMMSS.csv (t, phase, cmd_*, IMU accels/gyros, vz_short =
per-segment short-horizon integral). Analysis fits the VQ2 vertical model from it.
"""

import csv
import math
import os
import time

from common.dynamics import send_attitude_setpoint, send_rate_attitude
from common.paths import DATASETS_DIR
from runtime.setup import setup_components

SIM_IP, SIM_PORT = "127.0.0.1", 14550
G = 9.81
HOVER0 = 0.265           # starting guess; phase 1 refines it

# (name, roll, pitch_deg, thrust or None=tilt-comp, duration s)
def _script():
    ph = []
    def settle(d=1.5):
        ph.append(("settle", 0.0, 0.0, HOVER0, d))
    # climbout to working altitude, then settle
    ph.append(("climb", 0.0, 0.0, 0.40, 2.2))
    settle(2.0)
    # 1) hover bracket
    for thr in (0.24, 0.26, 0.28, 0.30):
        ph.append((f"hov{thr:.2f}", 0.0, 0.0, thr, 1.5))
        settle()
    # 2) thrust curve at race collectives
    for thr in (0.32, 0.36, 0.40, 0.45):
        ph.append((f"thr{thr:.2f}", 0.0, 0.0, thr, 1.5))
        ph.append(("recover", 0.0, 0.0, 0.18, 1.0))
        settle()
    # 3) brake pulses at tilt-comp thrust (the exact law the tape flies)
    for deg in (-15, -25, 15, 25):
        thr = HOVER0 / max(math.cos(math.radians(deg)), 0.5)
        ph.append((f"pulse{deg:+d}", 0.0, float(deg), thr, 1.2))
        ph.append((f"counter{deg:+d}", 0.0, -0.6 * deg, HOVER0, 0.8))   # arrest the drift
        settle()
    settle(2.0)
    return ph


def main():
    shared = {"running": True}
    boot_ms = int(time.time() * 1000)
    comps = setup_components(shared, boot_ms, SIM_IP, SIM_PORT)
    conn = comps["sim_conn"]
    comps["controller"].arm()

    path = os.path.join(DATASETS_DIR, time.strftime("vq2_probe_%Y%m%d_%H%M%S.csv"))
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "phase", "cmd_roll", "cmd_pitch_deg", "cmd_thr",
                "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "imu_us"])
    print(f"vq2 probe -> {path}", flush=True)

    # wait for GO (no movement during the countdown = no DQ risk on a training attempt)
    print("waiting for race GO ...", flush=True)
    while True:
        rs = shared.get("race_status") or {}
        start = rs.get("race_start_boot_time_ms", -1)
        now = rs.get("sim_boot_time_ms", 0)
        if start is not None and start >= 0 and now >= start:
            break
        send_rate_attitude(conn, boot_ms, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.02)

    script = _script()
    total = sum(p[4] for p in script)
    print(f"GO - flying {len(script)} phases, ~{total:.0f}s", flush=True)
    t0 = time.time()
    n = 0
    try:
        while True:
            t = time.time() - t0
            acc = 0.0
            cur = None
            for p in script:
                if t < acc + p[4]:
                    cur = p
                    break
                acc += p[4]
            if cur is None:
                break
            name, roll, pdeg, thr, _ = cur
            send_attitude_setpoint(conn, boot_ms, roll, math.radians(pdeg), math.radians(97.1), thr)
            imu = shared.get("highres_imu") or {}
            w.writerow([f"{t:.4f}", name, f"{roll:.3f}", f"{pdeg:.1f}", f"{thr:.3f}",
                        imu.get("xacc", ""), imu.get("yacc", ""), imu.get("zacc", ""),
                        imu.get("xgyro", ""), imu.get("ygyro", ""), imu.get("zgyro", ""),
                        imu.get("time_usec", "")])
            n += 1
            if n % 30 == 0:
                f.flush()
                print(f"  t={t:5.1f}s phase={name}", flush=True)
            time.sleep(1.0 / 90.0)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
    print(f"done -> {path}", flush=True)


if __name__ == "__main__":
    main()
