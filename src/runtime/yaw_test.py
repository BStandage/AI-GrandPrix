"""
Open-loop YAW SIGN test - zero perception, zero roll.

Commands a steadily increasing POSITIVE yaw angle through the SAME attitude path the race pilot uses
(send_attitude_setpoint), with level roll/pitch and hover thrust. No gates, no offsets, no tracking.

Just watch the sim and note which way it rotates:
    rotates RIGHT / clockwise (CW)      -> +yaw command turns RIGHT
    rotates LEFT  / counter-clockwise   -> +yaw command turns LEFT

That pins the actuator sign with nothing else moving. Then, since we want to turn toward the gate,
we pick RACE_KP_YAW's sign from this + which way the gate sits.

Run (with the sim up):  python -m runtime.yaw_test
Start the race (spacebar) if it just sits on the ground. Ctrl-C to stop.
"""

import time

from runtime.setup import setup_components
from common.dynamics import send_attitude_setpoint

SIM_IP, SIM_PORT = "127.0.0.1", 14550
HOVER    = 0.30      # collective thrust to hold altitude while it yaws
YAW_RATE = 0.4       # rad/s - how fast the commanded heading ramps (~23 deg/s, clearly visible)
HZ       = 100.0

def main():
    shared = {"running": True}
    system_boot_ms = int(time.time() * 1000)
    comps = setup_components(shared, system_boot_ms, SIM_IP, SIM_PORT)
    conn = comps["sim_conn"]
    comps["controller"].arm()

    print("\n>>> Commanding POSITIVE yaw (ramping). Watch the sim:", flush=True)
    print(">>>   rotates RIGHT / clockwise      = +yaw turns RIGHT", flush=True)
    print(">>>   rotates LEFT  / anticlockwise  = +yaw turns LEFT", flush=True)
    print(">>> (start the race with spacebar if it sits on the ground)  Ctrl-C to stop.\n", flush=True)

    yaw = 0.0
    try:
        while True:
            yaw += YAW_RATE / HZ                                   # ramp the commanded heading up
            send_attitude_setpoint(conn, system_boot_ms, 0.0, 0.0, yaw, HOVER)
            time.sleep(1.0 / HZ)
    except KeyboardInterrupt:
        print("\nstopped.", flush=True)
    finally:
        shared["running"] = False


if __name__ == "__main__":
    main()
