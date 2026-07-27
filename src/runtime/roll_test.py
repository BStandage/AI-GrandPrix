"""
Open-loop ROLL SIGN test - zero perception, zero yaw.

Commands a CONSTANT positive roll angle through the same attitude path the race pilot uses
(send_attitude_setpoint), level pitch/yaw, hover thrust. No gates, no offsets, no tracking.

Watch the sim and note which way the drone banks / drifts:
    banks & drifts RIGHT  -> +roll command = RIGHT
    banks & drifts LEFT   -> +roll command = LEFT

That pins the roll actuator sign with nothing else moving. Combined with "gate on the right =
+offset_x", it fixes whether RACE_KP_ROLL should be negative or positive to center.

Run (sim up):  python -m runtime.roll_test     (spacebar to start the race if it sits)  Ctrl-C to stop.
"""

import time

from runtime.setup import setup_components
from common.dynamics import send_attitude_setpoint

SIM_IP, SIM_PORT = "127.0.0.1", 14550
HOVER = 0.30
ROLL  = 0.30      # rad (~17 deg) constant positive roll

def main():
    shared = {"running": True}
    system_boot_ms = int(time.time() * 1000)
    comps = setup_components(shared, system_boot_ms, SIM_IP, SIM_PORT)
    conn = comps["sim_conn"]
    comps["controller"].arm()

    print("\n>>> Commanding CONSTANT +roll (%.2f rad). Watch the sim:" % ROLL, flush=True)
    print(">>>   banks/drifts RIGHT = +roll is RIGHT", flush=True)
    print(">>>   banks/drifts LEFT  = +roll is LEFT", flush=True)
    print(">>> (spacebar to start the race if it sits on the ground)  Ctrl-C to stop.\n", flush=True)

    try:
        while True:
            send_attitude_setpoint(conn, system_boot_ms, ROLL, 0.0, 0.0, HOVER)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nstopped.", flush=True)
    finally:
        shared["running"] = False


if __name__ == "__main__":
    main()
