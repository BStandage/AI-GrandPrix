#
# Sample Python client for the AI GP controller
#

import time

from runtime.setup import setup_components

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

# time since sim started ms
system_boot_ms = int(time.time() * 1000)

# arbitrary shared data between the various components
shared_data = {"running": True}

# setup components
components = setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)
controller = components['controller']
ts_loop = components['ts_loop']
mavlink_rx = components['mavlink_rx']
vision_rx = components['vision_rx']
logger = components['logger']

print("Arming drone...", flush=True)
controller.arm()
from runtime.controller import CONTROL_MODE
print(f"Starting control loop... (CONTROL_MODE = {CONTROL_MODE})", flush=True)
if CONTROL_MODE == "vision":
    print("  VISION: flying from the CAMERA alone - NO ground-truth gates (Round-1 goal).", flush=True)
    print("  Detects the red gate, strafes to centre it in frame, climbs to aim, punches through.", flush=True)
    print("  Slow on purpose (accuracy-limited). Be ready to press ESC to abort.", flush=True)
elif CONTROL_MODE == "trajectory":
    print("  TRAJECTORY: following a pre-planned racing line through the gates (fast).", flush=True)
    print("  Builds a spline through the gate centres and tracks a carrot along it.", flush=True)
    print("  Be ready to press ESC to abort.", flush=True)
elif CONTROL_MODE == "characterize":
    print("  CHARACTERIZE: flying a scripted profile to measure the drone's physics.", flush=True)
    print("  It WILL fly aggressively (full/zero thrust, 30deg lean, max rates) and may", flush=True)
    print("  crash - that's fine, we want the data. Logs to datasets/characterize_*.csv.", flush=True)
    print("  When done: python -m analysis.analyze_performance", flush=True)
else:
    print("  Manual flight (rate mode):", flush=True)
    print("    R/F = throttle up/down (press R to take off)", flush=True)
    print("    UP/DOWN = pitch  |  LEFT/RIGHT = roll  |  Q/E = yaw  |  ESC = quit", flush=True)
try:
    while shared_data["running"]:
        controller.update()
except KeyboardInterrupt:
    print("Interrupted.", flush=True)

# exit
ts_loop.get_thread_for_join().join(timeout=1.0)
mavlink_rx.get_thread_for_join().join(timeout=1.0)
vision_rx.get_thread_for_join().join(timeout=1.0)
logger.close()

print("Client exited!", flush=True)
