#
# Sample Python client for the AI GP controller
#

import time

from runtime.setup import setup_components
from debug_camera import start_debug_camera

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


# Debug camera window 
# Live 3-panel viewer (raw | HSV mask | detections); see debug_camera.py.
# Toggle with the DEBUG_CAMERA flag below; set to False for competition runs.
DEBUG_CAMERA = True

if DEBUG_CAMERA:
    start_debug_camera(shared_data)


print("Arming drone...", flush=True)
controller.arm()
from runtime.controller import CONTROL_MODE
print(f"Starting control loop... (CONTROL_MODE = {CONTROL_MODE})", flush=True)
if CONTROL_MODE == "vision":
    print("  VISION: flying from the camera alone.", flush=True)

elif CONTROL_MODE == "trajectory":
    print("  TRAJECTORY: following a pre-planned racing line through the gates (fast).", flush=True)
    print("  Builds a spline through the gate centres and tracks a carrot along it.", flush=True)

elif CONTROL_MODE == "characterize":
    print("  CHARACTERIZE: flying a scripted profile to measure the drone's physics.", flush=True)
    print("  It will fly aggressively (full/zero thrust, 30deg lean, max rates) and may", flush=True)
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
shared_data["running"] = False      # make sure debug thread exits if Ctrl-C hit
ts_loop.get_thread_for_join().join(timeout=1.0)
mavlink_rx.get_thread_for_join().join(timeout=1.0)
vision_rx.get_thread_for_join().join(timeout=1.0)
logger.close()

print("Client exited!", flush=True)