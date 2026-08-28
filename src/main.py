#
# AI-GP client entry. Deadline-paced control loop (do not busy-spin).
#

import ctypes
import time

# Windows default timer granularity is ~15.6 ms. Without this, sleep rounds up
# and open-loop tape replay skips rows (run-to-run gate scatter).
ctypes.windll.winmm.timeBeginPeriod(1)

from common.dynamics import CONTROL_HZ
from runtime.setup import setup_components
from debug_camera import start_debug_camera

SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

system_boot_ms = int(time.time() * 1000)
shared_data = {"running": True}

components = setup_components(
    shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)
controller = components["controller"]
ts_loop = components["ts_loop"]
mavlink_rx = components["mavlink_rx"]
vision_rx = components["vision_rx"]
logger = components["logger"]

# Live 3-panel viewer — keep False for race runs (steals CPU from sim/send).
DEBUG_CAMERA = True
if DEBUG_CAMERA:
    start_debug_camera(shared_data)

print("Arming drone...", flush=True)
controller.arm()
from runtime.controller import CONTROL_MODE
print(f"Starting control loop... (CONTROL_MODE = {CONTROL_MODE})", flush=True)
if CONTROL_MODE == "steady":
    print("  STEADY: pure vision (PQ default - map / race unknown course)", flush=True)
elif CONTROL_MODE == "ace":
    print("  ACE: open-loop tape_WORK.json (only after PQ map exists)", flush=True)
elif CONTROL_MODE == "giga":
    print("  GIGA: open-loop tape + est logs (only after PQ map exists)", flush=True)

# Deadline pace at CONTROL_HZ. Sleeping a full period AFTER update work made the
# real tick ~13 ms vs tape step 11.1 ms -> ~17% of tape rows skipped, which rows
# depending on start phase (gate-hit lottery). Sleep only the residual.
_period = 1.0 / CONTROL_HZ
_next = time.perf_counter()
print(
    f"Control pace: {_period*1000:.2f} ms deadline ({CONTROL_HZ} Hz).",
    flush=True,
)
try:
    while shared_data["running"]:
        controller.update()
        _next += _period
        delay = _next - time.perf_counter()
        if delay > 0.0004:
            time.sleep(delay - 0.0004)
            while time.perf_counter() < _next:
                pass
        elif delay < -_period:
            # Fell more than one tick behind (GC/hitch) — resync, don't spiral.
            _next = time.perf_counter()
except KeyboardInterrupt:
    print("Interrupted.", flush=True)

shared_data["running"] = False
ts_loop.get_thread_for_join().join(timeout=1.0)
mavlink_rx.get_thread_for_join().join(timeout=1.0)
vision_rx.get_thread_for_join().join(timeout=1.0)
logger.close()
print("Client exited!", flush=True)
