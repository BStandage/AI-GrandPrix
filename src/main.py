#
# Sample Python client for the AI GP controller
#

import time
import threading

import cv2

from runtime.setup import setup_components
from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections, annotate

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


# ── Debug camera window ───────────────────────────────────────────────────────
# Runs on its own thread so the cv2.imshow loop never blocks the control loop.
# Toggle with the DEBUG_CAMERA flag below; set to False for competition runs.
DEBUG_CAMERA = True

def _debug_camera_thread():
    """Show a 3-panel live view: raw frame | HSV mask | annotated detections.
    Pressing Q or ESC inside the window stops the whole program cleanly."""
    print("[debug] Camera window starting — press Q or ESC in the window to quit.", flush=True)
    while shared_data["running"]:
        frame = shared_data.get("latest_frame")
        if frame is None:
            time.sleep(0.01)
            continue

        # Run the same HSV segmentation the flight controller uses
        mask = gate_mask(frame)
        dets = mask_to_detections(mask, frame.shape)
        annotated = annotate(frame.copy(), dets)

        # Stack: raw (left) | mask (centre) | annotated (right)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = cv2.hconcat([frame, mask_bgr, annotated])

        # Status label — gate bearing, distance, area
        if dets:
            best = dets[0]
            label = (f"gate  offset={best.offset_x:+.2f}  dist={best.distance_m:.1f}m  "
                     f"area={best.area:.0f}  opening={'Y' if best.has_opening else 'N'}")
        else:
            label = "no gate detected"
        cv2.putText(combined, label, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        cv2.imshow("AI-GP debug  |  raw  |  HSV mask  |  detections", combined)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):     # Q or ESC closes everything
            shared_data["running"] = False
            break

    cv2.destroyAllWindows()
    print("[debug] Camera window closed.", flush=True)

if DEBUG_CAMERA:
    debug_thread = threading.Thread(target=_debug_camera_thread, daemon=True)
    debug_thread.start()
# ─────────────────────────────────────────────────────────────────────────────


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