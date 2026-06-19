#
# Live camera feed debug viewer
# Shows the raw simulator frame alongside the HSV-processed gate mask in real time.
#
# Run from main.py: set DEBUG_CAMERA = True

import threading
import time

import cv2

from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections, annotate


def _debug_camera_loop(shared_data):
    """Show a 3-panel live view: raw frame | HSV mask | annotated detections.
    Pressing Q or ESC inside the window stops the whole program cleanly."""
    print("[debug] Camera window starting - press Q or ESC in the window to quit.", flush=True)
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

        # Status label - gate bearing, distance, area
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


def start_debug_camera(shared_data):
    """Launch the debug viewer on its own daemon thread so the cv2.imshow loop
    never blocks the control loop. Returns the thread."""
    thread = threading.Thread(target=_debug_camera_loop, args=(shared_data,), daemon=True)
    thread.start()
    return thread


if __name__ == "__main__":
    # Standalone: spin up our own components and run the viewer on the main thread.
    from runtime.setup import setup_components

    SIM_SERVER_UDP_IP = "127.0.0.1"
    SIM_SERVER_UDP_PORT = 14550

    system_boot_ms = int(time.time() * 1000)
    shared_data = {"running": True}

    setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)

    print("Starting live camera debug view. Press Q or ESC to quit.", flush=True)
    _debug_camera_loop(shared_data)
