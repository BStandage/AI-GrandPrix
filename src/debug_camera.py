#
# Live camera feed debug viewer
# Shows the raw simulator frame alongside the HSV-processed gate mask in real time.
#
# Run separately from main.py (while the sim + server are running):
#   python debug_camera.py
#

import cv2
import time

from runtime.setup import setup_components
from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections, annotate

SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

system_boot_ms = int(time.time() * 1000)
shared_data = {"running": True}

components = setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)
vision_rx = components['vision_rx']

print("Starting live camera debug view. Press Q or ESC to quit.", flush=True)

while shared_data["running"]:
    frame = shared_data.get("latest_frame")   
    if frame is None:
        time.sleep(0.01)
        continue

    mask = gate_mask(frame)
    dets = mask_to_detections(mask, frame.shape)
    annotated = annotate(frame.copy(), dets)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    combined = cv2.hconcat([frame, mask_bgr, annotated])

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
    if key in (ord('q'), ord('Q'), 27):
        shared_data["running"] = False
        break

cv2.destroyAllWindows()
print("Debug viewer closed.", flush=True)