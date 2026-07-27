"""
Camera receiver. The simulator streams the drone's forward (FPV) camera over the network, and this is
where those frames arrive and become something the pilot can use.

Each camera frame is a JPEG image. A JPEG is much bigger than one UDP packet can hold, so the
simulator splits each frame into numbered chunks and sends them as separate packets. _vision_loop
collects the chunks for a frame (keyed by frame_id) until it has them all, reassembles the JPEG,
decodes it into an image, and hands that image to process_frame.

process_frame is the heart of the file: it runs the active gate detector on the image, stores the
detections where the pilot can read them, and (when enabled) logs everything for offline analysis.
All of this runs on its own background thread, so it never blocks the control loop.
"""

import csv
import os
import socket
import struct
import threading

import cv2
import numpy as np

from common.race import race_is_live
from perception.detectors import active_detector
from perception.vision_pose import shadow_compare
from perception.vision_data_collector import VisionDataCollector, COLLECT_VISION_DATA

SHADOW_CSV_HEADER = ["frame_id", "sim_time_ns", "gate_id",
                     "cam_fwd", "cam_right", "cam_down", "gt_fwd", "gt_right", "gt_down",
                     "cam_dist", "gt_dist", "pos_err", "dist_err"]

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 5600

class VisionRX:
    """Receives camera frames from the simulator over UDP and runs perception on each one.

    Built once at startup. It creates the active detector, opens the logging files, and starts a
    background thread (_vision_loop) that listens for packets and calls process_frame per frame."""

    def __init__(self, data, logger=None):
        """Set up the detector and logging, then start the receive thread. `data` is the shared state
        dict the rest of the system reads, and this writes the detections into it. `logger` is
        optional and enables the per-frame dataset and the shadow CSV when present."""
        self.data = data
        self.logger = logger
        # The active gate detector (set by $GATE_DETECTOR, default hsv_classic). Built once.
        self.detector = active_detector()
        self._det_warned = False    # so a broken detector logs once, not every frame
        print(f"Gate detector: {type(self.detector).__name__}", flush=True)
        self._shadow_n = 0          # vision-shadow frame counter (throttles the console summary)
        self._shadow_errs = []      # accumulated camera-vs-truth position errors (for the summary)
        # Vision-shadow goes to a CSV in the session folder; the console only gets a rare summary.
        self._shadow_f = self._shadow_w = None
        if logger is not None:
            try:
                path = os.path.join(logger.session_dir, "vision_shadow.csv")
                self._shadow_f = open(path, "w", newline="")
                self._shadow_w = csv.writer(self._shadow_f)
                self._shadow_w.writerow(SHADOW_CSV_HEADER)
                print(f"Vision-shadow logging to {path}", flush=True)
            except OSError:
                self._shadow_f = self._shadow_w = None
        # Comprehensive per-frame data collector (perception + ground truth + control intent).
        self.vlog = None
        if COLLECT_VISION_DATA and logger is not None:
            try:
                self.vlog = VisionDataCollector(logger.session_dir)
            except OSError:
                self.vlog = None
        self.thread = threading.Thread(
            target=self._vision_loop,
            daemon=True
        )
        self.is_running = True
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _vision_loop(self):
        """Receive camera packets over UDP and reassemble each frame from its chunks.

        A frame's JPEG is too big for one UDP packet, so the simulator splits it into numbered chunks,
        each sent as its own packet (a fixed-size header followed by a slice of the JPEG bytes). We
        accumulate chunks per frame_id until all of them arrive, stitch the JPEG back together, decode
        it, and call process_frame. A frame missing any packet is discarded."""
        header_format = "<IHHIIQ"
        header_sz = struct.calcsize(header_format)
        frames = {}  # frame_id -> received associated frame data

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT))
        # timeout so the loop can notice is_running going False and exit cleanly
        # (otherwise recvfrom blocks forever and Ctrl-C can't stop the process)
        sock.settimeout(0.5)
        print("Listening for camera frames...")

        while self.is_running:
            try:
                packet, addr = sock.recvfrom(65536)  # max UDP size
            except socket.timeout:
                continue

            header = packet[:header_sz]
            payload = packet[header_sz:]

            # Unpack the packet header:
            #   frame_id      which frame this packet belongs to
            #   chunk_id      this packet's index within that frame
            #   total_chunks  how many chunks make up the whole frame
            #   jpeg_size     full size of the assembled JPEG
            #   payload_size  size of this packet's slice
            #   sim_time_ns   frame timestamp (ns) on the server
            frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns = struct.unpack(header_format, header)

            if frame_id not in frames:
                frames[frame_id] = {
                    "chunks": {},
                    "total": total_chunks,
                    "size": jpeg_size,
                    "time": sim_time_ns
                }

            frames[frame_id]["chunks"][chunk_id] = payload

            # Check if frame is complete
            if len(frames[frame_id]["chunks"]) == total_chunks:
                jpeg_bytes = bytearray()

                frame_complete = True
                for i in range(total_chunks):
                    if i not in frames[frame_id]["chunks"]:
                        print('Missing packet %s in frame %s' % (i, frame_id,))
                        frame_complete = False
                        continue
                    jpeg_bytes.extend(frames[frame_id]["chunks"][i])

                if not frame_complete:
                    del frames[frame_id]
                    continue

                img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                # if an image is successfully decoded, process it, otherwise print an error and skip
                if image is not None:
                    sim_time_ns = frames[frame_id]["time"]
                    self.process_frame(frame_id, image, sim_time_ns) # This is where our processing begins
                else:
                    print(f"Failed to decode frame: {frame_id}")

                del frames[frame_id]

        if self.vlog is not None:
            self.vlog.close()

    def process_frame(self, frame_id, img, sim_time_ns):
        """Handle one decoded camera frame. img is the (height, width, 3) BGR image.

        Three things happen here:
          1. run the gate detector and publish the detections for the pilot to read,
          2. log a rich per-frame record for offline analysis (when collection is on),
          3. run the vision-shadow check, comparing camera-only pose to ground truth (when available).

        sim_time_ns is the frame's server timestamp. Keeping it with the frame lets perception be
        aligned against telemetry (pose, etc.) for offline labeling.
        """
        # make the latest frame available to other components (e.g. the controller)
        self.data["latest_frame"] = img
        self.data["latest_frame_id"] = frame_id

        # Detect gates from the camera (no ground truth needed). Store the nearest gate as the target
        # and the full list for look-ahead. offset_x in [-1, 1] is the gate's horizontal bearing in
        # the image (+ = right), the signal a vision controller turns on to centre the gate.
        try:
            gates = self.detector.process(img)
        except Exception as e:           # a broken detector must not take down the vision thread
            if not self._det_warned:
                self._det_warned = True
                print(f"[detector] {type(self.detector).__name__} error (suppressed further): {e!r}",
                      flush=True)
            gates = []
        self.data["vision_gates"] = gates
        self.data["vision_target"] = gates[0] if gates else None

        # Comprehensive data collection: one rich record per frame (perception, ground truth, and
        # control intent), for offline diagnosis of why the vision pilot misses gates.
        if self.vlog is not None:
            self.vlog.log(frame_id, img, sim_time_ns, self.data)

        # Vision-shadow: while the oracle flies (on ground truth), measure how close the camera-only
        # gate pose (detect, then PnP) is to the ground truth on the same frame, so there is no replay
        # mismatch. Every comparison is written to vision_shadow.csv. The console only gets a brief
        # running-median summary every ~5 s so it does not flood.
        cmp = shadow_compare(self.data, img)
        if cmp is not None:
            self._shadow_n += 1
            self._shadow_errs.append(cmp["pos_err"])
            e, g = cmp["est"], cmp["gt"]
            if self._shadow_w is not None:
                self._shadow_w.writerow([
                    frame_id, sim_time_ns, cmp["gate_id"],
                    f"{e[0]:.2f}", f"{e[1]:.2f}", f"{e[2]:.2f}",
                    f"{g[0]:.2f}", f"{g[1]:.2f}", f"{g[2]:.2f}",
                    f"{np.linalg.norm(e):.2f}", f"{np.linalg.norm(g):.2f}",
                    f"{cmp['pos_err']:.2f}", f"{cmp['dist_err']:.2f}"])
                self._shadow_f.flush()
            if self._shadow_n % 150 == 0:
                med = sorted(self._shadow_errs)[len(self._shadow_errs) // 2]
                print(f"[vision-shadow] {self._shadow_n} samples, median err {med:.1f} m "
                      f"(latest g{cmp['gate_id']} err {cmp['pos_err']:.1f} m), see vision_shadow.csv",
                      flush=True)

        # record the frame to the dataset - ONLY once the race timer is live, numbered 0,1,2,... from the
        # start (short, sequential names instead of the sim's giant global frame_id). Publish the seq so
        # the pilot CSV can map each row straight to frames/<seq>.jpg.
        if self.logger is not None and race_is_live(self.data):
            self.data["latest_frame_seq"] = self.logger.log_frame(frame_id, sim_time_ns, img)