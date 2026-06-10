import csv
import os
import socket
import struct
import threading

import cv2
import numpy as np

from perception.gate_detector import detect_gates
from perception.vision_pose import shadow_compare
from perception.vision_logger import VisionFrameLogger, COLLECT_VISION_DATA

SHADOW_CSV_HEADER = ["frame_id", "sim_time_ns", "gate_id",
                     "cam_fwd", "cam_right", "cam_down", "gt_fwd", "gt_right", "gt_down",
                     "cam_dist", "gt_dist", "pos_err", "dist_err"]

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 5600

class VisionRX:

    def __init__(self, data, logger=None):
        self.data = data
        self.logger = logger
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
                print(f"Vision-shadow logging -> {path}", flush=True)
            except OSError:
                self._shadow_f = self._shadow_w = None
        # Comprehensive per-frame data collector (perception + ground truth + control intent).
        self.vlog = None
        if COLLECT_VISION_DATA and logger is not None:
            try:
                self.vlog = VisionFrameLogger(logger.session_dir)
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

            # frame_id - identifier for this vision frame
            # chunk_id - identifier for this chunk packet of data of this frame
            # total_chunks - total number of chunk packets that make up this frame
            # jpeg_size - full size of jpeg data
            # payload_size - size of this packet
            # sim_time_ns - frame's epoch timestamp in ns on the server
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
        """
        The input var img is a numpy array representing the decoded image frame from the simulator's FPV camera.
        This is where we will call all helper functions to process the image and extract information for our pilot agent.

        sim_time_ns is the frame's server timestamp - keep it with the frame so it
        can be aligned against telemetry (pose, etc.) for offline labeling.
        """
        # make the latest frame available to other components (e.g. the controller)
        self.data["latest_frame"] = img
        self.data["latest_frame_id"] = frame_id

        # VISION: detect gates from the camera (no ground truth needed). Store the
        # nearest gate as the target and the full list for look-ahead. offset_x in
        # [-1,1] is the gate's horizontal bearing in the image (+ = right) - the
        # signal a vision-based controller yaws on to centre the gate.
        gates, _ = detect_gates(img)
        self.data["vision_gates"] = gates
        self.data["vision_target"] = gates[0] if gates else None

        # COMPREHENSIVE DATA COLLECTION: one rich record per frame (perception + ground truth +
        # control intent), for offline diagnosis of why the vision pilot misses gates.
        if self.vlog is not None:
            self.vlog.log(frame_id, img, sim_time_ns, self.data)

        # VISION-SHADOW: while the oracle flies (on ground truth), measure how close the
        # camera-only gate pose (detect -> PnP) is to the LIVE ground truth - same frame, so no
        # replay frame-mismatch. Every comparison is written to vision_shadow.csv; the console
        # only gets a brief running-median summary every ~5 s so it doesn't flood.
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
                      f"(latest g{cmp['gate_id']} err {cmp['pos_err']:.1f} m) -> vision_shadow.csv",
                      flush=True)

        # record the frame to the dataset for offline training/labeling
        if self.logger is not None:
            self.logger.log_frame(frame_id, sim_time_ns, img)