#
# Dataset logger for AI GP flight sessions.
#
# Each run creates a timestamped session folder under datasets/:
#
#   datasets/session_YYYYMMDD_HHMMSS/
#     frames/0000000123.jpg   <- raw FPV camera frames (one JPEG per frame)
#     frames.jsonl            <- one record per frame: id, timestamps, file, size
#     telemetry.jsonl         <- one record per MAVLink message (pose, imu, collisions...)
#     gates.json              <- ground-truth track layout (gate positions/sizes), written once
#
# Frame records carry sim_time_ns and telemetry records carry their MAVLink
# timestamps, so the two streams can be aligned in time after the fact. That
# alignment is what lets us auto-label gates later: take the drone pose at a
# frame's timestamp + the known gate geometry, and project gate corners into
# the image to generate detection labels for free.
#

import json
import os
import threading
import time

import cv2

from common.paths import DATASETS_DIR


class DataLogger:

    def __init__(self, base_dir=None):
        # Default to a "datasets" folder next to this file, so sessions always
        # land in src/datasets regardless of the current working directory
        # (running from the repo root vs. from src would otherwise scatter
        # them in different places).
        if base_dir is None:
            base_dir = DATASETS_DIR
        session = time.strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(base_dir, session)
        self.frames_dir = os.path.join(self.session_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)

        # one writer lock; frames come from the vision thread, telemetry from
        # the mavlink thread, so writes need to be serialized
        self._lock = threading.Lock()
        self._frames_f = open(os.path.join(self.session_dir, "frames.jsonl"), "w")
        self._telem_f = open(os.path.join(self.session_dir, "telemetry.jsonl"), "w")

        self.frame_count = 0
        self.gates_written = False

        print(f"Logging dataset to {self.session_dir}", flush=True)

    def log_frame(self, frame_id, sim_time_ns, img):
        # frame_id from the sim is a uint32; zero-pad so files sort in order
        filename = f"{frame_id:010d}.jpg"
        cv2.imwrite(os.path.join(self.frames_dir, filename), img)

        height, width = img.shape[:2]
        record = {
            "frame_id": frame_id,
            "sim_time_ns": sim_time_ns,
            "recv_time_ns": time.time_ns(),
            "file": f"frames/{filename}",
            "width": width,
            "height": height,
        }
        with self._lock:
            self._frames_f.write(json.dumps(record) + "\n")
            self._frames_f.flush()
            self.frame_count += 1
            if self.frame_count % 300 == 0:    # ~every 10 s at 30 Hz (was every 30 = spammy)
                print(f"  logged {self.frame_count} frames", flush=True)

    def log_telemetry(self, kind, fields):
        record = {"kind": kind, "recv_time_ns": time.time_ns()}
        record.update(fields)
        with self._lock:
            self._telem_f.write(json.dumps(record) + "\n")
            self._telem_f.flush()

    def log_gates(self, gates):
        # the track layout is sent once; write it as a standalone file
        with self._lock:
            with open(os.path.join(self.session_dir, "gates.json"), "w") as f:
                json.dump({"num_gates": len(gates), "gates": gates}, f, indent=2)
            self.gates_written = True
        print(f"  wrote track layout: {len(gates)} gates", flush=True)

    def close(self):
        with self._lock:
            self._frames_f.close()
            self._telem_f.close()
        print(f"Dataset closed: {self.frame_count} frames in {self.session_dir}", flush=True)
