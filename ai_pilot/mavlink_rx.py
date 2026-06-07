import struct
import time
import threading

from pymavlink import mavutil

ENCAPSULATED_RACE_STATUS_MSG_ID = 1
ENCAPSULATED_TRACK_INFO_MSG_ID  = 2

class MAVLinkRX:

    def __init__(self, mavlink_connection, data, logger=None):
        self.mavlink_conn = mavlink_connection
        self.data = data
        self.logger = logger
        self.thread = None
        self.is_running = False

        self.track_chunks = {}
        self.expected_num_track_chunks = {}
        self._assembled_transfers = set()   # transfer_ids already parsed (don't re-log/re-parse)

    @classmethod
    def create_mavlink_rx(cls, mavlink_connection, data, logger=None):
        rx = cls(mavlink_connection, data, logger)
        rx.thread = threading.Thread(
            target=rx.mavlink_receive_loop,
            daemon = True
        )
        rx.is_running = True
        rx.thread.start()
        return rx

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def mavlink_receive_loop(self):
        """
        Continuously receive MAVLink messages without blocking.
        """
        while self.is_running:

            try:
                msg = self.mavlink_conn.recv_match(blocking=False)
            except ConnectionResetError:
                print('WARNING: ConnectionResetError was thrown. No longer listening to MAVLink port.')
                return

            if msg is None:
                time.sleep(0.001)
                continue

            msg_type = msg.get_type()

            if msg_type == "BAD_DATA":
                continue

            # --------------------------------------------------------------------------------------
            # HEARTBEAT
            # --------------------------------------------------------------------------------------
            if msg_type == "HEARTBEAT":
                self.on_heartbeat(msg)

            # --------------------------------------------------------------------------------------
            # TIMESYNC
            # --------------------------------------------------------------------------------------
            elif msg_type == "TIMESYNC":
                self.on_timesync(msg)

            # --------------------------------------------------------------------------------------
            # ATTITUDE
            # --------------------------------------------------------------------------------------
            elif msg_type == "ATTITUDE":
                self.on_attitude(msg)

            # --------------------------------------------------------------------------------------
            # LOCAL_POSITION_NED
            # --------------------------------------------------------------------------------------
            elif msg_type == "LOCAL_POSITION_NED":
                self.on_local_position_ned(msg)

            # --------------------------------------------------------------------------------------
            # ODOMETRY
            # --------------------------------------------------------------------------------------
            elif msg_type == "ODOMETRY":
                self.on_odometry(msg)

            # --------------------------------------------------------------------------------------
            # HIGHRES_IMU
            # --------------------------------------------------------------------------------------
            elif msg_type == "HIGHRES_IMU":
                self.on_highres_imu(msg)

            # --------------------------------------------------------------------------------------
            # ENCAPSULATED_DATA
            # --------------------------------------------------------------------------------------
            elif msg_type == "ENCAPSULATED_DATA":
                self.on_encapsulated_data(msg)

            # --------------------------------------------------------------------------------------
            # ACTUATOR_OUTPUT_STATUS
            # --------------------------------------------------------------------------------------
            elif msg_type == "ACTUATOR_OUTPUT_STATUS":
                self.on_actuator_output_status(msg)

            # --------------------------------------------------------------------------------------
            # COLLISION
            # --------------------------------------------------------------------------------------
            elif msg_type == "COLLISION":
                self.on_collision(msg)

            # --------------------------------------------------------------------------------------
            # DATA_TRANSMISSION_HANDSHAKE - Repurposed and used for upcoming 'Track Data' packets
            # --------------------------------------------------------------------------------------
            elif msg.get_type() == "DATA_TRANSMISSION_HANDSHAKE":
                # The handshake announces how many chunks the track spans. It's just a HINT:
                # don't wipe chunks that arrived before it (UDP can reorder), and don't depend
                # on it - reassembly below is self-describing. setdefault, never reset.
                tid = msg.width
                self.track_chunks.setdefault(tid, {})
                self.expected_num_track_chunks[tid] = msg.packets
                print(f"[track] handshake: transfer {tid}, expecting {msg.packets} chunk(s)", flush=True)
                self._try_assemble_track(tid)

    def on_heartbeat(self, msg):
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        # print only on change so we can see whether the arm command took effect
        if armed != self.data.get("armed"):
            print(f"[heartbeat] armed = {armed}", flush=True)
        self.data["armed"] = armed

    def on_timesync(self, msg):
        request_time = msg.ts1
        response_time = msg.tc1

    def _log(self, kind, fields):
        # mirror the latest value into shared_data (for live use by the controller)
        # and append it to the telemetry log (for offline training/labeling)
        self.data[kind] = fields
        if self.logger is not None:
            self.logger.log_telemetry(kind, fields)

    def on_attitude(self, msg):
        self._log("attitude", {
            "roll": msg.roll,
            "pitch": msg.pitch,
            "yaw": msg.yaw,
            "rollspeed": msg.rollspeed,
            "pitchspeed": msg.pitchspeed,
            "yawspeed": msg.yawspeed,
            "time_boot_ms": msg.time_boot_ms,
        })

    def on_local_position_ned(self, msg):
        self._log("local_position_ned", {
            "x": msg.x, "y": msg.y, "z": msg.z,
            "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
            "time_boot_ms": msg.time_boot_ms,
        })

    def on_odometry(self, msg):
        # quaternion stored (w, x, y, z); MAVLink delivers q[0]=w
        self._log("odometry", {
            "x": msg.x, "y": msg.y, "z": msg.z,
            "qw": msg.q[0], "qx": msg.q[1], "qy": msg.q[2], "qz": msg.q[3],
            "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
            "rollspeed": msg.rollspeed,
            "pitchspeed": msg.pitchspeed,
            "yawspeed": msg.yawspeed,
            "time_usec": msg.time_usec,
            "reset_counter": msg.reset_counter,
        })

    def on_highres_imu(self, msg):
        self._log("highres_imu", {
            "xacc": msg.xacc, "yacc": msg.yacc, "zacc": msg.zacc,
            "xgyro": msg.xgyro, "ygyro": msg.ygyro, "zgyro": msg.zgyro,
            "time_usec": msg.time_usec,
        })

    def on_encapsulated_data(self, msg):
        if msg:
            raw_payload = bytes(msg.data)
            data_type = raw_payload[0]

            if int(data_type) == ENCAPSULATED_RACE_STATUS_MSG_ID:
                self.on_race_status(msg)
            elif int(data_type) == ENCAPSULATED_TRACK_INFO_MSG_ID:
                self.on_track_data_packet(msg)

    def on_race_status(self, msg):
        raw_payload = bytes(msg.data)
        # data_type - ID of this message
        # sim_boot_time_ms - elapsed ms on server since sim boot
        # race_start_boot_time_ms - elapsed ms on server since sim boot when race started. None or < 0 if race has not started
        # race_finish_time_ns - elapsed ns on server since sim boot when race finished. None or < 0 if race is ongoing
        # active_gate_index - current index of target race gate
        # last_gate_race_time - race time in seconds when last gate was passed
        data_type, sim_boot_time_ms, race_start_boot_time_ms, race_finish_time_ns, active_gate_index, last_gate_race_time = struct.unpack_from(
            "<BQqqIq", raw_payload)
        self._log("race_status", {
            "sim_boot_time_ms": sim_boot_time_ms,
            "race_start_boot_time_ms": race_start_boot_time_ms,
            "race_finish_time_ns": race_finish_time_ns,
            "active_gate_index": active_gate_index,
            "last_gate_race_time": last_gate_race_time,
        })

    def on_track_data_packet(self, msg):
        raw_payload = bytes(msg.data)
        if len(raw_payload) < 3:
            return
        # header: data_type (B), transfer_id (H). The rest is this chunk's slice of the track.
        # IMPORTANT: store the chunk even if we never saw the handshake for this transfer_id.
        # The old code dropped it ("if transfer_id not in expected: return"), so a single lost
        # handshake packet silently discarded the ENTIRE track - and a 6-gate track is one
        # chunk, so that one loss = no gates = a wasted run (sessions 132007/130923).
        data_type, transfer_id = struct.unpack_from("<BH", raw_payload)
        chunk = raw_payload[3:]
        self.track_chunks.setdefault(transfer_id, {})[msg.seqnr] = chunk
        self._try_assemble_track(transfer_id)

    def _try_assemble_track(self, transfer_id):
        """Reassemble + parse the track as soon as we have it, WITHOUT needing the handshake.
        The payload is self-describing: it starts with num_gates (uint16), so the full length
        is 2 + 38*num_gates bytes. We concatenate the longest contiguous run of chunks from
        seqnr 0 and parse the moment we have enough bytes. (A lost MIDDLE chunk still stalls -
        only a fresh re-broadcast recovers that - but the common single-chunk track is now
        robust to a lost handshake, which was the actual failure.)"""
        if transfer_id in self._assembled_transfers:
            return
        chunks = self.track_chunks.get(transfer_id)
        if not chunks or 0 not in chunks:
            return   # need at least chunk 0 (it holds num_gates)
        ordered = bytes()
        i = 0
        while i in chunks:           # longest contiguous run from the start
            ordered += chunks[i]
            i += 1
        if len(ordered) < 2:
            return
        num_gates, = struct.unpack_from("<H", ordered)
        if num_gates == 0 or num_gates > 100:
            return   # implausible -> chunk 0 is corrupt/partial, wait for more
        needed = 2 + 38 * num_gates
        expected = self.expected_num_track_chunks.get(transfer_id)
        count_complete = expected is not None and len(chunks) >= expected
        if len(ordered) < needed and not count_complete:
            return   # not all the bytes yet
        if len(ordered) < needed:
            return   # handshake count reached but bytes short (a chunk was lost) -> wait
        self._assembled_transfers.add(transfer_id)
        self.track_chunks.pop(transfer_id, None)
        self.expected_num_track_chunks.pop(transfer_id, None)
        self.on_track_data(ordered)

    def on_track_data(self, payload):
        # header:
        #   num_gates - track gate count
        num_gates, = struct.unpack_from("<H", payload)
        payload = payload[2:]
        gates = []
        for i in range(num_gates):
            # Gate Info
            #   gate_id - range is 0 - num_gates
            #   position_ned_x, position_ned_y, position_ned_z - Position of gate in NED coordinates
            #   orientation_ned_w, orientation_ned_x, orientation_ned_y, orientation_ned_z - Orientation of gate in NED coordinates
            #   width - gate width in metres
            #   height - gate height in metres
            gate_id, position_ned_x, position_ned_y, position_ned_z, orientation_ned_w, orientation_ned_x, orientation_ned_y, orientation_ned_z, width, height = struct.unpack_from(
                "<Hfffffffff", payload)
            payload = payload[38:]
            gates.append({
                "gate_id": gate_id,
                "position_ned": [position_ned_x, position_ned_y, position_ned_z],
                "orientation_ned": [orientation_ned_w, orientation_ned_x, orientation_ned_y, orientation_ned_z],
                "width": width,
                "height": height,
            })

        # the ground-truth track layout: keep it live, and persist it once.
        # NOTE: these are sim-only world coordinates - use them to auto-label
        # frames and for development, not as input to the final vision pilot
        # (the real race provides no GPS/absolute coordinates).
        self.data["gates"] = gates
        g0 = next((g for g in gates if g.get("gate_id") == 0), gates[0] if gates else None)
        if g0 is not None:
            p = g0["position_ned"]
            print(f"[track] LIVE TRACK received: {len(gates)} gates "
                  f"(gate0 at [{p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}])", flush=True)
        if self.logger is not None and not self.logger.gates_written:
            self.logger.log_gates(gates)

    def on_actuator_output_status(self, msg):
        # the drone's actual motor outputs - i.e. what the pilot (you, or a
        # policy) actually commanded. These are the action labels for imitation
        # learning when paired with the camera frames.
        self._log("actuator_output_status", {
            "time_usec": msg.time_usec,
            "motor_front_left": msg.actuator[0],
            "motor_front_right": msg.actuator[1],
            "motor_back_left": msg.actuator[2],
            "motor_back_right": msg.actuator[3],
        })

    def on_collision(self, msg):
        # Collision IDs
        # 1001 - Gate
        # 1002 - Environment
        self._log("collision", {
            "collision_id": msg.id,
            "threat_level": msg.threat_level,   # 1-2, 2 = higher impact
            "impact": msg.horizontal_minimum_delta,  # impulse magnitude in kg m/s (not a delta)
        })