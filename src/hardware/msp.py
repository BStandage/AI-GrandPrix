"""
MultiWii Serial Protocol (MSP v1) client for Betaflight 4.5.

The Orin talks to the Archer's flight controller over /dev/ttyTHS1 at
115200 baud; the sim's Betaflight SITL exposes the same protocol on TCP
port 5761. Both are request/response: we send a frame, the FC answers.
The FC never pushes.

Frame (v1):   '$' 'M' '<'  size  cmd  payload[size]  crc
              crc = XOR of size, cmd and every payload byte
Response:     '$' 'M' '>'  size  cmd  payload  crc      (or '!' = error)

Payload layouts are Betaflight 4.5 (src/main/msp/msp.c). Every decode
here is bounds-checked; a shorter payload than expected yields None for
the missing fields instead of an exception, because firmware minor
versions add trailing fields.

Channel order for MSP_SET_RAW_RC is the FC's `map` (default AETR1234):
roll, pitch, throttle, yaw, aux1 (arm), aux2 ... - the same order the sim
bridge uses. Confirm `map` in the Archer's `diff all`.
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

# --- command ids ------------------------------------------------------------
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_BATTERY_STATE = 130
MSP_STATUS_EX = 150
MSP_BOXNAMES = 116      # ';'-separated names of the ENABLED boxes, in flightModeFlags bit order
MSP_SET_RAW_RC = 200

# Betaflight 4.5 arming-disable flag bits, in order (src/main/fc/runtime_config.h)
ARMING_DISABLE_FLAGS = [
    "NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "RUNAWAY", "CRASH",
    "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI", "CMS",
    "BST", "MSP", "PARALYZE", "GPS", "RESCUE_SW", "RPMFILTER", "REBOOT_REQD",
    "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO", "ARMSWITCH",
]

# Betaflight box ids in permanent order; MSP_STATUS flightModeFlags bit i = box i active
# (only the first few are stable across versions; enough to see ARM / ANGLE / HORIZON)
FLIGHT_MODE_BOXES = ["ARM", "ANGLE", "HORIZON", "MAG", "HEADFREE", "PASSTHRU", "FAILSAFE", "GPSRESCUE"]

MSP_BOXIDS = 119        # PERMANENT box ids, in flightModeFlags bit order

# The qualifier FC (BF 4.4.3, BF_BLOCK2) never answers MSP_BOXNAMES - three
# timeouts at 0.25, 1.0 and 2.0 s on d45, 2026-09-20 - so box_names() came back
# empty, active_modes fell back to the 8-entry list above, and msp_override was
# permanently False. The runtime waits on that flag, so it would have sat at
# "waiting for the pilot" forever. MSP_BOXIDS does answer, and a permanent id
# is a better key than a name string anyway.
BOX_ID_NAMES = {0: "ARM", 1: "ANGLE", 2: "HORIZON", 6: "CAMSTAB", 7: "PASSTHRU",
                8: "BEEPERON", 13: "SERVO1", 19: "3D", 20: "FPVANGLEMIX",
                26: "PREARM", 27: "BEEPGPSCOUNT", 30: "USER1", 31: "USER2",
                32: "USER3", 33: "USER4", 34: "PIDAUDIO", 35: "ACROTRAINER",
                36: "VTXCONTROLDISABLE", 37: "LAUNCHCONTROL", 39: "STICKCOMMANDDISABLE",
                40: "BEEPERMUTE", 41: "READY", 43: "LAPTIMERRESET", 45: "GPSRESCUE",
                46: "AIRMODE", 48: "OSD", 49: "TELEMETRY", 50: "MSP OVERRIDE",
                51: "BLACKBOX", 52: "FAILSAFE", 53: "CAMERA1"}
BOX_ID_ARM, BOX_ID_ANGLE, BOX_ID_MSP_OVERRIDE = 0, 1, 50

RC_CENTER = 1500
RC_MIN = 1000
RC_MAX = 2000
RC_CHANNELS = 16


class MspError(Exception):
    pass


class MspTimeout(MspError):
    pass


# --- framing ------------------------------------------------------------------

def encode(cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= cmd <= 254:
        raise ValueError(f"MSP v1 command out of range: {cmd}")
    if len(payload) > 255:
        raise ValueError("MSP v1 payload > 255 bytes")
    crc = len(payload) ^ cmd
    for b in payload:
        crc ^= b
    return b"$M<" + bytes([len(payload), cmd]) + payload + bytes([crc])


@dataclass
class Frame:
    cmd: int
    payload: bytes
    error: bool = False


class Parser:
    """Incremental v1 response parser. Feed bytes, pop frames."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_crc = 0
        self.resync = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buf += data
        out = []
        while True:
            i = self._buf.find(b"$M")
            if i < 0:
                if len(self._buf) > 1:
                    self.resync += len(self._buf) - 1
                    del self._buf[:-1]
                break
            if i > 0:
                self.resync += i
                del self._buf[:i]
            if len(self._buf) < 3:
                break
            direction = self._buf[2:3]
            if direction not in (b">", b"!", b"<"):
                # a '$M' inside garbage: drop the false header, keep searching
                self.resync += 2
                del self._buf[:2]
                continue
            if len(self._buf) < 5:
                break
            size = self._buf[3]
            total = 5 + size + 1
            if len(self._buf) < total:
                break
            cmd = self._buf[4]
            payload = bytes(self._buf[5:5 + size])
            crc = self._buf[5 + size]
            calc = size ^ cmd
            for b in payload:
                calc ^= b
            del self._buf[:total]
            if direction == b"<":
                self.resync += total          # our own request echoed back (loopback)
                continue
            if calc != crc:
                self.bad_crc += 1
                continue
            out.append(Frame(cmd=cmd, payload=payload, error=(direction == b"!")))
        return out


# --- transports ---------------------------------------------------------------

class TcpTransport:
    """The sim's Betaflight SITL: MSP on tcp://<host>:5761."""

    def __init__(self, host: str, port: int = 5761, timeout: float = 2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.0)
        self.name = f"tcp://{host}:{port}"

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        try:
            return self.sock.recv(max_bytes)
        except (BlockingIOError, socket.timeout):
            return b""

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class SerialTransport:
    """The Archer: MSP on the Orin's UART (/dev/ttyTHS1 @ 115200)."""

    def __init__(self, port: str, baud: int = 115200):
        try:
            import serial  # pyserial; on the Orin it ships with the organizers' tools
        except ImportError as e:
            raise MspError("pyserial is required for a serial port (pip install pyserial)") from e
        self.ser = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0.2)
        self.name = f"{port}@{baud}"

    def write(self, data: bytes) -> None:
        self.ser.write(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        n = self.ser.in_waiting
        return self.ser.read(min(n, max_bytes)) if n else b""

    def close(self) -> None:
        try:
            self.ser.close()
        except OSError:
            pass


def open_transport(port: Optional[str] = None, tcp: Optional[str] = None, baud: int = 115200):
    """`tcp` = 'host[:port]' for the SITL, else `port` = a serial device."""
    if tcp:
        host, _, p = tcp.partition(":")
        return TcpTransport(host, int(p) if p else 5761)
    if not port:
        raise MspError("give --port /dev/ttyTHS1 (serial) or --tcp host:5761 (SITL)")
    return SerialTransport(port, baud)


# --- decoded telemetry ----------------------------------------------------------

@dataclass
class Attitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float          # heading 0..360, clockwise from north (Betaflight)


@dataclass
class RawImu:
    acc: tuple            # raw units; 1 g ~ 512 on Betaflight 4.x (see acc_1G)
    gyro: tuple           # deg/s
    mag: tuple


@dataclass
class Status:
    cycle_time_us: int
    i2c_errors: int
    sensors: int
    flight_mode_flags: int
    pid_profile: int
    cpu_load: int
    arming_disable_flags: Optional[int] = None
    armed: bool = False
    box_names: Optional[list] = None   # from MSP_BOXNAMES: bit i of flight_mode_flags = box_names[i]
    box_ids: Optional[list] = None     # from MSP_BOXIDS: bit i -> permanent box id

    @property
    def active_ids(self) -> list[int]:
        """Permanent box ids that are on right now. Empty when the FC did not
        give us MSP_BOXIDS."""
        ids = self.box_ids or []
        return [b for i, b in enumerate(ids) if self.flight_mode_flags & (1 << i)]

    @property
    def active_modes(self) -> list[str]:
        if self.box_ids:
            return [BOX_ID_NAMES.get(b, f"BOX{b}") for b in self.active_ids]
        names = self.box_names or FLIGHT_MODE_BOXES
        return [n for i, n in enumerate(names) if self.flight_mode_flags & (1 << i)]

    @property
    def angle_mode(self) -> bool:
        if self.box_ids:
            return BOX_ID_ANGLE in self.active_ids
        return "ANGLE" in self.active_modes

    @property
    def msp_override(self) -> bool:
        """The MSP OVERRIDE box is on: our sticks are being used. ARM, the
        override switch and the mode switches stay on the transmitter
        (msp_override_channels_mask = 15 covers channels 1-4 only).

        By permanent id 50 when the FC gives us MSP_BOXIDS, because this
        firmware does not answer MSP_BOXNAMES and the name fallback silently
        reported False forever."""
        if self.box_ids:
            return BOX_ID_MSP_OVERRIDE in self.active_ids
        return any("OVERRIDE" in n.upper() for n in self.active_modes)

    @property
    def arming_blockers(self) -> list[str]:
        if self.arming_disable_flags is None:
            return []
        return [n for i, n in enumerate(ARMING_DISABLE_FLAGS)
                if self.arming_disable_flags & (1 << i)]


@dataclass
class Altitude:
    alt_m: float
    vario_mps: float


@dataclass
class Battery:
    voltage_v: Optional[float]
    current_a: Optional[float]
    mah_drawn: Optional[int]
    cells: Optional[int] = None


def _u16(b, i): return struct.unpack_from("<H", b, i)[0]
def _i16(b, i): return struct.unpack_from("<h", b, i)[0]
def _u32(b, i): return struct.unpack_from("<I", b, i)[0]
def _i32(b, i): return struct.unpack_from("<i", b, i)[0]


def decode_attitude(p: bytes) -> Attitude:
    if len(p) < 6:
        raise MspError("MSP_ATTITUDE payload too short")
    return Attitude(_i16(p, 0) / 10.0, _i16(p, 2) / 10.0, float(_i16(p, 4)))


def decode_raw_imu(p: bytes) -> RawImu:
    if len(p) < 18:
        raise MspError("MSP_RAW_IMU payload too short")
    v = struct.unpack_from("<9h", p, 0)
    return RawImu(acc=v[0:3], gyro=v[3:6], mag=v[6:9])


def decode_status(p: bytes) -> Status:
    """MSP_STATUS_EX (150) or MSP_STATUS (101); both start the same way."""
    if len(p) < 11:
        raise MspError("MSP_STATUS payload too short")
    st = Status(cycle_time_us=_u16(p, 0), i2c_errors=_u16(p, 2), sensors=_u16(p, 4),
                flight_mode_flags=_u32(p, 6), pid_profile=p[10],
                cpu_load=_u16(p, 11) if len(p) >= 13 else 0)
    st.armed = bool(st.flight_mode_flags & 1)         # box 0 = ARM
    # STATUS_EX: u8 pidProfileCount, u8 rateProfile, then u8 n + n bytes of
    # extra flight-mode flags, then u8 count + u32 arming disable flags.
    i = 13
    if len(p) >= i + 2:
        i += 2                                          # pidProfileCount, rateProfile
        if len(p) >= i + 1:
            n = p[i]; i += 1 + n
            if len(p) >= i + 5:
                i += 1                                  # armingDisableFlagsCount
                st.arming_disable_flags = _u32(p, i)
    return st


def decode_altitude(p: bytes) -> Altitude:
    if len(p) < 6:
        raise MspError("MSP_ALTITUDE payload too short")
    return Altitude(_i32(p, 0) / 100.0, _i16(p, 4) / 100.0)


def decode_analog(p: bytes) -> Battery:
    if len(p) < 7:
        raise MspError("MSP_ANALOG payload too short")
    v_legacy = p[0] / 10.0
    mah = _u16(p, 1)
    amps = _i16(p, 5) / 100.0
    v = _u16(p, 7) / 100.0 if len(p) >= 9 else v_legacy
    return Battery(voltage_v=v, current_a=amps, mah_drawn=mah)


def decode_battery_state(p: bytes) -> Battery:
    if len(p) < 8:
        raise MspError("MSP_BATTERY_STATE payload too short")
    cells = p[0]
    v_legacy = p[3] / 10.0
    mah = _u16(p, 4)
    amps = _i16(p, 6) / 100.0
    v = _u16(p, 9) / 100.0 if len(p) >= 11 else v_legacy
    return Battery(voltage_v=v, current_a=amps, mah_drawn=mah, cells=cells)


# MSP_RC reports rcData in Betaflight's INTERNAL order: roll, pitch, yaw,
# throttle, aux1, aux2 ... (after the `map` has been applied). That is not
# the order of MSP_SET_RAW_RC, which is the map order (AETR = roll, pitch,
# throttle, yaw, aux...). Confirmed against the SITL 2026-09-16.
RC_ECHO_ROLL, RC_ECHO_PITCH, RC_ECHO_YAW, RC_ECHO_THROTTLE, RC_ECHO_AUX1, RC_ECHO_AUX2 = range(6)


def decode_rc(p: bytes) -> list[int]:
    return list(struct.unpack_from(f"<{len(p) // 2}H", p, 0))


def encode_rc(channels: Sequence[int]) -> bytes:
    ch = [int(min(RC_MAX, max(RC_MIN, c))) for c in channels][:RC_CHANNELS]
    return struct.pack(f"<{len(ch)}H", *ch)


# --- the client -----------------------------------------------------------------

@dataclass
class LinkStats:
    requests: int = 0
    responses: int = 0
    timeouts: int = 0
    errors: int = 0
    bad_crc: int = 0
    last_rtt_ms: float = 0.0
    rtt_ms_max: float = 0.0


class FlightController:
    """Synchronous MSP client: one request in flight at a time."""

    def __init__(self, transport, timeout_s: float = 0.25):
        self.t = transport
        self.timeout_s = timeout_s
        self.parser = Parser()
        self.stats = LinkStats()

    def close(self):
        self.t.close()

    def request(self, cmd: int, payload: bytes = b"", timeout_s: Optional[float] = None) -> bytes:
        """Send one command and return the response payload (b"" for acks)."""
        timeout_s = self.timeout_s if timeout_s is None else timeout_s
        self.t.write(encode(cmd, payload))
        self.stats.requests += 1
        t0 = time.perf_counter()
        deadline = t0 + timeout_s
        while True:
            data = self.t.read()
            if data:
                for f in self.parser.feed(data):
                    if f.cmd != cmd:
                        continue                # a stale reply from an earlier timeout
                    rtt = (time.perf_counter() - t0) * 1000.0
                    self.stats.responses += 1
                    self.stats.last_rtt_ms = rtt
                    self.stats.rtt_ms_max = max(self.stats.rtt_ms_max, rtt)
                    self.stats.bad_crc = self.parser.bad_crc
                    if f.error:
                        self.stats.errors += 1
                        raise MspError(f"FC rejected MSP command {cmd}")
                    return f.payload
            if time.perf_counter() > deadline:
                self.stats.timeouts += 1
                raise MspTimeout(f"no reply to MSP {cmd} within {timeout_s * 1000:.0f} ms")
            time.sleep(0.0005)

    # typed helpers -------------------------------------------------------------
    def api_version(self) -> tuple[int, int, int]:
        p = self.request(MSP_API_VERSION)
        return (p[0], p[1], p[2]) if len(p) >= 3 else (0, 0, 0)

    def fc_variant(self) -> str:
        return self.request(MSP_FC_VARIANT).decode("ascii", "replace")

    def fc_version(self) -> tuple[int, int, int]:
        p = self.request(MSP_FC_VERSION)
        return (p[0], p[1], p[2]) if len(p) >= 3 else (0, 0, 0)

    def board_info(self) -> str:
        p = self.request(MSP_BOARD_INFO)
        return p[:4].decode("ascii", "replace") if len(p) >= 4 else ""

    def box_names(self) -> list[str]:
        """Names of the enabled boxes in flightModeFlags bit order (cached)."""
        if getattr(self, "_box_names", None) is None:
            try:
                raw = self.request(MSP_BOXNAMES).decode("ascii", "replace")
                self._box_names = [n for n in raw.split(";") if n]
            except MspError:
                self._box_names = []
        return self._box_names

    def box_ids(self) -> list[int]:
        """Permanent box ids in flightModeFlags bit order (cached). Preferred
        over box_names(): this firmware answers MSP_BOXIDS but not
        MSP_BOXNAMES."""
        if getattr(self, "_box_ids", None) is None:
            try:
                self._box_ids = list(self.request(MSP_BOXIDS))
            except MspError:
                self._box_ids = []
        return self._box_ids

    def status(self) -> Status:
        try:
            st = decode_status(self.request(MSP_STATUS_EX))
        except MspError:
            st = decode_status(self.request(MSP_STATUS))
        ids = self.box_ids()
        if ids:
            st.box_ids = ids
            st.armed = BOX_ID_ARM in st.active_ids
            return st
        names = self.box_names()
        if names:
            st.box_names = names
            st.armed = "ARM" in st.active_modes
        return st

    def attitude(self) -> Attitude:
        return decode_attitude(self.request(MSP_ATTITUDE))

    def raw_imu(self) -> RawImu:
        return decode_raw_imu(self.request(MSP_RAW_IMU))

    def altitude(self) -> Altitude:
        return decode_altitude(self.request(MSP_ALTITUDE))

    def battery(self) -> Battery:
        try:
            return decode_battery_state(self.request(MSP_BATTERY_STATE))
        except MspError:
            return decode_analog(self.request(MSP_ANALOG))

    def rc(self) -> list[int]:
        return decode_rc(self.request(MSP_RC))

    def motors(self) -> list[int]:
        return decode_rc(self.request(MSP_MOTOR))

    def set_raw_rc(self, channels: Sequence[int]) -> None:
        """Send RC channels (map order, default roll, pitch, throttle, yaw,
        aux1..). The FC acks with an empty payload."""
        self.request(MSP_SET_RAW_RC, encode_rc(channels))
