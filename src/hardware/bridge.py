"""
The flight-controller bridge: RC out, telemetry in, one background thread.

    from hardware.bridge import FcBridge
    br = FcBridge.open(port="/dev/ttyTHS1")        # or tcp="172.x.x.x:5761"
    br.start()
    br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000)
    s = br.state()          # latest attitude / imu / altitude / battery / link
    br.stop()               # sends disarm frames, then closes the port

Rules the thread enforces, because the link is the only thing between
our code and the motors:

- Every cycle sends the CURRENT channels with MSP_SET_RAW_RC at `rc_hz`
  and polls MSP_ATTITUDE. Slower items (IMU, altitude, battery, status)
  are round-robined so the RC stream never waits on them.
- STALE COMMAND -> DISARM. If nobody has called set_rc() for `stale_s`,
  the channels are forced to throttle 1000 / arm 1000 and stay there
  until a fresh command arrives. A hung caller cannot leave the drone
  armed at the last throttle it heard.
- stop() sends the disarm frame `disarm_frames` times before closing.
- Link failures (timeouts) are counted and exposed; after
  `max_consecutive_timeouts` the bridge marks itself unhealthy, forces
  the disarm channels, and discards the last command: when the link
  returns it keeps sending the safe channels until set_rc() is called
  again. An outage never re-arms by itself.

The channel order is the FC's `map` (default AETR1234 = roll, pitch,
throttle, yaw, aux1 = arm, aux2 ...), identical to the sim bridge.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from typing import Optional

from hardware import msp

AUX_ARM = 4     # channel index of the arm switch (aux1), matches the sim's RCCommand


@dataclass
class FcState:
    t: float = 0.0                        # monotonic time of the last attitude sample
    attitude: Optional[msp.Attitude] = None
    imu: Optional[msp.RawImu] = None
    altitude: Optional[msp.Altitude] = None
    battery: Optional[msp.Battery] = None
    status: Optional[msp.Status] = None
    rc_echo: list = field(default_factory=list)   # MSP_RC as the FC sees it
    attitude_hz: float = 0.0
    rc_hz: float = 0.0
    healthy: bool = False
    stale_disarm: bool = False
    consecutive_timeouts: int = 0
    link: msp.LinkStats = field(default_factory=msp.LinkStats)


class FcBridge:
    def __init__(self, fc: msp.FlightController, rc_hz: float = 50.0, stale_s: float = 0.5,
                 max_consecutive_timeouts: int = 10, disarm_frames: int = 10):
        self.fc = fc
        self.rc_hz = rc_hz
        self.stale_s = stale_s
        self.max_consecutive_timeouts = max_consecutive_timeouts
        self.disarm_frames = disarm_frames
        self._lock = threading.Lock()
        self._channels = self.safe_channels()
        self._last_cmd_t = 0.0                  # never commanded -> stale from the start
        self._state = FcState()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    @classmethod
    def open(cls, port: Optional[str] = None, tcp: Optional[str] = None,
             baud: int = 115200, **kw) -> "FcBridge":
        return cls(msp.FlightController(msp.open_transport(port, tcp, baud)), **kw)

    @staticmethod
    def safe_channels() -> list[int]:
        ch = [msp.RC_CENTER] * msp.RC_CHANNELS
        ch[2] = msp.RC_MIN            # throttle
        ch[AUX_ARM] = msp.RC_MIN      # disarmed
        return ch

    # --- caller API ------------------------------------------------------------
    def set_rc(self, throttle: int, roll: int = 1500, pitch: int = 1500, yaw: int = 1500,
               arm: int = 1000, aux2: int = 1500, aux3: int = 1500, aux4: int = 1500) -> None:
        """Same fields as the sim's RCCommand."""
        with self._lock:
            self._channels[0] = int(roll)
            self._channels[1] = int(pitch)
            self._channels[2] = int(throttle)
            self._channels[3] = int(yaw)
            self._channels[4] = int(arm)
            self._channels[5] = int(aux2)
            self._channels[6] = int(aux3)
            self._channels[7] = int(aux4)
            self._last_cmd_t = time.monotonic()

    def set_channels(self, channels) -> None:
        with self._lock:
            for i, c in enumerate(channels[:msp.RC_CHANNELS]):
                self._channels[i] = int(c)
            self._last_cmd_t = time.monotonic()

    def state(self) -> FcState:
        with self._lock:
            return replace(self._state)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="fc-bridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._send_disarm()
        self.fc.close()

    # --- the loop --------------------------------------------------------------
    def _send_disarm(self) -> None:
        for _ in range(self.disarm_frames):
            try:
                self.fc.set_raw_rc(self.safe_channels())
            except msp.MspError:
                pass
            time.sleep(1.0 / self.rc_hz)

    def _run(self) -> None:
        period = 1.0 / self.rc_hz
        slow = (self._poll_imu, self._poll_altitude, self._poll_battery, self._poll_status, self._poll_rc)
        k = 0
        att_count, rc_count, win_t0 = 0, 0, time.monotonic()
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                stale = (now - self._last_cmd_t) > self.stale_s
                unhealthy = self._state.consecutive_timeouts >= self.max_consecutive_timeouts
                ch = self.safe_channels() if (stale or unhealthy) else list(self._channels)
                self._state.stale_disarm = stale
            try:
                self.fc.set_raw_rc(ch)
                rc_count += 1
                att = self.fc.attitude()
                att_count += 1
                with self._lock:
                    self._state.attitude = att
                    self._state.t = time.monotonic()
                    self._state.consecutive_timeouts = 0
                    self._state.healthy = True
                slow[k % len(slow)]()
                k += 1
            except msp.MspTimeout:
                with self._lock:
                    self._state.consecutive_timeouts += 1
                    if self._state.consecutive_timeouts >= self.max_consecutive_timeouts:
                        self._state.healthy = False
                        # an outage invalidates the last command: stay on the
                        # safe channels until the caller commands again
                        self._last_cmd_t = 0.0
            except msp.MspError:
                pass
            with self._lock:
                self._state.link = replace(self.fc.stats)
                if now - win_t0 >= 1.0:
                    self._state.attitude_hz = att_count / (now - win_t0)
                    self._state.rc_hz = rc_count / (now - win_t0)
                    att_count, rc_count, win_t0 = 0, 0, now
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _poll_imu(self):
        v = self.fc.raw_imu()
        with self._lock:
            self._state.imu = v

    def _poll_altitude(self):
        try:
            v = self.fc.altitude()
        except msp.MspError:
            return
        with self._lock:
            self._state.altitude = v

    def _poll_battery(self):
        try:
            v = self.fc.battery()
        except msp.MspError:
            return
        with self._lock:
            self._state.battery = v

    def _poll_status(self):
        v = self.fc.status()
        with self._lock:
            self._state.status = v

    def _poll_rc(self):
        v = self.fc.rc()
        with self._lock:
            self._state.rc_echo = v
