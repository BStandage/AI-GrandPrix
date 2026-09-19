"""MSP framing, decoding and the bridge's safety rules, against a fake FC."""
import struct
import time
import unittest

import _paths  # noqa: F401

from hardware import msp
from hardware.bridge import FcBridge


def response(cmd, payload=b"", error=False):
    crc = len(payload) ^ cmd
    for b in payload:
        crc ^= b
    return b"$M" + (b"!" if error else b">") + bytes([len(payload), cmd]) + payload + bytes([crc])


class FakeFc:
    """Transport that answers like a Betaflight: parses requests, replies."""

    def __init__(self):
        self.name = "fake"
        self.rx = bytearray()
        self.tx = bytearray()
        self.parser = msp.Parser()
        self.rc = [1500] * 16
        self.armed = False
        self.set_rc_count = 0
        self.drop_next = 0

    def write(self, data):
        for f in self._parse_requests(data):
            self._handle(f)

    def _parse_requests(self, data):
        # requests are '$M<' frames; reuse the parser by flipping direction
        return self.parser.feed(data.replace(b"$M<", b"$M>"))

    def _handle(self, f):
        if self.drop_next:
            self.drop_next -= 1
            return
        if f.cmd == msp.MSP_SET_RAW_RC:
            self.rc = msp.decode_rc(f.payload) + self.rc[len(f.payload) // 2:]
            self.armed = self.rc[4] >= 1700 and self.rc[2] < 1050 if not self.armed else self.rc[4] >= 1700
            self.set_rc_count += 1
            self.tx += response(f.cmd)
        elif f.cmd == msp.MSP_ATTITUDE:
            self.tx += response(f.cmd, struct.pack("<hhh", -123, 456, 270))
        elif f.cmd == msp.MSP_RAW_IMU:
            self.tx += response(f.cmd, struct.pack("<9h", 0, 0, 512, 1, -2, 3, 0, 0, 0))
        elif f.cmd == msp.MSP_ALTITUDE:
            self.tx += response(f.cmd, struct.pack("<ih", 135, -20))
        elif f.cmd == msp.MSP_STATUS_EX:
            flags = 1 if self.armed else 0
            adf = (1 << 9) | (1 << 25)      # BOOTGRACE | ARMSWITCH
            p = struct.pack("<HHHIBHBB", 125, 0, 0x03, flags, 0, 7, 3, 0) + bytes([0]) + bytes([26]) + struct.pack("<I", adf) + bytes([0])
            self.tx += response(f.cmd, p)
        elif f.cmd == msp.MSP_RC:
            self.tx += response(f.cmd, struct.pack("<16H", *self.rc))
        elif f.cmd == msp.MSP_BATTERY_STATE:
            self.tx += response(f.cmd, bytes([4]) + struct.pack("<H", 1300) + bytes([165]) + struct.pack("<Hh", 40, 250) + bytes([1]) + struct.pack("<H", 1652))
        elif f.cmd == msp.MSP_API_VERSION:
            self.tx += response(f.cmd, bytes([0, 1, 46]))
        elif f.cmd == msp.MSP_FC_VARIANT:
            self.tx += response(f.cmd, b"BTFL")
        elif f.cmd == msp.MSP_FC_VERSION:
            self.tx += response(f.cmd, bytes([4, 5, 5]))
        else:
            self.tx += response(f.cmd, error=True)

    def read(self, n=4096):
        out = bytes(self.tx[:n]); del self.tx[:n]; return out

    def close(self):
        pass


class TestFraming(unittest.TestCase):
    def test_encode_matches_reference_frame(self):
        # MSP_ATTITUDE request, no payload: $M< 0 108 crc(0^108)
        self.assertEqual(msp.encode(108), b"$M<" + bytes([0, 108, 108]))

    def test_parser_roundtrip_and_resync(self):
        p = msp.Parser()
        junk = b"\x00garbage$M"
        frames = p.feed(junk + response(108, b"\x01\x02\x03\x04\x05\x06") + b"$M>")
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].cmd, 108)
        self.assertEqual(frames[0].payload, b"\x01\x02\x03\x04\x05\x06")

    def test_bad_crc_is_dropped_and_counted(self):
        p = msp.Parser()
        bad = bytearray(response(108, b"\x01\x02\x03\x04\x05\x06")); bad[-1] ^= 0xFF
        self.assertEqual(p.feed(bytes(bad)), [])
        self.assertEqual(p.bad_crc, 1)

    def test_split_delivery(self):
        p = msp.Parser()
        r = response(105, struct.pack("<8H", *range(1000, 1008)))
        self.assertEqual(p.feed(r[:4]), [])
        self.assertEqual(p.feed(r[4:9]), [])
        f = p.feed(r[9:])
        self.assertEqual(len(f), 1)
        self.assertEqual(msp.decode_rc(f[0].payload), list(range(1000, 1008)))

    def test_rc_encode_clamps(self):
        self.assertEqual(msp.decode_rc(msp.encode_rc([500, 2500, 1500])), [1000, 2000, 1500])


class TestDecoders(unittest.TestCase):
    def test_attitude(self):
        a = msp.decode_attitude(struct.pack("<hhh", -123, 456, 270))
        self.assertAlmostEqual(a.roll_deg, -12.3); self.assertAlmostEqual(a.pitch_deg, 45.6); self.assertEqual(a.yaw_deg, 270)

    def test_status_ex_arming_flags(self):
        fake = FakeFc(); fake._handle(msp.Frame(msp.MSP_STATUS_EX, b""))
        st = msp.decode_status(msp.Parser().feed(bytes(fake.tx))[0].payload)
        self.assertFalse(st.armed)
        self.assertEqual(st.arming_blockers, ["BOOTGRACE", "ARMSWITCH"])
        self.assertEqual(st.cycle_time_us, 125)

    def test_status_short_payload_has_no_flags(self):
        st = msp.decode_status(struct.pack("<HHHIB", 125, 0, 3, 0, 0))
        self.assertIsNone(st.arming_disable_flags)
        self.assertEqual(st.arming_blockers, [])

    def test_battery_state(self):
        fake = FakeFc(); fake._handle(msp.Frame(msp.MSP_BATTERY_STATE, b""))
        b = msp.decode_battery_state(msp.Parser().feed(bytes(fake.tx))[0].payload)
        self.assertEqual(b.cells, 4); self.assertAlmostEqual(b.voltage_v, 16.52); self.assertAlmostEqual(b.current_a, 2.5)

    def test_altitude(self):
        a = msp.decode_altitude(struct.pack("<ih", 135, -20))
        self.assertAlmostEqual(a.alt_m, 1.35); self.assertAlmostEqual(a.vario_mps, -0.2)


class TestClient(unittest.TestCase):
    def test_request_response_and_stats(self):
        fc = msp.FlightController(FakeFc(), timeout_s=0.2)
        self.assertEqual(fc.fc_variant(), "BTFL")
        self.assertEqual(fc.fc_version(), (4, 5, 5))
        self.assertEqual(fc.api_version()[1:], (1, 46))
        fc.set_raw_rc([1500, 1500, 1000, 1500, 1000])
        self.assertEqual(fc.rc()[:5], [1500, 1500, 1000, 1500, 1000])
        self.assertEqual(fc.stats.responses, 5)

    def test_timeout_and_stale_reply_ignored(self):
        fake = FakeFc(); fc = msp.FlightController(fake, timeout_s=0.05)
        fake.drop_next = 1
        with self.assertRaises(msp.MspTimeout):
            fc.attitude()
        self.assertEqual(fc.stats.timeouts, 1)
        self.assertEqual(fc.attitude().yaw_deg, 270)     # link recovers

    def test_error_frame(self):
        fc = msp.FlightController(FakeFc(), timeout_s=0.05)
        with self.assertRaises(msp.MspError):
            fc.request(250)


class TestBridge(unittest.TestCase):
    def test_stale_command_forces_disarm(self):
        fake = FakeFc()
        br = FcBridge(msp.FlightController(fake, timeout_s=0.05), rc_hz=200.0, stale_s=0.1)
        br.start()
        try:
            for _ in range(20):
                br.set_rc(throttle=1600, arm=1800); time.sleep(0.01)
            self.assertEqual(fake.rc[2], 1600); self.assertEqual(fake.rc[4], 1800)
            time.sleep(0.3)                                  # nobody commands
            s = br.state()
            self.assertTrue(s.stale_disarm)
            self.assertEqual(fake.rc[2], 1000); self.assertEqual(fake.rc[4], 1000)
            self.assertTrue(s.healthy); self.assertIsNotNone(s.attitude)
        finally:
            br.stop()
        self.assertEqual(fake.rc[4], 1000)               # stop() sent disarm frames

    def test_unhealthy_link_forces_disarm(self):
        fake = FakeFc()
        br = FcBridge(msp.FlightController(fake, timeout_s=0.02), rc_hz=200.0, stale_s=5.0,
                      max_consecutive_timeouts=3)
        br.start()
        try:
            br.set_rc(throttle=1600, arm=1800); time.sleep(0.05)
            fake.drop_next = 50
            time.sleep(0.4)
            self.assertFalse(br.state().healthy)
            fake.drop_next = 0
            time.sleep(0.1)
            self.assertTrue(br.state().healthy)
            self.assertEqual(fake.rc[4], 1000)           # link is back, still safe: the old command is void
            br.set_rc(throttle=1600, arm=1800); time.sleep(0.05)
            self.assertEqual(fake.rc[4], 1800)           # a fresh command re-enables
        finally:
            br.stop()


if __name__ == "__main__":
    unittest.main()
