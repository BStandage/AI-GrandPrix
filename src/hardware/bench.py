"""
Bench tool for the flight-controller link. Run from src/.

    python -m hardware.bench --tcp 172.20.0.2:5761 info          # the sim's SITL
    python -m hardware.bench --port /dev/ttyTHS1 info            # the Archer
    python -m hardware.bench --port /dev/ttyTHS1 telemetry --hz 20 --seconds 10
    python -m hardware.bench --port /dev/ttyTHS1 rc-test --props-off
    python -m hardware.bench --port /dev/ttyTHS1 arm-test --props-off

info       identity, API version, sensors, armed state, arming blockers, battery
telemetry  attitude / gyro / altitude / battery at --hz, plus link rates
rc-test    streams DISARMED sticks through the bridge and reads MSP_RC back:
           proves the RC path end to end without ever arming. Then walks
           the roll stick +-200 and shows the echo.
arm-test   arms for --seconds with throttle at minimum, reports the ARMED
           flag and the arming blockers, disarms. Requires --props-off.

Nothing here raises the throttle. `hover` is deliberately absent until
the altitude source and the ANGLE-mode output exist.
"""

from __future__ import annotations

import argparse
import sys
import time

from hardware import msp
from hardware.bridge import FcBridge


def _open(args):
    return FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud, rc_hz=args.rc_hz)


def cmd_info(args) -> int:
    fc = msp.FlightController(msp.open_transport(args.port, args.tcp, args.baud))
    try:
        api = fc.api_version()
        print(f"link        {fc.t.name}")
        print(f"MSP API     {api[1]}.{api[2]}  (protocol {api[0]})")
        print(f"firmware    {fc.fc_variant()} {'.'.join(map(str, fc.fc_version()))}  board {fc.board_info()}")
        st = fc.status()
        print(f"cycle time  {st.cycle_time_us} us   cpu {st.cpu_load}%   i2c errors {st.i2c_errors}")
        print(f"sensors     0x{st.sensors:04x}   armed {st.armed}   modes {', '.join(st.active_modes) or 'none'}")
        print(f"arm blocks  {', '.join(st.arming_blockers) or 'none'}"
              + ("" if st.arming_disable_flags is not None else "  (flags not in this MSP_STATUS)"))
        try:
            b = fc.battery()
            print(f"battery     {b.voltage_v:.2f} V  {b.current_a:.2f} A  {b.mah_drawn} mAh"
                  + (f"  {b.cells}S" if b.cells else ""))
        except msp.MspError as e:
            print(f"battery     n/a ({e})")
        a = fc.attitude()
        print(f"attitude    roll {a.roll_deg:+.1f}  pitch {a.pitch_deg:+.1f}  yaw {a.yaw_deg:.0f}")
        print(f"rtt         {fc.stats.last_rtt_ms:.1f} ms")
        return 0
    finally:
        fc.close()


def cmd_telemetry(args) -> int:
    br = _open(args)
    br.start()
    t_end = time.monotonic() + args.seconds
    period = 1.0 / args.hz
    try:
        print(f"{'t':>6} {'roll':>7} {'pitch':>7} {'yaw':>6} {'gx':>6} {'gy':>6} {'gz':>6} "
              f"{'alt':>6} {'vbat':>5} {'att_hz':>6} {'rc_hz':>5} {'rtt':>5} {'to':>3}")
        t0 = time.monotonic()
        while time.monotonic() < t_end:
            s = br.state()
            a = s.attitude
            g = s.imu.gyro if s.imu else (0, 0, 0)
            alt = s.altitude.alt_m if s.altitude else float("nan")
            v = s.battery.voltage_v if s.battery and s.battery.voltage_v else float("nan")
            if a:
                print(f"{time.monotonic() - t0:6.2f} {a.roll_deg:+7.1f} {a.pitch_deg:+7.1f} {a.yaw_deg:6.0f} "
                      f"{g[0]:6d} {g[1]:6d} {g[2]:6d} {alt:6.2f} {v:5.2f} {s.attitude_hz:6.1f} "
                      f"{s.rc_hz:5.1f} {s.link.last_rtt_ms:5.1f} {s.link.timeouts:3d}")
            time.sleep(period)
        return 0
    finally:
        br.stop()


def _require_props_off(args) -> None:
    if not args.props_off:
        print("refusing: this command drives the RC channels. Take the props off and pass --props-off.")
        sys.exit(2)


def cmd_rc_test(args) -> int:
    _require_props_off(args)
    br = _open(args)
    br.start()
    try:
        print("streaming DISARMED neutral sticks for 2 s ...")
        for _ in range(int(2 * args.rc_hz)):
            br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000)
            time.sleep(1.0 / args.rc_hz)
        s = br.state()
        print(f"  FC echo (MSP_RC): {s.rc_echo[:8]}   rc {s.rc_hz:.0f} Hz  attitude {s.attitude_hz:.0f} Hz  "
              f"timeouts {s.link.timeouts}")
        # MSP_RC echo is roll, pitch, yaw, throttle, aux1, aux2 (FC internal order)
        ok = (len(s.rc_echo) >= 6 and abs(s.rc_echo[msp.RC_ECHO_ROLL] - 1500) < 30
              and s.rc_echo[msp.RC_ECHO_THROTTLE] < 1050 and s.rc_echo[msp.RC_ECHO_AUX1] < 1100)
        print("  echo matches:", "YES" if ok else "NO - check `map`, msp_override_channels_mask and the receiver type in diff all")
        for roll in (1300, 1700, 1500):
            for _ in range(int(0.5 * args.rc_hz)):
                br.set_rc(throttle=1000, roll=roll, arm=1000)
                time.sleep(1.0 / args.rc_hz)
            e = br.state().rc_echo
            print(f"  roll stick {roll} -> FC sees roll {e[0] if e else '?'}")
        print("stale-command rule check: not calling set_rc for 1 s ...")
        time.sleep(1.0)
        s = br.state()
        print(f"  bridge forced disarm channels: {s.stale_disarm}  FC echo throttle {s.rc_echo[msp.RC_ECHO_THROTTLE] if len(s.rc_echo) > 3 else '?'}")
        return 0 if ok else 1
    finally:
        br.stop()


def cmd_arm_test(args) -> int:
    _require_props_off(args)
    br = _open(args)
    br.start()
    try:
        for _ in range(int(1.0 * args.rc_hz)):
            br.set_rc(throttle=1000, arm=1000)
            time.sleep(1.0 / args.rc_hz)
        st = br.state().status
        print(f"before: armed {st.armed if st else '?'}  blockers {', '.join(st.arming_blockers) if st else '?'}")
        print(f"arming for {args.seconds:.0f} s, throttle at minimum ...")
        t_end = time.monotonic() + args.seconds
        armed_seen = False
        while time.monotonic() < t_end:
            br.set_rc(throttle=1000, arm=1800)
            st = br.state().status
            if st and st.armed:
                armed_seen = True
            time.sleep(1.0 / args.rc_hz)
        st = br.state().status
        print(f"during: armed seen {armed_seen}   now armed {st.armed if st else '?'}   "
              f"blockers {', '.join(st.arming_blockers) if st else '?'}")
        for _ in range(int(1.0 * args.rc_hz)):
            br.set_rc(throttle=1000, arm=1000)
            time.sleep(1.0 / args.rc_hz)
        st = br.state().status
        print(f"after:  armed {st.armed if st else '?'}")
        return 0 if armed_seen else 1
    finally:
        br.stop()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="serial device, e.g. /dev/ttyTHS1")
    ap.add_argument("--tcp", default=None, help="host[:port] of a Betaflight SITL (default port 5761)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rc-hz", type=float, default=50.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    t = sub.add_parser("telemetry")
    t.add_argument("--hz", type=float, default=10.0)
    t.add_argument("--seconds", type=float, default=10.0)
    r = sub.add_parser("rc-test")
    r.add_argument("--props-off", action="store_true")
    a = sub.add_parser("arm-test")
    a.add_argument("--props-off", action="store_true")
    a.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args(argv)
    try:
        return {"info": cmd_info, "telemetry": cmd_telemetry,
                "rc-test": cmd_rc_test, "arm-test": cmd_arm_test}[args.cmd](args)
    except msp.MspError as e:
        print(f"ERROR {e}")
        return 3


if __name__ == "__main__":
    sys.exit(main())
