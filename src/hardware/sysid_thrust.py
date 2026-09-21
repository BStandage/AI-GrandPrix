"""Throttle sysid on the real aircraft: measure the PWM -> thrust curve.

    python3 -m hardware.sysid_thrust --port /dev/ttyTHS1 --dry-run
    python3 -m hardware.sysid_thrust --port /dev/ttyTHS1 --arm

WHY. The curve we fly came from the organizers' blackbox - THEIR aircraft's
specific thrust - and was corrected on 2026-09-21 by -60 PWM from two points
in a crash log. It is right at hover: 1228 predicted, 1227 measured, four
times. It is NOT right above hover. Sally's clean flight implies about
11.9 m/s^2 at 1250 where the curve says 10.95, and the whole -60 shift rests
on two samples from an aircraft that was hitting a net.

Everything vertical is built on this table. AltitudeLoop inverts it to turn a
thrust demand into a PWM, so a wrong curve means every altitude command is
wrong by a factor nobody can see.

HOW. Hover, then short THROTTLE PULSES away from hover, one at a time,
returning to hover between each. The aircraft's own accelerometer measures the
acceleration each pulse produces - which IS specific thrust minus g, directly,
with no barometer anywhere in it. A pulse costs about a quarter of a metre.

    python3 -m hardware.sysid_thrust --analyze out/sysid/hw_thrust_000.csv

writes a curve_pwm/curve_acc pair straight into the format the tomls use.

SAFETY, because the last week earned it:
  * throttle NEVER leaves [hover - band, hover + band]. The pulses are the
    experiment; they are not permission to go to full throttle.
  * vertical speed over --vz-max for 0.3 s aborts the whole run
  * each pulse is --pulse-s long and followed by a recovery back to hover
  * the barometer is not in any loop here. It is recorded, not believed.
  * MSP OVERRIDE off is the abort, as always
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

G = 9.80665


def analyze(csv_path):
    """Turn a pulse log into curve_pwm / curve_acc."""
    import numpy as np
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    # PULSES ONLY. The recovery segments sit at hover PWM but are actively
    # controlled, so their slope is what the velocity hold was doing, not what
    # the motors produce open-loop. Including them puts a fake point right at
    # hover - the one value we most need to be honest.
    if "phase" in (d.dtype.names or ()):
        keep = d["phase"] == "pulse"
        if keep.any():
            d = d[keep]
        else:
            print("no rows tagged 'pulse' - analysing everything")
    t, vz, pwm = d["t"], d["vz"], d["pwm_cmd"]
    print(f"{'pwm':>5s} {'n':>4s} {'a_measured':>11s} {'thrust':>9s} {'g':>6s}")
    rows = []
    for p in sorted(set(int(x) for x in pwm)):
        m = pwm == p
        idx = np.flatnonzero(m)
        # contiguous runs only: each pulse is its own measurement
        splits = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        accs = []
        for run in splits:
            if len(run) < 4:
                continue
            # slope of vz over the pulse IS the specific acceleration
            a = float(np.polyfit(t[run], vz[run], 1)[0])
            accs.append(a)
        if not accs:
            continue
        a_med = float(np.median(accs))
        rows.append((p, len(accs), a_med, a_med + G))
        print(f"{p:5d} {len(accs):4d} {a_med:+11.2f} {a_med + G:9.2f} "
              f"{(a_med + G) / G:6.2f}")
    if not rows:
        print("no usable pulses in that file")
        return 1
    print("\nmeasured on THIS aircraft, paste into the [thrust] section:")
    print(f"curve_pwm = {[r[0] for r in rows]}")
    print(f"curve_acc = {[round(r[3], 2) for r in rows]}")
    hov = [r for r in rows if r[3] >= G]
    if hov and len(rows) > 1:
        import numpy as _n
        hp = float(_n.interp(G, [r[3] for r in rows], [r[0] for r in rows]))
        print(f"hover_pwm = {hp:.0f}")
    print("\nCompare with the curve currently flown before replacing it. A "
          "point that disagrees by more than about 1 m/s^2 is the one to "
          "explain, not to paste over.")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--analyze", default=None, help="offline: read a pulse log")
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--config", default=None)
    ap.add_argument("--pulses", type=int, nargs="*", default=None,
                    help="PWM offsets from hover to test. Default: a spread "
                         "inside --band, both directions.")
    ap.add_argument("--band", type=int, default=80,
                    help="hard limit either side of hover_pwm. The pulses are "
                         "the experiment, not permission to go to full throttle.")
    ap.add_argument("--pulse-s", type=float, default=0.40)
    ap.add_argument("--recover-s", type=float, default=1.20,
                    help="hover between pulses, to get the height back")
    ap.add_argument("--settle-s", type=float, default=3.0,
                    help="velocity hold before the first pulse")
    ap.add_argument("--vz-max", type=float, default=2.0,
                    help="abort if vertical speed exceeds this for 0.3 s")
    ap.add_argument("--vz-gain", type=float, default=2.5)
    ap.add_argument("--takeoff-pwm", type=int, default=1250)
    ap.add_argument("--climb-s", type=float, default=0.8)
    ap.add_argument("--rc-hz", type=float, default=50.0)
    ap.add_argument("--acc-lsb-per-g", default="auto")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--arm", action="store_true")
    args = ap.parse_args(argv)

    if args.analyze:
        return analyze(args.analyze)
    if not (args.dry_run or args.arm):
        raise SystemExit("pass --dry-run or --arm")

    import numpy as np

    from hardware.bridge import FcBridge
    from hardware.state import FcStateSource
    from raceline.config import AIGP_REPO, load_config

    cfg = load_config(args.config) if args.config else load_config(None)
    hover = int(cfg.thrust.hover_pwm)
    lo, hi = hover - args.band, hover + args.band
    pulses = args.pulses if args.pulses else [-60, -40, -20, 20, 40, 60, 80]
    pulses = [p for p in pulses if lo <= hover + p <= hi]

    br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
    br.start()
    src = FcStateSource(br)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        s = br.state()
        if s.attitude is not None and s.altitude is not None:
            break
        time.sleep(0.05)
    if br.state().attitude is None:
        print("ERROR no attitude from the FC"); br.stop(); return 2

    if args.acc_lsb_per_g == "auto":
        mags, t0c = [], time.monotonic()
        while time.monotonic() - t0c < 1.0:
            st = br.state()
            if st.imu is not None:
                mags.append(math.sqrt(sum(float(v) ** 2 for v in st.imu.acc)))
            time.sleep(0.02)
        if not mags:
            print("ERROR no MSP_RAW_IMU"); br.stop(); return 3
        med, spread = float(np.median(mags)), max(mags) - min(mags)
        if spread > 0.1 * med:
            print(f"ERROR accelerometer not at rest (spread {spread:.0f}). "
                  f"Put it down still."); br.stop(); return 4
        src.acc_lsb_per_g = med
        print(f"accelerometer: {med:.0f} counts per g")
    else:
        src.acc_lsb_per_g = float(args.acc_lsb_per_g)

    print(f"hover {hover} PWM, throttle bounded to {lo}..{hi}")
    print(f"pulses at hover{'' if not pulses else ' ' + ', '.join(f'{p:+d}' for p in pulses)}"
          f"  ({args.pulse_s:.2f} s each, {args.recover_s:.2f} s recovery)")
    a_lo = float(np.interp(lo, cfg.thrust.curve_pwm, cfg.thrust.curve_acc))
    a_hi = float(np.interp(hi, cfg.thrust.curve_pwm, cfg.thrust.curve_acc))
    print(f"which the CURRENT curve says is {a_lo / G:.2f}..{a_hi / G:.2f} g - "
          f"this flight finds out whether that is true")

    out = AIGP_REPO / "out" / "sysid"
    out.mkdir(parents=True, exist_ok=True)
    n = 0
    while (out / f"hw_thrust_{n:03d}.csv").exists():
        n += 1
    path = out / f"hw_thrust_{n:03d}.csv"
    f = open(path, "w", encoding="utf-8")
    f.write("t,phase,pwm_cmd,z,vz,az_world,vbat,amps\n")
    print(f"log -> {path}")

    if args.dry_run:
        print("DRY RUN: the FC stays disarmed and gets neutral sticks")
    else:
        print("LIVE: waiting for the pilot. Throttle low, ARM, then MSP OVERRIDE on.")
        while True:
            st = br.state().status
            br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)
            if st is not None and st.armed and st.msp_override:
                src.zero_altitude()
                print("armed, MSP OVERRIDE on: altitude re-zeroed, lifting off")
                break
            time.sleep(0.05)

    period = 1.0 / args.rc_hz
    t0 = time.monotonic()
    airborne = False
    vz_bad = 0.0
    # phases: climb -> settle -> (pulse, recover) * N -> land
    seq, tphase, ip = "climb", t0, 0
    thr = args.takeoff_pwm
    try:
        while True:
            t = time.monotonic()
            est = src.estimate()
            if est is None:
                time.sleep(period); continue
            vz = float(est.v[2])
            az_w = (float((est.R @ src.accel_body)[2]) - G
                    if src.accel_body is not None else 0.0)
            if not airborne and vz > 0.5:
                airborne = True

            if seq == "climb":
                thr = args.takeoff_pwm
                if t - t0 >= args.climb_s:
                    seq, tphase = "settle", t
                    print(f"  climb done (vz={vz:+.2f}), settling")
            elif seq == "settle":
                thr = _hold(cfg, args, vz, 0.0)
                if t - tphase >= args.settle_s:
                    seq, tphase = "pulse", t
            elif seq == "pulse":
                thr = hover + pulses[ip]
                if t - tphase >= args.pulse_s:
                    seq, tphase = "recover", t
            elif seq == "recover":
                thr = _hold(cfg, args, vz, 0.0)
                if t - tphase >= args.recover_s:
                    ip += 1
                    if ip >= len(pulses):
                        seq, tphase = "land", t
                        print("  all pulses done, landing")
                    else:
                        seq, tphase = "pulse", t
            else:  # land
                thr = _hold(cfg, args, vz, -0.4)
                if t - tphase > 4.0:
                    break

            thr = int(max(lo, min(hi, thr)))
            done = seq == "land" and t - tphase > 4.0
            out_rc = dict(throttle=(1000 if done else thr), roll=1500, pitch=1500,
                          yaw=1500, arm=(1000 if done else 1800), aux2=1500)
            src.note_throttle(out_rc["throttle"], t)
            if args.arm:
                br.set_rc(**out_rc)
            else:
                br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500,
                          arm=1000, aux2=1500)

            st = br.state()
            f.write(f"{t - t0:.3f},{seq},{out_rc['throttle']},"
                    f"{float(est.p[2]):.3f},{vz:.3f},{az_w:.3f},"
                    f"{st.battery.voltage_v if st.battery else ''},"
                    f"{st.battery.current_a if st.battery else ''}\n")

            if airborne and abs(vz) > args.vz_max:
                vz_bad += period
                if vz_bad > 0.3:
                    print(f"\n  VERTICAL SPEED {vz:+.1f} m/s for 0.3 s "
                          f"(limit {args.vz_max:.1f}). Aborting, PILOT TAKE OVER.")
                    br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500,
                              arm=1000, aux2=1500)
                    break
            else:
                vz_bad = 0.0
            if args.arm and st.status is not None and not st.status.msp_override:
                print("pilot took MSP OVERRIDE off: stopping"); break
            if seq == "pulse" and int((t - tphase) / period) == 0:
                print(f"  pulse {ip + 1}/{len(pulses)}: {hover + pulses[ip]} PWM "
                      f"(hover{pulses[ip]:+d})")
            time.sleep(max(0.0, period - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for _ in range(10):
            br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500,
                      arm=1000, aux2=1500)
            time.sleep(0.02)
        br.stop()
        f.close()

    print(f"\n  {path}")
    print(f"  python3 -m hardware.sysid_thrust --analyze {path}")
    return 0


def _hold(cfg, args, vz, vz_target):
    """Velocity hold, the only mode that has flown cleanly. No barometer."""
    a_cmd = max(-4.0, min(4.0, args.vz_gain * (vz_target - vz)))
    return int(round(cfg.pwm_for_thrust(G + a_cmd)))


if __name__ == "__main__":
    raise SystemExit(main())
