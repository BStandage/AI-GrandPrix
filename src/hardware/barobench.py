"""Is the barometer bad, or are we reading it too slowly? Answer it with no props.

    python3 -m hardware.barobench --port /dev/ttyTHS1 --seconds 30 --label quiet

NOTHING SPINS AND NOTHING IS COMMANDED. This does not open a bridge and never
sends an RC frame: it talks to the FC directly and asks for altitude, as fast
as the link will answer. Props off, drone on the bench, disarmed.

THE QUESTION. d44's height estimate implied 20.8 m/s of vertical motion between
two samples on 2026-09-21. An aircraft that makes 2 g at its throttle clamp
cannot do that, so the estimate was wrong. Two explanations fit:

  A. the barometer spikes under prop wash
  B. we read it at 6.6 Hz (33 Hz link / 5 round-robin pollers in bridge.py)

"It is clean on the bench" does not separate them - with no disturbance above
3.3 Hz there is nothing to fold down, so B predicts a clean bench too.

WHAT FOLD-DOWN ACTUALLY DOES, and why it is worse than noise. Aliasing KEEPS
amplitude and FOLDS frequency. Sampled at 6.6 Hz, a 13 Hz wobble reappears at
0.2 Hz - BELOW the estimator's own 0.48 Hz corner (vert_w = 3.0 rad/s). So it
does not arrive looking like noise the filter can reject. It arrives looking
SLOW, which is exactly what the filter is built to pass through as real motion.
Measured on a synthetic 0.15 m tone, decimating 90 Hz to 6.6 Hz multiplied the
believed motion by 24x at 13 Hz and 34x at 20 Hz. Prop and motor vibration is
broadband and certain to have energy near multiples of 6.6 Hz, and which
harmonic lands there moves with RPM - which is also why the error is
throttle-coupled, exactly as state.py already observed.

WHAT THIS TOOL CAN AND CANNOT TELL YOU, props off:

  CAN     the actual sample rate the link delivers - the one fact the whole
          argument rests on, and it has nothing to do with flying
  CAN     the quiet-bench noise floor
  CAN     how much a disturbance you can actually make (a fan) folds down
  CANNOT  clear the sample rate. The disturbance most likely to be folding
          down in flight is STRUCTURAL VIBRATION, and with no props there is
          none. A fan gives pressure, not vibration.

So a clean run here is not an all-clear. It is a rate measurement plus a floor.
The causal answer comes from a flight with the estimator's own counters logged.
"""

from __future__ import annotations

import argparse
import statistics
import time

NL = "\n"


def _lowpass(ts, zs, w=3.0):
    """The estimator's own vertical corner, vert_w = 3.0 rad/s (0.48 Hz). What
    survives this is what the aircraft BELIEVES is real motion, so this - not
    the raw spread - is the number that matters."""
    out, y = [], zs[0]
    for i, z in enumerate(zs):
        dt = ts[i] - ts[i - 1] if i else 0.0
        y += min(1.0, w * dt) * (z - y)
        out.append(y)
    return out


def _stats(ts, zs):
    gaps = [ts[i] - ts[i - 1] for i in range(1, len(ts))]
    rate = 1.0 / sorted(gaps)[len(gaps) // 2] if gaps else 0.0
    return {"n": len(ts), "hz": rate,
            "sd": statistics.pstdev(zs),
            "pp": max(zs) - min(zs),
            "believed": statistics.pstdev(_lowpass(ts, zs))}


def _decimate(ts, zs, hz):
    """Keep the first sample past each 1/hz tick: what bridge.py's round-robin
    hands the estimator, taken from this very same capture."""
    out_t, out_z, step, nxt = [], [], 1.0 / hz, ts[0]
    for t, z in zip(ts, zs):
        if t >= nxt:
            out_t.append(t)
            out_z.append(z)
            nxt += step
    return out_t, out_z


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--decimate-hz", type=float, default=6.6,
                    help="the rate hardware.runtime actually gets: 33 Hz link / 5 pollers")
    ap.add_argument("--label", default="", help="quiet / fan / etc, for the printout")
    ap.add_argument("--csv", default=None, help="write the raw capture here")
    args = ap.parse_args(argv)

    from hardware import msp

    fc = msp.FlightController(msp.open_transport(args.port, args.tcp, args.baud))
    tag = (" - " + args.label) if args.label else ""
    print(f"polling MSP_ALTITUDE directly for {args.seconds:.0f} s{tag}.")
    print("No RC frames are sent. Props off, disarmed.")
    print()

    ts, zs = [], []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            try:
                a = fc.altitude()
            except msp.MspError:
                continue
            ts.append(time.monotonic() - t0)
            zs.append(float(a.alt_m))
    except KeyboardInterrupt:
        pass
    finally:
        fc.close()

    if len(ts) < 20:
        print(f"only {len(ts)} samples: that is a link problem, not a barometer problem")
        return 1

    fast = _stats(ts, zs)
    dt, dz = _decimate(ts, zs, args.decimate_hz)
    slow = _stats(dt, dz) if len(dt) > 2 else None

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as fh:
            fh.write("t,alt_m" + NL)
            for t, z in zip(ts, zs):
                fh.write(f"{t:.4f},{z:.3f}" + NL)
        print(f"raw capture -> {args.csv}")
        print()

    print(f"{'':24s}{'rate':>9}{'sd raw':>10}{'BELIEVED':>11}")
    print(f"{'as fast as we can':24s}{fast['hz']:8.1f}H{fast['sd']:9.3f}m"
          f"{fast['believed']:10.4f}m")
    if slow:
        label = f"same data @ {args.decimate_hz:.1f} Hz"
        print(f"{label:24s}{slow['hz']:8.1f}H{slow['sd']:9.3f}m{slow['believed']:10.4f}m")
    print()
    print("  BELIEVED is the sd after the estimator's own 0.48 Hz corner: what the")
    print("  aircraft treats as real vertical motion. Raw spread is NOT the number")
    print("  that matters. Aliasing keeps amplitude and folds frequency, so it does")
    print("  not arrive looking noisy - it arrives looking SLOW, and slow is exactly")
    print("  what the filter passes through as real.")
    print()

    if fast["hz"] < args.decimate_hz * 1.5:
        print(f"  The link manages only {fast['hz']:.1f} Hz even polling altitude alone, so this")
        print("  capture says nothing about decimation - the ceiling is the link itself.")
        print("  Look at the baud rate, or at letting Betaflight close the vertical loop")
        print("  on its own sensors at its own rate.")
    elif slow and fast["believed"] > 1e-9:
        ratio = slow["believed"] / fast["believed"]
        print(f"  Decimating to {args.decimate_hz:.1f} Hz multiplied BELIEVED motion by {ratio:.1f}x,")
        print("  on the same seconds of the same sensor.")
        if ratio > 2.0:
            print("  That is fold-down: the sample rate is manufacturing apparent motion.")
        else:
            print("  No fold-down in this capture - but read the caveat.")

    print()
    print("  CAVEAT, and it is a big one: props off there is no STRUCTURAL VIBRATION,")
    print("  which is the disturbance most likely to be folding down in flight. A fan")
    print("  gives pressure, not vibration. A clean result here does NOT clear the")
    print("  sample rate. It gives you the RATE and the quiet floor, and nothing more.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
