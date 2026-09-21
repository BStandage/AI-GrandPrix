"""Vertical-channel simulator: fly the REAL altitude code against a barometer
that behaves like ours.

    python3 -m hardware.vertsim --mode baro     --alt 0.6
    python3 -m hardware.vertsim --mode no-baro  --climb-s 0.6
    python3 -m hardware.vertsim --sweep

WHY THIS EXISTS. Five attempts at commanded-altitude hover on hardware, two
broken airframes, zero successes. Two attempts at velocity hold, two clean
flights. The difference is the barometer, and it cannot be studied on a bench
because it only misbehaves with props turning. Every question about the
vertical channel has therefore cost a flight slot, and there are three slots
left.

This runs `raceline.rc_backend.AltitudeLoop` and `seeker.dr_estimator.
VerticalFilter` - the actual flight code, not a model of it - against a plant
and a sensor fitted to the real logs. Being wrong here is free.

THE BAROMETER MODEL, and every number in it was measured:

  quantisation   0.076 m     1 Pa steps (d45, table test)
  drift          0.25 m/min  at rest
  white noise    0.02 m      what is left on a bench
  THROTTLE-COUPLED SPIKES                 <- the one that matters

That last one is the whole point. The sensor is clean when the throttle is
steady and violent when it moves: d44 logged a clean trace under --no-baro at
a constant ~1227 PWM, then spiked to 1.76 m minutes later with the altitude
loop chasing. d45 read -3.86 m mid-launch. So the spike magnitude here scales
with how fast the throttle is changing, which closes a feedback loop through
the sensor's own disturbance - the loop reacts, the throttle moves, the wash
moves, the sensor spikes again.

A model is not evidence. What this can tell you is whether a change makes
things better or worse under the pathology we measured, which is the question
worth answering before spending an airframe on it.
"""

from __future__ import annotations

import argparse
import math
import random

G = 9.80665


class Barometer:
    """A barometer that lies the way ours lies."""

    LSB = 0.076          # 1 Pa, measured on d45
    DRIFT_MPS = 0.25 / 60.0
    WHITE = 0.02

    def __init__(self, spike_gain=0.010, spike_rate=2.0, seed=0,
                 quiet=False):
        # spike_gain: metres of spike per PWM/s of throttle movement.
        # Fitted so that a loop chasing hard (~400 PWM in 0.3 s, i.e. 1300
        # PWM/s) produces the ~1 to 4 m excursions both aircraft logged, while
        # a steady throttle produces none - which is exactly what separates
        # d44's clean --no-baro trace from its altitude-hold aborts.
        self.spike_gain = spike_gain
        self.spike_rate = spike_rate      # expected spikes per second at full chase
        self.rng = random.Random(seed)
        self.drift = 0.0
        self.prev_thr = None
        self.quiet = quiet                # a perfect sensor, for comparison

    def read(self, z_true: float, thr: float, dt: float) -> float:
        if self.quiet:
            return z_true
        self.drift += self.DRIFT_MPS * dt
        z = z_true + self.drift + self.rng.gauss(0.0, self.WHITE)
        if self.prev_thr is not None and dt > 0:
            rate = abs(thr - self.prev_thr) / dt          # PWM per second
            if self.rng.random() < self.spike_rate * dt * min(1.0, rate / 1300.0):
                z += self.rng.gauss(0.0, self.spike_gain * rate)
        self.prev_thr = thr
        return round(z / self.LSB) * self.LSB


def run(cfg, mode, alt, seconds, climb_s, takeoff_pwm, baro_w, vz_gain,
        gate_el_deg=None, seed=0, quiet_baro=False, dt=0.02, baro_hz=10.0,
        ceiling=None, verbose=False, thr_slew=0.0):
    """One flight. Returns a dict of what happened."""
    import numpy as np

    from raceline.rc_backend import AltitudeLoop, StateEstimate
    from seeker.dr_estimator import VerticalFilter
    from hardware.state import rot_zyx

    th = cfg.thrust
    kz = (cfg.vehicle.drag_quad_z or cfg.vehicle.drag_quad) / cfg.vehicle.mass_kg

    baro = Barometer(seed=seed, quiet=quiet_baro)
    vert = VerticalFilter(w=baro_w)
    loop = AltitudeLoop(cfg)
    R = rot_zyx(0.0, 0.0, 0.0)

    z_true = vz_true = 0.0
    thr = 1000
    thr_prev = None
    med, z_meas, next_baro = [], 0.0, 0.0
    phase, t_phase, z_t = "climb", 0.0, 0.0
    airborne = False
    t = 0.0
    hist = []
    ceiling = ceiling if ceiling is not None else alt + 1.0
    hit_ceiling = False

    n = int((seconds + climb_s + 3.0) / dt)
    for _ in range(n):
        t += dt
        # --- sensors -------------------------------------------------------
        fresh = t >= next_baro
        if fresh:
            next_baro += 1.0 / baro_hz
            raw = baro.read(z_true, thr, 1.0 / baro_hz)
            med.append(raw)
            med = med[-3:]
            z_meas = sorted(med)[1] if len(med) == 3 else raw
        # specific force the accelerometer would feel, plus its measured bias
        a_thrust = float(np.interp(thr, th.curve_pwm, th.curve_acc))
        az_world = a_thrust - G - kz * vz_true * abs(vz_true)
        z_f, vz_f = vert.update(dt, az_world + 0.02, z_meas, fresh)

        if not airborne and (z_f > 0.25 or vz_f > 0.7):
            airborne = True

        # --- control, the real code ----------------------------------------
        if phase == "climb":
            if mode == "no-baro":
                if t >= climb_s:
                    phase, t_phase = "hold", t
            else:
                z_t = min(alt, z_t + 0.3 * dt)
                if z_f >= alt - 0.15:
                    phase, t_phase, z_t = "hold", t, alt
        elif phase == "hold" and t - t_phase > seconds:
            break

        if mode == "cascade":
            # CASCADED, the textbook quadrotor structure: the height error
            # sets a VELOCITY TARGET, and a fast inner loop on the
            # accelerometer tracks that. The barometer therefore only ever
            # moves a target that is clamped to +-vz_cap, and can never make a
            # fast throttle change however badly it spikes. Our single-loop
            # version put it straight into the throttle, which is the feedback
            # path that broke five flights.
            if not airborne:
                thr = takeoff_pwm
            else:
                vz_cap = 0.5
                z_t2 = alt if phase != "climb" else min(alt, z_t)
                vz_ff = max(-vz_cap, min(vz_cap, 1.0 * (z_t2 - z_f)))
                a_cmd = max(-4.0, min(4.0, vz_gain * (vz_ff - vz_f)))
                thr = int(round(max(th.pwm_min, min(th.pwm_max,
                                                    cfg.pwm_for_thrust(G + a_cmd)))))
        elif mode == "no-baro":
            if not airborne:
                thr = takeoff_pwm
            else:
                vz_ff = 0.0
                if phase == "hold" and gate_el_deg is not None:
                    vz_ff = max(-0.30, min(0.30, 0.03 * gate_el_deg))
                a_cmd = max(-4.0, min(4.0, vz_gain * (vz_ff - vz_f)))
                thr = int(round(max(th.pwm_min, min(th.pwm_max,
                                                    cfg.pwm_for_thrust(G + a_cmd)))))
        else:
            est = StateEstimate(p=np.array([0.0, 0.0, z_f]),
                                v=np.array([0.0, 0.0, vz_f]), R=R, yaw=0.0)
            vz_ff = 0.3 if phase == "climb" else 0.0
            thr = loop.throttle(t, est, z_t, vz_ff, airborne, integrate=True)

        # THROTTLE SLEW LIMIT. The barometer's spikes scale with how fast the
        # throttle moves, so capping that rate caps the disturbance the loop
        # can inflict on its own sensor. Sally's clean flight moved 1250 ->
        # 1212 -> 1198 -> 1192 -> 1174, about 400 PWM/s and smooth. Randy's
        # crash went 1350 -> 1837 -> 1290 -> 1228 -> 1000 in half a second,
        # roughly 5000 PWM/s, and the barometer read -3.86 m through it.
        if thr_slew > 0 and thr_prev is not None:
            step = thr_slew * dt
            thr = int(round(max(thr_prev - step, min(thr_prev + step, thr))))
        thr_prev = thr

        # --- plant ---------------------------------------------------------
        a_thrust = float(np.interp(thr, th.curve_pwm, th.curve_acc))
        acc = a_thrust - G - kz * vz_true * abs(vz_true)
        vz_true += acc * dt
        z_true += vz_true * dt
        if z_true < 0.0:
            z_true, vz_true = 0.0, max(0.0, vz_true)
        hist.append((t, z_true, vz_true, z_f, z_meas, thr))
        if z_true > ceiling:
            hit_ceiling = True
            break

    held = [h for h in hist if h[0] > (climb_s if mode == "no-baro" else 1.0)]
    zs = [h[1] for h in held] or [0.0]
    target = alt if mode != "no-baro" else (sum(zs) / len(zs))
    return {
        "hit_ceiling": hit_ceiling,
        "peak": max(h[1] for h in hist) if hist else 0.0,
        "max_vz": max(abs(h[2]) for h in hist) if hist else 0.0,
        "wander": max(zs) - min(zs),
        "err": max(abs(z - target) for z in zs),
        "hist": hist,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default="../config/ladder/vehicle_k025_cam20_75.toml")
    ap.add_argument("--mode", choices=("baro", "no-baro", "cascade"), default="baro")
    ap.add_argument("--alt", type=float, default=0.6)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--climb-s", type=float, default=0.6)
    ap.add_argument("--takeoff-pwm", type=int, default=1250)
    ap.add_argument("--baro-w", type=float, default=1.0)
    ap.add_argument("--vz-gain", type=float, default=2.5)
    ap.add_argument("--gate-el", type=float, default=None,
                    help="hold this elevation error instead of zero vertical speed")
    ap.add_argument("--thr-slew", type=float, default=0.0,
                    help="cap on throttle change, PWM per second. 0 = no limit. "
                         "Sally's clean flight moved about 400 PWM/s; Randy's crash "
                         "about 5000.")
    ap.add_argument("--runs", type=int, default=20, help="seeds to average over")
    ap.add_argument("--sweep", action="store_true",
                    help="compare both modes, with and without a lying barometer")
    args = ap.parse_args(argv)

    from raceline.config import load_config
    cfg = load_config(args.config)

    def trial(mode, quiet, **kw):
        rs = [run(cfg, mode, args.alt, args.seconds, args.climb_s,
                  args.takeoff_pwm, args.baro_w, args.vz_gain,
                  gate_el_deg=args.gate_el, seed=s, quiet_baro=quiet,
                  thr_slew=kw.pop("thr_slew", args.thr_slew), **kw)
              for s in range(args.runs)]
        return {
            "ceiling": sum(r["hit_ceiling"] for r in rs),
            "peak": max(r["peak"] for r in rs),
            "vz": max(r["max_vz"] for r in rs),
            "wander": sum(r["wander"] for r in rs) / len(rs),
        }

    if args.sweep:
        print(f"{args.runs} runs each. hover_pwm {cfg.thrust.hover_pwm}, "
              f"takeoff {args.takeoff_pwm}, baro_w {args.baro_w}\n")
        print(f"{'':28} {'ceiling hits':>13} {'peak m':>8} {'max vz':>8} {'wander m':>9}")
        cases = [("single loop, no slew limit ", "baro", False, 0.0),
                 ("single loop, 800 PWM/s     ", "baro", False, 800.0),
                 ("single loop, 400 PWM/s     ", "baro", False, 400.0),
                 ("single loop, 200 PWM/s     ", "baro", False, 200.0),
                 ("cascade,     400 PWM/s     ", "cascade", False, 400.0),
                 ("velocity hold (no baro)    ", "no-baro", False, 0.0)]
        for label, mode, quiet, sl in cases:
            r = trial(mode, quiet, thr_slew=sl)
            print(f"  {label}  {r['ceiling']:>4}/{args.runs:<8} "
                  f"{r['peak']:>8.2f} {r['vz']:>8.2f} {r['wander']:>9.2f}")
        print("\n  'ceiling hits' is flights that ran away. The gate opening is")
        print("  1.5 m, so wander under about 0.7 m would fly the course.")
        return 0

    r = trial(args.mode, False)
    print(f"mode {args.mode}, {args.runs} runs")
    print(f"  ran away:   {r['ceiling']}/{args.runs}")
    print(f"  peak:       {r['peak']:.2f} m")
    print(f"  max vz:     {r['vz']:.2f} m/s")
    print(f"  wander:     {r['wander']:.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
