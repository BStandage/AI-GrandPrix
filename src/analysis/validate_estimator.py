"""
Replay the ace_pilot estimator OFFLINE against ground truth from a Tab 7 flight.

    python -m analysis.validate_estimator datasets/sysid_YYYYMMDD_HHMMSS/tab7_ace_envelope.csv

Feeds the recorded attitude COMMANDS and raw IMU accels through exactly the models ace flies
(96 ms first-order attitude lag, two-term drag, unleaked IMU vertical integrator) and compares
the result against odometry truth per tick. Outputs the numbers the first three ace flights
never had: velocity model error during transients, vertical-channel fidelity, and position
drift rate.

Sign conventions are resolved EMPIRICALLY per axis (correlation of replayed vs true velocity)
and reported - if a sign is flipped here, it is flipped in flight.
"""

import csv
import math
import sys

G = 9.81
ATT_TAU = 0.096
DRAG_C1, DRAG_C2 = 0.1141, 0.0192
VZ_TAU = 45.0


CLAMP = math.radians(45.0)   # the sim's measured per-axis attitude clamp
HOVER = 0.27


def replay(rows, pitch_sign, roll_sign, psi0, h_model="tan", v_att="cmd", vz_sign=+1,
           a_up_max=12.0):
    """h_model: 'tan' (g*tan(tilt), hover-equilibrium assumption) or
               'thr' (cmd_thr/hover * g * sin-decomposed - uses our own commanded collective).
       v_att:  attitude used to rotate the IMU accels for the vertical channel:
               'cmd' = the lagged command (ace's current), 'imu' = complementary filter on
               gyros+accels (steady's proven _imu_attitude)."""
    th = ph = 0.0                    # lagged COMMAND attitude (clamped like the sim clamps)
    eth = eph = 0.0                  # IMU complementary-filter attitude
    vx = vy = vz = 0.0
    alt = float(rows[0]["alt"])
    x, y = float(rows[0]["x"]), float(rows[0]["y"])
    fwd = (math.cos(psi0), math.sin(psi0))
    right = (-math.sin(psi0), math.cos(psi0))
    out = []
    t_prev = None
    for r in rows:
        t = float(r["timestamp_ms"]) * 1e-3
        dt = 0.0 if t_prev is None else t - t_prev
        t_prev = t
        if not (0.0 < dt <= 0.1):
            out.append((vx, vy, vz, alt, x, y))
            continue
        a = 1.0 - math.exp(-dt / ATT_TAU)
        cmd_p = max(-CLAMP, min(CLAMP, pitch_sign * math.radians(float(r["cmd_pitch_deg"]))))
        cmd_r = max(-CLAMP, min(CLAMP, roll_sign * math.radians(float(r["cmd_roll_deg"]))))
        th += (cmd_p - th) * a
        ph += (cmd_r - ph) * a
        xacc, yacc, zacc = float(r["xacc"]), float(r["yacc"]), float(r["zacc"])
        # complementary-filter attitude (steady's recipe: gyro integrate, accel correct)
        mag = math.sqrt(xacc * xacc + yacc * yacc + zacc * zacc)
        valid = abs(mag - G) <= 2.5
        roll_a = math.atan2(yacc, -zacc)
        pitch_a = math.atan2(-xacc, math.hypot(yacc, zacc))
        eth_g = eth + float(r["pitchspeed"]) * dt   # body rates (odometry) = what gyros measure
        eph_g = eph - float(r["rollspeed"]) * dt
        if valid:
            eth = 0.98 * eth_g + 0.02 * pitch_a
            eph = 0.98 * eph_g + 0.02 * roll_a
        else:
            eth, eph = eth_g, eph_g
        # horizontal accel model
        sp = math.hypot(vx, vy)
        drag = DRAG_C1 + DRAG_C2 * sp
        if h_model == "thr":
            tw = float(r["cmd_thr"]) / HOVER          # thrust in g units
            af = tw * G * math.cos(ph) * math.sin(th)
            ar = tw * G * math.cos(th) * math.sin(ph)
        else:
            af, ar = G * math.tan(th), G * math.tan(ph)
        vx += (af * fwd[0] + ar * right[0] - drag * vx) * dt
        vy += (af * fwd[1] + ar * right[1] - drag * vy) * dt
        x += vx * dt
        y += vy * dt
        # vertical: rotate raw accels through the chosen attitude
        ath, aph = (eth, eph) if v_att == "imu" else (th, ph)
        sp_, cp_ = math.sin(vz_sign * ath), math.cos(ath)
        sr_, cr_ = math.sin(vz_sign * aph), math.cos(aph)
        wd = sp_ * xacc + cp_ * sr_ * yacc + cp_ * cr_ * zacc
        a_up = -(wd + G)
        if abs(a_up) <= a_up_max:
            vz = (vz + a_up * dt) * math.exp(-dt / VZ_TAU)
            vz = max(-35.0, min(35.0, vz))
            alt += vz * dt
        out.append((vx, vy, vz, alt, x, y))
    return out


def corr(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    num = sum((p - ma) * (q - mb) for p, q in zip(a, b))
    da = math.sqrt(sum((p - ma) ** 2 for p in a)) or 1e-9
    db = math.sqrt(sum((q - mb) ** 2 for q in b)) or 1e-9
    return num / (da * db)


def main():
    f = sys.argv[1]
    rows = list(csv.DictReader(open(f)))
    rows = [r for r in rows if r.get("timestamp_ms") not in (None, "", "0")]
    print(f"{len(rows)} rows")

    # spawn/settle heading: mean measured yaw over the first 100 rows
    psi0 = sum(float(r["yaw"]) for r in rows[:100]) / 100
    print(f"psi0 (measured, first settle) = {math.degrees(psi0):+.1f} deg")

    tvx = [float(r["vx_w"]) for r in rows]
    tvy = [float(r["vy_w"]) for r in rows]

    t_all = [float(r["timestamp_ms"]) * 1e-3 for r in rows]
    dur = t_all[-1] - t_all[0]

    def score(rep):
        evh = [abs(math.hypot(v[0], v[1]) - float(r["vh"])) for v, r in zip(rep, rows)]
        evz = [abs(v[2] - float(r["climb_up"])) for v, r in zip(rep, rows)]
        ealt = [abs(v[3] - float(r["alt"])) for v, r in zip(rep, rows)]
        ex = [math.hypot(v[4] - float(r["x"]), v[5] - float(r["y"])) for v, r in zip(rep, rows)]
        return (sum(evh) / len(evh), sum(evz) / len(evz), sum(ealt) / len(ealt),
                ex[-1] / max(dur, 1), max(evh))

    # resolve command signs on the baseline model first
    best = None
    for ps in (+1, -1):
        for rs in (+1, -1):
            rep = replay(rows, ps, rs, psi0)
            c = corr([v[0] for v in rep], tvx) + corr([v[1] for v in rep], tvy)
            if best is None or c > best[0]:
                best = (c, ps, rs)
    c, ps, rs = best
    print(f"sign resolution: pitch_sign={ps:+d} roll_sign={rs:+d} (summed corr {c:+.3f})")

    print(f"\n{'variant':38s} {'vh_err':>7s} {'vz_err':>7s} {'alt_err':>8s} {'xy_drift':>9s}")
    variants = [("h=tan v=cmd (ace current)", dict(h_model='tan', v_att='cmd')),
                ("h=thr v=cmd", dict(h_model='thr', v_att='cmd')),
                ("h=tan v=imu", dict(h_model='tan', v_att='imu')),
                ("h=thr v=imu", dict(h_model='thr', v_att='imu')),
                ("h=thr v=imu vz_sign=-1", dict(h_model='thr', v_att='imu', vz_sign=-1)),
                ("h=thr v=imu a_up_max=30", dict(h_model='thr', v_att='imu', a_up_max=30.0)),
                ("h=thr v=cmd vz_sign=-1", dict(h_model='thr', v_att='cmd', vz_sign=-1))]
    reps = {}
    for name, kw in variants:
        rep = replay(rows, ps, rs, psi0, **kw)
        reps[name] = rep
        vh_e, vz_e, alt_e, xy_d, vh_max = score(rep)
        print(f"{name:38s} {vh_e:7.2f} {vz_e:7.2f} {alt_e:8.2f} {xy_d:9.3f}")

    # per-segment detail for the best thrust-model variant
    rep = reps["h=thr v=imu a_up_max=30"]
    segs = {}
    for i, r in enumerate(rows):
        segs.setdefault(r["seg"], []).append(i)
    print("\nper-segment end-of-hold (h=thr v=imu a_up_max=30): model vs truth")
    for seg, idx in segs.items():
        if seg in ("level",) or seg.startswith(("rec",)):
            continue
        i_end = idx[-1]
        vm = math.hypot(rep[i_end][0], rep[i_end][1])
        vt = float(rows[i_end]["vh"])
        vzm, vzt = rep[i_end][2], float(rows[i_end]["climb_up"])
        print(f"  {seg:18s} vh {vm:6.2f}/{vt:6.2f} ({vm - vt:+5.2f})   "
              f"vz {vzm:+6.2f}/{vzt:+6.2f} ({vzm - vzt:+5.2f})")


if __name__ == "__main__":
    main()
