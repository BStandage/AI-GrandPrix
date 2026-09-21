"""Which known failure fired? Check a flight log against all of them at once.

    python -m raceline.triage ../flightlogs/2026-09-22/hw_follower_XXX.csv [race_00N.csv]

Called automatically by scripts/pull_flight.sh. Every check below is a failure
this team has ALREADY had, or one a change made on 2026-09-21 could newly
cause. Each prints the numbers behind its verdict, because a verdict you
cannot check is a verdict you cannot act on.

The point is speed. After a flight you have minutes, not an evening, and the
difference between "she climbed again" and "vert_live was 0 for the whole
approach, so she was never on the camera at all" is the difference between
guessing at a fix and making one.
"""

from __future__ import annotations

import csv
import sys

OK, WARN, BAD = "ok  ", "WARN", "FAIL"


def _f(r, k):
    v = r.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _col(rows, k):
    return [v for v in (_f(r, k) for r in rows) if v is not None]


def check(rows, race):
    """Yield (verdict, headline, detail) for every known failure mode."""
    n = len(rows)
    t = _col(rows, "t")
    dur = (t[-1] - t[0]) if len(t) > 1 else 0.0

    # --- the vertical channel: was it ever actually in charge? -------------
    if "vert_live" in (rows[0] if rows else {}):
        live = _col(rows, "vert_live")
        seen = [r for r in rows if _f(r, "el_deg") is not None]
        frac = (sum(live) / len(live)) if live else 0.0
        if seen and frac < 0.25:
            yield (BAD, f"VERTICAL NEVER WENT LIVE: vert_live=1 on only {100*frac:.0f}% of ticks",
                   f"The camera measured an elevation on {len(seen)} ticks but the aircraft was "
                   f"allowed to use it on {int(sum(live))}. She flew a velocity hold. "
                   f"FIX: raise MATCH_STALE_S in hardware/runtime.py (2 min), or check `unm` below.")
        elif seen:
            yield (OK, f"vertical live on {100*frac:.0f}% of ticks with a detection", "")
        else:
            yield (WARN, "no el_deg anywhere: the vertical channel saw no gate at all",
                   "Either --vert vision was not passed, or the detector never matched a gate.")

    # --- the flight-4 runaway: does the command grow as the gate nears? ----
    pairs = [(_f(r, "det_range"), _f(r, "el_deg")) for r in rows]
    pairs = [(a, b) for a, b in pairs if a and b is not None and a < 12.0]
    if len(pairs) > 20:
        far = [b for a, b in pairs if a >= 5.0]
        near = [b for a, b in pairs if a <= 4.0]
        if far and near:
            fm, nm = sorted(far)[len(far)//2], sorted(near)[len(near)//2]
            if abs(nm) > abs(fm) + 3.0:
                yield (BAD, f"RUNAWAY SIGNATURE: elevation {fm:+.1f} deg far out -> {nm:+.1f} deg close in",
                       "The error grows as the range closes - that is a clipped ring, not a real "
                       "height error. FIX: AIGP_GATE_COMMIT_FRAC=0.70 (env var, no sync).")
            else:
                yield (OK, f"elevation stable on approach: {fm:+.1f} deg far -> {nm:+.1f} deg near", "")

    # --- the same runaway, seen through the OLD columns. A log written before
    #     2026-09-21 has no el_deg, and a triage that cannot read the flight it
    #     was built from is not a triage. dz = z_target - z is the command
    #     itself, and it is in every race_NNN.csv ever written.
    if race and not any(_f(r, "el_deg") is not None for r in rows):
        dzs = []
        for r in race:
            zt, z = _f(r, "z_target"), _f(r, "z")
            if zt is not None and z is not None:
                dzs.append((_f(r, "t"), zt - z))
        if dzs:
            peak_t, peak = max(dzs, key=lambda x: abs(x[1]))
            capped = [x for x in dzs if abs(x[1]) >= 0.34]
            if capped:
                yield (BAD, f"VERTICAL COMMAND SATURATED: {peak:+.2f} m at t={peak_t:.1f}s "
                            f"({len(capped)} ticks at the cap)",
                       "The +-0.35 m cap is only reached when the camera is badly wrong - it is "
                       "the flight-4 signature. FIX: AIGP_GATE_COMMIT_FRAC=0.70 (env var).")
            elif abs(peak) > 0.20:
                yield (WARN, f"vertical command peaked at {peak:+.2f} m (t={peak_t:.1f}s)", "")
            else:
                yield (OK, f"vertical command stayed small: peak {peak:+.2f} m", "")

    # --- commit: did it fire, and at a sane range? -------------------------
    com = [(_f(r, "det_range"), r.get("commit_why", "")) for r in rows]
    fired = [(a, w) for a, w in com if w not in ("", "-", None)]
    if fired:
        rngs = [a for a, _ in fired if a]
        why = fired[0][1]
        if rngs:
            first = max(rngs)
            if first > 5.5:
                yield (WARN, f"commit fires early, at {first:.1f} m ({why})",
                       "Long blind run to the gate. FIX: AIGP_GATE_COMMIT_FRAC=0.92 (env var).")
            elif first < 2.5:
                yield (BAD, f"commit fires late, at {first:.1f} m ({why})",
                       "A correction begun this close has nowhere to go. "
                       "FIX: AIGP_GATE_COMMIT_FRAC=0.70 (env var).")
            else:
                yield (OK, f"commit fired at {first:.1f} m ({why})", "")
    elif any(_f(r, "det_range") for r in rows):
        yield (WARN, "commit never fired", "She steered on the gate all the way in. Expected around 3.7 m.")

    # --- the barometer, which we did NOT fix -------------------------------
    z = [(_f(r, "t"), _f(r, "z")) for r in rows]
    z = [(a, b) for a, b in z if a is not None and b is not None]
    jumps = [(abs(z[i][1] - z[i-1][1]) / max(1e-3, z[i][0] - z[i-1][0]), z[i][0])
             for i in range(1, len(z))]
    if jumps:
        worst, when = max(jumps)
        if worst > 10.0:
            yield (BAD, f"HEIGHT ESTIMATE BROKEN: {worst:.0f} m/s implied at t={when:.1f}s",
                   "The airframe makes ~10 m/s at its clamp. This is the barometer, and there is "
                   "no fast fix - it is the VIO-shaped problem. Send me alt_gated/alt_rejected.")
        elif worst > 5.0:
            yield (WARN, f"height estimate rough: {worst:.0f} m/s implied at t={when:.1f}s", "")
        else:
            yield (OK, f"height estimate sane: worst {worst:.1f} m/s implied", "")
    for k, label in (("alt_gated", "baro samples dropped for a moving throttle"),
                     ("alt_rejected", "baro samples rejected as impossible"),
                     ("cam_rej", "detections dropped as implausible")):
        v = _col(rows, k)
        if v and v[-1] > 0:
            yield (WARN if v[-1] > n * 0.3 else OK, f"{label}: {v[-1]:.0f}", "")

    # --- takeoff ------------------------------------------------------------
    bounce = [(_f(r, "t"), _f(r, "z")) for r in rows
              if _f(r, "throttle") == 1250 and (_f(r, "z") or 0) < -0.2 and (_f(r, "t") or 0) > 1.5]
    if bounce:
        yield (BAD, f"TAKEOFF BRANCH RE-ENTERED {len(bounce)} times mid-flight",
               f"first at t={bounce[0][0]:.1f}s, z={bounce[0][1]:.2f}. The airborne latch is "
               f"bouncing. FIX: the latch test in solvers/follower.py.")

    # --- lateral, and the gates themselves ---------------------------------
    crossings = []
    last = None
    for r in rows:
        ev = _f(r, "cross_ev")
        if ev is not None and ev != last:
            if last is not None:
                crossings.append((ev, _f(r, "cross_lat"), _f(r, "cross_dz")))
            last = ev
    if crossings:
        bad = [c for c in crossings if c[1] is not None and abs(c[1]) > 0.75]
        msg = ", ".join(f"g{int(a)} {b:+.2f}m" for a, b, _ in crossings[:6])
        yield (BAD if bad else OK, f"{len(crossings)} crossings: {msg}",
               "Anything past +-0.75 m was outside the opening." if bad else "")
    else:
        yield (WARN, "no gate crossings recorded", "")

    # --- health -------------------------------------------------------------
    for k, lo, label in (("att_hz", 25, "FC link rate"), ("cam_fps", 50, "camera rate")):
        v = _col(rows, k)
        if v:
            m = sorted(v)[len(v)//2]
            yield (WARN if m < lo else OK, f"{label} median {m:.0f}",
                   "Recording may be starving it: try --record-step 2." if m < lo else "")
    to = _col(rows, "timeouts")
    if to and to[-1] > 0:
        yield (WARN, f"link timeouts: {to[-1]:.0f}", "")
    unm, fixes = _col(rows, "unm"), _col(rows, "fixes")
    if unm and fixes:
        yield (WARN if unm[-1] > max(fixes[-1], 1) else OK,
               f"map fixes {fixes[-1]:.0f} accepted, {unm[-1]:.0f} unmatched",
               "More rejected than accepted: the detector is seeing things that are not gates."
               if unm[-1] > max(fixes[-1], 1) else "")

    yield (OK, f"{n} rows over {dur:.1f} s", "")


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__.splitlines()[2].strip())
        return 2
    rows = list(csv.DictReader(open(argv[0], encoding="utf-8")))
    race = list(csv.DictReader(open(argv[1], encoding="utf-8"))) if len(argv) > 1 else []
    if not rows:
        print("empty log")
        return 1

    results = list(check(rows, race))
    bad = [r for r in results if r[0] == BAD]
    print()
    for verdict, headline, detail in results:
        print(f"  [{verdict}] {headline}")
        if detail:
            for line in detail.split(". "):
                if line.strip():
                    print(f"         {line.strip().rstrip('.')}.")
    print()
    warn = [r for r in results if r[0] == WARN]
    if bad:
        print(f"  {len(bad)} FAILURE(S), {len(warn)} warning(s). Start with the first failure -")
        print("  they cascade, and the later ones are often consequences of the first.")
    elif warn:
        print(f"  No known FAILURE, but {len(warn)} warning(s) above. If the flight went badly")
        print("  anyway, the cause is not on the list yet - send the csv, the .log and what")
        print("  you saw, and it gets added.")
    else:
        print("  Clean against every known failure mode.")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
