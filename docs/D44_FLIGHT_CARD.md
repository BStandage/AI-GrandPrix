# d44 flight card - "Sally the Brave"

**Scope: HOVER ONLY.** No camera, no course. `hover.py` without `--gate-z`
never opens the camera, so none of d45's optical numbers are needed or used.
Nothing here depends on a calibration Sally has not had.

For the course, she needs her own camera numbers - see the bottom.

---

## What actually matters for a hover

| | state |
|---|---|
| mask 15 | verified on d44, 2026-09-19 |
| ANGLE always on | aux row **2** on d44 - d45 uses row 5, do NOT paste one drone's aux line into the other |
| accelerometer scale | measured automatically at startup, and it refuses to run if she is not still |
| pitch sign | assume NOSE DOWN positive, same firmware as d45. **Check it - 1 minute, below** |
| override test | **never run on d44.** Mandatory before anything spins |
| all-up weight | irrelevant here: thrust comes off the curve in m/s^2, already mass-independent |

d45's `fy`, `hfov`, principal point and mount tilt are **not used by a plain
hover**. Ignore them rather than assuming them.

---

## 1. Sync

```
cd ~/GitRepos/AI-GrandPrix
scripts/sync_drone.sh d44 cam20_75
```

## 2. Foam over the flight controller

d45's barometer read **-3.86 m while hovering at 0.4 m** with props running,
and the altitude loop answered that with 4.35 g and flew to 2 m. Sally is the
same airframe with the same exposed sensor.

Canopy on, or foam over the board. Two minutes, and it is the single highest
value thing on this page.

## 3. Attitude signs - 1 minute, props off

```
ssh d44
cd ~/AI-GrandPrix/src
python3 -m hardware.tiltcheck --port /dev/ttyTHS1
```

Tilt her 30 degrees each way, holding each pose still. Must say **PASS**.

Not a formality. d45 had its pitch sign inverted for months: the error is
EXACTLY ZERO when the aircraft is level - which is how every bench check gets
done - and about -5 m/s^2 at 30 degrees. Same firmware, so Sally is very
likely fine. "Very likely" is what cost us a camera.

If it FAILS, add `--pitch-nose-up-positive` to every command.

## 4. Modes decode

```
python3 -c "
from hardware.bridge import FcBridge
br = FcBridge.open(port='/dev/ttyTHS1'); st = br.fc.status()
print('active:', st.active_modes, '| override:', st.msp_override, '| angle:', st.angle_mode)
br.stop()"
```

Must list ANGLE. Must not print an empty list.

## 5. Override test - with a transmitter, PROPS OFF

```
python3 -m hardware.override_test --port /dev/ttyTHS1
```

Must PASS. **This has never been run on d44.** It is the only abort there is,
and d45 only survived its runaway because it works. Full procedure in
`docs/OVERRIDE_TEST.md`.

---

## 6. The hover

```
python3 -m hardware.hover --port /dev/ttyTHS1 --alt 0.4 --seconds 20 \
  --ceiling 1.0 --config ../config/ladder/vehicle_k025_cam20_75.toml --arm
```

Props on, fresh battery, aircraft **level** on the ground.

1. Run it. It prints the accelerometer figures, then `LIVE: waiting for the pilot`.
2. Throttle stick **fully down**.
3. **ARM on.**
4. **MSP OVERRIDE on.** She lifts off by herself.
5. Hands on the sticks, throttle near hover, touch nothing.
6. Climbs to 0.4 m, holds 20 s, descends, disarms herself.

**ABORT: MSP OVERRIDE off.** Keep the throttle stick near hover position
throughout so taking over is a handoff, not a drop.

### Then read the log before doing anything else

```
tail -15 ../out/flightlogs/hover_*.csv
```

**The `z` column is the whole test.** A smooth ramp 0 to 0.4 means the
barometer is behaving. Metre-sized jumps mean the foam did not take.

| also watch | good |
|---|---|
| `vz` during the climb | reads real numbers, not near zero |
| the landing | lands and disarms, no bounce at knee height |
| `CEILING HIT` | should not appear at all |

### What is protecting her

| | |
|---|---|
| barometer readings implying over 12 m/s | rejected, last good value held |
| the altitude loop | closes on accel+baro FUSED, not the raw sensor |
| climb demand | capped at 12 m/s^2 (2.2 g) whatever the error |
| ceiling | 1.0 m on the RAW barometer - throttle to minimum, disarm |
| MSP OVERRIDE | the one that has never failed |

---

## Later: what she needs to fly the course

None of this is needed for a hover. Twenty-five minutes, props off, no
transmitter, whenever there is bench time.

1. `hardware.camcal_board grab` then `solve --square-mm 25.0` - 20 min.
   Tilt the board 30-45 deg in BOTH axes; face-on views measure nothing.
   Wants RMS under 0.5 px and fx within 1 percent of fy.
2. `hardware.camtilt --fy <new> --cy <new> --dist ... --lens-h ... --target-h ...`
   at two distances, aircraft LEVEL.
3. `hardware.bench drift` for the heading drift.
4. Weigh her all-up.

Full procedure in `docs/CAMERA_CALIBRATION.md`. **None of d45's optical numbers
transfer** - the principal point in particular sat 14 and 18 px off centre in
different directions on d45's own two cameras, so copying one aircraft's
offset to another can double the bias rather than remove it.
