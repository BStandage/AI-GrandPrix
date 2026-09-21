# Flight logs, 20-21 September 2026

The first autonomous flights. Saved here because they live on the drones'
SD-less filesystems and the drones get reflashed, swapped and crashed.

**Two complete CSVs and several console-only records.** The console ones are
here because a run that aborts before writing much still tells you why.

---

## Complete CSVs

### `hover_20260920_214011.csv` - Randy, the flight that broke his camera

Six rows, and the third one is the whole story:

```
0.181  z=-0.100   amps 26.8
0.281  z=-3.860   amps 34.0   <- 3.8 m step in 100 ms = 37 m/s
0.382  z=-0.490
0.582  z=+1.000   thr=1000
```

Target was 0.76 m. At `t=0.281` the aircraft believed it was 3.9 m LOW and
commanded `a_cmd +32.9` -> throttle 1837, near full. It reached about 3.3 m
and fell. **The control responded correctly to a barometer reading that was
fiction.**

This is the log the barometer outlier rejection, the climb cap and the
`--no-baro` mode were all written from.

### `hover_20260921_003652.csv` - Sally, the first clean autonomous hover

`--no-baro --takeoff-pwm 1250 --climb-s 0.6`. Forty-three rows, and
everything good we know came out of it:

- **vertical speed held inside 0.11 m/s for five seconds**, no oscillation
- **steady hover throttle 1224-1229, median 1227** against the 1228 the
  corrected curve predicts - derived independently from Randy's crash log an
  hour earlier, on a different aircraft
- **the barometer is CLEAN here**: 0.51, 0.62, 0.69, 0.71, 0.73, 0.65, 0.57.
  Same airframe type, same conditions, 17 A. Randy was drawing 35-55 A with
  the wrong curve making him climb
- `t=0.382`: vz jumps 0.000 -> 0.806, the launch detector firing. The seed
  implies a real acceleration of 2.11 m/s^2 where the curve predicts 1.14, so
  **the thrust curve still under-reads above hover** even after the -60 PWM
  correction. That is an open loose end

---

## Console-only records

Runs that aborted before the CSV was worth keeping. Transcribed from the
terminal.

### `hover_20260920_233559` - Randy, ceiling at 1.29 m

```
at altitude (1.00 m) after 0.5 s, holding
CEILING HIT: z=1.29 m > 1.00
```

Reached 1.00 m in half a second where 1.32 g predicts 0.40. First sign that
something was badly wrong with the thrust model, read at the time as a
barometer fault.

### `hover_20260921_001558` - Randy, ceiling at 1.06 m

```
at altitude (0.85 m) after 0.8 s, holding
CEILING HIT: z=1.06 m > 1.00
```

### `hover_20260921_002631` - Randy, the crash that broke his arm

`--no-baro`, and the log that produced the thrust curve correction:

```
climb done after 0.45 s, vz=+2.79
t=1.0   vz=+2.07   thr=1209   a_cmd=-4.00
t=2.0   vz=+1.23   thr=1228   a_cmd=-3.06
t=3.0   vz=+1.17   thr=1231   a_cmd=-2.93
t=4.0   vz=+1.19   thr=1230   a_cmd=-2.99
```

Climbed to 5.1 m, drifted left, hit the net. **A commanded -4 m/s^2 that
produces no descent means the throttle chosen for "4 below hover" WAS hover.**
Two points from this run - vz constant at ~1228 PWM, and 1350 PWM reaching
2.79 m/s in 0.45 s - gave -63 and -57 PWM, and the -60 shift was applied to
every real-aircraft config from it.

### `hover_20260921_030353` - Sally, ceiling on a RAW barometer spike

```
at altitude (0.52 m) after 0.7 s, holding
CEILING HIT: z=1.76 m raw / 0.91 filtered > 1.60
```

The fusion was right and the ceiling threw the flight away anyway, because it
tripped on either value. Fixed: the raw barometer must now stay over the line
for 0.35 s, the fused height still trips at once.

### `hover_20260921_030610` - Sally, the one that broke her leg

`--takeoff-pwm 1250 --climb 0.3`, target 0.6 m:

```
at altitude (0.45 m) after 0.7 s, holding
t=1.0 hold z=1.41 (target 0.60) vz=+3.04 thr=1000 a_cmd=-19.91
CEILING HIT (fused): z=1.68 m raw / 1.64 filtered > 1.60
```

**The CSV for this one was never recovered and it is the gap in the record.**
The rows between 0.4 s and 1.0 s would say whether the barometer dropped out,
whether vz was lying, or whether the thrust is simply higher than the curve
claims. Without them, why a gentle 1.12 g takeoff reached 3 m/s is
unexplained.

---

## What to do with these

`hardware.vertsim` models a barometer fitted to these logs and runs the real
`AltitudeLoop` and `VerticalFilter` against it. Any change to the vertical
channel should be swept there first - being wrong offline is free, and five of
these runs cost an airframe between them.
