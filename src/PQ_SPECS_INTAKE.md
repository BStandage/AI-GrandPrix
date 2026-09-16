# PQ tech specs intake - VADR-TS-004 / 00.01 (2026-08-18)

Source: `docs/specs/20260818_PQ_Technical_Spec_0001.pdf` (the published course and Orin quickstart PDFs are in `docs/specs/` too)

## Locked from spec

| Item | Spec |
|------|------|
| Daily access | **15 min** slots per team per day |
| Complete run | **2 laps**, no human intervention |
| Qualifying window | Times from **final two days** only |
| Ranking | Complete runs by time; incomplete by gates passed; tie = time to last gate |
| Track footprint | **60 m x 21 m** |
| Gates | Outer 2700x2700x260 mm; inner opening **1500x1500x260 mm**; includes a **double gate** |
| Airframe | Archer B2, Betaflight FC, UART to AI board |
| Compute | Jetson Orin NX 16 GB, Seeed A603, JetPack 6.2 |
| Sensors | Arducam 12.3 MP HQ (M12); IMU |

## FAQ answers - organizer responses (2026-08-28)

### Betaflight / FC
- Full config access (rates, PIDs, filters, telemetry): **yes**. Reflash: **no**.
- Extract config yourself via Betaflight CLI - run `diff all` on day 1.
- Version constraint as first stated: "2026.6.1 or earlier (MSP override
  breaks above)". **2026-09-15: asked "what Betaflight firmware is on the
  Archer, is there a fork?", the organizers sent configurator 10.10.0 and
  no fork -> the drones are on stock Betaflight 4.5.x** (10.10.0 is the
  4.5 configurator and cannot talk to 2026.x firmware; the 2026.6.1
  figure was presumably the app). The sim's SITL stays on 2026.6.0: a
  4.5.5 SITL stalls in lockstep (sim branch `feature/betaflight-4.5`,
  parked). Every CLI setting we use exists in both. Install configurator
  10.10.0 from the GitHub release tag; it coexists with the 2026.6.1 app.

### FC <-> Jetson
- **UART. TX: RC control commands (Jetson->FC). RX: IMU data (FC->Jetson).**
  That is the WHOLE interface - no attitude, no pose. Matches the elodin
  sim architecture exactly (our code emits RC sticks, gets IMU back).
- Jetson: 25 W power mode, **root access yes**.

### Airframe
- **8" Archer Block 2**, props 8x4.1. IMU/motors/ESCs/battery/AUW: TBD
  (detailed spec promised) -> these become `vehicle.toml` `[vehicle]`/
  `[thrust]` on day-1 sysid.

### Camera
- Rolling shutter, **1920x1080 @ 60 fps** (mode 1). Exposure + gain
  controllable. Frames timestamped on the Orin (CLOCK_MONOTONIC).
- **Intrinsics/extrinsics NOT provided** -> on-site checkerboard
  calibration is a day-1 HARD DEPENDENCY before any PnP/gate ranging.
- **IMU position + orientation in the drone ARE provided** -> the
  camera-to-IMU lever arm isn't a guess; fusion extrinsic is half-solved.

### Gates
- **Double gate = one gate flown through twice** (confirms the
  out-and-back model; it is organizer gate 9 = our g8 on the published
  map).
- Gate depth discrepancy: spec 260 mm vs diagram 140 mm - **under
  review**. Do not hard-code depth-sensitive logic.
- Gate pictures to be provided ahead of time (detector training data).

### Logistics
- **3 slots/day PLUS a training cage**; manual piloting **allowed**.
- 4 drones per team (not shared); Neros runs a repair workshop.
- No data sharing between teams.

### Still open (push organizers)
- [x] **Angle limit / ACRO - largely self-answered (2026-08-28):**
      Betaflight's angle limit only exists in ANGLE mode; ACRO has no
      attitude cap at all. The FAQ grants full config access (rates,
      PIDs, filters) with no reflash and version <= 2026.6.1 - so if the
      units ship in ANGLE, we set `angle_limit` (BF default 55 deg) or
      configure ACRO ourselves. The VQ sim's 45 deg clamp was a property
      of the old attitude-setpoint API, NOT of PQ's RC-over-UART
      interface. Residual question only: does any competition RULE pin
      specific FC settings?
- [x] Training cage = **5 x 5 m** (confirmed 2026-08-28). Big enough for:
      hover/throttle sysid, SHORT-BURST attitude steps (~0.5 s, +-2.5 m of
      room), camera calibration, detector imagery. NOT big enough for
      sustained speed runs. Still open: cage height, gates inside?, time
      limits.
- [ ] Gate depth resolution (260 vs 140 mm).
- [ ] Exact Betaflight 4.5.x patch level on the Archer, and the MSP
      override setup in their `diff all`: which channels we may override
      (`msp_override_channels_mask`), whether arming stays on the
      pilot's transmitter, and what happens when our RC stream pauses.
- [ ] Double gate 9: is it crossed twice per lap (south top / north low)
      and what is the top-opening height? (4.05 m is our estimate.)
- [x] UART protocol = **MSP over /dev/ttyTHS1 @ 115200** (Orin quickstart,
      2026-09-15). Still open: sustainable MSP_SET_RAW_RC rate,
      staleness/failsafe behavior if our RC stream hiccups.
- [~] IMU on the FC->Jetson link: MSP polled, 30-50 Hz attitude realistic
      (quickstart). Still open: raw gyro/accel rate vs attitude-only.
- [ ] Geofence / kill-switch / auto-disarm behavior our stack must
      account for.
- [ ] Bench/tethered powered testing allowed outside flight windows?

## Published track (2026-09-15) - `Drone_Race_Track_Gate_Coordinates_with_doublegate.pdf`

The organizer published ground-truth gate coordinates. `data/course_map.json`
IS that table now (source `published`); the 2026-08-27 overhead estimate is
kept at `data/course_map_overhead_estimate.json` and agreed with it to
< 0.5 m after a rigid fit (0.06 deg, +0.34/+0.24 m), so the estimate era's
geometry work carries over. What changed:

- **Boundary 85 x 165 ft = 25.9 x 50.3 m** (the tech spec's "60 x 21 m"
  was wrong or a different layout). Origin = top-left of the figure, X
  right, Y down, rotation = flight direction in deg CW from "up". Our map
  frame is ENU with the origin at the SW corner: `x = X ft * 0.3048`,
  `y = (165 - Y ft) * 0.3048`, `heading = 90 - rot`.
- **10 gates, flown in numerical order from gate 1.** The overhead
  extraction had 11 "gates": its order-1 bar at (12 ft, 93 ft) is an
  orange marker behind gate 1 (launch pad?), not a gate.
- **The start moved.** Gate 1 = (12, 71) ft northbound; the old start
  (3.8, 16.6 m) is organizer gate 10, the LAST gate of the lap.
- **The drone starts on the DASHED orange line behind gate 1** (Brian,
  2026-09-15), ~(12, 93) ft = (3.2, 21.4) m, 7.3 m south of gate 1 on its
  centreline (`meta.start` in the map; the sim spawns there). The solid
  orange bar 1.5 m past gate 1 at (12, 66) ft is probably the timing line.
  The takeoff leg is therefore ~7 m, not the 3 m the sim assumed.
- **Gates 5 and 8 are flown at rot 215 (down-left)**, not straight down:
  the estimate had gate 5 heading due south (37 deg off).
- **Gate 9 at (39.7, 147.7) ft rot 180 is the double gate** (filename).
  The table lists it ONCE, southbound. Our model stays: south through the
  top opening (4.05 m, height NOT published - estimate), U-turn, north
  through the low opening (1.35 m). Confirm both the second pass and the
  top-opening height on site.
- Code labels are traversal order: **gK = organizer gate K+1** (g0 = gate
  1, g8 = the double gate 9, g9 = gate 10). Planner knobs, line_search,
  sim tests and the viewer were relabelled by physical identity. The
  28.30 s estimate-map plan is archived
  (`out/plans/archive_20260915_estimate_map/`); `plan_RACE.json` is now
  the published-map plan, flown clean 23/23 in **29.55 s** (race_160,
  2026-09-15, laps 15.72 + 13.84; sim on ground truth). The speed ladder
  (`raceline.ladder`, 60/50/40/35 s rungs with centred crossings) is the
  race-day binary-search set - see `PQ_PROCEDURE.md`.

| Gate | X ft | Y ft | rot | ENU x m | ENU y m | heading | label |
|---|---|---|---|---|---|---|---|
| 1 START | 12.0 | 71.0 | 0 | 3.66 | 28.65 | N | g0 |
| 2 | 16.0 | 39.0 | 0 | 4.88 | 38.40 | N | g1 |
| 3 | 40.0 | 17.0 | 90 | 12.19 | 45.11 | E | g2 |
| 4 | 72.0 | 40.0 | 180 | 21.95 | 38.10 | S | g3 |
| 5 | 66.0 | 70.0 | 215 | 20.12 | 28.96 | SW (-125 deg) | g4 |
| 6 | 41.0 | 86.0 | 90 | 12.50 | 24.08 | E | g5 |
| 7 | 70.5 | 106.6 | 180 | 21.49 | 17.80 | S | g6 |
| 8 | 60.0 | 129.0 | 215 | 18.29 | 10.97 | SW (-125 deg) | g7 |
| 9 double | 39.7 | 147.7 | 180 | 12.10 | 5.27 | S (then N) | g8-top / g8-low |
| 10 | 13.0 | 110.0 | 0 | 3.96 | 16.76 | N | g9 |

Six orange circles (cones/pylons) sit on the boundary at X = 0 and ~49 ft,
Y = 0, ~58, ~115 ft - eyeballed from the figure into `meta.cones_xy`.

## Orin NX board (2026-09-15) - `orin-nx-quickstart.pdf` (Manual I, rev 2026-09)

Answers the open UART/protocol questions. Facts we build on:

- **Board**: Orin NX 16 GB on Seeed A603, JetPack 6.2 / L4T r36.4.3,
  kernel 5.15.148-tegra, NVMe, 25 W nvpmodel. Login `dcl`/`dcl`, root via
  sudo. Reach it over the USB gadget: `ssh dcl@192.168.55.1` (laptop side
  192.168.55.100/24; serial console fallback /dev/ttyACM0 at 115200).
- **FC link = MSP over `/dev/ttyTHS1` at 115200**, request/response, the
  FC never pushes. Organizer's own words: attitude polling at 30-50 Hz is
  realistic, a hard real-time loop is not - "keep anything that has to
  close a loop at flight rates inside Betaflight". So: RC sticks down at
  whatever MSP_SET_RAW_RC rate the link sustains, attitude/IMU back at
  30-50 Hz, and the rate/attitude loops stay in Betaflight (ANGLE/ACRO
  choice matters more than ever). Once per board:
  `sudo ~/target/msp/setup_jetson_uart.sh --apply` frees the UART from the
  Linux console.
- **Libraries to import: `~/target/msp/msp.py`, `msp_rc.py`.** Bench:
  `msp_bench.py --port /dev/ttyTHS1 info | telemetry --hz 20 | rc`.
  `imu_check.py` = IMU acceptance test. `companion_listener_msp.py` = live
  telemetry table. `msp_bench.py ... demo --props-off` is the ONLY tool
  that ARMS (throttle profile) - props off, everything else is read-only
  or holds disarmed.
- **Camera = IMX477 raw Bayer on CSI-2, `/dev/video0` (V4L2, RG10, 10-bit)**.
  ISP debayer/AE/AWB = Argus via `nvarguscamerasrc`, which needs an
  EGL/display context: over plain SSH it fails (`Failed to initialize
  EGLDisplay`) and tools fall back to software debayer with no AE/AWB
  (flat grey image = expected, not a fault). For the real ISP image run
  with `DISPLAY=:0 XAUTHORITY=<file>` from a desktop session on the board.
  Raw path for "is the sensor alive": `raw-view.py --ctrl exposure=20000
  --ctrl gain=200` or `v4l2-ctl --set-fmt-video=width=1920,height=1080,
  pixelformat=RG10 --stream-mmap --stream-count=100`. **Our capture
  pipeline must be decided: nvarguscamerasrc (needs a display context at
  boot - a headless X/EGL session) vs raw V4L2 + our own debayer/AE.**
- **Camera and FC share NO clock and NO hardware trigger** ("the single most
  important architectural fact on this board"). Frame timestamps come from
  `live-view-pts.py` / `frame-timestamps.py` (hardware capture PTS, jitter
  CSV); IMU samples are timestamped on arrival. Visual-inertial sync is a
  software estimate - measure the offset, do not assume it.
- **Do NOT `pip install opencv-python`** (JetPack's cv2 has GStreamer, the
  wheel does not, `VideoCapture(..., CAP_GSTREAMER)` fails silently). Use
  the system python3. Do NOT touch the device tree (bootloader partition,
  re-flash = organizers only). `sudo shutdown -h now` and wait for the LED
  before pulling the battery (real SSD).
- Diagnostics in order: `sudo ~/target/bringup-check.sh` (stops at the
  first broken layer), `camera-bind-check.sh`, `usb-device-mode.sh`,
  `signoff.sh` (factory acceptance PDF). `tegrastats` for thermals.
- Companion `SOFTWARE-GUIDE.md` (on the board) has every option and the
  camera/IMU timing constraints - read it on day 0.

## Diff vs VQ / what we must re-validate on-site

- [ ] Vision stream still usable by detector / aim (FOV, rate, color under PQ lighting)
- [ ] Attitude setpoint interface unchanged (signs: +roll = RIGHT, +pitch = nose down)
- [ ] Race clock / gate counter / timing line available for ticks + lap count (need 2 laps)
- [ ] Double gate = organizer gate 9 (g8): confirm the second (northbound,
      low opening) pass and the top-opening height on site
- [x] Track footprint: published 85 x 165 ft (25.9 x 50.3 m) - the spec's
      60 x 21 m is superseded; map is `data/course_map.json` (published)
- [ ] Transport / arming / laptop <-> Jetson timing

## Strategy (not steady-as-race)

Steady (or any slow visual flyer) is **survey only**. Race path is:

**survey -> map (accept) -> solve -> fly solved policy**

Open-loop tape without a trusted PQ map is archive/practice only.
