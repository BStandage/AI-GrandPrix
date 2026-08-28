# PQ tech specs intake - VADR-TS-004 / 00.01 (2026-08-18)

Source: `20260818_PQ_Technical_Spec_0001.pdf`

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
- Version constraint: **2026.6.1 or earlier** (MSP override breaks above).

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
- **Double gate = one gate flown through twice** (confirms the g10
  out-and-back model).
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
- [ ] UART protocol details: MSP or other? Expected command rate;
      staleness/failsafe behavior if our RC stream hiccups.
- [ ] IMU message set + rates on the FC->Jetson link.
- [ ] Geofence / kill-switch / auto-disarm behavior our stack must
      account for.
- [ ] Bench/tethered powered testing allowed outside flight windows?

## Diff vs VQ / what we must re-validate on-site

- [ ] Vision stream still usable by detector / aim (FOV, rate, color under PQ lighting)
- [ ] Attitude setpoint interface unchanged (signs: +roll = RIGHT, +pitch = nose down)
- [ ] Race clock / gate counter / timing line available for ticks + lap count (need 2 laps)
- [ ] Double-gate association in map extract (two openings, one structure)
- [ ] Track fits 60x21 - survey map extent / leg lengths sane vs footprint
- [ ] Transport / arming / laptop <-> Jetson timing

## Strategy (not steady-as-race)

Steady (or any slow visual flyer) is **survey only**. Race path is:

**survey -> map (accept) -> solve -> fly solved policy**

Open-loop tape without a trusted PQ map is archive/practice only.
