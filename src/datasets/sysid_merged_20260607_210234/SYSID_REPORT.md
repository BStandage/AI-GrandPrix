# Flight Dynamics System-Identification Report

Source: `sysid_merged_20260607_210234` (merged)

Per-tab provenance (best clean run chosen for each, since long sessions degrade):
- `tab1_rotational` ← `sysid_20260607_185548`
- `tab2_drag` ← `sysid_20260607_193845`
- `tab3_recovery` ← `sysid_20260607_205111`
- `tab4_feasibility` ← `sysid_20260607_202832`

> All force quantities are **accelerations (m/s²)**. The sim exposes no vehicle mass, so absolute Newtons / drag coefficients are not recoverable; the controller only needs accelerations regardless. Motor columns are **observed normalized outputs** (saturation proxy), not commanded RPM — the interface commands body rates + collective thrust, not the motor mixer.

## Master envelope summary

| Quantity | Measured | dynamics.py (current) |
|---|---|---|
| Hover thrust | (see drag/vertical runs) | 0.299 |
| roll ω_max (rad/s) | 29.23 | MAX_RATE=6.0 (clamp) |
| pitch ω_max (rad/s) | 29.19 | MAX_RATE=6.0 (clamp) |
| yaw ω_max (rad/s) | 19.48 | MAX_RATE=6.0 (clamp) |
| roll α_max (rad/s²) | 902.2 | — |
| pitch α_max (rad/s²) | 945.8 | — |
| yaw α_max (rad/s²) | 398.4 | — |
| Terminal fwd speed (m/s) | 9.38 | SPEED_LEAN_TABLE top 9.0 |
| Terminal lateral speed (m/s) | 8.33 | — |
| Inverted-dive terminal vz (m/s, +down) | 37.27 | free-fall ≈ 10.2 |
| Quadratic drag k (a=k·v², 1/m) | 0.0343 | — |

## 1. Rotational dynamics

| Axis | ω_max (rad/s) | α_max (rad/s²) | Lag to 63% (ms) | Cross-axis coupling |
|---|---|---|---|---|
| roll | 29.23 | 902.2 | 96 | 0.00 |
| pitch | 29.19 | 945.8 | 96 | 0.00 |
| yaw | 19.48 | 398.4 | 153 | 0.00 |

## 2. MPC No-Go-Zone (minimum recovery airspace)

Constraint for the planner:  `Z_drone − Z_floor > Δz_recovery(vz)`  — the altitude that must be reserved to arrest a descent of downward speed `vz`.

- **continuous**: `Δz_recovery = 0.0083·vz² + 0.080·vz + 1.909`  (n=11)
- **snap**: `Δz_recovery = 0.0090·vz² + 0.056·vz + 1.980`  (n=11)

## 3. Maneuver efficiency audit (continuous vs zero-thrust snap)

| entry_vz | strategy | axis | Δz_loss (m) | horizon-cross (s) | arrested |
|---|---|---|---|---|---|
| 10.6 | continuous | roll | 4.16 | 0.55 | True |
| 10.2 | continuous | pitch | 2.57 | 0.55 | True |
| 10.7 | snap | roll | 4.19 | 0.54 | True |
| 10.3 | snap | pitch | 2.92 | 0.51 | True |
| 15.5 | continuous | roll | 6.17 | 0.61 | True |
| 15.2 | continuous | pitch | 4.84 | 0.66 | True |
| 15.0 | snap | roll | 6.00 | 0.64 | True |
| 15.3 | snap | pitch | 4.01 | 0.76 | True |
| 20.0 | continuous | roll | 8.02 | 0.73 | True |
| 20.3 | continuous | pitch | 6.25 | 0.83 | True |
| 20.5 | snap | roll | 8.20 | 0.75 | True |
| 20.1 | snap | pitch | 5.04 | 1.08 | True |
| 25.3 | continuous | roll | 10.52 | 0.86 | True |
| 25.0 | continuous | pitch | 7.00 | 2.22 | True |
| 25.1 | snap | roll | 10.42 | 0.86 | True |
| 25.0 | snap | pitch | 8.17 | 1.14 | True |
| 30.1 | continuous | roll | 13.00 | 1.03 | True |
| 30.0 | continuous | pitch | 10.14 | 2.33 | True |
| 30.2 | snap | roll | 13.14 | 1.07 | True |
| 30.0 | snap | pitch | 10.28 | 2.32 | True |
| 35.1 | continuous | roll | 15.34 | 1.41 | True |
| 31.2 | continuous | pitch | 7.22 | 4.47 | False |
| 35.1 | snap | roll | 15.08 | 1.43 | True |
| 31.2 | snap | pitch | 6.93 | 4.47 | False |

## 4. Actuator saturation warnings (motor pinned > 400 ms)

- `rec_20_continuous_pitch` — 412.0 ms at saturation (structural loss of attitude authority).
- `rec_20_snap_pitch` — 612.0 ms at saturation (structural loss of attitude authority).
- `rec_25_continuous_pitch` — 1624.0 ms at saturation (structural loss of attitude authority).
- `rec_25_snap_pitch` — 672.0 ms at saturation (structural loss of attitude authority).
- `rec_30_continuous_roll` — 576.0 ms at saturation (structural loss of attitude authority).
- `rec_30_continuous_pitch` — 1712.0 ms at saturation (structural loss of attitude authority).
- `rec_30_snap_roll` — 588.0 ms at saturation (structural loss of attitude authority).
- `rec_30_snap_pitch` — 1712.0 ms at saturation (structural loss of attitude authority).
- `rec_35_continuous_roll` — 916.0 ms at saturation (structural loss of attitude authority).
- `rec_35_continuous_pitch` — 3576.0 ms at saturation (structural loss of attitude authority).
- `rec_35_snap_roll` — 904.0 ms at saturation (structural loss of attitude authority).
- `rec_35_snap_pitch` — 3588.0 ms at saturation (structural loss of attitude authority).

## 5. Kinematic feasibility cone (15 m gate spacing)

Max lateral acceleration from a 29° bank: **5.4 m/s²**.

| Entry speed (m/s) | Lateral offset, flown (m) | Lateral offset, derived (m) |
|---|---|---|
| 7.6 | 0.00 | 10.55 |
| 9.0 | 3.48 | 7.42 |

> NB: this airframe tops out ≈ 9 m/s (SPEED_LEAN_TABLE), far below the blueprint's 15–35 m/s assumption. The cone is mapped over realistic speeds; high-speed gate displacement that would force an aerobatic flip does not arise at these speeds.
