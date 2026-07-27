# Coordinate Conventions — Ground Truth

Single source of truth for sign/gain conventions in this codebase. **Every statement below is
derived from one cited source.** Where evidence is absent or contradictory, the entry is marked
`UNKNOWN — needs flight test` rather than guessed.

Confirmed sources:
1. **SYSID** — `src/datasets/sysid_merged_20260607_210234/SYSID_REPORT.md`
2. **LOG-A** — `src/datasets/race_pilot_dbg_20260630_154329.csv` (pure-seeker flight, `RACE_KP_ROLL=+0.40`, `RACE_OFFY_TARGET=0.58`)
3. **LOG-B** — `src/datasets/race_pilot_dbg_20260630_155617.csv` (pure-seeker flight, `RACE_KP_ROLL=-20.0`, `RACE_OFFY_TARGET=0.0`)
4. **DYN** — `src/common/dynamics.py` (measured sign/thrust constants)
5. **CAM** — Tech Spec VADR-TS-003 §3.7/§3.8: 640×360, 20° camera uptilt, pinhole `fx=fy=320`, `[cx,cy]=[320,180]`, VFoV 90°
6. **BRIEF** — session "WHAT HAS BEEN PROVEN IN FLIGHT" notes (multi-iteration flight history)

> **Control path matters.** The race pilot sends **ATTITUDE (quaternion) setpoints** via
> `send_attitude_setpoint()` (DYN:123). That path runs `roll/pitch/yaw` through `_euler_to_quat()`
> **unmodified** — the `ROLL_SIGN`/`PITCH_SIGN`/`YAW_SIGN` constants (DYN:31–35) are applied only in
> the **rate-mode** feedback loop (`send_rate_attitude`), which the race pilot does **not** use.
> So for the race pilot, the operative angle conventions are standard NED quaternion, not the
> `*_SIGN` constants. The `*_SIGN` values are documented here only to prevent cross-contamination.

---

## Camera frame (offset_x, offset_y)

- **Range:** each axis is normalized to roughly `[-1, +1]` (image edges). Zero = image center.
  Evidence: CAM `[cx,cy]=[320,180]` (center pixel); config comment `offset_x (+right)`,
  `offset_y: -1 top .. +1 bottom` (`config.py:14,17`).
- **offset_x:**
  - `0` = gate at horizontal image center (pixel col 320).
  - **Increasing offset_x (+) = gate to the drone's RIGHT.** Evidence: CAM (`cx=320`, standard
    pinhole) + `config.py` comment `(+right)` + live behaviour (drone tracks a right-side gate 2 at
    `+0.46..+0.81`).
  - **CORRECTION (open-loop roll test 2026-06-30, `runtime.roll_test`):** a POSITIVE roll command
    banks the drone LEFT, so a NEGATIVE roll command banks RIGHT. LOG-B's `offset_x=+0.011 →
    roll_cmd=-12.53°` was therefore a **RIGHT** bank (not "left" as an earlier version stated), which
    centered a **right-side** gate → confirms `+offset_x = RIGHT`. The earlier "empirical LEFT" entry
    was wrong because it mislabeled the `-roll` bank direction.
- **offset_y:**
  - `0` = gate at vertical image center (pixel row 180) = a ray **20° above horizontal** (because
    the camera is uptilted 20°, CAM §3.8). So **offset_y = 0 means the gate is ~20° ABOVE the
    drone**, not level.
  - **Increasing offset_y (+) = gate LOWER in the image (toward bottom) = gate physically lower
    relative to the drone** (drone is higher than / climbing above the gate).
  - **Level reference** (gate at the drone's own altitude, dead ahead) projects to
    **offset_y ≈ +0.65**, derived from CAM: pixel drop `= fy·tan(20°) = 320·0.364 = 116.5 px`
    below center; normalized `= 116.5 / 180 = 0.647`. Consistent with LOG-A behavior (below).
  - Evidence: CAM (`fy=320`, `cy=180`, 20° uptilt) + LOG-A.

---

## Roll

- **Sim actuator direction (open-loop, CONFIRMED): a POSITIVE `roll_cmd` banks/drifts the drone
  LEFT; a NEGATIVE `roll_cmd` banks RIGHT.** Evidence: **open-loop test 2026-06-30**
  (`runtime.roll_test`) — constant `+0.30 rad` roll, level pitch/yaw, hover, zero perception → drone
  banked LEFT. This is the sim's true roll sign and it is **inverted** from the naive NED-quaternion
  assumption (that `+roll = right`) — same inversion pattern as yaw. The `ROLL_SIGN=+1.0` (DYN:31)
  is rate-mode only and does **not** describe this attitude-path behaviour.
- **Centering law (`roll = RACE_KP_ROLL · offset_x`):** **`RACE_KP_ROLL` must be NEGATIVE** to
  center the gate. Evidence: Confirmed empirically in LOG-B and last flight (gate 0 passed cleanly
  with `RACE_KP_ROLL=-20.0`). Negative Kp: gate left (`+offset_x`) → negative `roll_cmd` (left bank)
  → drone moves left → gate centers. Geometry argument in earlier version was incorrect due to a
  wrong assumption about the camera x-axis orientation.

---

## Pitch

- **Race-pilot convention: NEGATIVE `pitch_cmd` = nose-down = FORWARD flight.** In standard NED
  quaternion (the `send_attitude_setpoint` path), positive pitch = nose-up; negative = nose-down =
  forward lean. Evidence: `config.py:19` (`RACE_FWD_LEAN`, applied as `pitch_cmd = -RACE_FWD_LEAN`,
  i.e. negative) + legacy config note *"Negative = forward; flip sign … if it flies BACKWARD"* +
  BRIEF architecture (`pitch = -FORWARD_LEAN` to fly at the gate; `pitch=0` is a coast).
  - Log arithmetic confirms the command (not the resulting direction): both LOG-A and LOG-B show a
    constant `pitch_cmd = -1.15°` for the whole run (`-RACE_FWD_LEAN = -0.02 rad = -1.146°`).
- ⚠️ **Do not confuse with `PITCH_SIGN = -1.0` (DYN:32).** That sign, and the DYN comment
  *"+pitch leans forward,"* apply to the **rate-mode** feedback loop, **not** the race pilot's
  attitude path. For the race pilot the operative rule is the one above: **negative = forward.**
- Residual caveat: forward-vs-backward at `-0.02 rad` has **not** been independently confirmed in
  these two logs (no position/velocity telemetry in Phase 2). Treated as established by BRIEF; if a
  future flight shows backward drift at negative pitch, this entry must be revisited.

---

## Yaw

- **Sim actuator direction (open-loop, CONFIRMED): a POSITIVE yaw command rotates the drone LEFT
  (counter-clockwise).** Evidence: **open-loop test 2026-06-30** (`runtime.yaw_test`) — commanded a
  ramping positive yaw via `send_attitude_setpoint` with level roll/pitch, hover thrust, and **zero
  perception**; drone span CCW. This is the ground-truth actuator sign, no tracking/roll confound.
  (So the sim's yaw is inverted vs a naive read, consistent with `YAW_SIGN=-1.0`, DYN:34.)
- **Race-pilot yaw law:** `yaw_cmd += RACE_KP_YAW · offset_x` each frame (integrated absolute heading,
  anti-wound: only integrates while `|offset_x| < RACE_YAW_MAX_OFF`).
- **`RACE_KP_YAW` must be NEGATIVE to yaw TOWARD the gate.** Derivation: a gate on the RIGHT
  (`+offset_x`, per config) must be faced by turning RIGHT = CW = a **negative** yaw command; with
  `yaw_cmd += KP_YAW·offset_x` that needs `KP_YAW < 0`. Cross-checks: (a) closed-loop isolated test —
  `+0.03` span away (unstable), `-0.03` yawed toward; (b) the open-loop CCW result above. **Use
  `-0.03`.**
- ⚠️ **Isolation matters:** yaw sign can't be read by eye while roll is also moving the gate (both
  change `offset_x`). Settle it open-loop (`runtime.yaw_test`), never from a mixed roll+yaw run.
- ✅ **`+offset_x = gate to the RIGHT`** — RESOLVED by the open-loop roll test (2026-06-30). LOG-B's
  old "empirical LEFT" was wrong (it mislabeled the `-roll` bank as "left" when the sim banks `+roll`
  left, so `-roll` = right). See the offset_x and Roll sections.

---

## Thrust

- **Phase 2 hover thrust ≈ 0.26** (collective, normalized 0..1). Evidence: `config.py:18`
  (`RACE_HOVER = 0.26`) + BRIEF (*"Hover thrust: ~0.26 (Phase 2 sim, bracketed in flight)"*).
- **`HOVER_THRUST = 0.299` (DYN:20) is the OLD June-7 build, not Phase 2** — BRIEF + DYN comment
  *"hover ~0.299"* tied to the `THRUST_CLIMB_TABLE` build.
- **Increasing thrust above hover = climb UP; below hover = sink/descend.** Evidence: DYN
  `THRUST_CLIMB_TABLE` (DYN:49–57) is monotonic and crosses zero climb at `0.299`
  (`0.20→-5.46 m/s`, `0.299→0`, `0.50→+16.9 m/s`). Cross-confirmed by LOG-A: thrust held above 0.26
  while `offset_y` rose monotonically (drone climbed); see vertical section.
- Range clamp: `MIN_THRUST = 0.0`, `MAX_THRUST = 1.0` (DYN:25–26).

---

## offset_y vertical channel

- **Positive offset_y = gate BELOW image center = gate is low relative to the drone** (drone is
  high / above the gate). (See Camera frame section for the derivation; `offset_y=0` is already
  ~20° above the drone due to uptilt, and `+0.65` is true level.) Evidence: CAM + `config.py:17`.
- **Thrust command produced:** `thrust = RACE_HOVER + RACE_KP_VERT · (RACE_OFFY_TARGET − offset_y)`
  (`race_pilot.py`). With `RACE_KP_VERT = +0.10`:
  - gate **above** target row (`offset_y < TARGET`) → `(TARGET − offset_y) > 0` → **thrust > hover →
    climb** toward it.
  - gate **below** target row (`offset_y > TARGET`) → **thrust < hover → descend.**
  - Evidence (arithmetic): LOG-B `t=0.000, offset_y=+0.144, TARGET=0.0 → thrust = 0.26 + 0.10·(0−0.144) = 0.2456`,
    logged `0.246`. ✓
- **Behavioral evidence (LOG-A, `TARGET=0.58`):** `offset_y` rose **monotonically** from `-0.02`
  (`t≈0`) through the `0.58` target (`t≈1.66`) to `+0.93` (`t≈2.13`), then the gate exited the
  bottom of frame; `area` peaked at only `~0.099` (never the `RACE_PASS_AREA=0.30` threshold) and
  `gates_passed` stayed `0`. Thrust ranged from `0.32` (climb) down to a minimum of only `0.225`
  (just `0.035` below hover). **Interpretation: the drone climbed above the gate and flew over the
  top** — the P-only vertical channel built climb velocity it could not arrest (no damping term).
  This is direct evidence that rising `offset_y` ↔ drone going high over the gate.
- ⚠️ **Live config:** `RACE_OFFY_TARGET = 0.0` (`config.py:17`). Since true level ≈ `+0.65`,
  targeting `offset_y = 0` aims the drone at a point **~20° above** each gate → it will tend to fly
  **low / below** the gate. Targeting the gate at true altitude requires `TARGET ≈ 0.65`
  (or the previously-flown `0.58`, a touch above level). Optimal value is a tuning question, not a
  convention — flagged here only so the sign/geometry is not misread.

---

## RACE_ROLL_MAX

- **Current value: `0.25` rad (`config.py:15`) = `14.32°`** (`0.25 · 180/π = 14.32`).
- Evidence (confirmed live): LOG-B `t=0.291, offset_x=-0.017` would command `-20·-0.017 = +0.34 rad`
  but logs `roll_cmd = +14.32°` — i.e. clamped exactly at `0.25 rad = 14.32°`. ✓
- Purpose (config comment): cap the roll setpoint so a side blob can't drive an unclamped panic
  bank (BRIEF: an earlier `0.85` shadow caused 33° banks into a crash).

---

## Summary table

| Quantity | Convention | Confidence | Source |
|---|---|---|---|
| `+offset_x` | gate to drone's RIGHT (open-loop corrected; config `(+right)`) | High | roll_test 2026-06-30 |
| `+offset_y` | gate BELOW center / drone high above gate | High | CAM + LOG-A |
| `offset_y = 0` | gate ~20° ABOVE drone (uptilt) | High | CAM |
| level (gate at drone altitude) | `offset_y ≈ +0.65` | High | CAM (derived) |
| `+roll_cmd` | banks/drifts LEFT (open-loop; sim roll is inverted) | High | roll_test 2026-06-30 |
| `RACE_KP_ROLL` sign to center | NEGATIVE (empirically confirmed) | High | LOG-B + last flight |
| `+yaw command` | rotates LEFT / CCW (open-loop confirmed) | High | yaw_test 2026-06-30 |
| `RACE_KP_YAW` sign to yaw toward gate | NEGATIVE (`-0.03`) | High | yaw_test + closed-loop 2026-06-30 |
| `-pitch_cmd` | forward flight (nose-down) | Medium | config + BRIEF |
| Phase 2 hover thrust | `≈ 0.26` | High | config + BRIEF |
| thrust > hover | climb up | High | DYN table + LOG-A |
| `RACE_ROLL_MAX` | `0.25 rad = 14.32°` | High (confirmed in LOG-B) | config + LOG-B |
| `ROLL/PITCH/YAW_SIGN` | rate-mode only — NOT used by race pilot | High | DYN |
