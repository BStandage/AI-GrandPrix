# Race card — 2026-09-21

**ABORT, ALWAYS: MSP OVERRIDE off.** Throttle stick near hover throughout, so
taking over is a handoff and not a drop.

**Two people.** One on the transmitter, finger on MSP OVERRIDE, eyes on the
aircraft. One on the laptop reading numbers. The person on the sticks never
looks at the screen.

---

## Numbers — do not cross these

| | **d44 "Sally"** | **d43** |
|---|---|---|
| `--fy` | **859** | **835.5** |
| `AIGP_CAM_CX` | 616.9 | 611.9 |
| `AIGP_CAM_CY` | 330.0 | 394.5 |
| `--cam-tilt` | **10** | **10** |
| `--heading-drift-dpm` | **5.0** | *omit* |
| **ANGLE switch** | **aux row 2** | **aux row 5** |

---

## d44 "Sally"

```
ssh d44
cd ~/AI-GrandPrix/src

python3 -m hardware.runtime --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_FLAT_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 859 --cam-tilt 10 --map-north here --heading-drift-dpm 5.0 \
    --vert vision --arm
```

Throttle **fully down** → **ARM** → **MSP OVERRIDE on** → **ANGLE row 2**.
The plan clock starts when the FC reports armed + override.

```
scripts/pull_flight.sh d44
```

---

## d43

```
ssh d43
cd ~/AI-GrandPrix/src

AIGP_CAM_CX=611.9 AIGP_CAM_CY=394.5 python3 -m hardware.runtime \
    --port /dev/ttyTHS1 --pilot follower \
    --traj ../out/plans/plan_FLAT_s15_cam20_75.json \
    --config ../config/ladder/vehicle_s15_cam20_75.toml \
    --fy 835.5 --cam-tilt 10 --map-north here \
    --vert vision --arm
```

Throttle **fully down** → **ARM** → **MSP OVERRIDE on** → **ANGLE row 5**.

**d43 is row 5, Sally is row 2.**

```
scripts/pull_flight.sh d43
```

---

## The abort call

The screen narrates what she believes:

```
gate 1 IN SIGHT at 7.5 m, 2.3 deg ABOVE me -> CLIMBING (commanding +0.14 m)
gate 1 COMMIT at 3.7 m (ring fills the frame) - height reference released
gate 1 CROSSED, 0.22 m to the left
```

**Abort if the degrees figure GROWS as the range shrinks.** That is the
flight-4 runaway, and it is the one thing we came to fix.

---

## Between flights, no sync needed

Prefix on the command line:

```
AIGP_GATE_COMMIT_FRAC=0.70    commit EARLIER — if she still climbs
AIGP_GATE_COMMIT_FRAC=0.92    commit LATER — if blind too long before the gate
AIGP_VERT=baro                abandon vision height — last resort only
```

---

## After each flight

`pull_flight.sh` lands everything in `flightlogs/<today>/`, prints the
narration, then names which known failure fired and its fix.

The columns to read first:

| column | reads |
|---|---|
| `el_deg` | what the camera measured |
| `vert_live` | 0/1 — was the aircraft allowed to use it |
| `commit_why` | `size` / `both_edges` / `width_clipped` / `lost` |
