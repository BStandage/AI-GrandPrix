# Status and what to run

Branch: `feature/hardware-seeker`. Sim repo: `elodin-sim-aigp` on `main`
(Betaflight SITL 2026.6.0; the Archer runs 4.5.x).

## Status (2026-09-16)

- Course: `data/course_map.json` is the organizer's published table.
  10 gates, gate 9 is the double (code labels g0..g9, g8-top/g8-low).
  Two laps = 23 crossings. Start = the dashed line 7.3 m behind gate 1.
- Sim, ground truth: `plan_RACE.json` clean in 29.55 s.
- Sim, no position sensor (vision-aided dead reckoning: FC accel and
  attitude, baro altitude, a position fix from every gate the camera
  sees): 60 s rung 57.2 s clean, 40 s rung 40.6 s clean, 35 s rung and
  plan_RACE fail with the 20 deg camera. With a 35 deg mount and a
  120 deg lens: 35 s rung 36.6 s clean, plan_RACE 29.4 s clean.
- Vision-only seeker (fallback, no plan, gate to gate): 215 s clean.
- Hardware: `src/hardware/` runs the follower over MSP, rehearsed
  against the sim's SITL disarmed. It has never flown a real drone.
  Unknowns to measure on site: camera mount tilt, focal length, real
  detector thresholds, FC accel scale and signs, pitch sign, the compass
  heading of map north.

## Sim (Docker Desktop running; commands from `AI-GrandPrix/src`)

```
python -m raceline.batch_fly ../out/plans/plan_LADDER_60s.json        # ground truth, watch the referee
AIGP_STATE_SOURCE=deadreckon python -m raceline.batch_fly --timeout 300 ../out/plans/plan_LADDER_60s.json   # no position sensor
AIGP_STATE_SOURCE=deadreckon AIGP_CAM_TILT_DEG=35 AIGP_CAM_HFOV_DEG=120 python -m raceline.batch_fly --timeout 300 ../out/plans/plan_RACE.json
python -m raceline.batch_fly --solver solvers.seeker --angle --timeout 400 ../out/plans/plan_RACE.json      # vision-only fallback
python -m raceline.ladder --targets 60 50 40 35      # rebuild rungs; one midpoint: --k 0.4
python race.py --plan-only                            # (repo root) new plan_NNN candidate; promote by copying over plan_RACE
```

Watch a flight instead: sim repo `run_race_docker.cmd` (flies plan_RACE).
Traces: `out/flightlogs/race_NNN.csv`, sim repo `race_result_NNN.json`.

## Rehearse the drone runtime against the sim (sim running, from `src`)

```
python -m hardware.bench --tcp 127.0.0.1:5761 info
python -m hardware.runtime --tcp 127.0.0.1:5761 --map-north 0 --pilot follower --traj ../out/plans/plan_LADDER_60s.json --no-camera --dry-run
```

## Archer (on the Orin, props OFF until the last line; from `AI-GrandPrix/src`)

```
python3 -m hardware.bench --port /dev/ttyTHS1 info                 # firmware, modes, arming blockers
python3 -m hardware.bench --port /dev/ttyTHS1 rc-test --props-off  # FC echoes our sticks
python3 -m hardware.bench --port /dev/ttyTHS1 arm-test --props-off # ARMED flag, then disarm
python3 -m hardware.bench --port /dev/ttyTHS1 telemetry --hz 5 --seconds 20   # heading while pointing along gate 1 = --map-north
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north <deg> --cam-tilt <deg> --fy <px> --pilot follower --traj ../out/plans/plan_LADDER_60s.json --dry-run
python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north <deg> --cam-tilt <deg> --fy <px> --pilot follower --traj ../out/plans/plan_LADDER_60s.json --arm
```

Fallback, no plan: same runtime with `--pilot seeker`. Add
`--pitch-nose-down-positive` if the bench telemetry reads pitch that way.
Bench measurements and the order on the day: `src/PQ_PROCEDURE.md`.

## Race day order

1. 60 s rung. Clean twice, fly the fastest rung you brought.
2. Fails, fly the midpoint (`raceline.ladder --k`).
3. plan_RACE only after the 35 s rung has held twice.
4. A complete slow run outranks every incomplete fast one.

## Where to read next

| You want to... | Go to |
|---|---|
| Bench numbers, day-1 measurements, the 15 min slot | `src/PQ_PROCEDURE.md` |
| The published course, the Orin board, firmware facts | `src/PQ_SPECS_INTAKE.md` |
| Set up the sim and fly a first race | `docs/GETTING_STARTED_RACING_LINE.md` |
| Betaflight, MSP, ANGLE vs ACRO | `docs/WHAT_IS_BETAFLIGHT.md` |
| How the stack is built | `docs/RACING_LINE_STACK.md` |
| Write or tune a solver | `src/solvers/README.md` |
| What is banned before planning | `RESTRICTIONS.md` |

Tune only `config/vehicle.toml`. Organizer PDFs: `docs/specs/`. Plans made
on the old estimated map: `out/plans/archive_20260915_estimate_map/`.

---

Everything below is the organizers' original dev-kit readme.

## AI Grand Prix (AI-GP) Development Kit
Conceived by Anduril founder Palmer Luckey and partnered with the Drone Champions League (DCL), Neros Technologies, and JobsOhio, AI-GP is a premier autonomous drone racing competition.
This global challenge invites elite engineers and teams of up to 8 people to design, build, and deploy autonomy software capable of piloting high-speed racing drones through professional-grade courses-with absolutely zero human intervention.
For complete competition details and updates, visit the official website at www.theaigrandprix.com.

## Competition Highlights

* The Stakes: Compete for a share of a $500,000 prize pool and career opportunities at Anduril.
* The Hardware: Complete competitive parity. All teams utilize identical racing drones built by Neros Technologies incorporating DCL's AI vector module.
* The Mission: Program the ultimate AI pilot to conquer dynamic, real-world flight conditions using onboard vision sensing-no GPS or absolute coordinate data will be provided.

------------------------------
## Repository Contents
This package contains the foundational tools required to develop, test, and qualify your autonomous flight software.
## 1. AIGP_X.zip (The Simulator)
This archive contains the official AI-GP flight simulator environment for Windows.

* Setup: Extract the ZIP archive to your local directory.
* Execution: Launch the simulator by running FlightSim.exe from the unzipped root folder.
* Authentication: Access the virtual qualifier within the simulator by logging in with your official simulator account credentials.

## 2. PyAIPilotExample.zip (The Code Template)
This archive provides a starter template to help you interface with the simulator and write your autonomous flight algorithms.

* Environment: Tested and verified on Python 3.14.2.
* Setup:
1. Unzip the archive.
   2. Install the required dependencies:

   pip install -r requirements.txt

   * Execution: Run the primary script to connect to the simulator:

python main.py


------------------------------
## System Requirements
The simulator environment has been successfully tested on Windows 11 with a GeForce RTX 3070. For stable performance, your system should meet or exceed the following hardware specifications:

| Requirement | Minimum Specification |
|---|---|
| OS | 64-bit Windows 10 / 11 |
| Processor | Intel Core i7 4770k (or AMD equivalent) |
| Memory | 8 GB RAM |
| Graphics | NVIDIA GeForce GTX 970 |
| Network | Broadband Internet connection |
| Storage | 12 GB available space |

------------------------------
## Timeline & Structure

* Virtual Qualifier Round 1: Simple, high-contrast, desaturated gate environment to test core flight logic.
* Virtual Qualifier Round 2: High-fidelity, visually complex 3D-scanned environments.
* Physical Qualifier (September 2026): Top teams advance to a live, indoor testing phase in Southern California.
* The Finals (November 2026): The premier AI Grand Prix live event in Ohio.

------------------------------
## Technical Specification & More Information
Can be found here:

https://www.theaigrandprix.com/previousupdates/