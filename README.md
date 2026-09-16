# Race day: what to run, and for what

**The honest state.** The only thing that can be commanded to fly today is
the simulator. The on-drone runtime is being built in `src/hardware/`:
the flight-controller link exists (`hardware.msp`, `hardware.bridge`,
`hardware.bench`: MSP over the Orin's UART or the sim SITL's TCP port,
RC out at 50 Hz, attitude/IMU/altitude/battery in, stale-command and
link-outage disarm rules, tested against a fake FC). Still missing: the
ANGLE-mode output for the follower, the estimator, and the runtime loop
that ties camera, estimator, follower and bridge together. When it
exists it takes the same two inputs as the sim: a plan JSON and a
vehicle toml.

**Which branch.** All current work is on `chore/dead-code` (it contains
`feature/pubmap-29s`, the two-lap stack on the published map, and the
docs branch) until those merge into `main`. In the sim repo, `main` is
the validated sim on Betaflight 2026.6.0; `feature/betaflight-4.5` is the
Archer's firmware generation and is validated when its step tests and
one race are clean.

**Which plan.** `out/plans/plan_RACE.json` is the race plan, always. It is
the only plan the launchers pick up by default. The speed-ladder rungs are
`out/plans/plan_LADDER_<T>s.json`, each with its own toml
`config/ladder/vehicle_<T>s.toml`. Numbered plans `plan_NNN.json` are
candidates; promote one by copying it over `plan_RACE.json` and `.png`.

| I want to... | Run (from the repo shown) | What it flies |
|---|---|---|
| Fly the race plan and WATCH it | sim repo: `run_race_docker.cmd` | `plan_RACE.json` (newest `plan_*.json` if it is missing), the toml recorded in the plan, 2 laps, `solvers.follower`. Opens the editor. |
| Fly a specific plan headless (a ladder rung, a candidate, several in a row) | `cd src` then `python -m raceline.batch_fly ../out/plans/plan_LADDER_60s.json` | The plan you name, with the toml it records (`--config <toml>` overrides). Prints the referee table: gates, contacts, time. |
| Same, from WSL without Docker | `python race.py --traj out/plans/plan_RACE.json` (add `--config config/ladder/vehicle_60s.toml` for a rung) | The plan you name. **Without `--traj` race.py REPLANS from the toml and flies a new numbered plan, not the race plan.** |
| Build the ladder rungs | `cd src` then `python -m raceline.ladder --targets 60 50 40 35`; a midpoint: `--k 0.4` | Writes the rung tomls and plans. Model times, centred crossings, zero contacts. |
| Make a new race-plan candidate | `python race.py --plan-only` | Writes `out/plans/plan_NNN.json` + `.png` and prints the CHECK lines. Frame contacts must be 0. |
| Read a run | `out/flightlogs/race_NNN.csv` (100 Hz trace) and the sim repo's `race_result_NNN.json` | Gates scored, first frame contact, lap times. |
| Fly the VISION SEEKER in the sim (no position, no plan: camera + heading + baro) | `cd src` then `python -m raceline.batch_fly --solver solvers.seeker --angle ../out/plans/plan_RACE.json` | The published course gate by gate on the HSV detector in ANGLE mode; the plan only sets the lap count. Trace in `out/flightlogs/seeker_NNN.csv`. This is the race-day pilot. |
| Fly a PLAN in the sim in ANGLE mode (the hardware control shape) | `cd src` then `python -m raceline.batch_fly --angle ../out/plans/plan_LADDER_60s.json` | Tilt-angle sticks, the FC levels itself; needed before any plan is trusted on the drone. |
| Fly the seeker on the ARCHER | on the Orin, `cd src` then `python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north <deg> --dry-run`, then `--arm` | Camera -> detector -> seeker -> MSP. `--map-north` = compass heading of the map's +y (point the drone along gate 1 on the start line and read the bench telemetry). Dry run streams disarmed sticks and prints what it would do. |
| Talk to a flight controller (the sim's SITL) | sim running, then `cd src` and `python -m hardware.bench --tcp 127.0.0.1:5761 info` (or `telemetry --hz 20`) | Firmware identity, arming blockers, attitude at the link rate. Read-only; the race keeps flying. |
| Talk to the Archer's flight controller (bench, props OFF) | on the Orin: `python3 -m hardware.bench --port /dev/ttyTHS1 info`, then `rc-test --props-off`, then `arm-test --props-off` | Proves the RC path end to end (the FC echoes our sticks back), then arms with throttle at minimum and disarms. Nothing raises the throttle. |

**The order on the day** (`src/PQ_PROCEDURE.md`): fly the slowest rung
(60 s). Clean twice, fly the fastest rung you brought. Fails, fly the
midpoint (`--k`). Only after the 35 s rung has held twice does
`plan_RACE.json` get a slot. A complete slow run outranks every incomplete
fast one.

---

> ## Team members: start here
>
> Everything below this box is the organizers' original dev-kit readme
> (the OLD Windows sim era). Our current stack races in the **elodin sim**
> on the organizers' **published** September course.
>
> **Where things stand (2026-09-15):** `data/course_map.json` is the
> organizer's published gate table (10 gates, 85 x 165 ft, gate 9 is the
> double; code labels are traversal order, g0 = gate 1, g8 = the double).
> The sim flies it clean: 23/23 crossings in 29.55 s over two laps
> (`out/plans/plan_RACE.json`). A speed ladder (`raceline.ladder`) builds
> slower, centred-crossing plans for race-day binary search. The Archer's
> flight controller is Betaflight 4.5.x; the sim's SITL is being pinned to
> 4.5.5 (sim branch `feature/betaflight-4.5`). Nothing flies on hardware
> yet: the MSP bridge, the ANGLE-mode output and the estimator are the
> open work.
>
> | You want to... | Go to |
> |---|---|
> | Just WATCH a flight, zero setup | Docker Desktop + `run_race_docker.cmd` in the sim repo (nothing else needed) |
> | Get set up and fly your first race | `docs/GETTING_STARTED_RACING_LINE.md` |
> | Understand drones/Betaflight from zero | `docs/WHAT_IS_BETAFLIGHT.md` |
> | Write or tune a solver | `src/solvers/README.md` |
> | Understand the stack's design | `docs/RACING_LINE_STACK.md` |
> | Know what's banned before planning | `RESTRICTIONS.md` (read it first) |
> | September physical-race facts, the published course, the Orin board | `src/PQ_SPECS_INTAKE.md` |
> | Day-0 / day-1 procedure and the race-day binary search | `src/PQ_PROCEDURE.md` |
> | The course in 3D | `viz/course_viewer.html` (keep private until after the qualifier) |
>
> Tune ONLY `config/vehicle.toml`. Race with `race.cmd` (or `python race.py`
> from the repo root). The `raceline.*` modules run from `src/`:
> `cd src && python -m raceline.batch_fly ../out/plans/plan_RACE.json`.
> The tape-era code (pilots, tape tools, MAVLink comms, the old runtime)
> and the overhead-image map extractor were removed on 2026-09-15; git
> history has them. Plans made on the pre-publication course estimate are
> archived under `out/plans/archive_20260915_estimate_map/`. Organizer
> spec PDFs live in `docs/specs/`.

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