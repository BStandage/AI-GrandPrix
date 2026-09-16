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