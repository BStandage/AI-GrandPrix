# Elodin PQ sim - full setup guide (Windows + WSL)

How to go from a bare Windows machine to watching a drone fly the published
PQ course in the Elodin simulator. Every step here was actually executed on
Brian's machine on 2026-08-27; the **Gotchas** notes are real failures we
hit and their fixes - read them before asking for help.

**Check first whether you need this at all**: the Docker path
(`run_race_docker.cmd` in the sim repo, or `docker compose up --build`)
gives you the identical sim with none of the steps below, on any OS. This
native WSL setup is for people iterating on the sim itself or who want
headless runs without Docker overhead.

Two repos are involved:

| repo | role |
|---|---|
| `AI-GrandPrix` (this one) | course map (`data/course_map.json`), map loader (`src/common/course_map.py`), extraction tooling |
| [`elodin-sim-aigp`](https://github.com/BStandage/elodin-sim-aigp) | the simulator (our fork of elodin-sys/ai-grand-prix): Elodin physics + Betaflight SITL + solver hook |

The sim repo imports this repo's loader at startup (`sim/pq_course.py` puts
`AI-GrandPrix/src` on `sys.path`), so they must live side by side.

---

## 1. WSL (Windows Subsystem for Linux)

The sim runs on Linux. In **PowerShell as admin**:

```powershell
wsl --install          # installs WSL2 + Ubuntu; reboot if asked
```

If WSL is already installed (`wsl -l -v` lists a distro), **still check
the kernel version**:

```bash
wsl uname -r
```

> **Gotcha (hard requirement):** the Elodin runtime uses io_uring, which
> needs Linux kernel **5.1+**. Old WSL installs ship kernel 4.19 and the
> sim dies at startup with
> `panicked at libs/stellarator/src/uring/mod.rs ... "Function not implemented"`.
> Fix from an **admin** PowerShell:
>
> ```powershell
> wsl --update --web-download
> wsl --shutdown
> ```

Set a Linux username/password when Ubuntu first opens - you'll need the
password for `sudo`.

## 2. Linux packages

Inside WSL (`wsl` from any terminal):

```bash
sudo apt update && sudo apt install -y build-essential libasound2t64 git-lfs curl
```

> **Gotcha:** if `apt install` throws `404 Not Found` errors, your package
> index is stale - run `sudo apt update` first (it's in the line above for
> exactly that reason).
>
> `libasound2t64` looks unrelated but is required: the Elodin CLI links
> against ALSA even for headless runs and dies with
> `libasound.so.2: cannot open shared object file` without it.

## 3. uv (Python manager), inside WSL

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Installs to `~/.local/bin/uv`. Open a new shell or `source ~/.local/bin/env`.

## 4. Clone the repos side by side

```bash
cd /mnt/c/Users/<you>/Documents/GitRepos     # or wherever
git clone https://github.com/bstandageusf/AI-GrandPrix.git AI-GrandPrix
git clone https://github.com/BStandage/elodin-sim-aigp.git elodin-sim-aigp
```

> **Gotcha:** the upstream sim repo's natural name `ai-grand-prix` differs
> from `AI-GrandPrix` only by case. Windows folders are case-insensitive by
> default, so the two would collide - our fork is named `elodin-sim-aigp`
> for that reason. Both repos look for each other as siblings (override
> with `AIGP_REPO` / `AIGP_SIM_REPO`).

## 5. Python env for the sim

```bash
cd elodin-sim
uv python pin 3.13     # already committed as .python-version if you got our fork
uv sync
```

> **Gotcha:** without the pin, uv may pick Python **3.14**, under which
> elodin 0.17.2 fails every `world.spawn` with `ValueError: Not spawnable`.
> 3.13 is required.

## 6. Betaflight SITL (the flight controller, built for desktop)

```bash
bash scripts/fetch_betaflight.sh     # git submodule init
bash scripts/build_betaflight.sh    # compiles obj/main/betaflight_SITL.elf
```

Needs the `build-essential` from step 2. `git status` showing
`M betaflight` afterwards is expected (the script enables lockstep sync in
the submodule's target.h).

**Firmware version matters.** The Archer's flight controller runs
Betaflight 4.5.x (the organizers ship configurator 10.10.0, the 4.5
configurator). The sim's submodule is pinned to tag 4.5.5 on the sim
branch `feature/betaflight-4.5` (2026-09-15; main still builds 2026.6.0
until that branch is validated and merged). After switching the submodule
version, delete `betaflight/obj/` and `eeprom.bin` so the build and the
config are regenerated - the Docker entrypoint does both when they are
missing.

## 7. Elodin CLI (runtime), inside WSL

```bash
bash scripts/install_elodin.sh
echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
elodin --version       # first run prints a welcome banner and exits; run again
```

## 8. Run the sim (headless)

```bash
cd elodin-sim
RACE_SOLVER=solver.pq_waypoints uv run -- elodin run sim/main.py
```

What you should see, in order:
- the **frame report** (map->sim transform, 11 crossings/lap, footprint OK),
- `[SITL] Bridge ready`,
- `[GATE] lap 0 g0 ...` lines as gates are crossed,
- a `[RACE]` summary + `race_result_XXX.json` written at the end.

`RACE_SOLVER` selects the autopilot module. `solver.pq_waypoints` flies the
extracted course's waypoints (2 laps incl. the double-gate out-and-back).
`solver.baseline` is the upstream demo (targets the OLD 3-gate course - it
will not score gates on the PQ course). A module living in
`AI-GrandPrix/src` (e.g. `pilots.my_pilot`) is also selectable because the
sim puts that tree on `sys.path`.

## 9. Visualize (Elodin editor, native Windows app)

The GUI editor runs on **Windows**, connecting to the sim in WSL over TCP.

One-time install, in PowerShell:

```powershell
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$dst = "$env:LOCALAPPDATA\Programs\elodin"; New-Item -ItemType Directory -Force $dst
Invoke-WebRequest https://github.com/elodin-sys/elodin/releases/download/v0.17.3/elodin-x86_64-pc-windows-msvc.zip -OutFile $env:TEMP\elodin.zip -UseBasicParsing
Expand-Archive $env:TEMP\elodin.zip -DestinationPath $dst -Force
```

Then, to watch a run:

1. **WSL terminal** - start the sim:
   `RACE_SOLVER=solver.pq_waypoints uv run -- elodin run sim/main.py`
2. **PowerShell** - connect the editor:
   `& "$env:LOCALAPPDATA\Programs\elodin\elodin.exe" editor 127.0.0.1:2240`

You get the 3D viewport (gates, cones, drone, chase cam), the drone's FPV
camera feed, and live telemetry graphs.

> If the editor can't connect, WSL's localhost forwarding may need mirrored
> networking: create `%USERPROFILE%\.wslconfig` containing
> `[wsl2]` / `networkingMode=mirrored`, then `wsl --shutdown` and retry.
>
> **Gotcha (Windows 10 only):** the editor crashes at startup with
> `panicked at ...ui/theme.rs ... "The system cannot find the file specified"`
> because it hard-codes `C:\Windows\Fonts\SegoeIcons.ttf` - a Windows 11
> font. Fix: download Microsoft's free Segoe Fluent Icons pack from
> `https://aka.ms/SegoeFluentIcons`, extract, and (as admin):
> `Copy-Item ".\Segoe Fluent Icons.ttf" "C:\Windows\Fonts\SegoeIcons.ttf"`
>
> Run the editor **from the sim repo directory** - it resolves its assets
> (`gate.glb` etc.) relative to the working directory.

## 10. Tests

```bash
uv run pytest                        # full suite (needs elodin)
python tests/test_pq_course.py       # course/tracker tests, no elodin needed
uv run python scripts/smoke_world.py # world-construction smoke, no Betaflight
uv run python scripts/render_pq_course.py  # out/course_layout.png top-down render
```

## Architecture in one paragraph

`sim/pq_course.py` loads `AI-GrandPrix/data/course_map.json` through
`common.course_map` (the only allowed map parser), applies the named
`MapToSim` transform (rotation 0; the drone spawn is the map's start
line, `meta.start`, 7.3 m behind gate g0 - maps without one fall back to
a 3 m standoff), and builds the 11-crossings-per-lap x 2-lap sequence
(published 10-gate map, 2026-09-15; 23 events including the start and
finish crossings of g0) - the stacked gate g8 (organizer gate 9) is two
crossings (top opening 4.05 m southbound, bottom 1.35 m northbound)
disambiguated by altitude. `sim/main.py` runs
Elodin physics and Betaflight SITL in lockstep at 1 kHz; each tick the
selected solver gets a `SensorUpdate` (IMU, baro, mag, 640x360 FPV frames,
race state) and returns an `RCCommand`. `RaceTracker` scores ordered
crossings host-side and writes `race_result_XXX.json`. Nothing anywhere
hardcodes gate positions - swap in official coordinates by replacing the
map JSON.
