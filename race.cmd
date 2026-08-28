@echo off
REM ==================================================================
REM  Racing-line race, double-clickable: plan -> fly headless -> report.
REM  Thin wrapper around race.py running in WSL (where the sim lives).
REM
REM  Any race.py flags pass through:
REM    race.cmd --plan-only
REM    race.cmd --traj out\plans\plan_002.json
REM  Double-click = full plan+fly+report with the default config.
REM ==================================================================
setlocal
cd /d "%~dp0"

REM bash would eat backslashes in paths like out\plans\plan_002.json
set "ARGS=%*"
if defined ARGS set "ARGS=%ARGS:\=/%"

wsl -e bash -lc "cd $(wslpath -a '%~dp0')../elodin-sim-aigp && ~/.local/bin/uv run python $(wslpath -a '%~dp0')race.py %ARGS%"

REM keep the window open when double-clicked (no args = likely a double-click)
if "%~1"=="" (
  echo.
  pause
)
endlocal
