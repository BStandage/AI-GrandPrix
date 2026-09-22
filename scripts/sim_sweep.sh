#!/usr/bin/env bash
# Mass test: fly the race build through the Betaflight sim N times with different
# synthetic-camera seeds, and tabulate what the referee and the follower said.
#
#   scripts/sim_sweep.sh 5                        # 5 seeds, race-day defaults
#   AIGP_BARO_PATHOLOGY=0 scripts/sim_sweep.sh 5  # ...with the d43-style barometer
#   AIGP_COMMIT_HOLD=1 scripts/sim_sweep.sh 5     # ...with the throttle hold after commit
#
# Each run is ~5 min on a laptop (the sim runs at ~0.5x real time). Results land
# in out/flightlogs/sweep_<stamp>.txt and print as they finish. Run from the repo
# root on the laptop with Docker up and ../elodin-sim-aigp next door.
set -uo pipefail
N="${1:-3}"
PLAN="${AIGP_SWEEP_PLAN:-out/plans/plan_FLAT_s15_cam20_75.json}"
CFG="${AIGP_SWEEP_CFG:-config/ladder/vehicle_s15_cam20_75.toml}"
cd "$(dirname "$0")/.."
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="out/flightlogs/sweep_${STAMP}.txt"
export AIGP_LAPS="${AIGP_LAPS:-2}" AIGP_VERT="${AIGP_VERT:-vision}" AIGP_STATE_SOURCE="${AIGP_STATE_SOURCE:-deadreckon}"
export AIGP_CAM_TILT_DEG="${AIGP_CAM_TILT_DEG:-10}" AIGP_CAM_HFOV_DEG="${AIGP_CAM_HFOV_DEG:-75}"
{
  echo "sweep $STAMP  plan=$PLAN  cfg=$CFG  N=$N"
  echo "env: VERT=$AIGP_VERT BARO_PATHOLOGY=${AIGP_BARO_PATHOLOGY:-1} COMMIT_HOLD=${AIGP_COMMIT_HOLD:-0} COMMIT_STRAIGHT=${AIGP_COMMIT_STRAIGHT:-1} SENSOR_NOISE=${AIGP_SENSOR_NOISE:-1}"
  printf "%-5s %-8s %-10s %-8s %s\n" seed passed status crash "commits / crossings (follower)"
} | tee "$OUT"
for ((i=0; i<N; i++)); do
  export AIGP_SEED=$i
  (cd ../elodin-sim-aigp && docker compose down --remove-orphans >/dev/null 2>&1)
  (cd src && python -m raceline.batch_fly "../$PLAN" --angle --config "../$CFG" --timeout 330 >/dev/null 2>&1)
  LOG=$(ls -t out/flightlogs/container_*.log | head -1)
  RACE=$(grep -m1 -o "gates_passed=[0-9]*/[0-9]* .*status=[A-Z]*" "$LOG" | sed 's/total_time=//; s/lap_times=\[[^]]*\] //')
  PASSED=$(echo "$RACE" | grep -o "gates_passed=[0-9]*" | cut -d= -f2)
  STATUS=$(echo "$RACE" | grep -o "status=[A-Z]*" | cut -d= -f2)
  CRASH=$(grep -m1 -o "hit [a-z0-9-]* frame at t=[0-9.]*s" "$LOG" | sed 's/ frame at t=/@/; s/hit //')
  EVENTS=$(grep -E "\[SIM\] (COMMIT|CROSSED)" "$LOG" | sed -E 's/.*COMMIT g([0-9]+) at det range ([0-9.]+) m \(t=([0-9.]+)\)/C\1@\2m/; s/.*CROSSED g([0-9]+) -> next g[0-9]+ \(t=([0-9.]+), lat ([+-][0-9.]+) m\)/X\1(\3)/' | tr '\n' ' ')
  printf "%-5s %-8s %-10s %-8s %s\n" "$i" "${PASSED:-?}" "${STATUS:-?}" "${CRASH:--}" "$EVENTS" | tee -a "$OUT"
done
echo "-> $OUT"
