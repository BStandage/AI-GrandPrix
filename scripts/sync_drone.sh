#!/usr/bin/env bash
# Push everything a drone needs to fly. Run from the repo root, on the LAPTOP.
#
#   scripts/sync_drone.sh d45
#   scripts/sync_drone.sh d45 cam20_75        # also push that camera's rungs
#
# Only ~3 MB: the six source packages, config, data, and the plans. Not
# src/datasets (869 MB of old recordings) and not the sim repo.
#
# Set up key auth first or it asks for the password on every file:
#   ssh-copy-id d45
set -euo pipefail

HOST="${1:-}"
CAM="${2:-cam20_75}"
if [ -z "$HOST" ]; then
    echo "usage: scripts/sync_drone.sh <ssh-host> [camera-tag]"; exit 2
fi
cd "$(dirname "$0")/.."

echo "== code -> $HOST"
ssh "$HOST" "mkdir -p ~/AI-GrandPrix/src ~/AI-GrandPrix/config/ladder ~/AI-GrandPrix/out/plans"
tar czf - --exclude='__pycache__' --exclude='*.pyc' \
    src/hardware src/perception src/raceline src/seeker src/solvers src/common \
    config/*.toml data \
  | ssh "$HOST" "tar xzf - -C ~/AI-GrandPrix"

echo "== rungs and plans ($CAM) -> $HOST"
shopt -s nullglob
TOMLS=(config/ladder/vehicle_k*_"$CAM".toml)
PLANS=(out/plans/plan_LADDER_k*_"$CAM".json)
# One compressed stream, not one scp connection per file. The plans are a few
# hundred KB each and scp died mid-transfer repeatedly on the venue WiFi
# (d45, 2026-09-20); tar over a single ssh session survives it and sends far
# fewer bytes, because the plan JSON compresses about 4:1.
if [ ${#TOMLS[@]} -eq 0 ] && [ ${#PLANS[@]} -eq 0 ]; then
    echo "   nothing matches *_$CAM.*"
else
    tar czf - "${TOMLS[@]}" "${PLANS[@]}"       | ssh "$HOST" "tar xzf - -C ~/AI-GrandPrix"
    echo "   ${#TOMLS[@]} rung configs, ${#PLANS[@]} plans"
fi

echo "== checking it landed"
ssh "$HOST" "cd ~/AI-GrandPrix/src && \
    python3 -c \"import numpy, cv2, serial; print('  cv2', cv2.__version__)\" && \
    python3 -c \"import tomllib\" 2>/dev/null || python3 -c \"import tomli; print('  tomli ok (python 3.10)')\" ; \
    echo '  configs:' \$(ls ~/AI-GrandPrix/config/ladder/*.toml 2>/dev/null | wc -l) ; \
    echo '  plans:  ' \$(ls ~/AI-GrandPrix/out/plans/*.json 2>/dev/null | wc -l) ; \
    du -sh ~/AI-GrandPrix | sed 's/^/  size: /'"

echo "== done"
