#!/usr/bin/env bash
# Pull the newest flight off a drone and triage it. Run from the repo root, on
# the LAPTOP, straight after landing.
#
#   scripts/pull_flight.sh d44               # newest flight, no video
#   scripts/pull_flight.sh d44 --video       # ...and the .avi (1-2 GB, slow)
#   scripts/pull_flight.sh d44 --nth 2       # the one before the newest
#
# Lands in flightlogs/<today>/ and then prints the triage: which of the known
# failure signatures fired, with the numbers behind each one. Read that before
# you read anything else.
#
# scp one file at a time was what made this slow enough to skip after a flight,
# and a debrief you skip is a debrief you do not have. One tar over one ssh
# session, same as sync_drone.sh in the other direction.
set -euo pipefail

HOST="${1:-}"
WANT_VIDEO=0
NTH=1
shift || true
while [ $# -gt 0 ]; do
    case "$1" in
        --video) WANT_VIDEO=1 ;;
        --nth)   NTH="$2"; shift ;;
        *) echo "unknown option $1"; exit 2 ;;
    esac
    shift
done
if [ -z "$HOST" ]; then
    echo "usage: scripts/pull_flight.sh <ssh-host> [--video] [--nth N]"; exit 2
fi
cd "$(dirname "$0")/.."

REMOTE=~/AI-GrandPrix/out/flightlogs
DEST="flightlogs/$(date +%Y-%m-%d)"
mkdir -p "$DEST"

# The run is named by its stamp: hw_<pilot>_<stamp>.{csv,log,avi,avi.idx}. Find
# the Nth newest CSV and take everything that shares its stem, plus the
# race_NNN.csv the follower wrote in the same run (newest, same reasoning).
STEM=$(ssh "$HOST" "ls -1t $REMOTE/hw_*.csv 2>/dev/null | sed -n '${NTH}p' | xargs -r basename | sed 's/\.csv\$//'")
if [ -z "$STEM" ]; then
    echo "no hw_*.csv found in $REMOTE on $HOST"; exit 1
fi
RACE=$(ssh "$HOST" "ls -1t $REMOTE/race_*.csv 2>/dev/null | sed -n '${NTH}p' | xargs -r basename || true")

echo "== flight $STEM${RACE:+ (+ $RACE)} -> $DEST"

FILES="$STEM.csv"
for extra in "$STEM.log" "$STEM.avi.idx" "$RACE"; do
    [ -n "$extra" ] && FILES="$FILES $extra"
done
# -h so a missing file is a warning, not a failure: half a debrief beats none
ssh "$HOST" "cd $REMOTE && tar czf - --ignore-failed-read $FILES" | tar xzf - -C "$DEST"

if [ "$WANT_VIDEO" = "1" ]; then
    echo "== video (this is the slow part)"
    ssh "$HOST" "cd $REMOTE && tar cf - --ignore-failed-read $STEM.avi" | tar xf - -C "$DEST"
else
    echo "   video left on the drone. To fetch it:"
    echo "     scripts/pull_flight.sh $HOST --video"
fi

echo
ls -la "$DEST/$STEM."* 2>/dev/null || true
echo
if [ -s "$DEST/$STEM.log" ]; then
    echo "== what it thought it was doing"
    cat "$DEST/$STEM.log"
    echo
fi
echo "== triage"
python -m raceline.triage "$DEST/$STEM.csv" ${RACE:+"$DEST/$RACE"} || \
    (cd src && python -m raceline.triage "../$DEST/$STEM.csv" ${RACE:+"../$DEST/$RACE"})
