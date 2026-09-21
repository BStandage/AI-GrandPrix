#!/usr/bin/env bash
# Put each aircraft's name up when you ssh in, so nobody flies the wrong drone.
#
#   scripts/install_motd.sh d44 sally
#   scripts/install_motd.sh d45 randy
#
# Two drones, identical hardware, both reporting hostname `dcl-orin`. The
# serial number is the only physical label and it is on the underside. This is
# cheaper than turning a drone over.
set -euo pipefail
HOST="${1:-}"; NAME="${2:-}"
if [ -z "$HOST" ] || [ -z "$NAME" ]; then
    echo "usage: scripts/install_motd.sh <ssh-host> <sally|randy>"; exit 2
fi
cd "$(dirname "$0")/.."
ART="scripts/motd/${NAME}.txt"
[ -f "$ART" ] || { echo "no art at $ART"; exit 2; }
# /etc/update-motd.d needs sudo; ~/.bashrc does not, and works over ssh
scp -q "$ART" "$HOST:~/.aircraft-name"
ssh "$HOST" "grep -q aircraft-name ~/.bashrc || \
    printf '\n# which drone am I\n[ -f ~/.aircraft-name ] && cat ~/.aircraft-name\n' >> ~/.bashrc"
echo "== $NAME installed on $HOST - ssh in to see it"
