#!/usr/bin/env bash
set -euo pipefail

WORK=/work/agilex/bunker-mini
/usr/bin/bash /deploy/host-can-bootstrap.sh
export BUNKER_CAN_INTERFACE=$(/usr/bin/bash /deploy/bunker-can.sh interface)
set +u
source /opt/ros/humble/setup.bash
set -u
# The persistent driver only needs raw SocketCAN access. Drop the bootstrap
# capabilities and host-root identity before starting the long-running process.
export HOME=/tmp
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/setpriv --reuid=65534 --regid=65534 --clear-groups \
    --bounding-set=-all,+net_raw --inh-caps=+net_raw --ambient-caps=+net_raw \
    -- python3 "$WORK/main.py"
