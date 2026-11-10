#!/usr/bin/env bash
set -euo pipefail

WORK=/work/agilex/bunker-pro
/usr/bin/bash /deploy/host-can-bootstrap.sh
export BUNKER_CAN_INTERFACE=$(/usr/bin/bash /deploy/bunker-can.sh interface)
source /opt/ros/humble/setup.bash
exec python3 "$WORK/main.py"
