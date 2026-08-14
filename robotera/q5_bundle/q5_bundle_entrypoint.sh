#!/usr/bin/env bash
# Run the Q5 vendor-facing driver. The driver itself starts the verified
# multiprocessing media/audio bridge on Domain 42; do not also start the old
# polling String bridge, since it would claim the same camera/audio topics
# with incompatible ROS message types.
# ROS setup scripts intentionally read optional variables that may be unset;
# nounset would abort while sourcing /opt/ros/humble/setup.bash.
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
if [[ -f /opt/teleop_client/install/setup.bash ]]; then
  source /opt/teleop_client/install/setup.bash
fi

driver_uri="${Q5_CYCLONEDDS_URI:-${CYCLONEDDS_URI:-}}"

(
  export ROS_DOMAIN_ID="${Q5_ROS_DOMAIN_ID:-211}"
  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  if [[ -n "${driver_uri}" ]]; then
    export CYCLONEDDS_URI="${driver_uri}"
  else
    unset CYCLONEDDS_URI
  fi
  exec python3 /work/main.py
) &
driver_pid=$!

shutdown() {
  trap - TERM INT EXIT
  kill -TERM "$driver_pid" 2>/dev/null || true
  wait "$driver_pid" 2>/dev/null || true
}

trap shutdown TERM INT EXIT

# The media bridge is a child of main.py and is shut down with it.
wait "$driver_pid"
exit_code=$?
exit "$exit_code"
