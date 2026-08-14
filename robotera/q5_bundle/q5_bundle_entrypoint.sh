#!/usr/bin/env bash
# Run the Q5 vendor-facing driver and Agent Core's JSON sensor bridge. The
# driver starts a separate typed media/audio bridge; q5_bus_bridge forwards
# only data/json topics, so it cannot claim camera/audio DDS topic types.
# ROS setup scripts intentionally read optional variables that may be unset;
# nounset would abort while sourcing /opt/ros/humble/setup.bash.
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
if [[ -f /q5_ws/install/setup.bash ]]; then
  source /q5_ws/install/setup.bash
fi
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

(
  export ROS_DOMAIN_ID="${AGENT_CORE_ROS_DOMAIN_ID:-42}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  export Q5_DRIVER_URL="${Q5_DRIVER_URL:-http://127.0.0.1:15793/mcp}"
  # Do not let the Q5 CycloneDDS configuration affect Fast DDS discovery.
  unset CYCLONEDDS_URI
  exec python3 /work/q5_bus_bridge.py
) &
bridge_pid=$!

shutdown() {
  trap - TERM INT EXIT
  kill -TERM "$bridge_pid" "$driver_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
  wait "$driver_pid" 2>/dev/null || true
}

trap shutdown TERM INT EXIT

# The media bridge is a child of main.py and is shut down with it.
wait -n "$driver_pid" "$bridge_pid"
exit_code=$?
exit "$exit_code"
