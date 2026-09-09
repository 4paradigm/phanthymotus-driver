#!/usr/bin/env bash
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
source /aimdk_x2_ws/install/setup.bash

(
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export NETWORK_INTERFACE="${NETWORK_INTERFACE:-develop0}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/work/agibot/AimDK_X2/resource/fastdds_develop0.xml}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  exec python3 /work/agibot/AimDK_X2/main.py
) &
driver_pid=$!

(
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/phanthy-motus/dds-local.xml}"
  export ROS_LOCALHOST_ONLY=0
  exec python3 /work/agibot/AimDK_X2/x2_socket_bridge.py
) &
bridge_pid=$!

shutdown() {
  trap - TERM INT EXIT
  kill -TERM "$bridge_pid" "$driver_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
  wait "$driver_pid" 2>/dev/null || true
}

trap shutdown TERM INT EXIT
wait -n "$driver_pid" "$bridge_pid"
exit_code=$?
shutdown
exit "$exit_code"
