#!/usr/bin/env bash
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
source /aimdk_x2_ws/install/setup.bash

robot_interface="${NETWORK_INTERFACE:-develop0}"
robot_profile_template="${ROBOT_FASTRTPS_DEFAULT_PROFILES_FILE:-/work/agibot/AimDK_X2/resource/fastdds_develop0.xml}"
robot_interface_ipv4="${ROBOT_INTERFACE_IPV4:-}"
if [ -z "$robot_interface_ipv4" ]; then
  command -v ip >/dev/null 2>&1 || {
    echo "[x2-entrypoint] 'ip' is required to discover ${robot_interface}'s IPv4 address" >&2
    exit 1
  }
  robot_interface_ipv4="$(ip -4 -o addr show dev "$robot_interface" scope global | awk 'NR == 1 {split($4, parts, "/"); print parts[1]}')"
fi
if ! printf '%s\n' "$robot_interface_ipv4" | awk -F. 'NF == 4 && $1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+$/ && $3 ~ /^[0-9]+$/ && $4 ~ /^[0-9]+$/ && $1 <= 255 && $2 <= 255 && $3 <= 255 && $4 <= 255 {exit 0} {exit 1}'; then
  echo "[x2-entrypoint] no usable IPv4 address for ${robot_interface}; set ROBOT_INTERFACE_IPV4 to override" >&2
  exit 1
fi
robot_profile_dir="/tmp/agibot_x2_bridge"
robot_profile="${robot_profile_dir}/fastdds_robot.xml"
mkdir -p "$robot_profile_dir"
grep -q '__ROBOT_INTERFACE_IPV4__' "$robot_profile_template" || {
  echo "[x2-entrypoint] FastDDS template has no robot IP placeholder: ${robot_profile_template}" >&2
  exit 1
}
sed "s/__ROBOT_INTERFACE_IPV4__/${robot_interface_ipv4}/g" "$robot_profile_template" > "$robot_profile"

(
  export ROS_DOMAIN_ID="${ROBOT_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export NETWORK_INTERFACE="$robot_interface"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$robot_profile"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  exec python3 /work/agibot/AimDK_X2/main.py
) &
driver_pid=$!

(
  export ROS_DOMAIN_ID="${CORE_ROS_DOMAIN_ID:-42}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${CORE_FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/phanthy-motus/dds-local.xml}"
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
