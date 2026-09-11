#!/usr/bin/env bash
# ROS Humble's generated setup files probe variables that may be unset. Enable
# nounset only after all setup files have been sourced.
set -Ee -o pipefail

source /opt/ros/humble/setup.bash
if [[ -f /ros_ws/install/setup.bash ]]; then
  source /ros_ws/install/setup.bash
fi
source /aimdk_x2_ws/install/setup.bash
set -u

socket_path="${X2_BRIDGE_SOCKET:-/tmp/agibot_x2_bridge/bridge_main.sock}"
robot_profile="${ROBOT_FASTRTPS_DEFAULT_PROFILES_FILE:-}"
core_profile="${CORE_FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/phanthy-motus/dds-local.xml}"

if [[ -z "${robot_profile}" ]]; then
  robot_profile="/tmp/agibot_x2_bridge/fastdds_robot.xml"
  profile_args=(
    --interface "${NETWORK_INTERFACE:-eth0}"
    --output "${robot_profile}"
  )
  if [[ -n "${ROBOT_INTERFACE_IP:-}" ]]; then
    profile_args+=(--address "${ROBOT_INTERFACE_IP}")
  fi
  python3 /work/agibot/AimDK_X2/x2_fastdds_profile.py "${profile_args[@]}"
fi

if [[ ! -f "${core_profile}" ]]; then
  core_profile="/work/agibot/AimDK_X2/resource/fastdds_bridge_local.xml"
fi

(
  export ROS_DOMAIN_ID="${CORE_DOMAIN_ID:-42}"
  export CORE_DOMAIN_ID="${CORE_DOMAIN_ID:-42}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${core_profile}"
  export FASTDDS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"
  export ROS_LOCALHOST_ONLY=0
  export X2_BRIDGE_SOCKET="${socket_path}"
  exec python3 /work/agibot/AimDK_X2/x2_socket_bridge.py
) &
bridge_pid=$!

for _ in {1..50}; do
  [[ -S "${socket_path}" ]] && break
  kill -0 "${bridge_pid}" 2>/dev/null || wait "${bridge_pid}"
  sleep 0.1
done
[[ -S "${socket_path}" ]] || { echo "[x2-entrypoint] socket bridge did not become ready" >&2; exit 1; }

(
  export ROS_DOMAIN_ID="${ROBOT_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${robot_profile}"
  export FASTDDS_DEFAULT_PROFILES_FILE="${robot_profile}"
  export ROS_LOCALHOST_ONLY=0
  export X2_BRIDGE_SOCKET="${socket_path}"
  exec python3 /work/agibot/AimDK_X2/main.py
) &
driver_pid=$!

shutdown() {
  trap - TERM INT EXIT
  kill -TERM "${driver_pid}" "${bridge_pid}" 2>/dev/null || true
  wait "${driver_pid}" 2>/dev/null || true
  wait "${bridge_pid}" 2>/dev/null || true
}

trap shutdown TERM INT EXIT
wait -n "${driver_pid}" "${bridge_pid}"
exit_code=$?
shutdown
exit "${exit_code}"
