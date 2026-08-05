#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REMOTE_MAP_ROOT="/home/unitree/phanthy-navigation-maps"
readonly SSH_OPTS=(
  -o ClearAllForwardings=yes
  -o ControlMaster=no
  -o ControlPath=none
  -o BatchMode=yes
  -o ConnectTimeout=8
)

usage() {
  echo "Usage: $0 <preflight|lio|rgb|map> <ssh-target> [ros-namespace] [network-interface]" >&2
  echo "Examples:" >&2
  echo "  $0 preflight g1-sh-wifi ubuntu eth0" >&2
  echo "  $0 lio g1-sh-wifi ubuntu eth0" >&2
  echo "  $0 rgb g1-sh-wifi ubuntu eth0" >&2
  echo "  G1_MAP_NAME=sh_n3_smoke $0 map g1-sh-wifi ubuntu eth0" >&2
}

if (( $# < 2 || $# > 4 )); then
  usage
  exit 2
fi

readonly MODE="$1"
readonly SSH_TARGET="$2"
readonly ROS_NAMESPACE_VALUE="${3:-ubuntu}"
readonly NETWORK_INTERFACE_VALUE="${4:-eth0}"

case "${MODE}" in
  preflight|lio|rgb|map) ;;
  *) usage; exit 2 ;;
esac

[[ "${SSH_TARGET}" =~ ^[a-zA-Z0-9_.@-]+$ ]] || {
  echo "invalid ssh target: ${SSH_TARGET}" >&2
  exit 2
}
[[ "${ROS_NAMESPACE_VALUE}" =~ ^[a-zA-Z0-9_]+$ ]] || {
  echo "invalid ROS namespace: ${ROS_NAMESPACE_VALUE}" >&2
  exit 2
}
[[ "${NETWORK_INTERFACE_VALUE}" =~ ^[a-zA-Z0-9_.:-]+$ ]] || {
  echo "invalid network interface: ${NETWORK_INTERFACE_VALUE}" >&2
  exit 2
}

MAP_NAME="${G1_MAP_NAME:-}"
if [[ "${MODE}" == map ]]; then
  [[ "${MAP_NAME}" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}$ ]] || {
    echo "G1_MAP_NAME must match [a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}" >&2
    exit 2
  }
fi
readonly MAP_NAME
readonly REMOTE_MAP_DIR="${REMOTE_MAP_ROOT}/${MAP_NAME}"

run_preflight() {
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test \"\$(uname -m)\" = aarch64
    test \"\$(timedatectl show -p NTPSynchronized --value)\" = yes
    echo 'acceptance=preflight target=${SSH_TARGET}'
    echo \"time=\$(date -Iseconds) uptime=\$(uptime -p) boot_id=\$(cat /proc/sys/kernel/random/boot_id) ntp=yes\"
    ip -br link show '${NETWORK_INTERFACE_VALUE}'
    docker compose version
    docker inspect embodied-unitree-g1 \
      --format 'main={{.Name}} image={{.Config.Image}} running={{.State.Running}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} started={{.State.StartedAt}} restarts={{.RestartCount}} oom={{.State.OOMKilled}}'
    for container in phanthy-navigation-sensors-shadow phanthy-fast-livo2-shadow; do
      if docker inspect \"\${container}\" >/dev/null 2>&1; then
        docker inspect \"\${container}\" \
          --format 'shadow={{.Name}} image={{.Config.Image}} running={{.State.Running}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} read_only={{.HostConfig.ReadonlyRootfs}} privileged={{.HostConfig.Privileged}} capdrop={{json .HostConfig.CapDrop}} devices={{json .HostConfig.Devices}} restarts={{.RestartCount}} oom={{.State.OOMKilled}} mode={{index .Config.Labels \"com.phanthy.navigation.mode\"}} map_name={{index .Config.Labels \"com.phanthy.navigation.map_name\"}}'
      else
        echo \"shadow=\${container} state=absent\"
      fi
    done
    df -h /home/unitree
  "
}

if [[ "${MODE}" == preflight ]]; then
  run_preflight
  echo "PREFLIGHT ACCEPTANCE COMPLETE: read-only checks only"
  exit 0
fi

if [[ "${MODE}" == lio ]]; then
  run_preflight
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{.State.Running}}')\" = true
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{.State.Health.Status}}')\" = healthy
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.Running}}')\" = true
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.Health.Status}}')\" = healthy
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{.HostConfig.ReadonlyRootfs}}')\" = true
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.HostConfig.ReadonlyRootfs}}')\" = true
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{.HostConfig.Privileged}}')\" = false
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.HostConfig.Privileged}}')\" = false
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{json .HostConfig.CapDrop}}')\" = '[\"ALL\"]'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{json .HostConfig.CapDrop}}')\" = '[\"ALL\"]'
    sensor_devices=\"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{json .HostConfig.Devices}}')\"
    livo_devices=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{json .HostConfig.Devices}}')\"
    test \"\${sensor_devices}\" = null || test \"\${sensor_devices}\" = '[]'
    test \"\${livo_devices}\" = null || test \"\${livo_devices}\" = '[]'
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{.RestartCount}}')\" -eq 0
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.RestartCount}}')\" -eq 0
    test \"\$(docker inspect phanthy-navigation-sensors-shadow --format '{{.State.OOMKilled}}')\" = false
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.OOMKilled}}')\" = false
    test \"\$(docker inspect embodied-unitree-g1 --format '{{.State.Running}}')\" = true

    sample_topic() {
      topic=\"\$1\"
      echo \"=== topic_hz \${topic} ===\"
      topic_output=\"\$(
        docker exec phanthy-fast-livo2-shadow /bin/bash -lc \
          \"source /opt/ros/humble/setup.bash && source /opt/fast_livo_ws/install/setup.bash && timeout --signal=INT 8 ros2 topic hz '\${topic}'\" \
          2>&1 || true
      )\"
      printf '%s\\n' \"\${topic_output}\"
      printf '%s\\n' \"\${topic_output}\" | grep -q 'average rate'
    }

    sample_topic '/${ROS_NAMESPACE_VALUE}/navigation/lidar_fast_livo'
    sample_topic '/${ROS_NAMESPACE_VALUE}/navigation/imu'
    sample_topic '/${ROS_NAMESPACE_VALUE}/navigation/odom'
    sample_topic '/${ROS_NAMESPACE_VALUE}/navigation/cloud_registered'

    echo '=== sensor_diagnostics ==='
    docker exec phanthy-fast-livo2-shadow /bin/bash -lc \
      \"source /opt/ros/humble/setup.bash && source /opt/fast_livo_ws/install/setup.bash && timeout 10 ros2 topic echo '/${ROS_NAMESPACE_VALUE}/navigation/sensor_diagnostics' --once\"

    echo '=== container_stats ==='
    docker stats --no-stream --format 'container={{.Name}} cpu={{.CPUPerc}} memory={{.MemUsage}} net={{.NetIO}} block={{.BlockIO}}' \
      phanthy-navigation-sensors-shadow phanthy-fast-livo2-shadow

    fast_logs=\"\$(docker logs --since 10m phanthy-fast-livo2-shadow 2>&1 || true)\"
    gap_recovery=\"\$(printf '%s\\n' \"\${fast_logs}\" | grep -c 'accepting current sample to resynchronize' || true)\"
    legacy_rejection=\"\$(printf '%s\\n' \"\${fast_logs}\" | grep -c 'imu time stamp Jumps' || true)\"
    sync_warning=\"\$(printf '%s\\n' \"\${fast_logs}\" | grep -c 'IMU and LiDAR not synced' || true)\"
    echo \"log_window=10m gap_recovery=\${gap_recovery} legacy_rejection=\${legacy_rejection} sync_warning=\${sync_warning}\"
    test \"\${legacy_rejection}\" -eq 0
  "
  echo "LIO ACCEPTANCE SAMPLE COMPLETE: no robot state changed"
  exit 0
fi

if [[ "${MODE}" == rgb ]]; then
  run_preflight
  rgb_sensor_status=0
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set +e
    readiness_status=0
    blocker() {
      echo \"BLOCKER: \$*\"
      readiness_status=1
    }

    echo 'acceptance=rgb_sensor topic=/${ROS_NAMESPACE_VALUE}/camera/rgb'
    d435_usb=\"\$(lsusb | grep -i '8086:0b3a' | head -n 1)\"
    if test -n \"\${d435_usb}\"; then
      echo \"d435_usb=present device=\${d435_usb}\"
    else
      echo 'd435_usb=missing expected=8086:0b3a'
      blocker 'D435i 未在 USB 枚举；先检查相机供电和 USB 线'
    fi

    camera_command=\"source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash\"
    camera_type=\"\$(docker exec embodied-unitree-g1 /bin/bash -lc \
      \"\${camera_command} && ros2 topic type '/${ROS_NAMESPACE_VALUE}/camera/rgb'\" 2>/dev/null)\"
    echo \"camera_type=\${camera_type:-absent}\"
    if test \"\${camera_type}\" != 'sensor_msgs/msg/CompressedImage'; then
      blocker '相机 topic 必须是 sensor_msgs/msg/CompressedImage'
    fi

    camera_info=\"\$(docker exec embodied-unitree-g1 /bin/bash -lc \
      \"\${camera_command} && ros2 topic info -v '/${ROS_NAMESPACE_VALUE}/camera/rgb'\" 2>&1)\"
    printf '%s\\n' \"\${camera_info}\"
    if ! printf '%s\\n' \"\${camera_info}\" | grep -Eq '^Publisher count: [1-9][0-9]*$'; then
      blocker 'D435i RGB topic 当前没有 publisher'
    else
      camera_hz=\"\$(docker exec embodied-unitree-g1 /bin/bash -lc \
        \"\${camera_command} && timeout --signal=INT 10 ros2 topic hz '/${ROS_NAMESPACE_VALUE}/camera/rgb' --wall-time\" 2>&1)\"
      printf '%s\\n' \"\${camera_hz}\"
      camera_rate=\"\$(printf '%s\\n' \"\${camera_hz}\" | sed -n 's/^average rate: //p' | tail -n 1)\"
      if test -z \"\${camera_rate}\" || ! awk -v rate=\"\${camera_rate}\" 'BEGIN { exit !(rate >= 10.0) }'; then
        blocker 'D435i RGB 实测频率必须至少 10 Hz'
      else
        echo \"camera_rate_hz=\${camera_rate}\"
      fi
    fi

    if test -f /home/unitree/.sensor-collector/calibration.json; then
      echo 'calibration_snapshot=present path=/home/unitree/.sensor-collector/calibration.json'
    else
      echo 'calibration_snapshot=absent'
      blocker '缺少 sensor-collector calibration.json'
    fi
    echo \"rgb_sensor_ready=\$((readiness_status == 0))\"
    exit \"\${readiness_status}\"
  " || rgb_sensor_status=$?

  rgb_calibration_status=0
  PYTHONDONTWRITEBYTECODE=1 python3 "${SCRIPT_DIR}/render-g1-livo-config.py" \
    <(ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
      'cat /home/unitree/.sensor-collector/calibration.json') \
    || rgb_calibration_status=$?

  if (( rgb_sensor_status != 0 || rgb_calibration_status != 0 )); then
    echo "RGB-LIVO READINESS BLOCKED: sensor=${rgb_sensor_status} calibration=${rgb_calibration_status}" >&2
    exit 1
  fi
  echo "RGB-LIVO READINESS COMPLETE: read-only checks only"
  exit 0
fi

run_preflight
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
  set -e
  test -d '${REMOTE_MAP_DIR}'
  pcd_count=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c | wc -l)\"
  pcd_bytes=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c -printf '%s\\n' | awk '{ total += \$1 } END { print total + 0 }')\"
  echo 'acceptance=map map_name=${MAP_NAME} map_dir=${REMOTE_MAP_DIR}'
  echo \"pcd_count=\${pcd_count} pcd_bytes=\${pcd_bytes}\"
  test \"\${pcd_count}\" -ge 1

  while IFS= read -r pcd; do
    test -n \"\${pcd}\"
    echo \"=== pcd \${pcd} ===\"
    stat --printf='size=%s modified=%y\\n' \"\${pcd}\"
    pcd_header=\"\$(sed -n '1,/^DATA /p' \"\${pcd}\")\"
    printf '%s\\n' \"\${pcd_header}\"
    printf '%s\\n' \"\${pcd_header}\" | grep -Eq '^POINTS [1-9][0-9]*$'
    printf '%s\\n' \"\${pcd_header}\" | grep -Eq '^DATA (ascii|binary|binary_compressed)$'
  done < <(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c -print | sort)

  if test -f /tmp/nav_result.json; then
    echo '=== nav_result.json ==='
    cat /tmp/nav_result.json
  else
    echo 'nav_result=absent'
  fi
"
echo "MAP ARTIFACT ACCEPTANCE COMPLETE: read-only checks only"
