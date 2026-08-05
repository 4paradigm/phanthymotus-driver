#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly NAV_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKER_CONTEXT="${LOCAL_DOCKER_CONTEXT:-desktop-linux}"
readonly FAST_LIVO_IMAGE="${1:-phanthy-fast-livo2:g1-1fcd0d0-n2gap1}"
readonly CONTAINER_NAME="fast-livo-imu-gap-smoke-$$"

cleanup() {
  docker --context "${DOCKER_CONTEXT}" stop --time 10 "${CONTAINER_NAME}" \
    >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker --context "${DOCKER_CONTEXT}" image inspect --platform linux/arm64 \
  "${FAST_LIVO_IMAGE}" >/dev/null

docker --context "${DOCKER_CONTEXT}" run -d --rm \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --read-only \
  --security-opt no-new-privileges:true \
  --cap-drop ALL \
  --tmpfs /tmp:rw,size=256m,mode=1777 \
  --tmpfs /opt/fast_livo_ws/src/fast_livo/Log:rw,size=256m,mode=1777 \
  -e ROS_DOMAIN_ID=92 \
  -e ROS_LOG_DIR=/tmp/ros-log \
  -v "${NAV_DIR}/g1_lio.yaml:/config/g1_lio.yaml:ro" \
  "${FAST_LIVO_IMAGE}" \
  /bin/bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/fast_livo_ws/install/setup.bash
    exec /opt/fast_livo_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping \
      --ros-args --params-file /config/g1_lio.yaml --log-level warn \
      -r /livox/lidar:=/gap_test/lidar \
      -r /livox/imu:=/gap_test/imu \
      -r /aft_mapped_to_init:=/gap_test/odom \
      -r /cloud_registered:=/gap_test/cloud_registered' >/dev/null

set +e
docker --context "${DOCKER_CONTEXT}" run --rm \
  --network host \
  --ipc host \
  --read-only \
  --security-opt no-new-privileges:true \
  --cap-drop ALL \
  --tmpfs /tmp:rw,size=128m,mode=1777 \
  -e ROS_DOMAIN_ID=92 \
  -e ROS_LOG_DIR=/tmp/ros-log \
  -v "${SCRIPT_DIR}/imu_gap_publisher.py:/test/imu_gap_publisher.py:ro" \
  "${FAST_LIVO_IMAGE}" \
  /bin/bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/fast_livo_ws/install/setup.bash
    exec python3 /test/imu_gap_publisher.py'
readonly PUBLISHER_STATUS="$?"
set -e

readonly GAP_WARNING_COUNT="$(
  docker --context "${DOCKER_CONTEXT}" logs "${CONTAINER_NAME}" 2>&1 \
    | grep -c "accepting current sample to resynchronize" || true
)"
readonly LEGACY_REJECTION_COUNT="$(
  docker --context "${DOCKER_CONTEXT}" logs "${CONTAINER_NAME}" 2>&1 \
    | grep -c "imu time stamp Jumps" || true
)"

if [[ "${PUBLISHER_STATUS}" -ne 0 ]]; then
  docker --context "${DOCKER_CONTEXT}" logs --tail 160 "${CONTAINER_NAME}" >&2
fi

test "${PUBLISHER_STATUS}" -eq 0
test "${GAP_WARNING_COUNT}" -ge 1
test "${LEGACY_REJECTION_COUNT}" -eq 0
echo "gap_warnings=${GAP_WARNING_COUNT} legacy_rejections=${LEGACY_REJECTION_COUNT}"
