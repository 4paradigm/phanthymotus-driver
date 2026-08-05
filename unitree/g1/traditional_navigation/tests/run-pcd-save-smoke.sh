#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly NAV_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly DOCKER_CONTEXT="${LOCAL_DOCKER_CONTEXT:-desktop-linux}"

set -a
# shellcheck disable=SC1091
source "${NAV_DIR}/source-lock.env"
set +a

readonly FAST_LIVO_IMAGE="${1:-phanthy-fast-livo2:g1-${FAST_LIVO2_IMAGE_TAG}}"
readonly CONTAINER_NAME="fast-livo-pcd-save-smoke-$$"
readonly OUTPUT_DIR="$(mktemp -d /private/tmp/g1-pcd-save-smoke.XXXXXX)"

cleanup() {
  docker --context "${DOCKER_CONTEXT}" rm -f "${CONTAINER_NAME}" \
    >/dev/null 2>&1 || true
  rm -rf -- "${OUTPUT_DIR}"
}
trap cleanup EXIT

pcd_count() {
  find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.pcd' -size +0c \
    | wc -l | tr -d ' '
}

docker --context "${DOCKER_CONTEXT}" image inspect --platform linux/arm64 \
  "${FAST_LIVO_IMAGE}" >/dev/null

docker --context "${DOCKER_CONTEXT}" run -d \
  --name "${CONTAINER_NAME}" \
  --platform linux/arm64 \
  --network host \
  --ipc host \
  --read-only \
  --security-opt no-new-privileges:true \
  --cap-drop ALL \
  --memory 6g \
  --tmpfs /tmp:rw,size=256m,mode=1777 \
  --tmpfs /opt/fast_livo_ws/src/fast_livo/Log:rw,size=512m,mode=1777 \
  -e ROS_DOMAIN_ID=94 \
  -e ROS_LOG_DIR=/tmp/ros-log \
  -v "${NAV_DIR}/g1_lio.yaml:/config/g1_lio.yaml:ro" \
  -v "${OUTPUT_DIR}:/opt/fast_livo_ws/src/fast_livo/Log/pcd:rw" \
  "${FAST_LIVO_IMAGE}" \
  /bin/bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/fast_livo_ws/install/setup.bash
    exec /opt/fast_livo_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping \
      --ros-args --params-file /config/g1_lio.yaml --log-level warn \
      -p pcd_save.pcd_save_en:=true \
      -p pcd_save.interval:=20 \
      -p pcd_save.type:=0 \
      -r /livox/lidar:=/gap_test/lidar \
      -r /livox/imu:=/gap_test/imu \
      -r /aft_mapped_to_init:=/gap_test/odom \
      -r /cloud_registered:=/gap_test/cloud_registered' >/dev/null

sleep 2

docker --context "${DOCKER_CONTEXT}" run --rm \
  --platform linux/arm64 \
  --network host \
  --ipc host \
  --read-only \
  --security-opt no-new-privileges:true \
  --cap-drop ALL \
  --tmpfs /tmp:rw,size=128m,mode=1777 \
  -e ROS_DOMAIN_ID=94 \
  -e ROS_LOG_DIR=/tmp/ros-log \
  -v "${SCRIPT_DIR}/imu_gap_publisher.py:/test/imu_gap_publisher.py:ro" \
  "${FAST_LIVO_IMAGE}" \
  /bin/bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/fast_livo_ws/install/setup.bash
    exec python3 /test/imu_gap_publisher.py' >/dev/null

readonly CHECKPOINT_COUNT="$(pcd_count)"
test "${CHECKPOINT_COUNT}" -ge 1

docker --context "${DOCKER_CONTEXT}" stop \
  --signal SIGINT --time 30 "${CONTAINER_NAME}" >/dev/null

readonly EXIT_CODE="$(
  docker --context "${DOCKER_CONTEXT}" inspect "${CONTAINER_NAME}" \
    --format '{{.State.ExitCode}}'
)"
readonly FINAL_COUNT="$(pcd_count)"

test "${EXIT_CODE}" -eq 0
test "${FINAL_COUNT}" -ge "${CHECKPOINT_COUNT}"
test -z "$(find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.tmp' -print -quit)"

while IFS= read -r pcd_file; do
  head -c 512 "${pcd_file}" | grep -a '^POINTS [1-9][0-9]*$' >/dev/null
  head -c 512 "${pcd_file}" | grep -a '^DATA binary$' >/dev/null
done < <(find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.pcd' -size +0c | sort)

readonly FIRST_PCD="$(
  find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.pcd' -size +0c \
    | sort | sed -n '1p'
)"
set +e
PARSER_OUTPUT="$(
  docker --context "${DOCKER_CONTEXT}" run --rm \
    --platform linux/arm64 \
    --network none \
    --read-only \
    --tmpfs /tmp:rw,size=64m,mode=1777 \
    -e ROS_DOMAIN_ID=95 \
    -v "${FIRST_PCD}:/test/map.pcd:ro" \
    "${FAST_LIVO_IMAGE}" \
    /bin/bash -lc '
      source /opt/ros/humble/setup.bash
      source /opt/fast_livo_ws/install/setup.bash
      timeout 3 ros2 run pcl_ros pcd_to_pointcloud --ros-args \
        -p file_name:=/test/map.pcd -p interval:=1.0' 2>&1
)"
PARSER_EXIT_CODE="$?"
set -e
if [[ "${PARSER_EXIT_CODE}" -ne 124 ]]; then
  printf '%s\n' "${PARSER_OUTPUT}" >&2
  exit 1
fi
grep -F 'Publishing data with' <<<"${PARSER_OUTPUT}" >/dev/null

docker --context "${DOCKER_CONTEXT}" logs "${CONTAINER_NAME}" 2>&1 \
  | grep -F 'PCD checkpoint completed' >/dev/null
docker --context "${DOCKER_CONTEXT}" logs "${CONTAINER_NAME}" 2>&1 \
  | grep -F 'PCD finalization completed' >/dev/null

echo "pcd_save_smoke=PASS checkpoints=${CHECKPOINT_COUNT} final_files=${FINAL_COUNT} exit=${EXIT_CODE}"
