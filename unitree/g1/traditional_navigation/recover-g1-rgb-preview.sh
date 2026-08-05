#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly MERGE_SCRIPT="${SCRIPT_DIR}/merge-g1-rgb-pcd.py"
readonly DEPLOY_SCRIPT="${SCRIPT_DIR}/deploy-g1-navigation-shadow.sh"
readonly SSH_TARGET="${1:-}"
readonly ROS_NAMESPACE_VALUE="${2:-ubuntu}"
readonly NETWORK_INTERFACE_VALUE="${3:-eth0}"
readonly MAP_NAME="${G1_MAP_NAME:-}"
readonly REMOTE_MAP_ROOT="/home/unitree/phanthy-navigation-maps"
readonly REMOTE_MAP_DIR="${REMOTE_MAP_ROOT}/${MAP_NAME}"
readonly RECOVERED_PCD="${REMOTE_MAP_DIR}/all_rgb_points.recovered.pcd"
readonly RECOVERY_MANIFEST="${REMOTE_MAP_DIR}/rgb-recovery-manifest.json"
readonly SSH_OPTS=(
  -o ClearAllForwardings=yes
  -o ControlMaster=no
  -o ControlPath=none
  -o BatchMode=yes
  -o ConnectTimeout=8
)

usage() {
  echo "Usage: G1_MAP_NAME=<map> CONFIRM_G1_SHADOW_WRITE=YES $0 <ssh-target> [ros-namespace] [network-interface]" >&2
}

if (( $# < 1 || $# > 3 )); then
  usage
  exit 2
fi
[[ "${SSH_TARGET}" =~ ^[a-zA-Z0-9_.@-]+$ ]] || {
  echo "invalid ssh target: ${SSH_TARGET}" >&2
  exit 2
}
[[ "${MAP_NAME}" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}$ ]] || {
  echo "G1_MAP_NAME must match [a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}" >&2
  exit 2
}
[[ "${ROS_NAMESPACE_VALUE}" =~ ^[a-zA-Z0-9_/-]+$ ]] || {
  echo "invalid ROS namespace: ${ROS_NAMESPACE_VALUE}" >&2
  exit 2
}
[[ "${NETWORK_INTERFACE_VALUE}" =~ ^[a-zA-Z0-9_.:-]+$ ]] || {
  echo "invalid network interface: ${NETWORK_INTERFACE_VALUE}" >&2
  exit 2
}
if [[ "${CONFIRM_G1_SHADOW_WRITE:-}" != "YES" ]]; then
  echo "Refusing robot write. Recovery creates an aggregate PCD and restores ordinary LIO." >&2
  exit 2
fi
test -f "${MERGE_SCRIPT}" || {
  echo "missing merge utility: ${MERGE_SCRIPT}" >&2
  exit 1
}
test -x "${DEPLOY_SCRIPT}" || {
  echo "missing deploy utility: ${DEPLOY_SCRIPT}" >&2
  exit 1
}

echo "[preflight] validate interrupted RGB preview identity"
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
  set -euo pipefail
  test \"\$(docker inspect embodied-unitree-g1 --format '{{.State.Running}}')\" = true
  actual_running=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.Running}}')\"
  actual_exit=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.ExitCode}}')\"
  actual_mode=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}')\"
  actual_map=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\"
  actual_dir=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{range .Mounts}}{{if eq .Destination \"/opt/fast_livo_ws/src/fast_livo/Log/pcd\"}}{{.Source}}{{end}}{{end}}')\"
  if ! { test \"\${actual_running}\" = false && \
         test \"\${actual_exit}\" = 255 && \
         test \"\${actual_mode}\" = rgb_preview && \
         test \"\${actual_map}\" = '${MAP_NAME}' && \
         test \"\${actual_dir}\" = '${REMOTE_MAP_DIR}'; }; then
    echo \"refusing RGB recovery identity mismatch: requested_map=${MAP_NAME} running=\${actual_running} exit=\${actual_exit} mode=\${actual_mode} actual_map=\${actual_map} actual_dir=\${actual_dir}\" >&2
    exit 1
  fi
  test -s '${REMOTE_MAP_DIR}/rgb-preview-calibration.json'
  test -s '${REMOTE_MAP_DIR}/lidar_poses.txt'
  source_count=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f \
    -regextype posix-extended -regex '.*/[0-9]+\.[0-9]+\.pcd' -size +0c | wc -l)\"
  test \"\${source_count}\" -ge 1
  echo \"map_name=${MAP_NAME} state=interrupted_rgb_preview exit=\${actual_exit} source_count=\${source_count}\"
"

echo "[recover] atomically merge RGB checkpoint PCDs"
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
  "python3 - \
    --input-dir '${REMOTE_MAP_DIR}' \
    --output '${RECOVERED_PCD}' \
    --manifest '${RECOVERY_MANIFEST}' \
    --skip-zero-filled-checkpoints" \
  < "${MERGE_SCRIPT}"

echo "[recover] restore ordinary read-only LIO"
CONFIRM_G1_SHADOW_WRITE=YES \
  "${DEPLOY_SCRIPT}" resume \
  "${SSH_TARGET}" "${ROS_NAMESPACE_VALUE}" "${NETWORK_INTERFACE_VALUE}"

echo "[verify] recovered RGB artifact"
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
  "python3 - \
    --validate \
    --output '${RECOVERED_PCD}' \
    --manifest '${RECOVERY_MANIFEST}'" \
  < "${MERGE_SCRIPT}"

echo "RGB preview recovered after unclean shutdown."
echo "Recovered map: ${RECOVERED_PCD}"
echo "Manifest: ${RECOVERY_MANIFEST}"
