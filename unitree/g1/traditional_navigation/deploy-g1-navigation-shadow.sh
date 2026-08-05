#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly NAV_DIR="${SCRIPT_DIR}"
readonly REMOTE_DIR="/home/unitree/phanthy-navigation-shadow-n2"
readonly REMOTE_MAP_ROOT="/home/unitree/phanthy-navigation-maps"
readonly LOCAL_DOCKER_CONTEXT="${LOCAL_DOCKER_CONTEXT:-desktop-linux}"
readonly -a LOCAL_DOCKER=(docker --context "${LOCAL_DOCKER_CONTEXT}")

usage() {
  echo "Usage: $0 <preflight|up|sensor-up|fast-up|fast-build-up|resume|down|start_mapping|stop_mapping|start_rgb_preview|stop_rgb_preview> <ssh-target> [ros-namespace] [network-interface]" >&2
  echo "Example: $0 preflight g1-sh-wifi ubuntu eth0" >&2
  echo "Mapping example: G1_MAP_NAME=sh_n3_smoke $0 start_mapping g1-sh-wifi ubuntu eth0" >&2
  echo "RGB preview: G1_MAP_NAME=sh_rgb_static $0 start_rgb_preview g1-sh-wifi ubuntu eth0" >&2
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
  preflight|up|sensor-up|fast-up|fast-build-up|resume|down|start_mapping|stop_mapping|start_rgb_preview|stop_rgb_preview) ;;
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
PCD_SAVE_INTERVAL_VALUE="${G1_PCD_SAVE_INTERVAL:-600}"
if [[ "${MODE}" == start_mapping || "${MODE}" == stop_mapping || \
      "${MODE}" == start_rgb_preview || "${MODE}" == stop_rgb_preview ]]; then
  [[ "${MAP_NAME}" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}$ ]] || {
    echo "G1_MAP_NAME must match [a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}" >&2
    exit 2
  }
fi
if [[ "${MODE}" == start_mapping || "${MODE}" == start_rgb_preview ]]; then
  [[ "${PCD_SAVE_INTERVAL_VALUE}" =~ ^[1-9][0-9]{0,5}$ ]] || {
    echo "G1_PCD_SAVE_INTERVAL must be an integer from 1 to 999999" >&2
    exit 2
  }
fi
readonly MAP_NAME
readonly PCD_SAVE_INTERVAL_VALUE
readonly REMOTE_MAP_DIR="${REMOTE_MAP_ROOT}/${MAP_NAME}"
readonly RGB_TIME_PROBE_PATH="${G1_RGB_TIME_PROBE:-}"
RGB_MOTION_POLICY_VALUE="stationary_only"
RGB_TIME_EVIDENCE_VALUE="unverified_callback_arrival"

for required_file in \
  source-lock.env \
  compose.shadow.yml \
  compose.mapping.yml \
  driver.shadow.yaml \
  g1_lio.yaml \
  Dockerfile.fast-livo2-hotfix \
  fast-livo2-runtime.patch \
  fast-livo2-pcd-save.patch; do
  test -f "${NAV_DIR}/${required_file}" || {
    echo "missing asset: ${NAV_DIR}/${required_file}" >&2
    exit 1
  }
done

if [[ "${MODE}" == start_rgb_preview || "${MODE}" == stop_rgb_preview ]]; then
  for required_file in \
    source-lock.rgb.env \
    compose.rgb-preview.yml \
    Dockerfile.fast-livo2-rgb-hotfix \
    fast-livo2-rgb-qos.patch \
    probe-g1-rgb-profile.py \
    probe-g1-rgb-time-offset.py \
    derive-g1-rgb-preview-calibration.py \
    render-g1-livo-config.py; do
    test -f "${NAV_DIR}/${required_file}" || {
      echo "missing RGB preview asset: ${NAV_DIR}/${required_file}" >&2
      exit 1
    }
  done
fi

if [[ "${MODE}" == start_rgb_preview && -n "${RGB_TIME_PROBE_PATH}" ]]; then
  test -f "${RGB_TIME_PROBE_PATH}" || {
    echo "missing G1_RGB_TIME_PROBE file: ${RGB_TIME_PROBE_PATH}" >&2
    exit 1
  }
  python3 "${NAV_DIR}/probe-g1-rgb-time-offset.py" \
    --validate "${RGB_TIME_PROBE_PATH}"
  RGB_MOTION_POLICY_VALUE="slow_manual_preview"
  RGB_TIME_EVIDENCE_VALUE="measured_callback_latency"
fi
readonly RGB_MOTION_POLICY_VALUE
readonly RGB_TIME_EVIDENCE_VALUE

set -a
# shellcheck disable=SC1091
source "${NAV_DIR}/source-lock.env"
set +a

if [[ "${MODE}" == start_rgb_preview || "${MODE}" == stop_rgb_preview ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${NAV_DIR}/source-lock.rgb.env"
  set +a
fi

readonly FAST_LIVO_IMAGE="phanthy-fast-livo2:g1-${FAST_LIVO2_IMAGE_TAG}"
readonly FAST_LIVO_BASE_IMAGE="phanthy-fast-livo2:g1-${FAST_LIVO2_BASE_IMAGE_TAG}"
readonly FAST_LIVO_RGB_IMAGE="${FAST_LIVO2_RGB_IMAGE:-}"
readonly FAST_LIVO_RGB_BASE_IMAGE="${FAST_LIVO2_RGB_BASE_IMAGE:-}"
readonly SSH_OPTS=(
  -o ClearAllForwardings=yes
  -o ControlMaster=no
  -o ControlPath=none
  -o BatchMode=yes
  -o ConnectTimeout=8
)

RGB_PREVIEW_TEMP_DIR=""
cleanup_rgb_preview_temp() {
  if [[ -n "${RGB_PREVIEW_TEMP_DIR}" && -d "${RGB_PREVIEW_TEMP_DIR}" && \
        "${RGB_PREVIEW_TEMP_DIR}" == /private/tmp/g1-rgb-preview.* ]]; then
    rm -rf -- "${RGB_PREVIEW_TEMP_DIR}"
  fi
}
trap cleanup_rgb_preview_temp EXIT

if [[ "${MODE}" == start_rgb_preview || "${MODE}" == stop_rgb_preview ]]; then
  test "${FAST_LIVO_RGB_BASE_IMAGE}" = "${FAST_LIVO_IMAGE}" || {
    echo "RGB preview base must equal the locked N3 map-save image" >&2
    exit 1
  }
fi

declare -a LOCAL_IMAGES=()
case "${MODE}" in
  resume|down|fast-build-up|start_mapping|stop_mapping|start_rgb_preview|stop_rgb_preview) ;;
  sensor-up) LOCAL_IMAGES=("${G1_DRIVER_IMAGE}") ;;
  fast-up) LOCAL_IMAGES=("${FAST_LIVO_IMAGE}") ;;
  *) LOCAL_IMAGES=("${G1_DRIVER_IMAGE}" "${FAST_LIVO_IMAGE}") ;;
esac

if (( ${#LOCAL_IMAGES[@]} == 0 )); then
  echo "[preflight] local images skipped (mode=${MODE}, no image transfer)"
else
  docker context inspect "${LOCAL_DOCKER_CONTEXT}" >/dev/null

  for image_name in "${LOCAL_IMAGES[@]}"; do
    if ! image_arch="$(
      "${LOCAL_DOCKER[@]}" image inspect --platform linux/arm64 "${image_name}" \
        --format '{{.Architecture}}' 2>/dev/null
    )"; then
      echo "missing local image in Docker context ${LOCAL_DOCKER_CONTEXT}: ${image_name}" >&2
      "${LOCAL_DOCKER[@]}" image ls --format '{{.Repository}}:{{.Tag}} {{.ID}}' >&2
      exit 1
    fi
    test "${image_arch}" = "arm64" || {
      echo "image is not arm64: ${image_name} (${image_arch})" >&2
      exit 1
    }
  done

  echo "[preflight] local images (context=${LOCAL_DOCKER_CONTEXT})"
  "${LOCAL_DOCKER[@]}" image inspect --platform linux/arm64 \
    "${LOCAL_IMAGES[@]}" \
    --format '{{.RepoTags}} id={{.Id}} arch={{.Architecture}} size={{.Size}}'
fi

echo "[preflight] remote target"
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" '
  set -e
  test "$(uname -m)" = aarch64
  test -w /home/unitree
  test "$(timedatectl show -p NTPSynchronized --value)" = yes
  echo "time=$(date -Iseconds) uptime=$(uptime -p) boot_id=$(cat /proc/sys/kernel/random/boot_id) ntp=yes"
  docker compose version
  docker inspect embodied-unitree-g1 \
    --format "current={{.Name}} image={{.Config.Image}} running={{.State.Running}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} started={{.State.StartedAt}}"
  df -h /home/unitree
'

if [[ "${MODE}" == fast-build-up ]]; then
  echo "[preflight] remote hotfix base"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test \"\$(docker image inspect '${G1_DRIVER_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_BASE_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_BASE_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.revision\"}}')\" = '${FAST_LIVO2_COMMIT}'
    test \"\$(docker image inspect '${FAST_LIVO_BASE_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-runtime-patch\"}}')\" = '${FAST_LIVO2_BASE_RUNTIME_PATCH_SHA256}'
  "
fi

if [[ "${MODE}" == start_mapping ]]; then
  echo "[preflight] mapping target"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test \"\$(docker image inspect '${G1_DRIVER_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-runtime-patch\"}}')\" = '${FAST_LIVO2_RUNTIME_PATCH_SHA256}'
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-pcd-save-patch\"}}')\" = '${FAST_LIVO2_PCD_SAVE_PATCH_SHA256}'
    current_mode=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}' 2>/dev/null || true)\"
    if test \"\${current_mode}\" = mapping || test \"\${current_mode}\" = rgb_preview; then
      current_map=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\"
      echo \"controlled write mode already active: mode=\${current_mode} map=\${current_map}\" >&2
      exit 1
    fi
    if test -e '${REMOTE_MAP_DIR}'; then
      test -d '${REMOTE_MAP_DIR}'
      test -z \"\$(find '${REMOTE_MAP_DIR}' -mindepth 1 -maxdepth 1 -print -quit)\" || {
        echo 'refusing to overwrite non-empty map directory: ${REMOTE_MAP_DIR}' >&2
        exit 1
      }
    fi
    echo 'map_name=${MAP_NAME} map_dir=${REMOTE_MAP_DIR} state=ready'
  "
fi

if [[ "${MODE}" == start_rgb_preview ]]; then
  echo "[preflight] RGB preview target and live nominal calibration"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test \"\$(docker image inspect '${G1_DRIVER_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_RGB_BASE_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_RGB_BASE_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.revision\"}}')\" = '${FAST_LIVO2_COMMIT}'
    test \"\$(docker image inspect '${FAST_LIVO_RGB_BASE_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-runtime-patch\"}}')\" = '${FAST_LIVO2_RUNTIME_PATCH_SHA256}'
    test \"\$(docker image inspect '${FAST_LIVO_RGB_BASE_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-pcd-save-patch\"}}')\" = '${FAST_LIVO2_PCD_SAVE_PATCH_SHA256}'
    current_mode=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}' 2>/dev/null || true)\"
    if test \"\${current_mode}\" = mapping || test \"\${current_mode}\" = rgb_preview; then
      current_map=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\"
      echo \"controlled write mode already active: mode=\${current_mode} map=\${current_map}\" >&2
      exit 1
    fi
    if test -e '${REMOTE_MAP_DIR}'; then
      test -d '${REMOTE_MAP_DIR}'
      test -z \"\$(find '${REMOTE_MAP_DIR}' -mindepth 1 -maxdepth 1 -print -quit)\" || {
        echo 'refusing to overwrite non-empty RGB preview directory: ${REMOTE_MAP_DIR}' >&2
        exit 1
      }
    fi
    test \"\$(docker inspect embodied-unitree-g1 --format '{{.State.Running}}')\" = true
    echo 'map_name=${MAP_NAME} map_dir=${REMOTE_MAP_DIR} state=ready'
  "

  RGB_PREVIEW_TEMP_DIR="$(mktemp -d /private/tmp/g1-rgb-preview.XXXXXX)"
  rgb_probe_ready=false
  for rgb_probe_attempt in 1 2 3; do
    : > "${RGB_PREVIEW_TEMP_DIR}/live-probe.json"
    : > "${RGB_PREVIEW_TEMP_DIR}/probe.stderr"
    : > "${RGB_PREVIEW_TEMP_DIR}/derive.stderr"
    rgb_probe_ssh_status=0
    ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
      "docker exec -i embodied-unitree-g1 python3 - '${NETWORK_INTERFACE_VALUE}' 1920 1080 15" \
      < "${NAV_DIR}/probe-g1-rgb-profile.py" \
      > "${RGB_PREVIEW_TEMP_DIR}/live-probe.json" \
      2> "${RGB_PREVIEW_TEMP_DIR}/probe.stderr" \
      || rgb_probe_ssh_status=$?
    rgb_probe_derive_status=not_run
    if [[ "${rgb_probe_ssh_status}" -eq 0 ]]; then
      rgb_derive_command=(
        python3 "${NAV_DIR}/derive-g1-rgb-preview-calibration.py"
        "${RGB_PREVIEW_TEMP_DIR}/live-probe.json"
        "${RGB_PREVIEW_TEMP_DIR}/rgb-preview-calibration.json"
      )
      if [[ -n "${RGB_TIME_PROBE_PATH}" ]]; then
        rgb_derive_command+=(--time-offset-probe "${RGB_TIME_PROBE_PATH}")
      fi
      if "${rgb_derive_command[@]}" \
          2> "${RGB_PREVIEW_TEMP_DIR}/derive.stderr"; then
        rgb_probe_ready=true
        break
      else
        rgb_probe_derive_status=$?
      fi
    fi
    rgb_probe_bytes="$(wc -c < "${RGB_PREVIEW_TEMP_DIR}/live-probe.json" | tr -d ' ')"
    echo "RGB probe attempt=${rgb_probe_attempt} invalid ssh_status=${rgb_probe_ssh_status} derive_status=${rgb_probe_derive_status} bytes=${rgb_probe_bytes}" >&2
    sed -n '1,20p' "${RGB_PREVIEW_TEMP_DIR}/probe.stderr" >&2
    sed -n '1,20p' "${RGB_PREVIEW_TEMP_DIR}/derive.stderr" >&2
    sleep 1
  done
  if [[ "${rgb_probe_ready}" != true ]]; then
    echo "RGB preview preflight failed: no valid live probe JSON after 3 attempts; no robot state was changed" >&2
    exit 1
  fi
  python3 "${NAV_DIR}/render-g1-livo-config.py" \
    "${RGB_PREVIEW_TEMP_DIR}/rgb-preview-calibration.json" \
    "${RGB_PREVIEW_TEMP_DIR}/g1_livo.rgb-preview.yaml" \
    --allow-nominal-preview
fi

if [[ "${MODE}" == stop_mapping ]]; then
  echo "[preflight] active mapping identity"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    actual_mode=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}')\"
    actual_map=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\"
    actual_dir=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{range .Mounts}}{{if eq .Destination \"/opt/fast_livo_ws/src/fast_livo/Log/pcd\"}}{{.Source}}{{end}}{{end}}')\"
    if ! { test \"\${actual_mode}\" = mapping && \
           test \"\${actual_map}\" = '${MAP_NAME}' && \
           test \"\${actual_dir}\" = '${REMOTE_MAP_DIR}'; }; then
      echo \"refusing stop_mapping identity mismatch: requested_map=${MAP_NAME} actual_mode=\${actual_mode} actual_map=\${actual_map} actual_dir=\${actual_dir}\" >&2
      exit 1
    fi
    mapping_interval=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.pcd_save_interval\"}}')\"
    mapping_user=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.Config.User}}')\"
    mapping_groups=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{json .HostConfig.GroupAdd}}')\"
    echo \"map_name=${MAP_NAME} map_dir=${REMOTE_MAP_DIR} pcd_save_interval=\${mapping_interval} container_user=\${mapping_user} group_add=\${mapping_groups} state=mapping\"
  "
fi

if [[ "${MODE}" == stop_rgb_preview ]]; then
  echo "[preflight] active RGB preview identity"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    actual_mode=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}')\"
    actual_map=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\"
    actual_dir=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{range .Mounts}}{{if eq .Destination \"/opt/fast_livo_ws/src/fast_livo/Log/pcd\"}}{{.Source}}{{end}}{{end}}')\"
    if ! { test \"\${actual_mode}\" = rgb_preview && \
           test \"\${actual_map}\" = '${MAP_NAME}' && \
           test \"\${actual_dir}\" = '${REMOTE_MAP_DIR}'; }; then
      echo \"refusing stop_rgb_preview identity mismatch: requested_map=${MAP_NAME} actual_mode=\${actual_mode} actual_map=\${actual_map} actual_dir=\${actual_dir}\" >&2
      exit 1
    fi
    preview_interval=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.pcd_save_interval\"}}')\"
    echo \"map_name=${MAP_NAME} map_dir=${REMOTE_MAP_DIR} pcd_save_interval=\${preview_interval} state=rgb_preview\"
  "
fi

if [[ "${MODE}" == preflight ]]; then
  echo "PREFLIGHT OK: no robot state changed"
  exit 0
fi

if [[ "${CONFIRM_G1_SHADOW_WRITE:-}" != "YES" ]]; then
  echo "Refusing robot write. Re-run with CONFIRM_G1_SHADOW_WRITE=YES." >&2
  exit 2
fi

if [[ "${MODE}" == down ]]; then
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test -d '${REMOTE_DIR}'
    cd '${REMOTE_DIR}'
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
      docker compose --env-file source-lock.env \
        -f compose.shadow.yml --profile lio-shadow down
  "
  echo "N2 shadow stopped; existing embodied-unitree-g1 was not modified."
  exit 0
fi

if [[ "${MODE}" == stop_mapping ]]; then
  echo "[mapping] stop and save map_name=${MAP_NAME}"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -u
    cd '${REMOTE_DIR}'

    mapping_stop_status=0
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    G1_MAP_NAME='${MAP_NAME}' \
    G1_MAP_DIR='${REMOTE_MAP_DIR}' \
      docker compose --env-file source-lock.env \
        -f compose.shadow.yml -f compose.mapping.yml \
        --profile lio-shadow stop fast-livo2 || mapping_stop_status=\$?

    mapping_exit_code=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.ExitCode}}' 2>/dev/null || echo inspect_failed)\"
    echo \"mapping_stop_status=\${mapping_stop_status} mapping_exit_code=\${mapping_exit_code}\"
    mapping_cleanup_status=0
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    G1_MAP_NAME='${MAP_NAME}' \
    G1_MAP_DIR='${REMOTE_MAP_DIR}' \
      docker compose --env-file source-lock.env \
        -f compose.shadow.yml -f compose.mapping.yml \
        --profile lio-shadow down || mapping_cleanup_status=\$?

    # Restore ordinary shadow before reporting map artifacts. Reporting errors
    # must never leave the read-only LIO sidecars stopped.
    mapping_restore_status=0
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
      docker compose --env-file source-lock.env \
        -f compose.shadow.yml --profile lio-shadow \
        up -d --no-build --wait --wait-timeout 90 || mapping_restore_status=\$?
    docker inspect phanthy-navigation-sensors-shadow phanthy-fast-livo2-shadow \
      --format '{{.Name}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} read_only={{.HostConfig.ReadonlyRootfs}} privileged={{.HostConfig.Privileged}} capdrop={{json .HostConfig.CapDrop}}' \
      || mapping_restore_status=\$?

    pcd_count=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c | wc -l)\"
    pcd_bytes=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c -printf '%s\\n' | awk '{ total += \$1 } END { print total + 0 }')\"
    echo \"map_name=${MAP_NAME} pcd_count=\${pcd_count} pcd_bytes=\${pcd_bytes}\"
    find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -printf '%f %s bytes\\n' | sort
    test \"\${mapping_stop_status}\" -eq 0
    test \"\${mapping_cleanup_status}\" -eq 0
    test \"\${mapping_restore_status}\" -eq 0
    test \"\${mapping_exit_code}\" = 0
    test \"\${pcd_count}\" -ge 1
  "
  echo "Mapping stopped and saved. Plain read-only LIO shadow was restored."
  exit 0
fi

if [[ "${MODE}" == stop_rgb_preview ]]; then
  echo "[rgb-preview] stop and save map_name=${MAP_NAME}"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -uo pipefail
    cd '${REMOTE_DIR}'

    preview_stop_status=0
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    G1_MAP_NAME='${MAP_NAME}' \
    G1_MAP_DIR='${REMOTE_MAP_DIR}' \
    G1_RGB_CONFIG_PATH='${REMOTE_DIR}/g1_livo.rgb-preview.yaml' \
      docker compose --env-file source-lock.env --env-file source-lock.rgb.env \
        -f compose.shadow.yml -f compose.mapping.yml -f compose.rgb-preview.yml \
        --profile lio-shadow stop fast-livo2 || preview_stop_status=\$?

    preview_exit_code=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.State.ExitCode}}' 2>/dev/null || echo inspect_failed)\"
    echo \"rgb_preview_stop_status=\${preview_stop_status} rgb_preview_exit_code=\${preview_exit_code}\"

    preview_cleanup_status=0
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    G1_MAP_NAME='${MAP_NAME}' \
    G1_MAP_DIR='${REMOTE_MAP_DIR}' \
    G1_RGB_CONFIG_PATH='${REMOTE_DIR}/g1_livo.rgb-preview.yaml' \
      docker compose --env-file source-lock.env --env-file source-lock.rgb.env \
        -f compose.shadow.yml -f compose.mapping.yml -f compose.rgb-preview.yml \
        --profile lio-shadow down || preview_cleanup_status=\$?

    # Always restore the ordinary LIO shadow before inspecting preview output.
    preview_restore_status=0
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
      docker compose --env-file source-lock.env \
        -f compose.shadow.yml --profile lio-shadow \
        up -d --no-build --wait --wait-timeout 90 || preview_restore_status=\$?
    docker inspect phanthy-navigation-sensors-shadow phanthy-fast-livo2-shadow \
      --format '{{.Name}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} read_only={{.HostConfig.ReadonlyRootfs}} privileged={{.HostConfig.Privileged}} capdrop={{json .HostConfig.CapDrop}}' \
      || preview_restore_status=\$?

    pcd_count=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c | wc -l)\"
    pcd_bytes=\"\$(find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -size +0c -printf '%s\\n' | awk '{ total += \$1 } END { print total + 0 }')\"
    rgb_pcd_count=0
    first_rgb_pcd=''
    for pcd_file in '${REMOTE_MAP_DIR}'/*.pcd; do
      test -f \"\${pcd_file}\" || continue
      if sed -n '1,12p' \"\${pcd_file}\" | grep -q '^FIELDS x y z rgb'; then
        rgb_pcd_count=\$((rgb_pcd_count + 1))
        if test -z \"\${first_rgb_pcd}\"; then
          first_rgb_pcd=\"\${pcd_file}\"
        fi
      fi
    done
    echo \"map_name=${MAP_NAME} pcd_count=\${pcd_count} rgb_pcd_count=\${rgb_pcd_count} pcd_bytes=\${pcd_bytes}\"
    find '${REMOTE_MAP_DIR}' -maxdepth 1 -type f -name '*.pcd' -printf '%f %s bytes\\n' | sort
    if test -n \"\${first_rgb_pcd}\"; then
      echo \"rgb_pcd_header=\$(basename -- \"\${first_rgb_pcd}\")\"
      sed -n '1,12p' \"\${first_rgb_pcd}\"
    fi
    test -s '${REMOTE_MAP_DIR}/rgb-preview-calibration.json'
    test \"\${preview_stop_status}\" -eq 0
    test \"\${preview_cleanup_status}\" -eq 0
    test \"\${preview_restore_status}\" -eq 0
    test \"\${preview_exit_code}\" = 0
    test \"\${rgb_pcd_count}\" -ge 1
  "
  echo "RGB preview stopped and saved. Plain read-only LIO shadow was restored."
  exit 0
fi

echo "[deploy] copy immutable configs"
COPYFILE_DISABLE=1 tar --no-xattrs -C "${NAV_DIR}" -cf - \
  source-lock.env \
  compose.shadow.yml \
  compose.mapping.yml \
  driver.shadow.yaml \
  g1_lio.yaml \
  Dockerfile.fast-livo2-hotfix \
  fast-livo2-runtime.patch \
  fast-livo2-pcd-save.patch \
  | ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
      "mkdir -p '${REMOTE_DIR}' &&
       tar -C '${REMOTE_DIR}' -xf - &&
       chmod 0644 \
         '${REMOTE_DIR}/source-lock.env' \
         '${REMOTE_DIR}/compose.shadow.yml' \
         '${REMOTE_DIR}/compose.mapping.yml' \
         '${REMOTE_DIR}/driver.shadow.yaml' \
         '${REMOTE_DIR}/g1_lio.yaml' \
         '${REMOTE_DIR}/Dockerfile.fast-livo2-hotfix' \
         '${REMOTE_DIR}/fast-livo2-runtime.patch' \
         '${REMOTE_DIR}/fast-livo2-pcd-save.patch'"

if [[ "${MODE}" == start_rgb_preview ]]; then
  echo "[deploy] copy immutable RGB preview assets"
  COPYFILE_DISABLE=1 tar --no-xattrs -cf - \
    -C "${NAV_DIR}" \
      source-lock.rgb.env \
      compose.rgb-preview.yml \
      Dockerfile.fast-livo2-rgb-hotfix \
      fast-livo2-rgb-qos.patch \
    -C "${RGB_PREVIEW_TEMP_DIR}" \
      rgb-preview-calibration.json \
      g1_livo.rgb-preview.yaml \
    | ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
        "tar -C '${REMOTE_DIR}' -xf - &&
         chmod 0644 \
           '${REMOTE_DIR}/source-lock.rgb.env' \
           '${REMOTE_DIR}/compose.rgb-preview.yml' \
           '${REMOTE_DIR}/Dockerfile.fast-livo2-rgb-hotfix' \
           '${REMOTE_DIR}/fast-livo2-rgb-qos.patch' \
           '${REMOTE_DIR}/rgb-preview-calibration.json' \
           '${REMOTE_DIR}/g1_livo.rgb-preview.yaml'"
fi

if [[ "${MODE}" == start_mapping ]]; then
  echo "[mapping] start map_name=${MAP_NAME}"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    mkdir -p '${REMOTE_MAP_DIR}'
    test -z \"\$(find '${REMOTE_MAP_DIR}' -mindepth 1 -maxdepth 1 -print -quit)\"
    test -w '${REMOTE_MAP_DIR}'
    map_uid=\"\$(stat -c '%u' '${REMOTE_MAP_DIR}')\"
    map_gid=\"\$(stat -c '%g' '${REMOTE_MAP_DIR}')\"
    test \"\${map_uid}\" -ge 0
    test \"\${map_gid}\" -ge 0
    cd '${REMOTE_DIR}'
    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    G1_MAP_NAME='${MAP_NAME}' \
    G1_MAP_DIR='${REMOTE_MAP_DIR}' \
    G1_PCD_SAVE_INTERVAL='${PCD_SAVE_INTERVAL_VALUE}' \
    G1_MAP_UID=\"\${map_uid}\" \
    G1_MAP_GID=\"\${map_gid}\" \
      docker compose --env-file source-lock.env \
        -f compose.shadow.yml -f compose.mapping.yml \
        --profile lio-shadow \
        up -d --no-build --force-recreate --wait --wait-timeout 90
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}')\" = mapping
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\" = '${MAP_NAME}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.pcd_save_interval\"}}')\" = '${PCD_SAVE_INTERVAL_VALUE}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_owner\"}}')\" = \"\${map_uid}:\${map_gid}\"
    test -z \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.Config.User}}')\"
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{json .HostConfig.GroupAdd}}')\" = \"[\\\"\${map_gid}\\\"]\"
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{range .Mounts}}{{if eq .Destination \"/opt/fast_livo_ws/src/fast_livo/Log/pcd\"}}{{.Source}}{{end}}{{end}}')\" = '${REMOTE_MAP_DIR}'
    docker exec phanthy-fast-livo2-shadow /bin/bash -lc \
      \"id -G | tr ' ' '\\n' | grep -qx '\${map_gid}'\"
    docker exec phanthy-fast-livo2-shadow test -w /opt/fast_livo_ws/src/fast_livo/Log/pcd
    docker inspect phanthy-navigation-sensors-shadow phanthy-fast-livo2-shadow \
      --format '{{.Name}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} user={{json .Config.User}} group_add={{json .HostConfig.GroupAdd}} read_only={{.HostConfig.ReadonlyRootfs}} privileged={{.HostConfig.Privileged}} capdrop={{json .HostConfig.CapDrop}} devices={{json .HostConfig.Devices}}'
  "
  echo "Mapping is active for ${MAP_NAME}. Manually drive the robot, then run:"
  echo "G1_MAP_NAME=${MAP_NAME} CONFIRM_G1_SHADOW_WRITE=YES $0 stop_mapping ${SSH_TARGET} ${ROS_NAMESPACE_VALUE} ${NETWORK_INTERFACE_VALUE}"
  exit 0
fi

if [[ "${MODE}" == start_rgb_preview ]]; then
  echo "[rgb-preview] build/reuse isolated RGB image and start map_name=${MAP_NAME}"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    cd '${REMOTE_DIR}'
    printf '%s  %s\n' '${FAST_LIVO2_RGB_QOS_PATCH_SHA256}' fast-livo2-rgb-qos.patch | sha256sum -c -
    grep -Fxq '# preview_only: true' g1_livo.rgb-preview.yaml

    installed_rgb_patch=\"\$(docker image inspect '${FAST_LIVO_RGB_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-rgb-qos-patch\"}}' 2>/dev/null || true)\"
    if test \"\${installed_rgb_patch}\" = '${FAST_LIVO2_RGB_QOS_PATCH_SHA256}'; then
      echo 'RGB preview image already matches source lock; reuse it'
    else
      docker build \
        --platform linux/arm64 \
        --pull=false \
        --network=none \
        --build-arg FAST_LIVO2_RGB_BASE_IMAGE='${FAST_LIVO_RGB_BASE_IMAGE}' \
        --build-arg FAST_LIVO2_COMMIT='${FAST_LIVO2_COMMIT}' \
        --build-arg FAST_LIVO2_RGB_QOS_PATCH_SHA256='${FAST_LIVO2_RGB_QOS_PATCH_SHA256}' \
        -t '${FAST_LIVO_RGB_IMAGE}' \
        -f Dockerfile.fast-livo2-rgb-hotfix \
        .
    fi
    test \"\$(docker image inspect '${FAST_LIVO_RGB_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_RGB_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.revision\"}}')\" = '${FAST_LIVO2_COMMIT}'
    test \"\$(docker image inspect '${FAST_LIVO_RGB_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-runtime-patch\"}}')\" = '${FAST_LIVO2_RUNTIME_PATCH_SHA256}'
    test \"\$(docker image inspect '${FAST_LIVO_RGB_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-pcd-save-patch\"}}')\" = '${FAST_LIVO2_PCD_SAVE_PATCH_SHA256}'
    test \"\$(docker image inspect '${FAST_LIVO_RGB_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-rgb-qos-patch\"}}')\" = '${FAST_LIVO2_RGB_QOS_PATCH_SHA256}'

    mkdir -p '${REMOTE_MAP_DIR}'
    test -z \"\$(find '${REMOTE_MAP_DIR}' -mindepth 1 -maxdepth 1 -print -quit)\"
    cp rgb-preview-calibration.json '${REMOTE_MAP_DIR}/rgb-preview-calibration.json'
    chmod 0444 '${REMOTE_MAP_DIR}/rgb-preview-calibration.json'
    map_uid=\"\$(stat -c '%u' '${REMOTE_MAP_DIR}')\"
    map_gid=\"\$(stat -c '%g' '${REMOTE_MAP_DIR}')\"
    test \"\${map_uid}\" -ge 0
    test \"\${map_gid}\" -ge 0

    ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
    NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    G1_MAP_NAME='${MAP_NAME}' \
    G1_MAP_DIR='${REMOTE_MAP_DIR}' \
    G1_PCD_SAVE_INTERVAL='${PCD_SAVE_INTERVAL_VALUE}' \
    G1_MAP_UID=\"\${map_uid}\" \
    G1_MAP_GID=\"\${map_gid}\" \
    G1_RGB_CONFIG_PATH='${REMOTE_DIR}/g1_livo.rgb-preview.yaml' \
    G1_RGB_TIME_EVIDENCE='${RGB_TIME_EVIDENCE_VALUE}' \
    G1_RGB_MOTION_POLICY='${RGB_MOTION_POLICY_VALUE}' \
      docker compose --env-file source-lock.env --env-file source-lock.rgb.env \
        -f compose.shadow.yml -f compose.mapping.yml -f compose.rgb-preview.yml \
        --profile lio-shadow \
        up -d --no-build --force-recreate --wait --wait-timeout 90

    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{.Config.Image}}')\" = '${FAST_LIVO_RGB_IMAGE}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}')\" = rgb_preview
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.map_name\"}}')\" = '${MAP_NAME}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.pcd_save_interval\"}}')\" = '${PCD_SAVE_INTERVAL_VALUE}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.rgb_evidence\"}}')\" = nominal_public_urdf
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.rgb_time_evidence\"}}')\" = '${RGB_TIME_EVIDENCE_VALUE}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.rgb_motion_policy\"}}')\" = '${RGB_MOTION_POLICY_VALUE}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{range .Mounts}}{{if eq .Destination \"/opt/fast_livo_ws/src/fast_livo/Log/pcd\"}}{{.Source}}{{end}}{{end}}')\" = '${REMOTE_MAP_DIR}'
    test \"\$(docker inspect phanthy-fast-livo2-shadow --format '{{json .HostConfig.GroupAdd}}')\" = \"[\\\"\${map_gid}\\\"]\"
    docker exec phanthy-fast-livo2-shadow /bin/bash -lc \
      \"id -G | tr ' ' '\\n' | grep -qx '\${map_gid}'\"
    docker exec phanthy-fast-livo2-shadow test -w /opt/fast_livo_ws/src/fast_livo/Log/pcd
    docker inspect phanthy-navigation-sensors-shadow phanthy-fast-livo2-shadow \
      --format '{{.Name}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} user={{json .Config.User}} group_add={{json .HostConfig.GroupAdd}} read_only={{.HostConfig.ReadonlyRootfs}} privileged={{.HostConfig.Privileged}} capdrop={{json .HostConfig.CapDrop}} devices={{json .HostConfig.Devices}}'
  "
  echo "RGB preview is active for ${MAP_NAME}; evidence=nominal_public_urdf, time_evidence=${RGB_TIME_EVIDENCE_VALUE}, motion_policy=${RGB_MOTION_POLICY_VALUE}."
  if [[ "${RGB_MOTION_POLICY_VALUE}" == slow_manual_preview ]]; then
    echo "Keep the robot stationary for the first 20 seconds, then drive slowly and avoid fast turns while building this nominal RGB preview map."
  else
    echo "Keep the robot stationary and inspect /${ROS_NAMESPACE_VALUE}/navigation/cloud_registered_rgb in RViz."
  fi
  echo "Stop command: G1_MAP_NAME=${MAP_NAME} CONFIRM_G1_SHADOW_WRITE=YES $0 stop_rgb_preview ${SSH_TARGET} ${ROS_NAMESPACE_VALUE} ${NETWORK_INTERFACE_VALUE}"
  exit 0
fi

if [[ "${MODE}" == resume ]]; then
  echo "[deploy] reuse previously loaded arm64 images"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    test \"\$(docker image inspect '${G1_DRIVER_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{.Architecture}}')\" = arm64
  "
elif [[ "${MODE}" == fast-up ]]; then
  echo "[deploy] reuse driver image and stream patched FAST-LIVO2 image"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
    "test \"\$(docker image inspect '${G1_DRIVER_IMAGE}' --format '{{.Architecture}}')\" = arm64"
  "${LOCAL_DOCKER[@]}" image save --platform linux/arm64 "${FAST_LIVO_IMAGE}" \
    | gzip -1 \
    | ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" 'gzip -dc | docker load'
elif [[ "${MODE}" == fast-build-up ]]; then
  echo "[deploy] build patched FAST-LIVO2 from validated remote base (no image transfer)"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
    set -e
    cd '${REMOTE_DIR}'
    printf '%s  %s\n' \
      '${FAST_LIVO2_RUNTIME_PATCH_SHA256}' \
      fast-livo2-runtime.patch | sha256sum -c -
    printf '%s  %s\n' \
      '${FAST_LIVO2_PCD_SAVE_PATCH_SHA256}' \
      fast-livo2-pcd-save.patch | sha256sum -c -
    docker build \
      --platform linux/arm64 \
      --pull=false \
      --network=none \
      --build-arg FAST_LIVO2_BASE_IMAGE='${FAST_LIVO_BASE_IMAGE}' \
      --build-arg FAST_LIVO2_COMMIT='${FAST_LIVO2_COMMIT}' \
      --build-arg FAST_LIVO2_RUNTIME_PATCH_SHA256='${FAST_LIVO2_RUNTIME_PATCH_SHA256}' \
      --build-arg FAST_LIVO2_PCD_SAVE_PATCH_SHA256='${FAST_LIVO2_PCD_SAVE_PATCH_SHA256}' \
      -t '${FAST_LIVO_IMAGE}' \
      -f Dockerfile.fast-livo2-hotfix \
      .
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{.Architecture}}')\" = arm64
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-runtime-patch\"}}')\" = '${FAST_LIVO2_RUNTIME_PATCH_SHA256}'
    test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{index .Config.Labels \"org.opencontainers.image.fast-livo2-pcd-save-patch\"}}')\" = '${FAST_LIVO2_PCD_SAVE_PATCH_SHA256}'
  "
elif [[ "${MODE}" == sensor-up ]]; then
  echo "[deploy] reuse FAST-LIVO2 image and stream patched sensor image"
  ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" \
    "test \"\$(docker image inspect '${FAST_LIVO_IMAGE}' --format '{{.Architecture}}')\" = arm64"
  "${LOCAL_DOCKER[@]}" image save --platform linux/arm64 "${G1_DRIVER_IMAGE}" \
    | gzip -1 \
    | ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" 'gzip -dc | docker load'
else
  echo "[deploy] stream arm64 images"
  "${LOCAL_DOCKER[@]}" image save --platform linux/arm64 \
    "${G1_DRIVER_IMAGE}" "${FAST_LIVO_IMAGE}" \
    | gzip -1 \
    | ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" 'gzip -dc | docker load'
fi

echo "[deploy] start read-only shadow sidecars"
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
  set -e
  cd '${REMOTE_DIR}'
  ROS_NAMESPACE='${ROS_NAMESPACE_VALUE}' \
  NETWORK_INTERFACE='${NETWORK_INTERFACE_VALUE}' \
    docker compose --env-file source-lock.env \
      -f compose.shadow.yml --profile lio-shadow \
      up -d --no-build --force-recreate --wait --wait-timeout 90
  docker inspect \
    phanthy-navigation-sensors-shadow \
    phanthy-fast-livo2-shadow \
    --format '{{.Name}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} read_only={{.HostConfig.ReadonlyRootfs}} privileged={{.HostConfig.Privileged}} capdrop={{json .HostConfig.CapDrop}} devices={{json .HostConfig.Devices}} pid={{.HostConfig.PidMode}}'
"

echo "LIO shadow is running. Existing embodied-unitree-g1 was not replaced."
echo "Stop command: CONFIRM_G1_SHADOW_WRITE=YES $0 down ${SSH_TARGET} ${ROS_NAMESPACE_VALUE} ${NETWORK_INTERFACE_VALUE}"
