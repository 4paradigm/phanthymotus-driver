#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PROBE_SCRIPT="${SCRIPT_DIR}/probe-g1-rgb-time-offset.py"
readonly SSH_TARGET="${1:-}"
readonly SAMPLE_COUNT="${2:-120}"
readonly OUTPUT_PATH="${G1_RGB_TIME_PROBE_OUTPUT:-/private/tmp/g1-rgb-time-offset.json}"
readonly DRIVER_CONTAINER="embodied-unitree-g1"
readonly SSH_OPTS=(
  -o ClearAllForwardings=yes
  -o ControlMaster=no
  -o ControlPath=none
  -o BatchMode=yes
  -o ConnectTimeout=8
)

usage() {
  echo "Usage: CONFIRM_G1_SHADOW_WRITE=YES $0 <ssh-target> [sample-count]" >&2
  echo "Output: G1_RGB_TIME_PROBE_OUTPUT=/private/tmp/g1-rgb-time-offset.json" >&2
}

if (( $# < 1 || $# > 2 )); then
  usage
  exit 2
fi
[[ "${SSH_TARGET}" =~ ^[a-zA-Z0-9_.@-]+$ ]] || {
  echo "invalid ssh target: ${SSH_TARGET}" >&2
  exit 2
}
if ! [[ "${SAMPLE_COUNT}" =~ ^[1-9][0-9]{1,2}$ ]] || \
   (( SAMPLE_COUNT < 30 || SAMPLE_COUNT > 600 )); then
  echo "sample-count must be an integer from 30 to 600" >&2
  exit 2
fi
test -f "${PROBE_SCRIPT}" || {
  echo "missing probe: ${PROBE_SCRIPT}" >&2
  exit 1
}
test -d "$(dirname -- "${OUTPUT_PATH}")" || {
  echo "output directory does not exist: $(dirname -- "${OUTPUT_PATH}")" >&2
  exit 1
}
if [[ "${CONFIRM_G1_SHADOW_WRITE:-}" != "YES" ]]; then
  echo "Refusing robot write. This probe temporarily stops and restores ${DRIVER_CONTAINER}." >&2
  exit 2
fi

temporary_output="$(mktemp /private/tmp/g1-rgb-time-offset.XXXXXX)"
cleanup() {
  rm -f -- "${temporary_output}"
}
trap cleanup EXIT

echo "[time-sync] temporarily release D435I, sample GLOBAL_TIME, then restore Driver" >&2
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "
  set -euo pipefail
  driver='${DRIVER_CONTAINER}'
  current_mode=\"\$(docker inspect phanthy-fast-livo2-shadow --format '{{index .Config.Labels \"com.phanthy.navigation.mode\"}}' 2>/dev/null || true)\"
  if test \"\${current_mode}\" = mapping || test \"\${current_mode}\" = rgb_preview; then
    echo \"refusing time probe while controlled write mode is active: \${current_mode}\" >&2
    exit 1
  fi
  test \"\$(docker inspect \"\${driver}\" --format '{{.State.Running}}')\" = true
  driver_image=\"\$(docker inspect \"\${driver}\" --format '{{.Config.Image}}')\"
  test -n \"\${driver_image}\"
  realsense_present=0
  for vendor_file in /sys/bus/usb/devices/*/idVendor; do
    device_dir=\"\${vendor_file%/idVendor}\"
    if test -r \"\${vendor_file}\" && \
       test -r \"\${device_dir}/idProduct\" && \
       test \"\$(cat \"\${vendor_file}\")\" = 8086 && \
       test \"\$(cat \"\${device_dir}/idProduct\")\" = 0b3a; then
      realsense_present=1
      break
    fi
  done
  test \"\${realsense_present}\" = 1 || {
    echo \"Intel RealSense D435I USB device 8086:0b3a is not present\" >&2
    exit 1
  }

  driver_stopped=0
  restore_driver() {
    if test \"\${driver_stopped}\" = 1; then
      docker start \"\${driver}\" >/dev/null
      driver_stopped=0
    fi
  }
  trap restore_driver EXIT INT TERM

  driver_stopped=1
  docker stop -t 20 \"\${driver}\" >/dev/null
  docker run --rm -i \
    --privileged \
    --network host \
    --ipc host \
    -v /dev:/dev \
    --entrypoint python3 \
    \"\${driver_image}\" - \
      --width 1920 --height 1080 --fps 15 \
      --samples '${SAMPLE_COUNT}' --warmup 30
  restore_driver
  trap - EXIT INT TERM
  test \"\$(docker inspect \"\${driver}\" --format '{{.State.Running}}')\" = true
" < "${PROBE_SCRIPT}" > "${temporary_output}"

summary="$(python3 "${PROBE_SCRIPT}" --validate "${temporary_output}")"
install -m 0600 "${temporary_output}" "${OUTPUT_PATH}"
printf '%s\n' "${summary}"
printf 'output=%s\n' "${OUTPUT_PATH}"
printf 'next_env=G1_RGB_TIME_PROBE=%q\n' "${OUTPUT_PATH}"
