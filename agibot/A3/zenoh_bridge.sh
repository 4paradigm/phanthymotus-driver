#!/usr/bin/env bash
set -euo pipefail

# A3 ADU: the robot DDS domain is 232; Agent Core and this driver use 42.
# The standalone bridge embeds CycloneDDS and therefore does not load the
# host's Jazzy Python/ROS environment. Only the host network/PID namespaces
# are needed for the robot-side participant.
readonly VERSION="1.10.1"
readonly URL="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${VERSION}/zenoh-bridge-ros2dds-${VERSION}-aarch64-unknown-linux-gnu-standalone.zip"
readonly SHA256="fdb64d942d4b6beccbe9b1f8a359a01a2fca95a023c27535bf97b233603fad5f"
readonly ROOT="/opt/phanthy-motus/data/a3-zenoh"
readonly BIN="${ROOT}/zenoh-bridge-ros2dds"
readonly VERSION_FILE="${ROOT}/version"
readonly HOST_PID="${ROOT}/host.pid"
readonly CONTAINER_PID="${ROOT}/container.pid"

log() { echo "[zenoh] $*"; }

install_bridge() {
  mkdir -p "${ROOT}"
  if [[ -x "${BIN}" ]] && [[ "$(cat "${VERSION_FILE}" 2>/dev/null || true)" = "${VERSION}" ]]; then
    return
  fi
  local tmp="${ROOT}/bridge.zip.tmp"
  log "downloading official bridge ${VERSION}"
  curl --fail --location --retry 3 --silent --show-error "${URL}" -o "${tmp}"
  echo "${SHA256}  ${tmp}" | sha256sum -c -
  unzip -p "${tmp}" zenoh-bridge-ros2dds > "${BIN}.tmp"
  chmod 0755 "${BIN}.tmp"
  mv -f "${BIN}.tmp" "${BIN}"
  printf '%s\n' "${VERSION}" > "${VERSION_FILE}"
  rm -f "${tmp}"
}

start() {
  install_bridge
  if [[ -f "${HOST_PID}" ]] && kill -0 "$(cat "${HOST_PID}")" 2>/dev/null; then
    log "host bridge already running pid=$(cat "${HOST_PID}")"
  else
    # Enter ADU's host namespaces. The binary is on /opt/phanthy-motus/data,
    # which is shared with the host by the deployment fragment.
    nsenter -t 1 -m -u -i -n -p -- env ROS_DOMAIN_ID=232 \
      "${BIN}" --mode router >"${ROOT}/host.log" 2>&1 &
    echo $! > "${HOST_PID}"
    log "host bridge started domain=232 pid=$(cat "${HOST_PID}")"
  fi
  if [[ -f "${CONTAINER_PID}" ]] && kill -0 "$(cat "${CONTAINER_PID}")" 2>/dev/null; then
    log "container bridge already running pid=$(cat "${CONTAINER_PID}")"
  else
    ROS_DOMAIN_ID=42 "${BIN}" --mode client -e tcp/127.0.0.1:7447 \
      >"${ROOT}/container.log" 2>&1 &
    echo $! > "${CONTAINER_PID}"
    log "container bridge started domain=42 pid=$(cat "${CONTAINER_PID}")"
  fi
}

stop() {
  for pidfile in "${HOST_PID}" "${CONTAINER_PID}"; do
    if [[ -f "${pidfile}" ]]; then
      kill "$(cat "${pidfile}")" 2>/dev/null || true
      rm -f "${pidfile}"
    fi
  done
}

case "${1:-start}" in
  start) start ;;
  stop) stop ;;
  *) echo "usage: $0 {start|stop}" >&2; exit 2 ;;
esac
