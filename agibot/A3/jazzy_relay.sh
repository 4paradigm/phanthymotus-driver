#!/usr/bin/env bash
set -euo pipefail
ROOT=/opt/phanthy-motus/data/a3-relay
PIDFILE=$ROOT/relay.pid
SOURCE=/work/agibot/A3/jazzy_relay.py
HOST_SOURCE=/proc/1/root/tmp/a3-jazzy-relay.py
mkdir -p "$ROOT"
case "${1:-start}" in
start)
  if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null \
      && grep -q "Jazzy input ready" "$ROOT/relay.log" 2>/dev/null; then exit 0; fi
  if [[ -f "$PIDFILE" ]]; then kill "$(cat "$PIDFILE")" 2>/dev/null || true; fi
  rm -f "$PIDFILE"
  # Keep the host-side log bounded; Docker's rotation does not cover this
  # file because the relay lives in the host mount namespace.
  if [[ -f "$ROOT/relay.log" ]] && [[ $(stat -c %s "$ROOT/relay.log" 2>/dev/null || echo 0) -gt 10485760 ]]; then
    mv -f "$ROOT/relay.log" "$ROOT/relay.log.1" || true
  fi
  : > "$ROOT/relay.log"
  # Files created in the container's mount namespace are invisible after a
  # mount-namespace switch.  Write the relay into the host root via /proc/1/root
  # instead, then enter the host root (without -m) so the host Jazzy install and
  # /tmp path are visible.  PID/network namespaces are still entered for DDS.
  cp "$SOURCE" "$HOST_SOURCE"
  # Do not let nsenter inherit a removed container cwd.  Also avoid a login
  # shell: host profiles can re-source the container's Humble environment.
  cd /tmp
  nsenter -t 1 -r -u -n -p -- /usr/bin/bash --noprofile --norc -c \
    'set -e
     cd /tmp
     # The launcher inherits the Humble process environment.  Jazzy setup
     # rejects a preselected ROS_DISTRO and its setup files use optional
     # variables that are not compatible with bash -u.
     unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH
     unset COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
     unset AMENT_TRACE_SETUP_FILES AMENT_TRACE_SETUP
     export ROS_HOME=/opt/phanthy-motus/data/a3-relay/ros-home
     export ROS_LOG_DIR=/opt/phanthy-motus/data/a3-relay/ros-log
     mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
     if [ -f /opt/ros/jazzy/setup.sh ]; then
       . /opt/ros/jazzy/setup.sh
     elif [ -f /opt/ros/jazzy/setup.bash ]; then
       . /opt/ros/jazzy/setup.bash
     else
       echo "[relay] ERROR: host Jazzy setup not found" >&2; exit 41
     fi
     python3 -c "import rclpy, sensor_msgs" || {
       echo "[relay] ERROR: host Jazzy rclpy/sensor_msgs unavailable" >&2; exit 42;
     }
     exec python3 /tmp/a3-jazzy-relay.py' \
    >>"$ROOT/relay.log" 2>&1 &
  echo $! > "$PIDFILE"
  for _ in {1..100}; do
    if grep -q "Jazzy input ready" "$ROOT/relay.log" 2>/dev/null; then
      echo "[relay] host Jazzy relay ready pid=$(cat "$PIDFILE") domain=232"
      exit 0
    fi
    if ! kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "[relay] ERROR: host Jazzy relay exited; see $ROOT/relay.log" >&2
      tail -20 "$ROOT/relay.log" >&2 || true
      exit 43
    fi
    sleep 0.1
  done
  echo "[relay] ERROR: host Jazzy relay did not become ready; see $ROOT/relay.log" >&2
  exit 44
  ;;
stop)
  kill "$(cat "$PIDFILE" 2>/dev/null || echo 0)" 2>/dev/null || true
  rm -f "$PIDFILE"
  ;;
*) echo "usage: $0 {start|stop}" >&2; exit 2;;
esac
