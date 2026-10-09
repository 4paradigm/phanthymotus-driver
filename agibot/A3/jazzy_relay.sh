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
  : > "$ROOT/relay.log"
  # Files created in the container's mount namespace are invisible after a
  # mount-namespace switch.  Write the relay into the host root via /proc/1/root
  # instead, then enter the host root (without -m) so the host Jazzy install and
  # /tmp path are visible.  PID/network namespaces are still entered for DDS.
  cp "$SOURCE" "$HOST_SOURCE"
  nsenter -t 1 -r -u -n -p -- /usr/bin/bash -lc \
    'set -eu
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
