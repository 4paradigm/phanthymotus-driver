#!/usr/bin/env bash
set -euo pipefail
ROOT=/opt/phanthy-motus/data/a3-relay
PIDFILE=$ROOT/relay.pid
SOURCE=/work/agibot/A3/jazzy_relay.py
mkdir -p "$ROOT"
case "${1:-start}" in
start)
  if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null \
      && grep -q "Jazzy input ready" "$ROOT/relay.log" 2>/dev/null; then exit 0; fi
  if [[ -f "$PIDFILE" ]]; then kill "$(cat "$PIDFILE")" 2>/dev/null || true; fi
  rm -f "$PIDFILE"
  cp "$SOURCE" "$ROOT/jazzy_relay.py"
  # Enter the host mount namespace so this process uses the host's Python 3.12
  # and Jazzy ROS installation.  Use the minimal POSIX shell available on the
  # ADU root filesystem; the previous bash paths were not present there.
  : > "$ROOT/relay.log"
  nsenter -t 1 -m -u -n -p -- /bin/sh -c \
    "set -e; if [ -f /opt/ros/jazzy/setup.sh ]; then . /opt/ros/jazzy/setup.sh; elif [ -f /opt/ros/jazzy/setup.bash ]; then . /opt/ros/jazzy/setup.bash; else echo '[relay] ERROR: /opt/ros/jazzy is not visible in host mount namespace' >&2; exit 41; fi; python3 -c 'import rclpy, sensor_msgs' || { echo '[relay] ERROR: host Jazzy rclpy/sensor_msgs unavailable' >&2; exit 42; }; exec python3 $ROOT/jazzy_relay.py" \
    >"$ROOT/relay.log" 2>&1 &
  echo $! > "$PIDFILE"
  echo "[relay] host Jazzy relay started pid=$(cat "$PIDFILE") domain=232"
  ;;
stop)
  if [[ -f "$PIDFILE" ]]; then kill "$(cat "$PIDFILE")" 2>/dev/null || true; rm -f "$PIDFILE"; fi
  ;;
*) echo "usage: $0 {start|stop}" >&2; exit 2;;
esac
