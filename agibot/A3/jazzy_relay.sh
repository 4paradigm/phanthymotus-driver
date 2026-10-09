#!/usr/bin/env bash
set -euo pipefail
ROOT=/opt/phanthy-motus/data/a3-relay
PIDFILE=$ROOT/relay.pid
SOURCE=/work/agibot/A3/jazzy_relay.py
RUNNER=$ROOT/relay-bash
mkdir -p "$ROOT"
case "${1:-start}" in
start)
  if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null \
      && grep -q "Jazzy input ready" "$ROOT/relay.log" 2>/dev/null; then exit 0; fi
  if [[ -f "$PIDFILE" ]]; then kill "$(cat "$PIDFILE")" 2>/dev/null || true; fi
  rm -f "$PIDFILE"
  cp "$SOURCE" "$ROOT/jazzy_relay.py"
  : > "$ROOT/relay.log"
  # The ADU runtime does not expose a usable systemd/D-Bus control path from
  # containers.  Place the interpreter in the shared data mount, then execute
  # it after entering PID 1's mount namespace.  The executable path exists in
  # both namespaces, avoiding nsenter's post-setns path lookup problem.
  cp /bin/bash "$RUNNER"
  chmod 0755 "$RUNNER"
  nsenter -t 1 -m -u -n -p -- "$RUNNER" -lc \
    "set -e; if [ -f /opt/ros/jazzy/setup.sh ]; then . /opt/ros/jazzy/setup.sh; elif [ -f /opt/ros/jazzy/setup.bash ]; then . /opt/ros/jazzy/setup.bash; else echo '[relay] ERROR: host Jazzy setup not found' >&2; exit 41; fi; python3 -c 'import rclpy, sensor_msgs' || { echo '[relay] ERROR: host Jazzy rclpy/sensor_msgs unavailable' >&2; exit 42; }; exec python3 $ROOT/jazzy_relay.py" \
    >>"$ROOT/relay.log" 2>&1 &
  echo $! > "$PIDFILE"
  echo "[relay] host Jazzy relay started via shared runner pid=$(cat "$PIDFILE") domain=232"
  ;;
stop)
  kill "$(cat "$PIDFILE" 2>/dev/null || echo 0)" 2>/dev/null || true
  rm -f "$PIDFILE"
  ;;
*) echo "usage: $0 {start|stop}" >&2; exit 2;;
esac
