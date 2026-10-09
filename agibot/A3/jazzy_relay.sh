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
  # The container already exposes the host's /opt/ros/jazzy tree.  Do not
  # enter the host mount namespace: ADU denies that namespace transition and
  # its minimal root does not contain the container shell path.  Enter only
  # the host PID/network namespaces so Jazzy DDS sees the robot interfaces.
  : > "$ROOT/relay.log"
  nsenter -t 1 -u -n -p -- /usr/bin/bash -lc \
    "source /opt/ros/jazzy/setup.bash; exec python3 $ROOT/jazzy_relay.py" \
    >"$ROOT/relay.log" 2>&1 &
  echo $! > "$PIDFILE"
  echo "[relay] host Jazzy relay started pid=$(cat "$PIDFILE") domain=232"
  ;;
stop)
  if [[ -f "$PIDFILE" ]]; then kill "$(cat "$PIDFILE")" 2>/dev/null || true; rm -f "$PIDFILE"; fi
  ;;
*) echo "usage: $0 {start|stop}" >&2; exit 2;;
esac
