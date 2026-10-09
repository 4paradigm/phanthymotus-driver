#!/usr/bin/env bash
set -euo pipefail
ROOT=/opt/phanthy-motus/data/a3-relay
PIDFILE=$ROOT/relay.pid
SOURCE=/work/agibot/A3/jazzy_relay.py
mkdir -p "$ROOT"
case "${1:-start}" in
start)
  if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then exit 0; fi
  rm -f "$PIDFILE"
  cp "$SOURCE" "$ROOT/jazzy_relay.py"
  # The host mount namespace is required for ADU's Jazzy installation.  Keep
  # the source under the shared data mount because /work only exists in the
  # container namespace.  /bin/bash is present on the ADU host; /usr/bin/bash
  # is not guaranteed there.
  nsenter -t 1 -m -u -n -p -- /bin/bash -lc \
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
