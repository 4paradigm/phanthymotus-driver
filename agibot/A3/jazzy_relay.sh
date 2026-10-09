#!/usr/bin/env bash
set -euo pipefail
ROOT=/opt/phanthy-motus/data/a3-relay
PIDFILE=$ROOT/relay.pid
SOURCE=/work/agibot/A3/jazzy_relay.py
UNIT=/host-systemd/system/agibot-a3-jazzy-relay.service
SYSTEMCTL=/usr/local/bin/host-systemctl
mkdir -p "$ROOT"
case "${1:-start}" in
start)
  if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null \
      && grep -q "Jazzy input ready" "$ROOT/relay.log" 2>/dev/null; then exit 0; fi
  if [[ -f "$PIDFILE" ]]; then kill "$(cat "$PIDFILE")" 2>/dev/null || true; fi
  rm -f "$PIDFILE"
  cp "$SOURCE" "$ROOT/jazzy_relay.py"
  : > "$ROOT/relay.log"
  if [[ ! -x "$SYSTEMCTL" || ! -S /run/dbus/system_bus_socket || ! -d /host-systemd/system ]]; then
    echo "[relay] ERROR: host systemd control mounts are unavailable" >&2
    exit 41
  fi
  cat > "$UNIT" <<EOF
[Unit]
Description=AgiBot A3 Jazzy media relay
After=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash; exec /usr/bin/python3 $ROOT/jazzy_relay.py'
Restart=always
RestartSec=2
Environment=ROS_DOMAIN_ID=232
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
StandardOutput=append:$ROOT/relay.log
StandardError=append:$ROOT/relay.log

[Install]
WantedBy=multi-user.target
EOF
  "$SYSTEMCTL" daemon-reload >>"$ROOT/relay.log" 2>&1
  "$SYSTEMCTL" enable --now agibot-a3-jazzy-relay.service >>"$ROOT/relay.log" 2>&1
  "$SYSTEMCTL" --no-pager --plain status agibot-a3-jazzy-relay.service >>"$ROOT/relay.log" 2>&1 || true
  echo "systemd" > "$PIDFILE"
  echo "[relay] host Jazzy relay requested via systemd domain=232"
  ;;
stop)
  if [[ -x "$SYSTEMCTL" && -S /run/dbus/system_bus_socket ]]; then
    "$SYSTEMCTL" disable --now agibot-a3-jazzy-relay.service >/dev/null 2>&1 || true
    "$SYSTEMCTL" daemon-reload >/dev/null 2>&1 || true
  fi
  rm -f "$PIDFILE"
  ;;
*) echo "usage: $0 {start|stop}" >&2; exit 2;;
esac
