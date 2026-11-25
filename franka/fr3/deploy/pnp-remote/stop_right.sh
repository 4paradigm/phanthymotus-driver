#!/bin/sh
# Let the supervisor interrupt the vendor scripts before systemd signals the cgroup.
set -eu
pid=${1:-}
case "$pid" in ''|*[!0-9]*|0|1) exit 0 ;; esac
kill -TERM "$pid" 2>/dev/null || exit 0
for _ in 1 2 3 4 5 6 7 8; do
    kill -0 "$pid" 2>/dev/null || exit 0
    sleep .5
done
