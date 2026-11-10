#!/usr/bin/env bash
set -euo pipefail

find_gs_usb_interface() {
    local path driver found=""
    for path in /sys/class/net/*; do
        [[ -e "$path/device/driver" ]] || continue
        driver=$(readlink -f "$path/device/driver" 2>/dev/null || true)
        if [[ "${driver##*/}" == gs_usb ]]; then
            [[ -z "$found" ]] || {
                echo "multiple gs_usb interfaces found; set up one USB CAN adapter only" >&2
                return 1
            }
            found=${path##*/}
        fi
    done
    [[ -n "$found" ]] || return 1
    printf '%s\n' "$found"
}

action=${1:-up}
case "$action" in
    up)
        /usr/sbin/modprobe gs_usb
        iface=
        for _ in $(seq 1 30); do
            iface=$(find_gs_usb_interface || true)
            [[ -n "$iface" ]] && break
            sleep 1
        done
        [[ -n "$iface" ]] || { echo "gs_usb CAN interface did not appear" >&2; exit 1; }
        /usr/sbin/ip link set "$iface" down 2>/dev/null || true
        /usr/sbin/ip link set "$iface" type can bitrate 500000
        /usr/sbin/ip link set "$iface" up
        printf '%s\n' "$iface"
        ;;
    down)
        iface=$(find_gs_usb_interface || true)
        [[ -z "$iface" ]] || /usr/sbin/ip link set "$iface" down
        ;;
    interface)
        find_gs_usb_interface
        ;;
    *)
        echo "usage: $0 {up|down}" >&2
        exit 2
        ;;
esac
