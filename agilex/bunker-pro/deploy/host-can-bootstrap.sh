#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
KERNEL_RELEASE=$(uname -r)
MODULE_DIR="/lib/modules/${KERNEL_RELEASE}"

fail() {
    echo "[bunker-can-setup] ERROR: $*" >&2
    exit 1
}

[[ "${EUID}" -eq 0 ]] || fail "container must run as root to provision the host CAN driver"
[[ -w "$MODULE_DIR" ]] || fail "host /lib/modules must be mounted read-write"
[[ -w /etc/modules-load.d && -w /etc/systemd/system && -w /usr/local/sbin ]] || \
    fail "host module/systemd directories must be mounted read-write"
[[ -S /run/dbus/system_bus_socket ]] || fail "host system D-Bus socket is not mounted"

usb_found=false
for vendor in /sys/bus/usb/devices/*/idVendor; do
    [[ -r "$vendor" ]] || continue
    if [[ "$(<"$vendor")" == 1d50 ]]; then
        product=${vendor%/idVendor}/idProduct
        [[ -r "$product" && "$(<"$product")" == 606f ]] && usb_found=true
    fi
done
$usb_found || fail "USB-CAN adapter 1d50:606f is not visible to the container"

if ! modinfo -k "$KERNEL_RELEASE" gs_usb >/dev/null 2>&1; then
    [[ -d "$MODULE_DIR/build" ]] || fail "gs_usb is missing and matching host headers are unavailable at ${MODULE_DIR}/build"
    build_dir=$(mktemp -d /tmp/bunker-gs-usb.XXXXXX)
    trap 'rm -rf "$build_dir"' EXIT
    cp "$SCRIPT_DIR/gs_usb.c" "$build_dir/gs_usb.c"
    cp "$SCRIPT_DIR/gs_usb-build/Makefile" "$build_dir/Makefile"
    make -C "$MODULE_DIR/build" M="$build_dir" modules
    install -D -m 0644 "$build_dir/gs_usb.ko" \
        "$MODULE_DIR/extra/gs_usb.ko"
    depmod -a "$KERNEL_RELEASE"
    echo "[bunker-can-setup] built gs_usb for ${KERNEL_RELEASE}"
fi

modprobe -S "$KERNEL_RELEASE" gs_usb || fail "could not load gs_usb for ${KERNEL_RELEASE}"

install -m 0644 "$SCRIPT_DIR/gs_usb.modules" /etc/modules-load.d/gs_usb.conf
install -m 0755 "$SCRIPT_DIR/bunker-can.sh" /usr/local/sbin/phanthy-bunker-can
install -m 0644 "$SCRIPT_DIR/bunker-can.service" \
    /etc/systemd/system/phanthy-bunker-can.service

# Use the host system bus directly. systemctl refuses to operate when the
# container's PID 1 is not systemd, even if the host D-Bus socket is mounted.
busctl=/usr/bin/busctl
bus_address=unix:path=/run/dbus/system_bus_socket
systemd_destination=org.freedesktop.systemd1
systemd_path=/org/freedesktop/systemd1
systemd_manager=org.freedesktop.systemd1.Manager
[[ -x "$busctl" ]] || fail "busctl is unavailable in the driver image"
"$busctl" --address="$bus_address" call "$systemd_destination" "$systemd_path" \
    "$systemd_manager" Reload || fail "could not reload host systemd over D-Bus"
"$busctl" --address="$bus_address" call "$systemd_destination" "$systemd_path" \
    "$systemd_manager" EnableUnitFiles as 1 phanthy-bunker-can.service false true || \
    fail "could not enable the persistent host CAN service over D-Bus"
# Restart on each container start so USB re-enumeration gets a fresh bitrate setup.
"$busctl" --address="$bus_address" call "$systemd_destination" "$systemd_path" \
    "$systemd_manager" RestartUnit ss phanthy-bunker-can.service replace || \
    fail "could not start the host CAN service over D-Bus"

iface=$(/usr/local/sbin/phanthy-bunker-can interface) || \
    fail "no network interface bound to gs_usb after module load"
details=$(/usr/sbin/ip -j -details link show dev "$iface")
python3 -c 'import json,sys; info=json.load(sys.stdin)[0]; can=info.get("linkinfo",{}).get("info_data",{}); state=can.get("state"); bitrate=can.get("bittiming",{}).get("bitrate"); assert info["flags"] and "UP" in info["flags"], "CAN link is down"; assert bitrate == 500000, "expected 500000 bit/s, got %s" % bitrate; print("[bunker-can-setup] %s %s %s bit/s" % (info["ifname"], state, bitrate))' <<<"$details"

export BUNKER_CAN_INTERFACE="$iface"
echo "[bunker-can-setup] host CAN ready: ${BUNKER_CAN_INTERFACE}"
