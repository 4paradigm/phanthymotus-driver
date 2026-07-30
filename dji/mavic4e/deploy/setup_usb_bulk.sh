#!/bin/bash
# setup_usb_bulk.sh — Configure USB gadget for DJI PSDK Bulk mode
#
# Modifies existing Jetson l4t gadget in-place (can't create new one).
# Must run with root/privileged.

set -x

UDC_NAME=$(ls /sys/class/udc/ 2>/dev/null | head -1)
if [ -z "$UDC_NAME" ]; then
    echo "[usb_bulk] ERROR: no UDC found"
    exit 1
fi

GADGET_DIR="/sys/kernel/config/usb_gadget/l4t"

if [ ! -d "$GADGET_DIR" ]; then
    echo "[usb_bulk] ERROR: no gadget found"
    exit 1
fi

cd "$GADGET_DIR"

# ── Step 1: Unbind UDC ───────────────────────────────────────────────
echo "" > UDC 2>/dev/null || true
sleep 1
echo "[usb_bulk] UDC unbound"

# ── Step 2: Remove existing function symlinks from config ────────────
for link in configs/c.1/ffs.* configs/c.1/acm.* configs/c.1/ncm.* configs/c.1/rndis.* configs/c.1/mass_storage.*; do
    [ -L "$link" ] && rm -f "$link"
done
echo "[usb_bulk] old function links removed"

# ── Step 3: Remove old functions ─────────────────────────────────────
for func in functions/acm.* functions/ncm.* functions/rndis.* functions/mass_storage.*; do
    [ -d "$func" ] && rmdir "$func" 2>/dev/null || true
done
echo "[usb_bulk] old functions removed"

# ── Step 4: Change VID/PID to DJI Bulk ───────────────────────────────
echo 0x2CA3 > idVendor
echo 0xF001 > idProduct
echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol
echo "[usb_bulk] VID/PID set to 2CA3:F001"

# ── Step 5: Create FunctionFS bulk functions ─────────────────────────
mkdir -p /dev/usb-ffs
for i in 1 2 3; do
    func="functions/ffs.bulk${i}"
    if [ ! -d "$func" ]; then
        mkdir -p "$func"
    fi
    if [ ! -L "configs/c.1/ffs.bulk${i}" ]; then
        ln -sf "$GADGET_DIR/$func" "configs/c.1/ffs.bulk${i}"
    fi
    mkdir -p "/dev/usb-ffs/bulk${i}"
    mountpoint -q "/dev/usb-ffs/bulk${i}" || \
        mount -o mode=0777,uid=2000,gid=2000 -t functionfs "bulk${i}" "/dev/usb-ffs/bulk${i}"
done
echo "[usb_bulk] FFS bulk functions created"

# ── Step 6: Initialize endpoints with startup_bulk ───────────────────
# NOTE: startup_bulk must be launched EXTERNALLY as a long-running daemon.
# It keeps ep0 open — if it dies, the gadget stops working.
# The calling process (main.py) handles this.
echo "[usb_bulk] FFS ready — startup_bulk must be launched externally"

# ── Step 7: Bind UDC ─────────────────────────────────────────────────
# NOTE: UDC binding is also done externally AFTER startup_bulk opens ep0.
echo "[usb_bulk] gadget configured — ready for startup_bulk + UDC bind"
