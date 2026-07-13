#!/bin/bash
# setup_usb_bulk.sh — Configure USB gadget for DJI PSDK Bulk mode
#
# Reference: DJI PSDK Raspberry Pi demo (raspi-usb-device-start.sh)
# Must run with root/privileged.

set -e

UDC_NAME=$(ls /sys/class/udc/ 2>/dev/null | head -1)
if [ -z "$UDC_NAME" ]; then
    echo "[usb_bulk] ERROR: no UDC found"
    exit 1
fi

GADGET_DIR="/sys/kernel/config/usb_gadget/dji_psdk"

# ── Step 1: Disable existing gadgets ──────────────────────────────────
# Unbind any existing gadget from UDC
for g in /sys/kernel/config/usb_gadget/*/UDC; do
    [ -f "$g" ] && echo "" > "$g" 2>/dev/null || true
done
sleep 1

# Try to cleanly remove l4t gadget (Jetson default)
if [ -d /sys/kernel/config/usb_gadget/l4t ]; then
    cd /sys/kernel/config/usb_gadget/l4t
    # Remove symlinks in configs
    find configs/ -type l -delete 2>/dev/null || true
    # Remove strings dirs
    rmdir configs/*/strings/0x409 2>/dev/null || true
    rmdir configs/c.1 2>/dev/null || true
    # Remove functions
    for f in functions/*; do
        [ -d "$f" ] && rmdir "$f" 2>/dev/null || true
    done
    rmdir strings/0x409 2>/dev/null || true
    cd /
    rmdir /sys/kernel/config/usb_gadget/l4t 2>/dev/null || true
    echo "[usb_bulk] removed l4t gadget"
fi

# ── Step 2: Create DJI PSDK gadget ───────────────────────────────────
if [ -d "$GADGET_DIR" ]; then
    # Already exists from previous run, just re-init endpoints
    echo "[usb_bulk] gadget already exists, re-using"
else
    mkdir -p "$GADGET_DIR"
    cd "$GADGET_DIR"

    echo 0x2CA3 > idVendor
    echo 0xF001 > idProduct
    echo 0x0001 > bcdDevice
    echo 0x0200 > bcdUSB
    echo 0xEF > bDeviceClass
    echo 0x02 > bDeviceSubClass
    echo 0x01 > bDeviceProtocol

    mkdir -p strings/0x409
    echo "psdk-jetson" > strings/0x409/serialnumber
    echo "PhanthyMotus" > strings/0x409/manufacturer
    echo "DJI-PSDK-Payload" > strings/0x409/product

    mkdir -p configs/c.1
    echo 0x80 > configs/c.1/bmAttributes
    echo 250 > configs/c.1/MaxPower

    # Create 3 FunctionFS bulk functions
    mkdir -p /dev/usb-ffs
    for i in 1 2 3; do
        mkdir -p /dev/usb-ffs/bulk${i}
        func="functions/ffs.bulk${i}"
        mkdir -p "$func"
        ln -sf "$GADGET_DIR/$func" "configs/c.1/ffs.bulk${i}"
        mount -o mode=0777,uid=2000,gid=2000 -t functionfs "bulk${i}" "/dev/usb-ffs/bulk${i}" 2>/dev/null || true
    done

    mkdir -p configs/c.1/strings/0x409
    echo "BULK1+BULK2+BULK3" > configs/c.1/strings/0x409/configuration

    echo "[usb_bulk] gadget created (VID=2CA3 PID=F001)"
fi

# ── Step 3: Initialize FFS endpoints ─────────────────────────────────
STARTUP_BULK="/usr/local/bin/startup_bulk"
if [ -x "$STARTUP_BULK" ]; then
    # Kill previous instances
    pkill -f startup_bulk 2>/dev/null || true
    sleep 0.5

    for i in 1 2 3; do
        "$STARTUP_BULK" "/dev/usb-ffs/bulk${i}" &
        sleep 1
    done
    echo "[usb_bulk] endpoints initialized"
else
    echo "[usb_bulk] ERROR: startup_bulk not found"
    exit 1
fi

# ── Step 4: Bind UDC ─────────────────────────────────────────────────
udevadm settle -t 5 2>/dev/null || true
echo "$UDC_NAME" > "$GADGET_DIR/UDC"
echo "[usb_bulk] bound to UDC: $UDC_NAME"
echo "[usb_bulk] done"
