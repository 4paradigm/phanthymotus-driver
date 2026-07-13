#!/bin/bash
# setup_usb_bulk.sh — Configure USB gadget for DJI PSDK Bulk mode
#
# Reuses existing Jetson l4t gadget — changes VID/PID and adds FFS bulk functions.
# Must run with root/privileged before psdk_bridge starts.

set -e

GADGET_DIR="/sys/kernel/config/usb_gadget/l4t"
UDC_NAME=$(ls /sys/class/udc/ 2>/dev/null | head -1)

if [ -z "$UDC_NAME" ]; then
    echo "[usb_bulk] ERROR: no UDC found"
    exit 1
fi

if [ ! -d "$GADGET_DIR" ]; then
    echo "[usb_bulk] ERROR: no l4t gadget found at $GADGET_DIR"
    exit 1
fi

# Unbind UDC first (required to modify gadget)
echo "" > "$GADGET_DIR/UDC" 2>/dev/null || true
sleep 1

# Remove existing function symlinks from config
cd "$GADGET_DIR/configs/c.1"
for link in $(find . -maxdepth 1 -type l); do
    rm -f "$link"
done

# Remove existing functions (ignore errors)
for func in "$GADGET_DIR/functions/"*; do
    [ -d "$func" ] && rmdir "$func" 2>/dev/null || true
done

# Set DJI VID/PID for USB Bulk
echo 0x2CA3 > "$GADGET_DIR/idVendor"
echo 0xF001 > "$GADGET_DIR/idProduct"
echo 0x0200 > "$GADGET_DIR/bcdUSB"
echo 0xEF > "$GADGET_DIR/bDeviceClass"
echo 0x02 > "$GADGET_DIR/bDeviceSubClass"
echo 0x01 > "$GADGET_DIR/bDeviceProtocol"

echo "[usb_bulk] VID/PID set to 2CA3:F001"

# Create 3 FunctionFS bulk functions
for i in 1 2 3; do
    func="$GADGET_DIR/functions/ffs.bulk${i}"
    mkdir -p "$func"
    ln -sf "$func" "$GADGET_DIR/configs/c.1/ffs.bulk${i}"
    mkdir -p "/dev/usb-ffs/bulk${i}"
    mount -t functionfs "bulk${i}" "/dev/usb-ffs/bulk${i}" 2>/dev/null || true
done

echo "[usb_bulk] 3 bulk FFS functions created"

# Initialize each bulk endpoint with USB descriptors
STARTUP_BULK="/usr/local/bin/startup_bulk"
if [ -x "$STARTUP_BULK" ]; then
    for i in 1 2 3; do
        "$STARTUP_BULK" "/dev/usb-ffs/bulk${i}" &
        sleep 0.5
    done
    echo "[usb_bulk] endpoints initialized"
else
    echo "[usb_bulk] WARNING: startup_bulk not found"
    exit 1
fi

# Re-bind UDC
sleep 1
echo "$UDC_NAME" > "$GADGET_DIR/UDC"
echo "[usb_bulk] bound to UDC: $UDC_NAME"
echo "[usb_bulk] done"
