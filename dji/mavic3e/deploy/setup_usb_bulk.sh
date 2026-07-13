#!/bin/bash
# setup_usb_bulk.sh — Configure USB gadget for DJI PSDK Bulk mode
#
# Creates USB gadget with VID=0x2CA3, PID=0xF001 and 3 FunctionFS bulk endpoints.
# Must run with root/privileged before psdk_bridge starts.
#
# Reference: DJI PSDK Raspberry Pi demo (raspi-usb-device-start.sh)

set -e

GADGET_DIR="/sys/kernel/config/usb_gadget/dji_psdk"
UDC_NAME=$(ls /sys/class/udc/ 2>/dev/null | head -1)

if [ -z "$UDC_NAME" ]; then
    echo "[usb_bulk] ERROR: no UDC found"
    exit 1
fi

# Remove existing Jetson l4t gadget if present (it conflicts)
if [ -d /sys/kernel/config/usb_gadget/l4t ]; then
    echo "" > /sys/kernel/config/usb_gadget/l4t/UDC 2>/dev/null || true
    sleep 1
    echo "[usb_bulk] disabled l4t gadget"
fi

# If our gadget already exists, just re-bind UDC
if [ -d "$GADGET_DIR" ]; then
    echo "$UDC_NAME" > "$GADGET_DIR/UDC" 2>/dev/null || true
    echo "[usb_bulk] gadget already configured, re-bound to $UDC_NAME"
    exit 0
fi

echo "[usb_bulk] creating DJI USB Bulk gadget..."

# Load modules
modprobe libcomposite 2>/dev/null || true

# Create gadget
mkdir -p "$GADGET_DIR"
cd "$GADGET_DIR"

echo 0x2CA3 > idVendor
echo 0xF001 > idProduct
echo 0x0001 > bcdDevice
echo 0x0200 > bcdUSB
echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

# Strings
mkdir -p strings/0x409
echo "psdk-jetson" > strings/0x409/serialnumber
echo "PhanthyMotus" > strings/0x409/manufacturer
echo "DJI PSDK Payload" > strings/0x409/product

# Configuration
mkdir -p configs/c.1
echo 0x80 > configs/c.1/bmAttributes
echo 250 > configs/c.1/MaxPower
mkdir -p configs/c.1/strings/0x409
echo "BULK" > configs/c.1/strings/0x409/configuration

# Create 3 FunctionFS bulk functions
for i in 1 2 3; do
    func="functions/ffs.bulk${i}"
    mkdir -p "$func"
    ln -sf "$func" "configs/c.1/"

    # Create mount point and mount functionfs
    mkdir -p "/dev/usb-ffs/bulk${i}"
    mount -t functionfs "bulk${i}" "/dev/usb-ffs/bulk${i}" -o mode=0777 2>/dev/null || true
done

echo "[usb_bulk] gadget created with 3 bulk endpoints"

# Initialize each bulk endpoint with USB descriptors (startup_bulk)
STARTUP_BULK="/usr/local/bin/startup_bulk"
if [ -x "$STARTUP_BULK" ]; then
    for i in 1 2 3; do
        "$STARTUP_BULK" "/dev/usb-ffs/bulk${i}" &
        sleep 0.5
    done
    echo "[usb_bulk] startup_bulk initialized all endpoints"
else
    echo "[usb_bulk] WARNING: startup_bulk not found, endpoints not initialized"
fi

# Bind to UDC
sleep 1
echo "$UDC_NAME" > UDC
echo "[usb_bulk] bound to UDC: $UDC_NAME"
echo "[usb_bulk] done"
