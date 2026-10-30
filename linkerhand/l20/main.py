#!/usr/bin/env python3
"""LinkerHand L20 USB-CAN MCP Driver entry point."""

from common.vendor_runtime import run_driver
from hand_device import build_plugins


if __name__ == "__main__":
    run_driver(
        __file__,
        "linkerhand-l20-driver",
        "linkerhand-l20-device-bundle",
        build_plugins,
    )
