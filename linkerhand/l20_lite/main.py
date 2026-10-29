#!/usr/bin/env python3
"""LinkerHand L20 Lite USB-RS485 MCP Driver entry point."""

from common.vendor_runtime import run_driver
from modbus_device import build_plugins


if __name__ == "__main__":
    run_driver(
        __file__,
        "linkerhand-l20-lite-driver",
        "linkerhand-l20-lite-device-bundle",
        build_plugins,
    )
