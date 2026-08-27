#!/usr/bin/env python3
"""RealMan RM75-6F-V MCP driver entry point."""

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(
        __file__,
        "realman-rm75-6f-v-driver",
        "realman-rm75-6f-v-device-bundle",
        build_plugins,
    )
