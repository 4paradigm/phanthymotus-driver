#!/usr/bin/env python3
"""Dobot CRA MCP entry point."""

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(
        __file__,
        driver_id="dobot-cra-driver",
        server_name="dobot-cra",
        build_plugins=build_plugins,
    )
