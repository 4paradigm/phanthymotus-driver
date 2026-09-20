#!/usr/bin/env python3
"""Generic embodiment simulator — MCP driver entry point."""

from simulator.generic.plugins import build_plugins

from common.vendor_runtime import run_driver

if __name__ == "__main__":
    run_driver(
        __file__,
        "simulator-generic",
        "simulator-generic-device-bundle",
        build_plugins,
    )
