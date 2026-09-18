#!/usr/bin/env python3
"""UBTECH U1 Pro MCP driver entry point."""

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(__file__, "ubtrobot-u1-pro-driver", "ubtrobot-u1-pro-device-bundle", build_plugins)
