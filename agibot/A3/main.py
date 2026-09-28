#!/usr/bin/env python3
"""AgiBot A3 (AimDK v3.2) MCP Driver 入口。"""

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "agibot-a3-driver", "agibot-a3-device-bundle", build_plugins)
