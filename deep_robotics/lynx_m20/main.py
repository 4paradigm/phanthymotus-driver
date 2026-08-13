#!/usr/bin/env python3
"""云深处山猫 M20 MCP Driver 入口。"""

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "deep-robotics-lynx-m20-driver", "deep-robotics-lynx-m20-device-bundle", build_plugins)
