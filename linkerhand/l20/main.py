#!/usr/bin/env python3
"""LinkerHand L20 MCP Driver 入口。"""

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "linkerhand-l20-driver", "linkerhand-l20-device-bundle", build_plugins)
