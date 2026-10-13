#!/usr/bin/env python3
"""BrainCo Revo 2 灵巧手 MCP Driver 入口。"""

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "brainco-revo2-driver", "brainco-revo2-device-bundle", build_plugins)
