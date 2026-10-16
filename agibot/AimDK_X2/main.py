#!/usr/bin/env python3
"""AgiBot X2 (AimDK) MCP Driver 入口。"""

import os
from pathlib import Path

os.environ.setdefault("NETWORK_INTERFACE", "develop0")

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "agibot-x2-driver", "agibot-x2-device-bundle", build_plugins)
