#!/usr/bin/env python3
"""云深处山猫 S10 MCP Driver 入口。"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "deep-robotics-lynx-s10-driver", "deep-robotics-lynx-s10-device-bundle", build_plugins)
