#!/usr/bin/env python3
"""UBTECH U1 Pro MCP driver entrypoint."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))

from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "ubtech-u1-pro-driver", "ubtech-u1-pro-device-bundle", build_plugins)
