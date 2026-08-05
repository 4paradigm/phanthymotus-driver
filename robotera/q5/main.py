#!/usr/bin/env python3
"""RobotEra Q5 MCP driver entrypoint."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(__file__, "robotera-q5-driver", "robotera-q5-device-bundle", build_plugins)
