#!/usr/bin/env python3
"""Run from repository root: PYTHONPATH=. python3 agilex/piper/main.py."""
import os

from common.vendor_runtime import run_driver
from agilex.piper.device import build_plugins

if __name__ == "__main__":
    server_name = os.environ.get("PIPER_MCP_SERVER_NAME", "").strip() or "agilex-piper"
    run_driver(__file__, "agilex-piper-driver", server_name, build_plugins)
