#!/usr/bin/env python3
"""Run from repository root: PYTHONPATH=. python3 agilex/piper/main.py."""
import os

from common.vendor_runtime import run_driver
from agilex.piper.device import build_plugins

if __name__ == "__main__":
    server_name = os.environ.get("PIPER_MCP_SERVER_NAME", "").strip() or "agilex-piper"
    registration_id = os.environ.get("PIPER_MCP_REGISTRATION_ID", "").strip() or "agilex-piper-driver"
    run_driver(__file__, registration_id, server_name, build_plugins)
