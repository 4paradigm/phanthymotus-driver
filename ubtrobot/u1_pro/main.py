#!/usr/bin/env python3
"""UBTECH U1 Pro MCP driver entry point."""

import os

from common.vendor_runtime import load_config, run_driver
from device import build_plugins


if __name__ == "__main__":
    config = load_config(__file__)
    uri = config.get("ros", {}).get("cyclonedds_uri")
    if uri:
        os.environ["CYCLONEDDS_URI"] = str(uri)
    run_driver(__file__, "ubtrobot-u1-pro-driver", "ubtrobot-u1-pro-device-bundle", build_plugins)
