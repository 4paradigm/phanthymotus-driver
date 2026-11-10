#!/usr/bin/env python3
"""Bunker Pro driver entry point."""

from common.vendor_runtime import run_driver
from device import BunkerProDevice


def build_plugins(config, namespace, ros2):
    return [BunkerProDevice(config, namespace, ros2)]


if __name__ == "__main__":
    run_driver(__file__, "agilex-bunker-pro-driver", "AgileX Bunker Pro", build_plugins)
