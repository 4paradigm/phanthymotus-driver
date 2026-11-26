#!/usr/bin/env python3
"""Bunker Mini driver entry point."""

from common.vendor_runtime import run_driver
from device import BunkerMiniDevice


def build_plugins(config, namespace, ros2):
    return [BunkerMiniDevice(config, namespace, ros2)]


if __name__ == "__main__":
    run_driver(__file__, "agilex-bunker-mini-driver", "AgileX Bunker Mini", build_plugins)
