#!/usr/bin/env python3
"""Franka entry point using the shared Driver runtime."""

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(
        __file__,
        driver_id="franka-fr3-driver",
        server_name="franka-fr3",
        build_plugins=build_plugins,
    )
