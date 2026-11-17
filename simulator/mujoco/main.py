#!/usr/bin/env python3
"""MCP host for the remote MuJoCo workbench."""

from common.vendor_runtime import run_driver
from simulator.mujoco.plugin import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "simulator-mujoco", "simulator-mujoco-device-bundle", build_plugins)
