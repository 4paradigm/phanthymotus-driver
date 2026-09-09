#!/usr/bin/env python3
"""RealMan 二指夹爪 MCP Driver 入口。"""

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(__file__, "realman-gripper-driver", "realman-gripper-device-bundle", build_plugins)
