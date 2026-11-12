#!/usr/bin/env python3
"""RealMan RM75-6F-V entry point using the shared Driver runtime."""

# Deploy Approval V153 fresh-lifecycle acceptance; no runtime behavior change.

from common.vendor_runtime import run_driver
from device import build_plugins


if __name__ == "__main__":
    run_driver(
        __file__,
        driver_id="realman-rm75-6f-v-driver",
        server_name="realman-rm75-6f-v",
        build_plugins=build_plugins,
    )
