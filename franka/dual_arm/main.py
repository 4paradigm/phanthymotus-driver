from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "franka-dual-arm-driver", "franka-dual-arm", build_plugins)
