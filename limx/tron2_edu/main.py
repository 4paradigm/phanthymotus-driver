from common.vendor_runtime import run_driver
from device import build_plugins

if __name__ == "__main__":
    run_driver(__file__, "limx-tron2-edu-driver", "limx-tron2-edu", build_plugins)
