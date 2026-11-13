#!/usr/bin/env python3
"""UniX AI Panther driver entry point."""

try:
    from common import logsafe
    logsafe.install()
except ImportError:
    pass

import os

from common.vendor_runtime import run_driver
from device import build_plugins


def pin_robot_dds():
    """Make `config.yaml`'s CycloneDDS settings the ones that apply.

    `common.vendor_runtime.configure_cyclonedds` only fills `CYCLONEDDS_URI` in when it is
    unset, so a value inherited from the image or the host would silently win. Here that
    would be the failure this driver is most exposed to: the robot's nodes are bound to
    loopback, and a container on any other interface sees every topic name and receives no
    data. An operator who really wants another configuration sets PANTHER_CYCLONEDDS_URI.
    """
    override = os.environ.get("PANTHER_CYCLONEDDS_URI")
    if override:
        os.environ["CYCLONEDDS_URI"] = override
    else:
        os.environ.pop("CYCLONEDDS_URI", None)


if __name__ == "__main__":
    # The guard matters: panther_bridge starts its child processes with "spawn", which
    # re-imports this module in each child, and the child must not start a second driver.
    pin_robot_dds()
    run_driver(__file__, "unixai-panther-driver", "unixai-panther-device-bundle", build_plugins)
