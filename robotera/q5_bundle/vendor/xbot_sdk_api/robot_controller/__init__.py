"""
Robot Controller Library
A Python library for controlling robots using ROS2 services and actions.
"""

from .robot_controller import RobotController
from .trajectory_controller import TrajectoryController
from .mpc_controller import MPCController
from .version import __version__, __author__

__all__ = [
    "RobotController",
    "TrajectoryController",
    "MPCController"
]

