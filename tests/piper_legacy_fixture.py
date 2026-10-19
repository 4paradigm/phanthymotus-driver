"""Preserve tests of historical/internal intent primitives after UI replacement.

New public contracts and production bundle are tested in test_piper_unified.py.
This fixture changes only the runtime class, using production monitor/camera wiring.
"""
from agilex.piper.intent_control import IntentControlPlugin
from agilex.piper.intent_monitor import add_intent_monitor
from agilex.piper.camera_live import add_camera_live


def build_plugins(config, namespace, ros2):
    plugins = add_intent_monitor([IntentControlPlugin(config, namespace, ros2)], config, ros2)
    return add_camera_live(plugins, config, ros2)
