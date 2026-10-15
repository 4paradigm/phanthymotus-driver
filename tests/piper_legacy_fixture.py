"""Preserve tests of historical/internal intent primitives after UI replacement.

New public contracts and production bundle are tested in test_piper_unified.py.
This fixture changes only the runtime class, using production monitor/camera wiring.
"""
from unittest.mock import patch
from agilex.piper.device import build_plugins as production_plugins
from agilex.piper.intent_control import IntentControlPlugin


def build_plugins(config, namespace, ros2):
    with patch('agilex.piper.unified_control.UnifiedControlPlugin', IntentControlPlugin):
        return production_plugins(config, namespace, ros2)
