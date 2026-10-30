"""The marketplace must advertise the configured bundle's real public tools."""
from pathlib import Path

import yaml

from deep_robotics.s10_pro.device import build_plugins
from test_s10_loco_move import loco


def test_marketplace_catalogue_matches_supervised_runtime_without_motion(loco):
    motion, _, bridge, _, transports = loco
    manifest = yaml.safe_load((Path(__file__).resolve().parents[1] /
        'deep_robotics/s10_pro/driver.yaml').read_text())
    sensors = build_plugins({}, 'test', bridge, lambda *args: None, motion=motion)
    try:
        tools = []
        for plugin in [*sensors, motion]:
            tools.extend(plugin.get_tools() if hasattr(plugin, 'get_tools')
                         else [plugin.get_tool()])
        listed = manifest['cards']
        assert len(listed) == len({c['name'] for c in listed})
        assert {c['name']: c['type'] for c in listed} == {
            t['name']: t['type'] for t in tools}
        assert next(c for c in listed if c['name'] == 'model')['type'] == 'resource'
        assert not any(t.states or t.velocities for t in transports)
    finally:
        for plugin in sensors:
            plugin.stop()
