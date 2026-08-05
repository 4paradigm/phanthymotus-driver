import unittest

from controlled_spatial import ControlledSpatialPlugin
from controlled_spatial_contract import (
    CONTROLLED_SPATIAL_ACTION_PARAMS,
    CONTROLLED_SPATIAL_ACTIONS,
    controlled_spatial_tool_definition,
)


EXPECTED_ACTION_PARAMS = {
    "start_mapping": ["map_name"],
    "stop_mapping": [],
    "tag_place": ["name", "description"],
    "untag_place": ["name"],
    "list_tags": [],
    "list_maps": [],
    "delete_map": ["map_name"],
    "load_map": ["map_name"],
    "navigate_to_tag": ["tag_name", "speed", "mode"],
    "navigate_to_pose": ["x", "y", "yaw", "speed", "mode"],
    "wait_navigation_done": ["stall_timeout"],
    "pause_nav": [],
    "resume_nav": [],
    "stop_nav": [],
}


class ControlledSpatialContractTest(unittest.TestCase):
    def test_runtime_plugin_uses_the_canonical_card(self):
        plugin = ControlledSpatialPlugin.__new__(ControlledSpatialPlugin)
        self.assertEqual(plugin._tool_def(), controlled_spatial_tool_definition())

    def test_card_matches_the_controlled_spatial_standard(self):
        tool = controlled_spatial_tool_definition()
        schema = tool["inputSchema"]

        self.assertEqual(tool["name"], "controlled_spatial")
        self.assertEqual(tool["type"], "actuator")
        self.assertFalse(tool["multiInstance"])
        self.assertEqual(schema["required"], ["action"])
        self.assertEqual(
            tuple(schema["properties"]["action"]["enum"]),
            CONTROLLED_SPATIAL_ACTIONS,
        )
        self.assertEqual(set(schema["x-action-params"]), set(CONTROLLED_SPATIAL_ACTIONS))
        self.assertEqual(
            {
                action: spec["params"]
                for action, spec in schema["x-action-params"].items()
            },
            EXPECTED_ACTION_PARAMS,
        )
        self.assertEqual(schema["x-action-params"], CONTROLLED_SPATIAL_ACTION_PARAMS)

    def test_each_call_gets_an_independent_contract_copy(self):
        first = controlled_spatial_tool_definition()
        second = controlled_spatial_tool_definition()
        first["inputSchema"]["properties"]["action"]["enum"].append("invalid")
        self.assertNotEqual(first, second)
        self.assertNotIn(
            "invalid", second["inputSchema"]["properties"]["action"]["enum"]
        )

    def test_wait_navigation_done_uses_the_advertised_90_second_default(self):
        class SmartMotionStub:
            def __init__(self):
                self.stall_timeout = None

            def wait_nav_done(self, *, stall_timeout):
                self.stall_timeout = stall_timeout
                return {"status": "arrived"}

        smart_motion = SmartMotionStub()
        plugin = ControlledSpatialPlugin.__new__(ControlledSpatialPlugin)
        plugin._smart_motion = smart_motion

        result = plugin.dispatch("wait_navigation_done", {})

        self.assertEqual(result, {"status": "arrived"})
        self.assertEqual(smart_motion.stall_timeout, 90.0)


if __name__ == "__main__":
    unittest.main()
