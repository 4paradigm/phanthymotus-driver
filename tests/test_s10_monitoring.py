"""S10 monitor truthfulness, observed fault semantics and independent lifecycle."""
from __future__ import annotations

import copy
import time
import unittest

from test_s10_cards import cards, FakeBridge, Publications, eventually
from monitoring import ErrorObservations, DEVICE_ACTIONS


class PassiveMotion:
    events_topic = "/test_robot/s10_pro/motion_events"

    def __init__(self):
        self.events = [{"event": "requested", "timestamp_ms": int(time.time() * 1000) - 10000}]
        self.control_calls = []

    def status(self):
        return {"state": "idle", "phase": "locked", "events": copy.deepcopy(self.events)}

    def start(self):
        self.control_calls.append("start")

    def stop(self):
        self.control_calls.append("stop")


class S10MonitoringTests(unittest.TestCase):
    def setUp(self):
        self.bridge, self.output, self.motion = FakeBridge(), Publications(), PassiveMotion()
        self.plugins = cards.build_plugins({}, "test_robot", self.bridge, self.output, motion=self.motion)
        self.by_name = {plugin.name: plugin for plugin in self.plugins}

    def tearDown(self):
        for plugin in self.plugins:
            plugin.stop()

    def payload(self, name):
        return self.by_name[name]._payload()[0]

    def test_action_visibility_excludes_routing_fields_and_preserves_shared_settings(self):
        for plugin in self.plugins:
            schema = plugin.get_tool()["inputSchema"]
            actions = schema["x-action-params"]
            self.assertEqual([], actions["status"]["params"])
            self.assertEqual([], actions["stop"]["params"])
            self.assertEqual(set(plugin.get_tool()["configSchema"]["properties"]), set(actions["config"]["params"]))
            for action in actions.values():
                self.assertNotIn("instance_id", action["params"])
                self.assertNotIn("_tool_name", action["params"])
                self.assertNotIn("config", action["params"])

    def test_state_online_requires_motion_battery_and_hes_without_inventing_fault_clear(self):
        self.bridge.put("battery", {"data": [{"battery_level": 59}]})
        self.bridge.put("motion_info", {"data": {"motion_state": {"state": 17}, "gait_state": {"gait": 0},
                                                "vel_x": .2, "vel_y": 0., "vel_yaw": .1, "height": .3}})
        self.assertFalse(self.payload("state")["fresh"])
        self.bridge.put("hard_estop", {"value": 0})
        payload = self.payload("state")
        self.assertTrue(payload["fresh"])
        self.assertTrue(payload["summary"]["all_required_sources_fresh"])
        self.assertEqual("unknown", payload["summary"]["errors"]["current_fault_state"])
        self.assertEqual("unknown", payload["summary"]["motion"]["control_ownership"])
        self.assertEqual(17, payload["summary"]["motion"]["motion_state"])

    def test_motion_monitor_keeps_raw_data_and_unknown_height_units_and_modes(self):
        raw = {"data": {"motion_state": {"state": 0}, "gait_state": {"gait": 100},
                        "vel_x": .1, "vel_y": .2, "vel_yaw": .3, "height": .07}}
        self.bridge.put("motion_info", raw)
        payload = self.payload("motion_info")
        self.assertEqual(raw, payload["data"])
        self.assertEqual({"x": .1, "y": .2, "yaw": .3}, payload["summary"]["velocity"])
        self.assertEqual("unknown", payload["summary"]["motion_state_meaning"])
        self.assertEqual("unknown", payload["summary"]["gait_meaning"])
        self.assertIsNone(payload["summary"]["units"]["height"])
        self.assertEqual("unknown", payload["summary"]["usage_mode"])

    def test_hes_stale_or_missing_never_reports_released_or_safe(self):
        self.assertEqual("unknown", self.payload("hard_estop")["summary"]["indication"])
        for value, indication in ((0, "observed_released"), (1, "observed_triggered"), (8, "unknown")):
            self.bridge.put("hard_estop", {"value": value})
            summary = self.payload("hard_estop")["summary"]
            self.assertEqual(indication, summary["indication"])
            self.assertIsNone(summary["safe_to_move"])
            self.assertFalse(summary["manufacturer_mapping_verified"])
        self.bridge.put("hard_estop", {"value": 0}, age=10)
        summary = self.payload("hard_estop")["summary"]
        self.assertEqual("unknown", summary["indication"])
        self.assertEqual("observed_released", summary["last_observed_indication"])
        self.bridge.put("hard_estop", {"value": False})
        self.assertIsNone(self.payload("hard_estop")["summary"]["value"])

    def test_device_status_counts_observed_batteries_and_preserves_unknown_units(self):
        self.bridge.put("battery", {"data": [{"voltage": 7501, "current": -49, "battery_temperature": [23.2]}]})
        self.bridge.put("joints", {"data": {"joints_data": [{"name": [49, 48, 0], "motion_temp": 30., "driver_temp": 38.}]}})
        payload = self.payload("device_status")
        self.assertTrue(payload["fresh"])
        summary = payload["summary"]
        self.assertEqual(1, summary["battery_count"])
        self.assertEqual(7501, summary["batteries"][0]["voltage"])
        self.assertEqual("10", summary["joint_temperatures"][0]["joint"])
        self.assertTrue(all(unit is None for unit in summary["units"].values()))
        self.assertEqual("unknown", summary["charging"]["status"])
        self.assertEqual("unavailable", summary["compute_resources"]["cpu_status"]["status"])

    def test_resource_item_age_adds_envelope_age_and_never_refreshes_old_unit(self):
        self.bridge.put("cpu_status", {"latest": {"soc_id": "CPU_106", "avg_util": 25}, "items": [
            {"data": {"soc_id": "CPU_103", "avg_util": 5}, "age_s": 4.5, "received_at_ms": 10},
            {"data": {"soc_id": "CPU_106", "avg_util": 25}, "age_s": 0., "received_at_ms": 20},
        ], "truncated": True}, age=1.)
        resource = self.payload("device_status")["summary"]["compute_resources"]["cpu_status"]
        self.assertEqual(5.5, resource["items"][0]["age_s"])
        self.assertFalse(resource["items"][0]["fresh"])
        self.assertTrue(resource["items"][1]["fresh"])
        self.assertTrue(resource["truncated"])
        self.assertEqual("%", self.payload("device_status")["summary"]["compute_units"]["cpu_utilization"])

    def test_only_fresh_aggregate_establishes_current_active_fault_list(self):
        self.assertIsNone(self.payload("errors")["summary"]["current_faults"])
        self.bridge.put("fault_event", {"code": 42, "type": 1, "resources": ["0"], "source": ["motion"]})
        self.assertEqual("unknown", self.payload("errors")["health"])
        self.bridge.put("fault_aggregator", {"active_faults": []})
        payload = self.payload("errors")
        self.assertEqual("no_active_faults_reported", payload["health"])
        self.assertEqual([], payload["summary"]["current_faults"])
        self.bridge.put("fault_aggregator", {"active_faults": []}, age=10, sequence=2)
        self.assertEqual("unknown", self.payload("errors")["health"])

    def test_aggregate_logical_identity_deduplicates_code_resources_and_source(self):
        fault = {"name": "Motor", "code": 0x8001, "type": 1, "resources": ["0"], "source": ["driver"]}
        another = {**fault, "resources": ["1"]}
        self.bridge.put("fault_aggregator", {"active_faults": [fault, copy.deepcopy(fault), another]})
        summary = self.payload("errors")["summary"]
        self.assertEqual("active_faults_reported", summary["current_fault_state"])
        self.assertEqual(2, summary["active_fault_count"])
        self.assertEqual([fault, another], summary["current_faults"])
        self.bridge.put("fault_aggregator", {"active_faults": [None]}, sequence=2)
        self.assertIsNone(self.payload("errors")["summary"]["current_faults"])

    def test_fault_and_exception_event_history_classifies_recovery_without_all_clear(self):
        self.bridge.put("fault_event", {"type": 1, "code": 12, "timestamp": {"sec": 1}})
        first = self.payload("errors")["summary"]
        self.assertEqual("fault_started", first["history"][0]["classification"])
        for _ in range(5):
            self.payload("errors")
        self.assertEqual(1, self.payload("errors")["summary"]["history"][0]["observations"])
        self.bridge.put("fault_event", {"type": 2, "code": 12, "timestamp": {"sec": 2}}, sequence=2)
        self.bridge.put("exception", {"source": "motor", "description": "restored", "is_abnormal": False})
        summary = self.payload("errors")["summary"]
        self.assertEqual(2, len(summary["recovered_faults"]))
        self.assertEqual("unknown", summary["current_fault_state"])
        self.assertTrue(summary["semantics_verified"])
        self.assertIn("may miss", summary["note"])

    def test_error_history_is_bounded_and_periodic_equal_snapshot_is_deduplicated(self):
        self.bridge.put("fault_aggregator", {"active_faults": [], "timestamp": {"sec": 1}})
        self.payload("errors")
        self.bridge.put("fault_aggregator", {"active_faults": [], "timestamp": {"sec": 2}}, sequence=2)
        history = self.payload("errors")["summary"]["history"]
        self.assertEqual(1, len(history))
        self.assertEqual(2, history[0]["observations"])
        for sequence in range(70):
            self.bridge.put("fault_event", {"code": sequence, "type": 1}, sequence=sequence + 1)
            self.payload("errors")
        self.assertEqual(64, len(self.payload("errors")["summary"]["history"]))

    def test_local_cards_do_not_acquire_robot_sources_or_claim_robot_online(self):
        for name in ("capabilities", "device", "motion_events"):
            card = self.by_name[name]
            card.start({"refresh_hz": 30})
            eventually(lambda: bool(self.output.matching("/motion_events_view" if name == "motion_events" else "/" + name)))
            state = card.status()
            self.assertEqual("running", state["state"])
            self.assertFalse(state["connected"])
            self.assertFalse(state["fresh"])
            self.assertFalse(state["live_robot_data"])
            card.stop()
        self.assertEqual([], self.bridge.acquires)
        self.assertEqual([], self.bridge.releases)

    def test_device_is_actuator_but_every_mutation_is_explicitly_unavailable(self):
        card = self.by_name["device"]
        tool = card.get_tool()
        self.assertEqual("actuator", tool["type"])
        self.assertEqual(list(DEVICE_ACTIONS), tool["unavailable_actions"])
        self.assertEqual(["info", "start", "stop", "status", "config"], tool["inputSchema"]["properties"]["action"]["enum"])
        for action in DEVICE_ACTIONS:
            result = card.dispatch(action, {})
            self.assertEqual("unavailable", result["state"])
            self.assertFalse(result["accepted"])
            self.assertFalse(result["command_sent"])
        self.assertEqual([], self.bridge.acquires)

    def test_capabilities_exclude_charging_and_keep_optional_hardware_unregistered(self):
        data = self.payload("capabilities")["data"]
        self.assertFalse(data["motion"]["supported"])
        self.assertEqual("excluded", data["charging_action"]["status"])
        self.assertNotIn("charging", self.by_name)
        for name in ("lidar_rear", "camera_front", "camera_rear", "gps", "charge_status", "navigation"):
            self.assertNotIn(name, self.by_name)
            self.assertFalse(data["conditional"][name]["registered"])
            self.assertTrue(data["conditional"][name]["reason"])

    def test_motion_event_view_preserves_history_and_never_operates_or_notifies_motion(self):
        card = self.by_name["motion_events"]
        original = copy.deepcopy(self.motion.events)
        card.start({"refresh_hz": 30})
        eventually(lambda: len(self.output.matching("/motion_events_view")) >= 2)
        data = self.output.matching("/motion_events_view")[-1]["data"]
        self.assertEqual(original, data["events"])
        self.assertGreaterEqual(data["last_event_age_s"], 10.)
        self.assertEqual(0, data["completion_notifications_emitted_by_view"])
        self.assertEqual(self.motion.events_topic, data["event_source_topic"])
        self.assertNotEqual(self.motion.events_topic, card.topic)
        card.stop()
        self.assertEqual([], self.motion.control_calls)

    def test_ros_graph_preserves_discovery_data_and_never_claims_command_authority(self):
        raw = {"topics": [{"topic": "/GPS", "types": ["drdds/msg/NavSat"], "publisher_count": 1}],
               "sources": {"gps": {"topic_discovered": True, "last_received_at_ms": None}},
               "nodes": [], "read_only": True, "command_publishers_created": 0, "services_called": 0}
        self.bridge.put("ros_graph", raw)
        payload = self.payload("ros_graph")
        self.assertEqual(raw, payload["data"])
        self.assertEqual("unknown", payload["summary"]["control_ownership"])
        self.assertTrue(payload["summary"]["discovery_only"])
        self.assertEqual("ros2://graph", payload["source"]["ros_graph"]["topic"])


if __name__ == "__main__":
    unittest.main()
