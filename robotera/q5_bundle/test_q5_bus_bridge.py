"""Offline contract checks for the Q5 cross-domain sensor bridge."""

from __future__ import annotations

import json
import os
from pathlib import Path
import unittest
from unittest import mock

import q5_bus_bridge
from sensor_contract import topic_out


class _Mcp:
    def __init__(self):
        self.info_calls: list[str] = []

    def list_tools(self):
        return [
            {"name": "battery", "type": "sensor", "topic_out": [
                {"topic": "/q5/battery", "format": "data/json"},
                {"topic": "/q5/battery_alias", "format": "data/json"},
            ]},
            {"name": "base_drive", "type": "actuator", "topic_out": [
                {"topic": "/q5/base_drive", "format": "data/json"},
            ]},
            {"name": "q5_model", "type": "resource", "topic_out": [
                {"topic": "/q5/model", "format": "data/json"},
            ]},
            {"name": "no_topic", "type": "sensor", "topic_out": []},
        ]

    def sensor_info(self, name):
        self.info_calls.append(name)
        return {"name": name, "percentage": 63.0}


class Q5BusBridgeTests(unittest.TestCase):
    def test_sensor_topic_contract_does_not_depend_on_vendor_side_publisher(self):
        declared = topic_out("/nvidia_desktop/q5/battery", "data/json")
        self.assertEqual(declared, [{
            "topic": "/nvidia_desktop/q5/battery", "format": "data/json",
        }])
        self.assertEqual(
            q5_bus_bridge.select_sensor_tools([{
                "name": "battery", "type": "sensor", "topic_out": declared,
            }]),
            {"battery": ["/nvidia_desktop/q5/battery"]},
        )

    def test_single_container_entrypoint_keeps_dds_stacks_in_separate_processes(self):
        source = Path(__file__).with_name("q5_bundle_entrypoint.sh").read_text()
        self.assertIn('RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"', source)
        self.assertIn('RMW_IMPLEMENTATION="rmw_fastrtps_cpp"', source)
        self.assertIn('exec python3 /work/main.py', source)
        self.assertIn('exec python3 /work/q5_bus_bridge.py', source)
        self.assertIn('media/audio bridge', source)
        self.assertIn('wait -n "$driver_pid" "$bridge_pid"', source)

    def test_fastdds_bridge_uses_its_udp_profile_when_not_overridden(self):
        with mock.patch.dict(os.environ, {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}, clear=True):
            profile = q5_bus_bridge.configure_fastdds_transport()
            self.assertEqual(profile, str(q5_bus_bridge.DEFAULT_FASTDDS_PROFILE))
            self.assertEqual(os.environ["FASTDDS_DEFAULT_PROFILES_FILE"], profile)
            self.assertEqual(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"], profile)

    def test_fastdds_bridge_respects_a_deployment_profile(self):
        with mock.patch.dict(os.environ, {
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTDDS_DEFAULT_PROFILES_FILE": "/etc/fastdds/custom.xml",
        }, clear=True):
            self.assertEqual(q5_bus_bridge.configure_fastdds_transport(), "/etc/fastdds/custom.xml")
            self.assertNotIn("FASTRTPS_DEFAULT_PROFILES_FILE", os.environ)

    def test_only_sensor_tools_with_topics_are_selected(self):
        selected = q5_bus_bridge.select_sensor_tools(_Mcp().list_tools())
        self.assertEqual(selected, {
            "battery": ["/q5/battery", "/q5/battery_alias"],
        })

    def test_media_topics_are_reserved_for_the_typed_bridge(self):
        self.assertEqual(q5_bus_bridge.select_sensor_tools([{
            "name": "mic", "type": "sensor", "topic_out": [
                {"topic": "/q5/mic/audio", "format": "audio/pcm-16k"},
            ],
        }]), {})

    def test_mcp_envelope_parser_returns_data_not_protocol_envelope(self):
        response = {
            "jsonrpc": "2.0", "id": 7,
            "result": {"content": [{"type": "text", "text": json.dumps({
                "state": "running", "data": {"voltage": 61.9},
            })}]},
        }
        self.assertEqual(q5_bus_bridge.extract_data_payload(response), {"voltage": 61.9})

    def test_poll_never_calls_control_tools_and_publishes_data_payload(self):
        mcp = _Mcp()
        messages = []
        bridge = q5_bus_bridge.SensorBusBridge(mcp, lambda topic, data: messages.append((topic, data)))
        self.assertEqual(bridge.refresh(), {"battery": ["/q5/battery", "/q5/battery_alias"]})
        self.assertEqual(bridge.poll_once(), 2)
        self.assertEqual(mcp.info_calls, ["battery"])
        self.assertEqual([topic for topic, _ in messages], ["/q5/battery", "/q5/battery_alias"])
        self.assertEqual(json.loads(messages[0][1]), {"name": "battery", "percentage": 63.0})


if __name__ == "__main__":
    unittest.main()
