"""Offline contract checks for the Q5 cross-domain sensor bridge."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

import fastdds_transport
import q5_bus_bridge
import q5_media_bridge
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
            {"name": "joints", "type": "sensor", "topic_out": [
                {"topic": "/q5/joints", "format": "sensor/skeleton"},
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
    def test_skeleton_polling_defaults_to_driver_publish_rate(self):
        self.assertEqual(q5_bus_bridge.DEFAULT_POLL_HZ, 10.0)

    def test_media_bridge_keeps_a_bounded_latest_frame_queue(self):
        source = Path(q5_bus_bridge.__file__.replace("q5_bus_bridge.py", "q5_media_bridge.py")).read_text()
        self.assertIn('"rgb": self._ctx.Queue(maxsize=2)', source)
        self.assertIn('"depth_jpeg": self._ctx.Queue(maxsize=2)', source)
        self.assertIn('"pointcloud": self._ctx.Queue(maxsize=2)', source)
        self.assertIn("for media_q in media_qs.values():", source)
        self.assertIn("media_q.get_nowait()", source)

    def test_media_bridge_sets_fastdds_before_importing_rclpy(self):
        source = Path(q5_media_bridge.__file__).read_text()
        worker_start = source.index("def _run_bridge_subprocess")
        worker_source = source[worker_start:]
        self.assertLess(worker_source.index('os.environ["ROS_DOMAIN_ID"] = "42"'),
                        worker_source.index("import rclpy"))
        self.assertLess(worker_source.index('os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"'),
                        worker_source.index("import rclpy"))
        self.assertLess(worker_source.index("configure_bundled_fastdds_transport()"),
                        worker_source.index("import rclpy"))

    def test_media_bridge_forces_bundled_profile(self):
        with tempfile.NamedTemporaryFile() as configured, \
                mock.patch.dict(os.environ, {
                    "FASTRTPS_DEFAULT_PROFILES_FILE": configured.name,
                    "FASTDDS_DEFAULT_PROFILES_FILE": configured.name,
                    "FASTDDS_BUILTIN_TRANSPORTS": "DEFAULT",
                }, clear=True):
            profile = q5_media_bridge.configure_bundled_fastdds_transport()

            bundled = str(fastdds_transport.DEFAULT_FASTDDS_PROFILE)
            self.assertEqual(profile, bundled)
            self.assertEqual(os.environ["FASTDDS_DEFAULT_PROFILES_FILE"], bundled)
            self.assertEqual(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"], bundled)
            self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", os.environ)

    def test_fastdds_transport_uses_existing_deployment_profile(self):
        with tempfile.NamedTemporaryFile() as configured, \
                mock.patch.dict(os.environ, {
                    "FASTRTPS_DEFAULT_PROFILES_FILE": configured.name,
                    "FASTDDS_BUILTIN_TRANSPORTS": "DEFAULT",
                }, clear=True):
            profile = fastdds_transport.configure_fastdds_transport()

            self.assertEqual(profile, configured.name)
            self.assertEqual(os.environ["FASTDDS_DEFAULT_PROFILES_FILE"], configured.name)
            self.assertEqual(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"], configured.name)
            self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", os.environ)

    def test_fastdds_transport_prefers_fleet_legacy_profile(self):
        with tempfile.NamedTemporaryFile() as legacy, \
                tempfile.NamedTemporaryFile() as canonical, \
                mock.patch.dict(os.environ, {
                    "FASTDDS_DEFAULT_PROFILES_FILE": canonical.name,
                    "FASTRTPS_DEFAULT_PROFILES_FILE": legacy.name,
                }, clear=True):
            profile = fastdds_transport.configure_fastdds_transport()

            self.assertEqual(profile, legacy.name)
            self.assertEqual(os.environ["FASTDDS_DEFAULT_PROFILES_FILE"], legacy.name)
            self.assertEqual(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"], legacy.name)

    def test_fastdds_transport_falls_back_when_configured_profile_is_missing(self):
        with mock.patch.dict(os.environ, {
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/missing/dds-local.xml",
        }, clear=True), mock.patch("builtins.print") as log:
            profile = fastdds_transport.configure_fastdds_transport()

            fallback = str(fastdds_transport.DEFAULT_FASTDDS_PROFILE)
            self.assertEqual(profile, fallback)
            self.assertEqual(os.environ["FASTDDS_DEFAULT_PROFILES_FILE"], fallback)
            self.assertEqual(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"], fallback)
            self.assertTrue(any("falling back" in str(call) for call in log.call_args_list))

    def test_fastdds_transport_logs_selected_profile(self):
        with tempfile.NamedTemporaryFile() as configured, \
                mock.patch.dict(os.environ, {
                    "FASTDDS_DEFAULT_PROFILES_FILE": configured.name,
                }, clear=True), mock.patch("builtins.print") as log:
            fastdds_transport.configure_fastdds_transport()

            log.assert_called_once_with(
                f"[q5-dds] using Fast DDS profile: {configured.name}", flush=True)

    def test_fastdds_transport_rejects_missing_fallback(self):
        missing = Path("/missing/q5-fastdds-udp.xml")
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(fastdds_transport, "DEFAULT_FASTDDS_PROFILE", missing):
            with self.assertRaisesRegex(RuntimeError, "Fast DDS profile unavailable"):
                fastdds_transport.configure_fastdds_transport()

    def test_bundled_fastdds_profile_is_loopback_only(self):
        root = ET.parse(fastdds_transport.DEFAULT_FASTDDS_PROFILE).getroot()
        namespace = {"dds": root.tag.partition("}")[0].removeprefix("{")}
        addresses = [element.text for element in root.findall(
            ".//dds:interfaceWhiteList/dds:address", namespace)]
        builtin = root.find(".//dds:useBuiltinTransports", namespace)

        self.assertEqual(addresses, ["127.0.0.1"])
        self.assertIsNotNone(builtin)
        self.assertEqual(builtin.text.strip().lower(), "false")

    def test_bridges_use_their_expected_fastdds_selectors(self):
        self.assertIs(q5_media_bridge.configure_bundled_fastdds_transport,
                      fastdds_transport.configure_bundled_fastdds_transport)
        self.assertIs(q5_bus_bridge.configure_fastdds_transport,
                      fastdds_transport.configure_fastdds_transport)

    def test_media_bridge_child_installs_logsafe_before_ros(self):
        source = Path(q5_media_bridge.__file__).read_text()
        worker_source = source[source.index("def _run_bridge_subprocess"):]
        self.assertLess(worker_source.index("logsafe.install(check_fd=False)"),
                        worker_source.index("import rclpy"))

    def test_camera_worker_child_installs_logsafe_before_ros(self):
        source = Path(__file__).with_name("q5_camera_worker.py").read_text()
        worker_source = source[source.index("def _run_camera_worker"):]
        self.assertLess(worker_source.index("logsafe.install(check_fd=False)"),
                        worker_source.index("import rclpy"))

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

    def test_only_sensor_tools_with_topics_are_selected(self):
        selected = q5_bus_bridge.select_sensor_tools(_Mcp().list_tools())
        self.assertEqual(selected, {
            "battery": ["/q5/battery", "/q5/battery_alias"],
            "joints": ["/q5/joints"],
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
        self.assertEqual(bridge.refresh(), {
            "battery": ["/q5/battery", "/q5/battery_alias"],
            "joints": ["/q5/joints"],
        })
        self.assertEqual(bridge.poll_once(), 3)
        self.assertEqual(mcp.info_calls, ["battery", "joints"])
        self.assertEqual([topic for topic, _ in messages], [
            "/q5/battery", "/q5/battery_alias", "/q5/joints",
        ])
        self.assertEqual(json.loads(messages[0][1]), {"name": "battery", "percentage": 63.0})


if __name__ == "__main__":
    unittest.main()
