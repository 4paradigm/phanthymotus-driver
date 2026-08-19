from __future__ import annotations

import importlib.util
import json
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics/lynx_m20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


protocol = load("lynx_m20_basic_server", DRIVER / "basic_server.py")
m20 = load("lynx_m20_contract", DRIVER / "device.py")
nos_mapping = load("lynx_m20_nos_mapping", DRIVER / "nos_mapping.py")


class FakeNative:
    def __init__(self): self.velocity_calls = []
    def request(self, message_type, command, items=None):
        return {"type": message_type, "command": command, "items": items or {}}
    def send_velocity(self, command, items):
        self.velocity_calls.append((command, dict(items)))
        return {"command": command, "items": items}


class FakeNodes:
    def __init__(self):
        self.native = FakeNative()
        self.last_velocity = None
        self.motion_event_topic = "/host/lynx_m20/motion_events"
        self.events = []
        self.mapping_events = []
    def publish_motion_event(self, event, **data):
        payload = {"event": event, **data}
        self.events.append(payload)
        return payload
    def publish_motion_state(self, state): return {"motion_state": state}
    def publish_gait(self, gait): return {"gait": gait}
    def set_ros_velocity(self, x, y, yaw, duration=0):
        self.last_velocity = ("ros2", x, y, yaw, duration)
        return {"velocity": [x, y, yaw], "duration": duration, "auto_stop": duration > 0}
    def set_native_velocity(self, command, values, duration=0):
        self.last_velocity = ("native", command, values, duration)
        return {**self.native.send_velocity(command, values), "duration": duration, "auto_stop": duration > 0}
    def stop_velocity(self, reason="command"):
        self.last_velocity = ("stopped", reason)
        self.publish_motion_event("motion_stopped", reason=reason)
    def motion_summary(self): return {"state": 17}
    def begin_mapping_view(self, name): self.mapping_events.append(("begin", name))
    def mark_mapping_started(self): self.mapping_events.append(("started",))
    def mark_mapping_saving(self): self.mapping_events.append(("saving",))
    def finish_mapping_view(self, active_map=None): self.mapping_events.append(("finished", active_map))
    def fail_mapping_view(self, error): self.mapping_events.append(("failed", str(error)))


class LynxM20ContractTests(unittest.TestCase):
    def make_stream_nodes(self):
        nodes = object.__new__(m20.M20Nodes)
        nodes.config = {"velocity_command_timeout": 0.5, "topics": {"nav_cmd": "/NAV_CMD"}}
        nodes.native = FakeNative()
        nodes.native_velocity_command = 25
        nodes._motion_condition = threading.Condition(threading.Lock())
        nodes._motion_command = None
        nodes._motion_shutdown = False
        nodes._active_motion = None
        nodes.events = []
        nodes.nav_frames = []
        nodes.publish_motion_event = lambda event, **data: nodes.events.append({"event": event, **data})
        nodes._publish_nav_cmd = lambda velocity: nodes.nav_frames.append(tuple(velocity))
        nodes._motion_thread = threading.Thread(target=nodes._motion_stream_loop, daemon=True)
        nodes._motion_thread.start()
        return nodes

    def stop_stream_nodes(self, nodes):
        with nodes._motion_condition:
            nodes._motion_shutdown = True
            nodes._motion_condition.notify_all()
        nodes._motion_thread.join(1)

    def test_basic_server_json_frame_round_trip(self):
        frame = protocol.encode_frame(7, 2, 23, {"GaitParam": 0x3002})
        message_id, payload = protocol.decode_frame(frame)
        self.assertEqual(7, message_id)
        body = payload["PatrolDevice"]
        self.assertEqual((2, 23), (body["Type"], body["Command"]))
        self.assertEqual(0x3002, body["Items"]["GaitParam"])
        self.assertEqual(protocol.SYNC, frame[:4])

    def test_mapping_card_uses_only_documented_drmap_commands(self):
        class FakeMappingClient:
            def __init__(self): self.calls = []; self.state = "idle"
            def validate_map_name(self, name): return nos_mapping.NOSMappingClient.validate_map_name(name)
            def start_mapping(self, map_name, activate=True):
                self.calls.append(("start", map_name, activate)); self.state = "mapping"; return {"state": "mapping"}
            def stop_mapping(self): self.calls.append(("stop",)); self.state = "idle"; return {"state": "saved"}
            def status(self): return {"state": self.state}
            def list_maps(self): self.calls.append(("list",)); return {"maps": []}

        client = FakeMappingClient()
        completions = []
        completed = threading.Event()
        def notify(*args):
            completions.append(args)
            completed.set()
        plugin = m20.M20ProMappingPlugin(FakeNodes(), client=client, notifier=notify)
        tool_def = plugin.get_tool()
        self.assertEqual(("mapping", "actuator"), (tool_def["name"], tool_def["type"]))
        self.assertEqual(
            {"actions": ["start_mapping", "stop_mapping"], "timeout": 180},
            tool_def["inputSchema"]["x-completion"],
        )
        self.assertEqual(
            ["map_name", "activate"],
            tool_def["inputSchema"]["x-action-params"]["start_mapping"]["params"],
        )
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {}))
        self.assertEqual({"state": "idle"}, plugin.dispatch("stop", {}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("info", {}))
        accepted = plugin.dispatch("start_mapping", {"map_name": "floor_1", "activate": False})
        self.assertEqual("accepted", accepted["state"])
        self.assertTrue(accepted["action_id"].startswith("m20_mapping_"))
        self.assertTrue(completed.wait(1))
        self.assertEqual("completed", completions[-1][1])
        completed.clear()
        plugin.dispatch("stop_mapping", {})
        self.assertTrue(completed.wait(1))
        plugin.dispatch("list_maps", {})
        self.assertEqual(
            [("start", "floor_1", False), ("stop",), ("list",)],
            client.calls,
        )
        self.assertEqual(
            [("begin", "floor_1"), ("started",), ("saving",), ("finished", None)],
            plugin.nodes.mapping_events,
        )
        with self.assertRaisesRegex(ValueError, "boolean"):
            plugin.dispatch("start_mapping", {"map_name": "floor_1", "activate": "false"})

    def test_mapping_worker_reports_failure_to_acp(self):
        class FailingClient:
            @staticmethod
            def validate_map_name(name): return name
            def start_mapping(self, **kwargs): raise RuntimeError("ssh failed")

        completions = []
        completed = threading.Event()
        def notify(*args): completions.append(args); completed.set()
        plugin = m20.M20ProMappingPlugin(FakeNodes(), client=FailingClient(), notifier=notify)
        accepted = plugin.dispatch("start_mapping", {"map_name": "floor1", "activate": True})
        self.assertEqual("accepted", accepted["state"])
        self.assertTrue(completed.wait(1))
        self.assertEqual((accepted["action_id"], "failed"), completions[0][:2])
        self.assertIn("ssh failed", completions[0][2]["error"])
        self.assertEqual(("failed", "ssh failed"), plugin.nodes.mapping_events[-1])

    def test_live_mapping_pointcloud_layout_and_map_transform(self):
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            SimpleNamespace(name="z", offset=8, datatype=7, count=1),
        ]
        raw = struct.pack("<fffIfffI", 1.0, 2.0, 3.0, 99, -1.0, 0.5, 0.0, 42)
        cloud = SimpleNamespace(
            fields=fields, point_step=16, width=2, height=1,
            is_bigendian=False, data=raw,
        )
        self.assertEqual([(1.0, 2.0, 3.0), (-1.0, 0.5, 0.0)], m20._pointcloud_xyz(cloud))

        half = 2 ** -0.5
        pose = {"x": 10.0, "y": 20.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": half, "qw": half}
        transformed = m20._transform_point((1.0, 0.0, 0.0), pose)
        self.assertAlmostEqual(10.0, transformed[0], places=6)
        self.assertAlmostEqual(21.0, transformed[1], places=6)
        self.assertAlmostEqual(0.0, transformed[2], places=6)

    def test_mapping_packet_carries_map_name_and_runtime_metadata(self):
        metadata = {
            "version": 3,
            "state": "live_mapping",
            "requested_map": "floor_1",
            "active_map": None,
        }
        packet = m20._mapping_packet([(1, 2, 3)], {"x": 4, "y": 5, "yaw": 0.25}, metadata)
        _, _, _, flags, point_count = struct.unpack_from("<fffBI", packet)
        self.assertEqual((0x07, 1), (flags, point_count))
        metadata_offset = 17 + point_count * 12
        metadata_size = struct.unpack_from("<I", packet, metadata_offset)[0]
        decoded = json.loads(packet[metadata_offset + 4:metadata_offset + 4 + metadata_size])
        self.assertEqual("floor_1", decoded["requested_map"])
        self.assertEqual("live_mapping", decoded["state"])

    def test_live_mapping_callback_accumulates_transformed_points(self):
        class FakePublisher:
            def __init__(self): self.messages = []
            def publish(self, message): self.messages.append(message)

        class FakeUInt8:
            def __init__(self): self.data = []

        nodes = object.__new__(m20.M20Nodes)
        nodes.config = {"mapping_visualization": {
            "voxel_size": 0.08, "max_buffer_points": 100, "max_points": 100, "publish_hz": 100,
        }}
        nodes.lock = threading.Lock()
        nodes.values = {}
        nodes._mapping_lock = threading.Lock()
        nodes._mapping_pose = None
        nodes._mapping_voxels = {}
        nodes._mapping_last_points = []
        nodes._mapping_last_grid_msg = None
        nodes._mapping_uint8_type = FakeUInt8
        nodes._mapping_last_publish = 0.0
        nodes._mapping_runtime = {
            "state": "idle", "requested_map": None, "active_map": None,
            "source": None, "source_frame": None, "target_frame": "map",
            "update_count": 0, "point_count": 0, "buffer_point_count": 0,
            "last_update_time": None,
        }
        nodes.mapping_pub = FakePublisher()
        nodes.begin_mapping_view("floor_2")

        quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        pose = SimpleNamespace(
            position=SimpleNamespace(x=10.0, y=20.0, z=0.0),
            orientation=quaternion,
        )
        odometry = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(pose=pose),
        )
        nodes._slam_odometry_callback(odometry)

        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            SimpleNamespace(name="z", offset=8, datatype=7, count=1),
        ]
        cloud = SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link"), fields=fields,
            point_step=16, width=1, height=1, is_bigendian=False,
            data=struct.pack("<fffI", 1.0, 2.0, 3.0, 0),
        )
        nodes._live_mapping_callback(FakeUInt8)(cloud)

        self.assertEqual(1, len(nodes.mapping_pub.messages))
        packet = bytes(nodes.mapping_pub.messages[0].data)
        _, _, _, flags, count = struct.unpack_from("<fffBI", packet)
        self.assertEqual((0x07, 1), (flags, count))
        self.assertEqual((11.0, 22.0, 3.0), struct.unpack_from("<fff", packet, 17))
        metadata_offset = 17 + count * 12
        metadata_size = struct.unpack_from("<I", packet, metadata_offset)[0]
        metadata = json.loads(packet[metadata_offset + 4:metadata_offset + 4 + metadata_size])
        self.assertEqual(("live_mapping", "floor_2", "grid_map_3d"), (
            metadata["state"], metadata["requested_map"], metadata["source"],
        ))

    def test_mapping_view_encodes_supported_canvas_packet(self):
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        origin = SimpleNamespace(position=SimpleNamespace(x=-1.0, y=2.0), orientation=quaternion)
        grid = SimpleNamespace(
            info=SimpleNamespace(width=3, height=2, resolution=0.5, origin=origin),
            data=[0, 50, -1, 100, 49, 80],
        )
        packet = m20.encode_occupancy_grid(grid, {"x": 1, "y": 2, "yaw": 0.25})
        robot_x, robot_y, robot_yaw, flags, count = m20.struct.unpack_from("<fffBI", packet)
        self.assertEqual((1.0, 2.0, -0.25, 0x03, 3), (robot_x, robot_y, robot_yaw, flags, count))
        points = m20.struct.unpack_from("<9f", packet, 17)
        self.assertEqual((-0.25, 2.25, 0.0), points[:3])
        self.assertEqual((-0.75, 2.75, 0.0), points[3:6])
        self.assertEqual((0.25, 2.75, 0.0), points[6:9])

    def test_mapping_view_static_and_dynamic_contract_match(self):
        nodes = FakeNodes()
        nodes.mapping_topic = "/host/lynx_m20/mapping_view"
        nodes.mapping_view_snapshot = lambda: {
            "state": "live_mapping", "requested_map": "floor_1", "point_count": 123,
        }
        plugin = m20.M20MappingViewPlugin(nodes)
        expected = [{"topic": nodes.mapping_topic, "format": "sensor/mapping"}]
        self.assertEqual(expected, plugin.get_tool()["topic_out"])
        started = plugin.dispatch("start", {})
        self.assertEqual(expected, started["topic_out"])
        self.assertEqual(
            ("running", "live_mapping", "floor_1", 123),
            (started["state"], started["mapping_state"], started["requested_map"], started["point_count"]),
        )
        info = plugin.dispatch("info", {})
        self.assertEqual(expected, info["topic_out"])
        self.assertEqual(("live_mapping", "floor_1", 123), (info["state"], info["requested_map"], info["point_count"]))
        self.assertEqual({"state": "idle"}, plugin.dispatch("stop", {}))

    def test_mapping_view_start_reports_running_when_mapping_is_idle(self):
        nodes = FakeNodes()
        nodes.mapping_topic = "/host/lynx_m20/mapping_view"
        nodes.mapping_view_snapshot = lambda: {
            "state": "idle", "requested_map": None, "point_count": 0,
        }
        started = m20.M20MappingViewPlugin(nodes).dispatch("start", {})
        self.assertEqual("running", started["state"])
        self.assertEqual("idle", started["mapping_state"])
        self.assertEqual(0, started["point_count"])

    def test_mapping_view_supports_live_and_latched_grid_publishers(self):
        source = (DRIVER / "device.py").read_text()
        self.assertIn("DurabilityPolicy.VOLATILE", source)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("ReliabilityPolicy.BEST_EFFORT", source)
        self.assertIn("ReliabilityPolicy.RELIABLE", source)

    def test_mapping_ssh_is_pinned_noninteractive_and_rejects_shell_input(self):
        class Completed:
            returncode = 0
            stdout = "Start mapping program"
            stderr = ""

        calls = []
        def runner(command, **kwargs):
            calls.append((command, kwargs))
            return Completed()

        client = nos_mapping.NOSMappingClient({
            "host": "10.21.31.106",
            "user": "user",
            "identity_file": "/secrets/key",
            "known_hosts_file": "/secrets/known_hosts",
        }, runner=runner)
        result = client.start_mapping("floor-1", activate=False)
        command = calls[0][0]
        self.assertEqual("mapping", result["state"])
        self.assertIn("BatchMode=yes", command)
        self.assertIn("HostKeyAlgorithms=ssh-ed25519", command)
        self.assertIn("StrictHostKeyChecking=yes", command)
        self.assertIn("UserKnownHostsFile=/secrets/known_hosts", command)
        self.assertEqual(
            "sudo -n /usr/local/sbin/phanthy-m20-mapping start floor-1 false",
            command[-1],
        )
        self.assertFalse(calls[0][1]["check"])
        for invalid in ("", "../map", "map name", "map;reboot", "a" * 65):
            with self.assertRaises(ValueError):
                client.start_mapping(invalid)

        client.stop_mapping()
        self.assertEqual(
            "sudo -n /usr/local/sbin/phanthy-m20-mapping stop",
            calls[-1][0][-1],
        )

    def test_nos_helper_has_a_strict_command_allowlist(self):
        helper = (DRIVER / "nos_mapping_helper.sh").read_text()
        self.assertIn('case "${action}" in', helper)
        self.assertIn('^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$', helper)
        self.assertIn('exec /usr/local/bin/drmap mapping -s -n "${map_name}"', helper)
        self.assertIn("exec /usr/local/bin/drmap stop_mapping", helper)

    def test_mapping_status_distinguishes_active_service(self):
        responses = iter([
            (0, "active\n"),
            (0, "/var/opt/robot/data/maps/floor-1\n"),
            (0, "floor-2\nfloor-1\n"),
        ])
        def runner(command, **kwargs):
            code, stdout = next(responses)
            return type("Completed", (), {"returncode": code, "stdout": stdout, "stderr": ""})()

        client = nos_mapping.NOSMappingClient({}, runner=runner)
        status = client.status()
        maps = client.list_maps()
        self.assertEqual(("mapping", "/var/opt/robot/data/maps/floor-1"), (status["state"], status["active_map"]))
        self.assertEqual(["floor-1", "floor-2"], maps["maps"])

    def test_tcp_stream_decoder_handles_fragmented_and_concatenated_frames(self):
        first = protocol.encode_frame(1, 100, 100)
        second = protocol.encode_frame(2, 1002, 6)
        decoder = protocol.StreamDecoder()
        self.assertEqual([], decoder.feed(first[:9]))
        decoded = decoder.feed(first[9:] + second)
        self.assertEqual([1, 2], [item[0] for item in decoded])

    def test_official_motion_commands_use_documented_values(self):
        plugin = m20.M20MotionPlugin(FakeNodes())
        self.assertEqual(1, plugin.dispatch("stand", {"transport": "native"})["items"]["MotionParam"])
        self.assertEqual(4, plugin.dispatch("lie", {"transport": "native"})["items"]["MotionParam"])
        gait = plugin.dispatch("gait", {"transport": "native", "gait": "agile_flat"})
        self.assertEqual((2, 23, 0x3002), (gait["type"], gait["command"], gait["items"]["GaitParam"]))
        axis = plugin.dispatch("axis", {"x": 0.5, "y": 0, "yaw": -0.2})
        self.assertEqual(21, axis["command"])
        timed = plugin.dispatch("axis", {"x": 0.1, "y": 0, "yaw": 0, "duration": 2})
        self.assertEqual((2, True), (timed["duration"], timed["auto_stop"]))
        safe_default = plugin.dispatch("axis", {"x": 0.1, "y": 0, "yaw": 0})
        self.assertEqual((1.0, True), (safe_default["duration"], safe_default["auto_stop"]))
        continuous = plugin.dispatch("velocity", {"x": 0.1, "duration": 0})
        self.assertEqual((0, False), (continuous["duration"], continuous["auto_stop"]))
        stopped = plugin.dispatch("stopmotion", {"transport": "native"})
        self.assertEqual(("stopped", {"state": "stopped"}), (plugin.nodes.last_velocity[0], stopped))
        with self.assertRaises(ValueError): plugin.dispatch("axis", {"x": 1.1, "y": 0, "yaw": 0})
        with self.assertRaises(ValueError): plugin.dispatch("velocity", {"duration": 61})

    def test_g1_style_motion_card_exposes_contextual_parameters(self):
        motion = m20.M20MotionPlugin(FakeNodes()).get_tool()
        self.assertEqual("actuator", motion["type"])
        self.assertEqual(["transport"], motion["inputSchema"]["x-action-params"]["stand"]["params"])
        self.assertEqual(["transport", "x", "y", "yaw", "duration"], motion["inputSchema"]["x-action-params"]["velocity"]["params"])
        self.assertEqual(["transport"], motion["inputSchema"]["x-action-params"]["stopmotion"]["params"])
        self.assertNotIn("stop", motion["inputSchema"]["x-action-params"])
        self.assertEqual((1, 60), (motion["inputSchema"]["properties"]["duration"]["default"], motion["inputSchema"]["properties"]["duration"]["maximum"]))

    def test_axis_schema_exposes_safe_hardware_guidance_without_motion_defaults(self):
        properties = m20.M20MotionPlugin(FakeNodes()).get_tool()["inputSchema"]["properties"]
        self.assertEqual([1, -1], properties["x"]["examples"])
        self.assertEqual([0.5, -0.5], properties["y"]["examples"])
        self.assertEqual([0.4, -0.4], properties["yaw"]["examples"])
        for axis in ("x", "y", "yaw"):
            self.assertEqual((-1, 1), (properties[axis]["minimum"], properties[axis]["maximum"]))
            self.assertNotIn("default", properties[axis])
            self.assertIn("blank=0", properties[axis]["description"][:40])
        self.assertEqual("Use +/-1; blank=0.", properties["x"]["description"].split(" Forward")[0])
        self.assertEqual("Use |y| >= 0.5; blank=0.", properties["y"]["description"].split(" Left")[0])
        self.assertEqual("Use +/-0.4; +/-1 ~= 45deg; blank=0.", properties["yaw"]["description"].split(" Normalized")[0])

    def test_motion_events_is_a_distinct_read_only_sensor(self):
        tool_def = m20.M20MotionEventsPlugin(FakeNodes()).get_tool()
        self.assertEqual(("motion_events", "sensor", False), (tool_def["name"], tool_def["type"], tool_def["multiInstance"]))
        self.assertEqual(
            [{"topic": "/host/lynx_m20/motion_events", "format": "data/json"}],
            tool_def["topic_out"],
        )

    def test_json_state_stream_publishes_string_payload_and_keeps_snapshot_value(self):
        class FakePublisher:
            def __init__(self): self.messages = []
            def publish(self, message): self.messages.append(message)

        class FakeString:
            def __init__(self): self.data = ""

        class FakeImu:
            def __init__(self):
                self.orientation = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}

        nodes = object.__new__(m20.M20Nodes)
        nodes.lock = threading.Lock()
        nodes.values = {}
        publisher = FakePublisher()
        callback = nodes._callback("imu", publisher, as_json=True, string_type=FakeString)
        callback(FakeImu())

        payload = json.loads(publisher.messages[0].data)
        self.assertEqual({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, payload["orientation"])
        self.assertEqual(payload, nodes.values["imu"])

    def test_json_state_streams_use_agent_core_compatible_format(self):
        source = (DRIVER / "device.py").read_text()
        self.assertNotIn('("imu", Imu, "/IMU", "data/imu"', source)
        self.assertIn('("imu", Imu, "/IMU", "data/json"', source)
        self.assertIn("core_msg_type = String if as_json else msg_type", source)

    def test_motion_events_separate_request_acceptance_from_feedback(self):
        nodes = FakeNodes()
        plugin = m20.M20MotionPlugin(nodes)
        plugin.dispatch("gait", {"transport": "native", "gait": "basic"})
        self.assertEqual(["motion_requested", "gait_requested"], [event["event"] for event in nodes.events])
        self.assertNotIn("gait_changed", [event["event"] for event in nodes.events])

    def test_motion_failure_is_published_and_reraised(self):
        nodes = FakeNodes()
        plugin = m20.M20MotionPlugin(nodes)
        with self.assertRaises(ValueError):
            plugin.dispatch("axis", {"x": 2, "y": 0, "yaw": 0})
        self.assertEqual("motion_command_failed", nodes.events[-1]["event"])

    def test_native_stream_uses_independent_thread_and_repeated_zero_stop(self):
        nodes = self.make_stream_nodes()
        try:
            result = nodes.set_native_velocity(21, {"X": 0.1, "Y": 0.0, "Yaw": 0.0}, duration=0.12)
            self.assertEqual((0.12, True), (result["duration"], result["auto_stop"]))
            deadline = time.monotonic() + 1
            while not any(event.get("reason") == "duration_expired" for event in nodes.events) and time.monotonic() < deadline:
                time.sleep(0.01)
            stopped = next(event for event in nodes.events if event.get("reason") == "duration_expired")
            self.assertLess(stopped["stop_delay"], 0.08)
            zero_calls = [items for _, items in nodes.native.velocity_calls if items and all(value == 0 for value in items.values())]
            self.assertEqual(3, len(zero_calls))
        finally:
            self.stop_stream_nodes(nodes)

    def test_switching_transport_stops_old_stream_before_new_command(self):
        nodes = self.make_stream_nodes()
        try:
            nodes.set_native_velocity(21, {"X": 0.2, "Y": 0.0, "Yaw": 0.0}, duration=0)
            nodes.set_ros_velocity(0.1, 0.0, 0.0, duration=0)
            native_zeros = [items for _, items in nodes.native.velocity_calls if items and all(value == 0 for value in items.values())]
            self.assertEqual(3, len(native_zeros))
            self.assertEqual((0.1, 0.0, 0.0), nodes.nav_frames[-1])
        finally:
            self.stop_stream_nodes(nodes)

    def test_missed_watchdog_stops_and_never_replays_stale_motion(self):
        nodes = self.make_stream_nodes()
        try:
            nodes.set_native_velocity(21, {"X": 0.2, "Y": 0.0, "Yaw": 0.0}, duration=0)
            with nodes._motion_condition:
                nodes._motion_command["last_send"] = time.monotonic() - 0.6
                nodes._motion_command["next_send"] = time.monotonic() - 0.5
                nodes._motion_condition.notify_all()
            deadline = time.monotonic() + 1
            while not any(event.get("reason") == "stream_watchdog_expired" for event in nodes.events) and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(any(event.get("reason") == "stream_watchdog_expired" for event in nodes.events))
            count_after_stop = len(nodes.native.velocity_calls)
            time.sleep(0.12)
            self.assertEqual(count_after_stop, len(nodes.native.velocity_calls))
            self.assertIsNone(nodes._motion_command)
        finally:
            self.stop_stream_nodes(nodes)

    def test_motion_stream_is_not_a_ros_timer(self):
        source = (DRIVER / "device.py").read_text()
        self.assertIn('name="m20-motion-stream"', source)
        self.assertNotIn("create_timer(0.05, self._send_active_native_velocity)", source)
        self.assertNotIn("create_timer(0.1, self._publish_active_velocity)", source)

    def test_tcp_wait_cannot_block_udp_velocity(self):
        class FakeUdp:
            def __init__(self): self.sent = []
            def sendto(self, payload, address): self.sent.append((payload, address))

        client = protocol.BasicServerClient({"basic_server": {"enabled": True}})
        client._udp = FakeUdp()
        completed = threading.Event()
        with client._tcp_lock:
            worker = threading.Thread(
                target=lambda: (client.send_velocity(21, {"X": 0.1}), completed.set()),
                daemon=True,
            )
            worker.start()
            self.assertTrue(completed.wait(0.2), "UDP velocity was blocked by the TCP request lock")
        worker.join(1)
        self.assertEqual(1, len(client._udp.sent))

    def test_device_commands_use_documented_native_protocol(self):
        plugin = m20.M20DevicePlugin(FakeNodes())
        lights = plugin.dispatch("lights", {"front": True, "rear": False})
        self.assertEqual((1101, 2, {"Front": 1, "Back": 0}), (lights["type"], lights["command"], lights["items"]))
        mode = plugin.dispatch("mode", {"mode": "assist"})
        self.assertEqual({"Mode": 2}, mode["items"])
        sleep = plugin.dispatch("auto_sleep", {"enabled": True, "minutes": 10})
        self.assertEqual({"Auto": True, "Sleep": False, "Time": 10}, sleep["items"])

    def test_standard_and_pro_boundaries_are_explicit(self):
        config = (DRIVER / "config.yaml").read_text()
        source = (DRIVER / "device.py").read_text()
        readme = (DRIVER / "README.md").read_text()
        self.assertIn('model_variant: "standard"', config)
        self.assertIn("if nodes.is_pro", source)
        self.assertIn("仅 M20 Pro", readme)
        self.assertIn("未提供舞蹈", readme)

    def test_ros2_uses_official_fastdds_topics_and_pinned_drdds(self):
        config = (DRIVER / "config.yaml").read_text()
        dockerfile = (DRIVER / "Dockerfile").read_text()
        for topic in ("/MOTION_STATE", "/GAIT", "/NAV_CMD", "/MOTION_INFO", "/IMU", "/LIDAR/POINTS", "/HES_STATUS", "/CHARGE", "/JOINTS_DATA"):
            self.assertIn(topic, config)
        self.assertIn("rmw_fastrtps_cpp", dockerfile)
        self.assertIn("ARG APT_MIRROR=", dockerfile)
        self.assertIn('--build-arg "APT_MIRROR=${APT_MIRROR}"', (ROOT / "build.sh").read_text())
        self.assertIn("a0d1a29eec5c4db5a9107595bb51e3be8122b86c", dockerfile)
        self.assertIn("deep-robotics-msg-${DEEP_ROBOTICS_MSG_REV}.zip", dockerfile)
        self.assertIn("1d268a76e80af8ea5aa3dc28de0c236de87bce55a5d397fdad1e23515f02a537", dockerfile)
        self.assertIn("sha256sum -c -", dockerfile)
        self.assertIn("python3 -m zipfile -e", dockerfile)
        self.assertNotIn("apt-get install -y --no-install-recommends unzip", dockerfile)
        self.assertNotIn("git clone", dockerfile)

        archive = DRIVER / "deep-robotics-msg-a0d1a29eec5c4db5a9107595bb51e3be8122b86c.zip"
        self.assertTrue(archive.is_file())
        import hashlib
        self.assertEqual(
            "1d268a76e80af8ea5aa3dc28de0c236de87bce55a5d397fdad1e23515f02a537",
            hashlib.sha256(archive.read_bytes()).hexdigest(),
        )

    def test_container_preserves_repository_depth_for_entrypoint(self):
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("/work/deep_robotics/lynx_m20/", dockerfile)
        self.assertIn("python3 /work/deep_robotics/lynx_m20/main.py", dockerfile)
        self.assertNotIn("python3 /work/main.py", dockerfile)

    def test_no_stale_s10_identity_remains(self):
        old_model = "s" + "10"
        text_suffixes = {".md", ".py", ".yaml", ".yml", ".txt", ""}
        files = [path for path in DRIVER.glob("*") if path.suffix in text_suffixes]
        files += [ROOT / "README.md", ROOT / "README_zh.md"]
        stale = [str(path) for path in files if path.is_file() and old_model in path.read_text().lower()]
        self.assertEqual([], stale)

    def test_build_selector_metadata_is_english(self):
        metadata = (DRIVER / "driver.yaml").read_text()
        self.assertIn("name: DEEP Robotics Lynx M20 Driver", metadata)
        self.assertNotRegex(metadata, r"[\u4e00-\u9fff]")


if __name__ == "__main__":
    unittest.main()
