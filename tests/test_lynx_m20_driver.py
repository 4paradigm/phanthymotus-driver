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

    def test_continuous_state_stream_info_returns_authoritative_topic_out(self):
        class FakeStateNodes:
            streams = {
                "motion_info": {"robot_topic": "/MOTION_INFO", "topic": "/host/lynx_m20/motion_info", "format": "data/json"},
                "imu": {"robot_topic": "/IMU", "topic": "/host/lynx_m20/imu", "format": "data/json"},
                "lidar": {"robot_topic": "/grid_map_3d", "topic": "/host/lynx_m20/lidar", "format": "sensor/pointcloud"},
            }
            rtsp_streams = {
                "camera_front": {"url": "rtsp://10.21.31.103:8554/video1", "format": "video/h265"},
            }
            reset_called = False

            def reset_lidar_accumulation(self):
                self.reset_called = True

        plugin = m20.M20StatePlugin(FakeStateNodes())
        static_topics = {
            definition["name"]: definition["topic_out"]
            for definition in plugin.get_tools()
            if "topic_out" in definition
        }
        canvas_formats = {"data/json", "data/odometry", "sensor/pointcloud", "video/h265"}

        for topic_out in static_topics.values():
            for output in topic_out:
                self.assertIn(output["format"], canvas_formats)

        for name in (*FakeStateNodes.streams, *FakeStateNodes.rtsp_streams):
            info = plugin.dispatch("info", {"_tool_name": name})
            self.assertEqual("ready", info["state"])
            self.assertEqual(static_topics[name], info["topic_out"])
        self.assertEqual("running", plugin.dispatch("start", {"_tool_name": "lidar"})["state"])
        self.assertTrue(plugin.nodes.reset_called)
        self.assertEqual("ready", plugin.dispatch("start", {"_tool_name": "camera_front"})["state"])

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
        self.assertIn("core_msg_type = String if as_json else UInt8MultiArray if as_pointcloud else msg_type", source)

    def test_fused_lidar_stream_uses_live_ros_compatible_topic(self):
        source = (DRIVER / "device.py").read_text()
        self.assertIn('("lidar", PointCloud2, "/grid_map_3d", "sensor/pointcloud"', source)
        self.assertNotIn('("lidar_rear", PointCloud2', source)
        self.assertNotIn('"pointcloud/ros2"', source)

    def test_lidar_pointcloud_is_encoded_for_canvas_renderer(self):
        class Field:
            def __init__(self, name, offset):
                self.name = name
                self.offset = offset
                self.datatype = 7

        first = struct.pack("<fffI", 1.0, 2.0, 3.0, 10)
        second = struct.pack("<fffI", 4.0, 5.0, 6.0, 20)
        msg = SimpleNamespace(
            point_step=16,
            width=1,
            height=2,
            row_step=20,
            fields=[Field("x", 0), Field("y", 4), Field("z", 8)],
            data=first + b"pad!" + second,
        )
        payload, point_count = m20.encode_pointcloud(msg)
        self.assertEqual((12, 2), struct.unpack_from("<II", payload))
        self.assertEqual(2, point_count)
        self.assertEqual(
            (-1.0, 2.0, -3.0, -4.0, 5.0, -6.0),
            struct.unpack_from("<ffffff", payload, 8),
        )

    def test_lidar_pointcloud_rejects_unsupported_xyz_layout(self):
        field = lambda name, offset: SimpleNamespace(name=name, offset=offset, datatype=7)
        msg = SimpleNamespace(
            point_step=16, width=1, height=1, row_step=16,
            fields=[field("x", 4), field("y", 8), field("z", 12)],
            data=bytes(16),
        )
        with self.assertRaisesRegex(ValueError, "offsets 0/4/8"):
            m20.encode_pointcloud(msg)

    def test_lidar_pointcloud_rejects_big_endian_data(self):
        field = lambda name, offset: SimpleNamespace(name=name, offset=offset, datatype=7)
        msg = SimpleNamespace(
            point_step=16, width=1, height=1, row_step=16, is_bigendian=True,
            fields=[field("x", 0), field("y", 4), field("z", 8)],
            data=struct.pack(">fffI", 1.0, 2.0, 3.0, 0),
        )
        with self.assertRaisesRegex(ValueError, "little-endian"):
            m20.encode_pointcloud(msg)

    def test_lidar_callback_publishes_encoded_canvas_payload(self):
        class FakePublisher:
            def __init__(self): self.messages = []
            def publish(self, message): self.messages.append(message)

        class FakeUInt8MultiArray:
            def __init__(self): self.data = []

        field = lambda name, offset: SimpleNamespace(name=name, offset=offset, datatype=7)
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link"),
            point_step=16, width=1, height=1, row_step=16, is_bigendian=False,
            fields=[field("x", 0), field("y", 4), field("z", 8)],
            data=struct.pack("<fffI", 1.0, 2.0, 3.0, 0),
        )
        nodes = object.__new__(m20.M20Nodes)
        nodes.config = {"lidar_visualization": {
            "accumulate_frames": 5, "min_points": 1,
            "max_points": 100, "publish_hz": 5.0,
        }}
        nodes.lock = threading.Lock()
        nodes.values = {}
        nodes._pointcloud_lock = threading.Lock()
        nodes._pointcloud_frames = {}
        nodes._pointcloud_last_publish = {}
        nodes._lidar_pose = None
        nodes._lidar_voxels = {}
        publisher = FakePublisher()
        callback = nodes._callback(
            "lidar", publisher, as_pointcloud=True,
            pointcloud_type=FakeUInt8MultiArray,
        )
        callback(msg)

        self.assertEqual(1, len(publisher.messages))
        payload = bytes(publisher.messages[0].data)
        self.assertEqual((12, 1), struct.unpack_from("<II", payload))
        self.assertEqual((-1.0, 2.0, -3.0), struct.unpack_from("<fff", payload, 8))
        self.assertEqual(1, nodes.values["lidar"]["point_count"])

        nodes._pointcloud_last_publish["lidar"] = float("-inf")
        msg.data = struct.pack("<fffI", 4.0, 5.0, 6.0, 0)
        callback(msg)
        payload = bytes(publisher.messages[-1].data)
        self.assertEqual((12, 2), struct.unpack_from("<II", payload))
        self.assertEqual(
            (-1.0, 2.0, -3.0, -4.0, 5.0, -6.0),
            struct.unpack_from("<ffffff", payload, 8),
        )

        nodes._lidar_pose = {
            "x": 10.0, "y": 20.0, "z": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        }
        nodes._pointcloud_last_publish["lidar"] = float("-inf")
        msg.data = struct.pack("<fffI", 1.0, 2.0, 3.0, 0)
        callback(msg)
        payload = bytes(publisher.messages[-1].data)
        self.assertEqual((12, 1), struct.unpack_from("<II", payload))
        self.assertEqual((-11.0, 22.0, -3.0), struct.unpack_from("<fff", payload, 8))
        self.assertEqual("map", nodes.values["lidar"]["frame_id"])

        nodes.config["lidar_visualization"]["max_points"] = 1
        nodes._pointcloud_last_publish["lidar"] = float("-inf")
        msg.data = struct.pack("<fffI", 4.0, 5.0, 6.0, 0)
        callback(msg)
        payload = bytes(publisher.messages[-1].data)
        self.assertEqual((12, 1), struct.unpack_from("<II", payload))
        self.assertEqual((-11.0, 22.0, -3.0), struct.unpack_from("<fff", payload, 8))

    def test_lidar_slam_transform_rotates_points_into_map(self):
        half = 2 ** -0.5
        pose = {
            "x": 10.0, "y": 20.0, "z": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": half, "qw": half,
        }
        transformed = m20.transform_point((1.0, 0.0, 0.0), pose)
        self.assertAlmostEqual(10.0, transformed[0], places=6)
        self.assertAlmostEqual(21.0, transformed[1], places=6)
        self.assertAlmostEqual(0.0, transformed[2], places=6)

    def test_lidar_odometry_accepts_fixed_odom_frame(self):
        nodes = object.__new__(m20.M20Nodes)
        nodes._pointcloud_lock = threading.Lock()
        nodes._lidar_pose = None
        pose = SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="odom"),
            pose=SimpleNamespace(pose=pose),
        )
        nodes._lidar_odometry_callback(msg)
        self.assertEqual("odom", nodes._lidar_pose["frame_id"])

    def test_default_lidar_odometry_source_is_continuous_odom(self):
        config_path = Path(m20.__file__).with_name("config.yaml")
        config_text = config_path.read_text(encoding="utf-8")
        self.assertIn(
            'slam_odometry: "/ODOM"',
            config_text,
        )
        self.assertIn("max_points: 500000", config_text)
        self.assertIn("publish_max_points: 80000", config_text)

    def test_lidar_canvas_sampling_covers_complete_accumulation(self):
        points = list(range(20))
        self.assertEqual(points, m20.evenly_sample_points(iter(points), 20, 20))
        sampled = m20.evenly_sample_points(iter(points), 20, 5)
        self.assertEqual([0, 4, 8, 12, 16], sampled)

    def test_lidar_legacy_external_limit_is_migrated(self):
        self.assertEqual(
            (500000, 80000, True),
            m20.lidar_point_limits({"max_points": 50000}, 32),
        )
        self.assertEqual(
            (50000, 40000, False),
            m20.lidar_point_limits(
                {"max_points": 50000, "publish_max_points": 40000}, 32,
            ),
        )

    def test_lidar_pointcloud_rejects_frames_without_valid_points(self):
        field = lambda name, offset: SimpleNamespace(name=name, offset=offset, datatype=7)
        msg = SimpleNamespace(
            point_step=16, width=2, height=1, row_step=32, is_bigendian=False,
            fields=[field("x", 0), field("y", 4), field("z", 8)],
            data=struct.pack("<fffIfffI", 0.0, 0.0, 0.0, 0, float("nan"), 1.0, 2.0, 0),
        )
        with self.assertRaisesRegex(ValueError, "no finite non-zero"):
            m20.encode_pointcloud(msg)

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
        for topic in ("/MOTION_STATE", "/GAIT", "/NAV_CMD", "/MOTION_INFO", "/IMU", "/grid_map_3d", "/HES_STATUS", "/CHARGE", "/JOINTS_DATA"):
            self.assertIn(topic, config)
        self.assertIn("rmw_fastrtps_cpp", dockerfile)
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
