from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import math
import os
import re
import struct
import sys
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch
from pathlib import Path
from types import ModuleType, SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "unixai" / "panther"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from common.vendor_runtime import DriverBundle


def _load(name):
    spec = importlib.util.spec_from_file_location(name, DRIVER / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


contracts = _load("contracts")
core_bridge = _load("core_bridge")
plugins = _load("plugins")
camera = _load("camera")
device_module = _load("device")
audio = _load("audio")
laser_cloud = _load("laser_cloud")


class PantherAudioContractTest(unittest.TestCase):
    def test_canvas_audio_ports_and_pulse_commands(self):
        config = {"server": "unix:/run/user/1001/pulse/native",
                  "source": "alsa_input.platform-sound.analog-stereo",
                  "sink": "alsa_output.platform-sound.analog-stereo"}
        mic = audio.AudioPlugin("mic", "unixai_panther", config).get_tool()
        speaker = audio.AudioPlugin("speaker", "unixai_panther", config).get_tool()
        self.assertEqual(mic["topic_out"], [{"topic": "/unixai_panther/panther/mic/audio",
                                             "format": "audio/pcm-16k"}])
        self.assertEqual(speaker["topic_in"], [{"format": "audio/pcm-16k"}])
        self.assertEqual(speaker["inputSchema"]["x-action-params"]["start"]["params"],
                         ["input_topic"])
        self.assertIn("--device=alsa_input.platform-sound.analog-stereo",
                      audio._pulse_command("mic", config["source"]))
        self.assertIn("--device=alsa_output.platform-sound.analog-stereo",
                      audio._pulse_command("speaker", config["sink"]))
        self.assertEqual(audio._pulse_identity({"uid": "1001", "gid": "1002"}),
                         {"user": 1001, "group": 1002})

    def test_partial_pulse_reads_are_reassembled_before_publish(self):
        pending = bytearray()
        self.assertEqual(list(audio._complete_chunks(pending, b"a" * 300)), [])
        self.assertEqual(list(audio._complete_chunks(pending, b"b" * 1748)),
                         [b"a" * 300 + b"b" * 1748])
        self.assertEqual(pending, b"")


class PantherDriverTest(unittest.TestCase):
    def test_agent_core_can_discover_all_cards_over_mcp(self):
        card_names = ("joints", "robot_state", "battery", "imu", "odom", "laser_scan",
                      "camera_color", "camera_depth")

        class FakeDevice:
            topics = {
                name: {"topic": f"/test/{name}",
                       "format": "image/jpeg" if name == "camera_color" else
                                 "image/depth-zlib" if name == "camera_depth" else
                                 "data/json"}
                for name in (*card_names, "laser_cloud")
            }

            def __getattr__(self, _name):
                return lambda *_args, **_kwargs: None

        config = {"plugins": {
            **{name: {"enabled": True} for name in card_names},
            "base_drive": {"enabled": True, "max_linear_mps": 0.5,
                           "max_angular_deg_s": 45.0, "max_duration_s": 10.0},
            "arm_control": {"enabled": True, "max_duration_s": 20.0},
            "gripper": {"enabled": True},
            "lifting": {"enabled": True, "max_position_m": 0.8, "home_position_m": 0.6},
            "head": {"enabled": True, "low_speed_deg_s": 10.0, "high_speed_deg_s": 20.0},
        }}
        bundle = DriverBundle(plugins.build_plugins(config, "test", FakeDevice()))
        tools = bundle.get_all_tools()
        json.dumps({"jsonrpc": "2.0", "id": 1, "result": {"tools": tools}},
                   allow_nan=False)
        discovered = {tool["name"] for tool in tools}
        expected = {*card_names, "model", "base_drive", "arm_control",
                    "gripper", "lifting", "head"}
        self.assertEqual(discovered, expected)

    def test_control_cards_use_runtime_safety_checks_and_lifecycle(self):
        class FakeDevice:
            topics = {
                name: {"topic": f"/test/{name}", "format": "data/json"}
                for name in ("joints", "robot_state", "battery", "imu", "odom", "laser_scan",
                             "camera_color", "camera_depth")
            }

            def __getattr__(self, _name):
                return lambda *_args, **_kwargs: None

            @staticmethod
            def schema_joint_limits_deg():
                limits = contracts.parse_joint_limits(
                    (DRIVER / "resource" / "panther.urdf").read_text()
                )
                return {
                    name: (math.degrees(lower), math.degrees(upper))
                    for name, (lower, upper) in limits.items()
                }

        config = {"plugins": {
            "base_drive": {"enabled": True, "max_linear_mps": 0.5,
                           "max_angular_deg_s": 45.0, "max_duration_s": 10.0},
            "arm_control": {"enabled": True, "max_duration_s": 20.0},
            "gripper": {"enabled": True},
            "lifting": {"enabled": True, "max_position_m": 0.8, "home_position_m": 0.6},
            "head": {"enabled": True, "low_speed_deg_s": 10.0,
                     "high_speed_deg_s": 20.0},
        }}
        tools = {plugin.name: plugin.get_tool()
                 for plugin in plugins.build_plugins(config, "test", FakeDevice())}
        base = tools["base_drive"]["inputSchema"]
        self.assertNotIn("confirm_motion", base["properties"])
        self.assertEqual(base["x-completion"]["actions"], ["move"])
        self.assertEqual(base["properties"]["angular_z_deg_s"]["maximum"], 45.0)
        self.assertEqual(base["properties"]["duration_s"]["maximum"], 10.0)
        self.assertIn("+forward", base["properties"]["linear_x"]["description"])
        self.assertIn("+left", base["properties"]["linear_y"]["description"])
        arm = tools["arm_control"]["inputSchema"]
        self.assertTrue({"move", "move_separate", "home", "stop"} <= set(arm["properties"]["action"]["enum"]))
        self.assertTrue({"shoulder_rot", "wrist_3", "l_shoulder_rot", "l_wrist_3",
                         "r_shoulder_rot", "r_wrist_3"} <= set(arm["properties"]))
        self.assertEqual(
            arm["x-action-params"]["move"]["params"],
            ["side", *contracts.ARM_JOINT_FIELDS, "duration_s"],
        )
        self.assertEqual(
            arm["x-action-params"]["move_separate"]["params"],
            [*[f"l_{field}" for field in contracts.ARM_JOINT_FIELDS],
             *[f"r_{field}" for field in contracts.ARM_JOINT_FIELDS],
             "duration_s"],
        )
        self.assertTrue(arm["properties"]["j0_deg"]["x-ui-hidden"])
        self.assertNotIn("confirm_motion", arm["properties"])
        self.assertEqual(arm["properties"]["shoulder_raise"]["description"],
                         "L/B [0.0, 90.0] deg; R [-90.0, 0.0] deg")
        self.assertEqual(arm["properties"]["l_elbow_flex"]["description"],
                         "Range [-127.8, 0.0] deg")
        self.assertEqual(arm["properties"]["elbow_flex"]["title"],
                         "Elbow Flexion/Extension (deg)")
        self.assertEqual(arm["properties"]["l_shoulder_rot"]["title"],
                         "Left Shoulder Rotation (deg)")
        self.assertEqual(arm["properties"]["r_wrist_3"]["title"],
                         "Right Wrist 3 (deg)")
        self.assertNotIn("oneOf", arm["properties"]["action"])
        gripper = tools["gripper"]["inputSchema"]
        self.assertEqual(gripper["properties"]["side"]["enum"], ["left", "right", "both"])
        self.assertTrue({"set", "open", "close", "stop"} <= set(gripper["properties"]["action"]["enum"]))
        self.assertEqual(gripper["x-action-params"]["stop"]["params"], [])
        self.assertNotIn("mode", gripper["properties"])
        lifting = tools["lifting"]["inputSchema"]
        self.assertTrue({"home", "stop"} <= set(lifting["properties"]["action"]["enum"]))
        self.assertIn("0.6 m", lifting["x-action-params"]["home"]["description"])
        self.assertEqual(lifting["x-action-params"]["stop"]["params"], [])
        head = tools["head"]["inputSchema"]
        self.assertTrue({"look", "home", "stop"} <= set(head["properties"]["action"]["enum"]))
        self.assertTrue({"yaw_deg", "pitch_deg"} <= set(head["properties"]))
        self.assertEqual(head["properties"]["speed"]["enum"], ["low", "high"])
        self.assertEqual(head["properties"]["speed"]["default"], "high")
        self.assertEqual(head["x-action-params"]["home"]["params"], [])
        self.assertEqual(set(head["x-completion"]["actions"]), {"look", "home"})
        expected_hooks = {
            "on_interrupt_motion": {"action": "stop"},
            "on_interrupt_all": {"action": "stop"},
        }
        for tool_name in ("base_drive", "arm_control", "head"):
            self.assertEqual(tools[tool_name]["inputSchema"]["x-hooks"], expected_hooks)
        self.assertNotIn("x-hooks", tools["gripper"]["inputSchema"])
        self.assertNotIn("x-hooks", tools["lifting"]["inputSchema"])
        for tool_name in ("base_drive", "arm_control", "gripper", "lifting", "head"):
            schema = tools[tool_name]["inputSchema"]
            self.assertNotIn("confirm_motion", schema["properties"])
            self.assertNotIn("confirm_motion", json.dumps(schema))
        exposed = json.dumps(tools, ensure_ascii=False)
        self.assertIsNone(re.search(r"[一-鿿]", exposed))
        self.assertNotRegex(exposed, r'"(?:head[01]|la[0-7]|ra[0-7])_rad"')

    def test_one_shot_actuators_accept_canvas_stop_without_motion_confirmation(self):
        device = (DRIVER / "device.py").read_text()
        self.assertEqual(
            device.count("single position command: nothing is in flight in this driver"),
            2,
        )

    def test_actuator_dispatch_does_not_inject_driver_confirmation(self):
        captured = []
        plugin = plugins.ActuatorPlugin(
            None, "arm_control", "test", {}, lambda payload: captured.append(payload) or payload
        )
        result = plugin.dispatch("move", {"side": "left", "j0_deg": 1.0})
        self.assertNotIn("confirm_motion", result)
        plugin.dispatch("info", {})
        self.assertNotIn("confirm_motion", captured[-1])

    def test_runtime_motion_enters_readiness_checks_without_driver_confirmation(self):
        device = object.__new__(device_module.PantherDevice)
        device.robot_state = lambda: {"ready_for_motion": False}
        calls = (
            (device.drive, {"action": "move"}),
            (device.arm, {"action": "home"}),
            (device.gripper, {"action": "open"}),
            (device.lifting, {"action": "home"}),
            (device.head, {"action": "home"}),
        )
        for method, payload in calls:
            with self.subTest(method=method.__name__), self.assertRaisesRegex(
                RuntimeError, "fresh joint and hardware state"
            ):
                method(payload)

    def test_state_and_battery_are_separate_streams(self):
        device = (DRIVER / "device.py").read_text()
        robot_state = device.split("    def robot_state(self)", 1)[1].split(
            "    def lifting_state(self)", 1
        )[0]
        self.assertNotIn('"battery"', robot_state)
        self.assertIn('self._publish_limited(\n            "battery"', device)

    def test_lifting_home_and_head_speed_profiles_are_configured(self):
        config = (DRIVER / "config.yaml").read_text()
        device = (DRIVER / "device.py").read_text()
        self.assertIn("home_position_m: 0.6", config)
        self.assertIn("high_speed_deg_s: 20.0", config)
        self.assertIn("low_speed_deg_s: 10.0", config)
        self.assertIn('if action == "home" else', device)
        self.assertIn('speed_name = "low" if action == "home"', device)
        self.assertIn("math.ceil(distance / speed * rate)", device)

    def test_head_channels_follow_physical_robot_mapping(self):
        current = {"joint_head0": 0.0, "joint_head1": 0.0}
        target = contracts.head_target_from_degrees(
            {"yaw_deg": 30.0, "pitch_deg": -10.0}, current
        )
        self.assertAlmostEqual(target[0], math.radians(30.0))
        self.assertAlmostEqual(target[1], math.radians(-10.0))
        self.assertEqual(contracts.HEAD_YAW_JOINT, "joint_head0")
        self.assertEqual(contracts.HEAD_PITCH_JOINT, "joint_head1")

    def test_arm_degree_inputs_convert_at_the_ros_boundary(self):
        converted = contracts.degrees_vector_to_radians(
            [0.0, 30.0, -45.0, 90.0, 10.0, -10.0, 15.0, -15.0],
            "positions_deg", 8,
        )
        self.assertAlmostEqual(converted[1], math.radians(30.0))
        self.assertAlmostEqual(converted[2], math.radians(-45.0))
        self.assertAlmostEqual(converted[3], math.radians(90.0))

    def test_vendor_urdf_arm_mirror_signs_are_applied(self):
        left = [10.0, 20.0, 30.0, 40.0, -50.0, 60.0, 15.0, -25.0]
        self.assertEqual(
            contracts.mirrored_arm_degrees(left),
            [-10.0, 20.0, -30.0, -40.0, -50.0, -60.0, -15.0, -25.0],
        )

    def test_arm_runtime_builds_mirrored_and_independent_targets(self):
        def run(arguments, positions=None, model_limits=None, expected_duration=1.0):
            device = object.__new__(device_module.PantherDevice)
            device._cfg = {"plugins": {"arm_control": {
                "max_duration_s": 20.0, "min_duration_s": 1.0,
                "speed_deg_s": 20.0, "home_pose_deg": {"joint_la1": 0.0},
            }}}
            device._lock = threading.RLock()
            device._arm_lock = threading.RLock()
            device._arm_active = None
            names = [*contracts.LEFT_ARM_JOINTS, *contracts.RIGHT_ARM_JOINTS]
            device._model_joint_limits = (model_limits or
                                          dict.fromkeys(names, (-math.pi, math.pi)))
            device._joint_limits = dict.fromkeys(names, (-math.pi, math.pi))
            initial = [0.0] * len(names)
            for name, value in (positions or {}).items():
                initial[names.index(name)] = value
            joint_msg = SimpleNamespace(name=names, position=initial)
            device._cache = {"joints": (joint_msg, time.monotonic())}
            device._ensure_motion_ready = lambda: None
            captured = {}
            completed = threading.Event()

            def capture(active, targets):
                captured.update(targets)
                with device._arm_lock:
                    device._arm_active = None
                completed.set()

            device._run_arm = capture
            result = device.arm(arguments)
            self.assertEqual(result["state"], "queued")
            self.assertAlmostEqual(result["duration_s"], expected_duration)
            self.assertTrue(completed.wait(1.0))
            return captured

        mirrored = run({"action": "move", "side": "both", "shoulder_rot": 10.0,
                        "shoulder_abd": 5.0, "elbow_flex": -10.0})
        self.assertAlmostEqual(math.degrees(mirrored["left"][0]), 10.0)
        self.assertAlmostEqual(math.degrees(mirrored["right"][0]), -10.0)
        self.assertAlmostEqual(math.degrees(mirrored["left"][1]), 5.0)
        self.assertAlmostEqual(math.degrees(mirrored["right"][1]), 5.0)
        self.assertAlmostEqual(math.degrees(mirrored["left"][4]), -10.0)
        self.assertAlmostEqual(math.degrees(mirrored["right"][4]), -10.0)

        separate = run({"action": "move_separate", "l_shoulder_rot": 6.0,
                        "r_shoulder_rot": 8.0})
        self.assertAlmostEqual(math.degrees(separate["left"][0]), 6.0)
        self.assertAlmostEqual(math.degrees(separate["right"][0]), 8.0)

        vendor_limits = contracts.parse_joint_limits(
            (DRIVER / "resource" / "panther.urdf").read_text()
        )
        partial = run(
            {"action": "move", "side": "left", "shoulder_rot": 3.0},
            positions={"joint_la2": math.radians(-0.08)},
            model_limits=vendor_limits,
        )
        self.assertAlmostEqual(math.degrees(partial["left"][0]), 3.0)
        self.assertEqual(partial["left"][2], 0.0)

        # The original rejection was a 40° displacement of joint_la1.
        large_move = run({"action": "move", "side": "left", "shoulder_abd": 40.0},
                         model_limits=vendor_limits, expected_duration=2.0)
        self.assertAlmostEqual(math.degrees(large_move["left"][1]), 40.0)
        independent = run({"action": "move_separate", "l_shoulder_abd": 40.0, "r_shoulder_abd": -30.0},
                          model_limits=vendor_limits, expected_duration=2.0)
        self.assertAlmostEqual(math.degrees(independent["left"][1]), 40.0)
        self.assertAlmostEqual(math.degrees(independent["right"][1]), -30.0)
        home = run({"action": "home", "side": "left"}, {"joint_la1": math.radians(40.0)},
                   model_limits=vendor_limits, expected_duration=2.0)
        self.assertEqual(home["left"][1], 0.0)

    def test_imu_keeps_finite_vectors_when_zero_covariance_has_no_truth_value(self):
        class Covariance:
            def __init__(self, values):
                self.values = values

            def __len__(self):
                return len(self.values)

            def __getitem__(self, index):
                return self.values[index]

            def __bool__(self):
                raise ValueError("ambiguous covariance truth value")

        vector = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="imu_link"),
            orientation=SimpleNamespace(x=float("nan"), y=float("nan"),
                                        z=float("nan"), w=float("nan")),
            orientation_covariance=Covariance([0.0] * 9),
            angular_velocity=vector(-0.004, 0.0, 0.001),
            angular_velocity_covariance=Covariance([0.0] * 9),
            linear_acceleration=vector(0.0, 0.04903325, 9.83606995),
            linear_acceleration_covariance=Covariance([0.0] * 9),
        )
        payload = device_module.PantherDevice._imu_payload(msg, time.monotonic())
        self.assertIsNone(payload["orientation_xyzw"])
        self.assertIsNone(payload["euler_rpy_rad"])
        self.assertEqual(payload["angular_velocity_rad_s"], [-0.004, 0.0, 0.001])
        self.assertEqual(payload["linear_acceleration_m_s2"], [0.0, 0.04903325, 9.83606995])

    def test_ros_byte_fields_decode_by_value(self):
        self.assertEqual(contracts.byte_value(b"\x02", "level"), 2)
        self.assertEqual(contracts.byte_value(bytearray([3]), "level"), 3)
        self.assertEqual(contracts.byte_value(1, "level"), 1)
        with self.assertRaisesRegex(ValueError, "exactly one byte"):
            contracts.byte_value(b"12", "level")

    def test_sensor_start_arms_while_waiting_for_first_sample(self):
        class FakeDevice:
            topics = {"imu": {"topic": "/test/imu", "format": "data/json"}}

            @staticmethod
            def sensor_info(_name):
                return {"available": False, "fresh": False, "source_topic": "/imu"}

        plugin = plugins.SensorPlugin(FakeDevice(), "imu", "test", ["imu"])
        started = plugin.dispatch("start", {})
        self.assertEqual(started["state"], "waiting_for_data")
        self.assertTrue(started["armed"])
        self.assertEqual(plugin.dispatch("info", {})["state"], "unavailable")

    def test_bundled_model_and_standardized_streams_are_packaged(self):
        model = DRIVER / "resource" / "panther.urdf"
        self.assertTrue(model.exists())
        model_text = model.read_text(encoding="utf-8")
        self.assertIn('<robot\n  name="pmm_urdf">', model_text)
        self.assertIn("joint_la0", model_text)
        self.assertIn("package://pmm_urdf/meshes/", model_text)
        self.assertEqual(
            hashlib.sha256(model_text.encode("utf-8")).hexdigest(),
            "abcd54d188f93c62247816b93c9998e112b1be36cd2bae582910d393c4f4902d",
        )
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("COPY resource/panther.urdf /work/resource/panther.urdf", dockerfile)
        device = (DRIVER / "device.py").read_text()
        self.assertIn("build_sample", device)
        self.assertIn("mean_twist", device)
        self.assertIn('"orientation_xyzw"', device)
        self.assertIn('"not_ready_reasons"', device)

    def test_bridge_uses_independent_latest_only_lanes(self):
        source = (DRIVER / "core_bridge.py").read_text()
        self.assertIn("maxsize=1", source)
        self.assertIn("target=_run_lane", source)
        self.assertIn('name=f"panther_core_{card}"', source)
        self.assertIn('"restarts"', source)

    def test_live_urdf_limits_are_parsed(self):
        urdf = """<robot name='p'><joint name='joint_la0' type='revolute'>
          <limit lower='-1.0' upper='2.0' effort='1' velocity='1'/>
        </joint></robot>"""
        self.assertEqual(contracts.parse_joint_limits(urdf), {"joint_la0": (-1.0, 2.0)})

    def test_arm_limits_are_vendor_live_intersection(self):
        names = ("joint_la0", "joint_la1")
        self.assertEqual(
            contracts.intersect_joint_limits(
                names,
                {"joint_la0": (-1.0, 1.0), "joint_la1": (-2.0, 2.0)},
                {"joint_la0": (-2.0, 2.0), "joint_la1": (-0.5, 0.75)},
            ),
            {"joint_la0": (-1.0, 1.0), "joint_la1": (-0.5, 0.75)},
        )
        with self.assertRaisesRegex(ValueError, "do not overlap"):
            contracts.intersect_joint_limits(
                ("joint_la0",), {"joint_la0": (-1.0, 0.0)},
                {"joint_la0": (0.5, 1.0)},
            )

    def test_arm_runtime_rejects_target_outside_vendor_limit_when_live_is_wider(self):
        device = object.__new__(device_module.PantherDevice)
        device._cfg = {"plugins": {"arm_control": {
            "max_duration_s": 20.0, "min_duration_s": 1.0,
            "speed_deg_s": 20.0,
        }}}
        device._lock = threading.RLock()
        device._arm_lock = threading.RLock()
        device._arm_active = None
        names = [*contracts.LEFT_ARM_JOINTS, *contracts.RIGHT_ARM_JOINTS]
        device._model_joint_limits = contracts.parse_joint_limits(
            (DRIVER / "resource" / "panther.urdf").read_text()
        )
        device._joint_limits = dict.fromkeys(names, (-math.pi, math.pi))
        joint_msg = SimpleNamespace(name=names, position=[0.0] * len(names))
        device._cache = {"joints": (joint_msg, time.monotonic())}
        device._ensure_motion_ready = lambda: None
        for target in (
            {"j0_deg": 70.0},
            {"positions_deg": [70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        ):
            with self.subTest(target=target), self.assertRaisesRegex(
                ValueError, "outside URDF limit"
            ):
                device.arm({
                    "action": "move", "side": "left", **target,
                    "duration_s": 1.0,
                })

    def test_render_model_and_joint_indices_share_one_snapshot(self):
        device = object.__new__(device_module.PantherDevice)
        device._lock = threading.RLock()
        device._cache = {}
        device._callback_errors = {}
        device._model_urdf = (DRIVER / "resource" / "panther.urdf").read_text(encoding="utf-8")
        device._model_joint_limits = contracts.parse_joint_limits(device._model_urdf)
        device._render_urdf, device._render_mimics = device_module.render_model(device._model_urdf)
        device._render_joint_indices = contracts.parse_joint_indices(device._render_urdf)
        device._render_model_sha256 = hashlib.sha256(device._render_urdf.encode()).hexdigest()
        device._render_source = "bundled articulated display model"
        live = '<robot><joint name="body_joint" type="fixed"><parent link="a"/><child link="b"/></joint></robot>'
        device._on_robot_description(SimpleNamespace(data=live))
        model = device.sensor_info("model")
        self.assertNotEqual(model["urdf"], live)
        self.assertEqual(device._urdf, live)

        def feedback(names, positions):
            return SimpleNamespace(name=names, position=positions, velocity=[], effort=[])

        device._put("lifting", feedback(["height"], [0.6]))
        device._put("gripper_left", feedback(["joint_lf"], [0.0]))
        device._cache["gripper_right"] = (feedback(["joint_rf"], [0.8]), time.monotonic() - 10)
        payload = device._joints_payload(feedback(["joint_head0", "joint_la0"], [0.2, 0.1]), time.monotonic())
        self.assertEqual(model["model_sha256"], payload["model_sha256"])
        rows = {joint["name"]: joint for joint in payload["joints"]}
        self.assertEqual(rows["body_joint"]["q"], 0.6)
        self.assertEqual(rows["joint_head0"]["q"], 0.2)
        self.assertEqual(rows["joint_la0"]["q"], 0.1)
        self.assertEqual(rows["joint_lf"]["q"], 1.0)
        self.assertEqual(rows["joint_lf02"]["q"], 1.0)
        self.assertNotIn("joint_rf", rows)
        self.assertEqual(rows["joint_lf"]["idx"], contracts.parse_joint_indices(model["urdf"])["joint_lf"])

        device._put("gripper_left", feedback(["joint_lf"], [1.0]))
        device._put("gripper_right", feedback(["joint_rf"], [0.2]))
        payload = device._joints_payload(feedback([], []), time.monotonic())
        rows = {joint["name"]: joint["q"] for joint in payload["joints"]}
        self.assertEqual(rows["joint_lf02"], 0.0)
        self.assertAlmostEqual(rows["joint_rf02"], -0.8)

        joints = {j.get("name"): j for j in ET.fromstring(model["urdf"]).findall("joint")}
        for arm in ("la", "ra"):
            self.assertEqual(joints[f"joint_{arm}3"].find("axis").get("xyz"), "0 0 1")
            self.assertEqual(joints[f"joint_{arm}4"].find("axis").get("xyz"), "0 1 0")
        self.assertEqual(joints["body_joint"].get("type"), "prismatic")
        self.assertEqual(joints["body_joint"].find("parent").get("link"), "base_link")
        self.assertEqual(joints["body_joint"].find("child").get("link"), "body_link")
        self.assertEqual([name for name, j in joints.items() if j.find("parent").get("link") == "base_link"],
                         ["body_joint"])
        self.assertFalse({"joint_fl_rev", "joint_fl_con", "joint_front_lidar", "joint_behind_lidar"} & joints.keys())
        self.assertEqual(model["skeleton_visual_links"], ["base_link"])
        base = ET.fromstring(model["urdf"]).find("link[@name='base_link']")
        self.assertEqual(len(base.findall("visual")), 2)
        box = base.find("visual[@name='display_base']")
        center_x, center_y, center_z = map(float, box.find("origin").get("xyz").split())
        length, width, height = map(float, box.find("geometry/box").get("size").split())
        self.assertEqual((center_y, center_z, width, height), (0.0, 0.1, 0.51, 0.2))
        self.assertAlmostEqual(center_x + length / 2, 0.325)
        rear_x = center_x - length / 2
        rail = base.find("visual[@name='display_column']")
        rail_x, _, center = map(float, rail.find("origin").get("xyz").split())
        self.assertAlmostEqual(rail_x - rear_x, (rail_x + 0.325) / 2)
        half_length = float(rail.find("geometry/cylinder").get("length")) / 2
        self.assertAlmostEqual(center - half_length, 0.2)
        self.assertAlmostEqual(center + half_length, 0.62 + 0.785998)
        for name in ("joint_head0", "joint_head1", "joint_rgbd"):
            self.assertEqual(float(joints[name].find("origin").get("xyz").split()[1]), 0.0)
        for name, axis in (("joint_head0", "0 0 1"), ("joint_head1", "0 1 0")):
            self.assertEqual(joints[name].find("axis").get("xyz"), axis)
        left_eye, right_eye = (tuple(map(float, joints[f"joint_{side}_eye"].find("origin").get("xyz").split()))
                               for side in ("left", "right"))
        self.assertEqual(left_eye, (right_eye[0], -right_eye[1], right_eye[2]))

    def test_skeleton_indices_follow_kinematic_urdf_order(self):
        urdf = """<robot name='p'>
          <joint name='fixed_base' type='fixed'><parent link='a'/><child link='b'/></joint>
          <joint name='joint_la0' type='revolute'><parent link='b'/><child link='c'/></joint>
          <ros2_control name='control' type='system'>
            <joint name='joint_la0'><command_interface name='position'/></joint>
          </ros2_control>
        </robot>"""
        self.assertEqual(
            contracts.parse_joint_indices(urdf),
            {"fixed_base": 0, "joint_la0": 1},
        )

    def test_joint_target_rejects_missing_limits_instead_of_guessing(self):
        with self.assertRaisesRegex(ValueError, "no limits"):
            contracts.validate_joint_target(("joint_la0",), [0.0], {}, {"joint_la0": 0.0})

    def test_joint_target_rejects_nonfinite_feedback(self):
        with self.assertRaisesRegex(ValueError, r"current\.joint_la0 must be finite"):
            contracts.validate_joint_target(
                ("joint_la0",), [0.8], {"joint_la0": (-1.0, 1.0)}, {"joint_la0": float("nan")}
            )

    def test_partial_arm_target_clamps_small_implicit_hold_limit_error(self):
        names = ("joint_la0", "joint_la2")
        limits = {"joint_la0": (-1.0, 1.0), "joint_la2": (0.0, 1.0)}
        current = {"joint_la0": 0.0, "joint_la2": math.radians(-0.08)}
        target = contracts.validate_joint_target(
            names,
            [math.radians(3.0), current["joint_la2"]],
            limits,
            current,
            commanded_joint_names={"joint_la0"},
            hold_limit_tolerance_rad=math.radians(0.5),
        )
        self.assertAlmostEqual(math.degrees(target[0]), 3.0)
        self.assertEqual(target[1], 0.0)

    def test_partial_arm_target_keeps_explicit_and_large_limit_errors_strict(self):
        limits = {"joint_la2": (0.0, 1.0)}
        current = {"joint_la2": math.radians(-0.08)}
        with self.assertRaisesRegex(ValueError, "target -0.08 deg"):
            contracts.validate_joint_target(
                ("joint_la2",), [current["joint_la2"]], limits, current,
                commanded_joint_names={"joint_la2"},
                hold_limit_tolerance_rad=math.radians(0.5),
            )
        far_current = {"joint_la2": math.radians(-2.0)}
        with self.assertRaisesRegex(ValueError, "calibration or recovery"):
            contracts.validate_joint_target(
                ("joint_la2",), [far_current["joint_la2"]], limits, far_current,
                commanded_joint_names=set(),
                hold_limit_tolerance_rad=math.radians(0.5),
            )

    def test_drive_caps_and_finite_duration(self):
        config = {"max_linear_mps": 0.5, "max_angular_deg_s": 45.0, "max_duration_s": 10.0}
        command = contracts.validate_drive(
            {"linear_x": 0.5, "linear_y": -0.5, "angular_z_deg_s": 45.0,
             "duration_s": 10.0}, config
        )
        self.assertEqual(command[:2], (0.5, -0.5))
        self.assertAlmostEqual(command[2], math.radians(45.0))
        self.assertEqual(command[3], 10.0)
        with self.assertRaisesRegex(ValueError, "duration_s"):
            contracts.validate_drive(
                {"linear_x": 0.0, "duration_s": -1}, config
            )

    def test_drive_stop_waits_for_match_then_republishes_zero(self):
        class FakePublisher:
            def __init__(self):
                self.matched = False
                self.messages = []

            def get_subscription_count(self):
                return int(self.matched)

            def publish(self, message):
                self.messages.append(message)

        geometry = ModuleType("geometry_msgs")
        geometry_msg = ModuleType("geometry_msgs.msg")
        geometry_msg.Twist = type("Twist", (), {})
        geometry.msg = geometry_msg
        publisher = FakePublisher()
        device = device_module.PantherDevice.__new__(device_module.PantherDevice)
        device._cfg = {"plugins": {"base_drive": {
            "publish_rate_hz": 100.0,
            "stop_repetitions": 2,
            "stop_publish_duration_s": 0.05,
            "stop_match_timeout_s": 0.05,
        }}}
        device._drive_lock = threading.Lock()
        device._drive_action_id = "move-1"
        device._drive_generation = 1
        device._drive_pub = publisher
        device._drive_topic = "/cmd_vel"
        device._drive_stop_request = None
        device._drive_stop_thread = None
        device._closed = False
        with patch.dict(sys.modules, {
            "geometry_msgs": geometry,
            "geometry_msgs.msg": geometry_msg,
        }), patch.object(device_module, "_notify_completion"):
            result = device.cancel_drive("driver_shutdown", wait_s=0.08)
            self.assertEqual(result["state"], "stop_unconfirmed")
            self.assertFalse(result["stop_confirmed"])
            self.assertGreater(len(publisher.messages), 0)
            # close() marks the device closed before other actuator workers
            # finish. The pending zero command must survive that transition.
            device._closed = True
            publisher.matched = True
            confirmed = device._wait_for_pending_drive_stop(1.0)
        self.assertTrue(confirmed["stop_confirmed"])
        self.assertGreaterEqual(len(publisher.messages), 2)

    def test_manifest_lists_expected_cards(self):
        driver_text = (DRIVER / "driver.yaml").read_text()
        cards = set(re.findall(r"name:\s*([a-z_]+)", driver_text))
        expected = {"joints", "model", "robot_state", "battery", "imu", "odom", "laser_scan",
                    "camera_color", "camera_depth", "base_drive", "arm_control",
                    "gripper", "lifting", "head"}
        self.assertTrue(expected <= cards)
        self.assertFalse({"robot_status", "gripper_state", "lifting_state",
                          "ultrasonic", "nav", "servo"} & cards)

    def test_bridge_exposes_consolidated_state_merged_lidar_and_typed_media(self):
        topics = core_bridge.output_topics("test")
        self.assertEqual(topics["robot_state"]["format"], "data/json")
        self.assertEqual(topics["battery"]["topic"], "/test/panther/battery")
        self.assertEqual(topics["laser_scan"]["topic"], "/test/panther/laser_scan")
        self.assertEqual(topics["laser_cloud"]["format"], "sensor/pointcloud")
        self.assertEqual(topics["camera_color"]["format"], "image/jpeg")
        self.assertEqual(topics["camera_depth"]["format"], "image/depth-zlib")
        self.assertFalse({"laser_front", "laser_rear", "laser_fused"} & set(topics))

    def test_agent_core_lane_creates_only_an_output_publisher(self):
        created = {"publishers": [], "subscriptions": [], "messages": []}

        class Messages:
            def __init__(self, payload):
                self.items = [payload, None]

            def get(self):
                return self.items.pop(0)

        class Publisher:
            def publish(self, message):
                created["messages"].append(message)

        class Node:
            def __init__(self, _name, **kwargs):
                self.kwargs = kwargs
                self.assert_output_only_options()

            def assert_output_only_options(self):
                self_test.assertFalse(self.kwargs["enable_rosout"])
                self_test.assertFalse(self.kwargs["start_parameter_services"])

            def create_publisher(self, message_type, topic, _qos):
                created["publishers"].append((message_type, topic))
                return Publisher()

            def create_subscription(self, *_args, **_kwargs):
                created["subscriptions"].append(True)
                raise AssertionError("Agent Core bridge must not create a DataReader")

            def destroy_node(self):
                pass

        class String:
            def __init__(self):
                self.data = ""

        class CompressedImage:
            pass

        self_test = self
        rclpy = ModuleType("rclpy")
        rclpy.init = lambda **_kwargs: None
        rclpy.ok = lambda: True
        rclpy.shutdown = lambda: None
        rclpy_node = ModuleType("rclpy.node"); rclpy_node.Node = Node
        rclpy_qos = ModuleType("rclpy.qos")
        rclpy_qos.HistoryPolicy = SimpleNamespace(KEEP_LAST=1)
        rclpy_qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
        rclpy_qos.QoSProfile = lambda **kwargs: kwargs
        sensor_msgs = ModuleType("sensor_msgs")
        sensor_msgs_msg = ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.CompressedImage = CompressedImage
        sensor_msgs.msg = sensor_msgs_msg
        std_msgs = ModuleType("std_msgs")
        std_msgs_msg = ModuleType("std_msgs.msg")
        std_msgs_msg.String = String
        std_msgs_msg.UInt8MultiArray = type("UInt8MultiArray", (), {})
        std_msgs.msg = std_msgs_msg
        modules = {
            "rclpy": rclpy, "rclpy.node": rclpy_node, "rclpy.qos": rclpy_qos,
            "sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs_msg,
            "std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg,
        }
        with patch.dict(sys.modules, modules), patch.dict(os.environ, {}, clear=False):
            core_bridge._run_lane(Messages({"source_topic": "/scan", "ranges": [1.0]}), "/profile.xml", 42, "test", "laser_scan")
            packet = laser_cloud.point_packet([(1, 2, 0)])
            core_bridge._run_lane(Messages(packet), "/profile.xml", 42, "test", "laser_cloud")
        self.assertEqual(len(created["publishers"]), 2)
        self.assertEqual(created["publishers"][0][1], "/test/panther/laser_scan")
        self.assertEqual(created["subscriptions"], [])
        self.assertEqual(json.loads(created["messages"][0].data)["source_topic"], "/scan")
        self.assertEqual(created["publishers"][1], (std_msgs_msg.UInt8MultiArray, "/test/panther/laser_scan/points"))
        self.assertEqual(bytes(created["messages"][1].data), packet)

    def test_lidar_publishes_only_current_scan_and_falls_back_to_front(self):
        device = object.__new__(device_module.PantherDevice)
        device._lock = threading.RLock()
        device._laser_lock = threading.Lock()
        device._cache = {}
        device._callback_errors = {}
        device._last_card_publish = {}
        device._laser_topics = ("/scan", "/lidar_front/scan")
        device._laser_source = None
        device._laser_seen = {}
        device._laser_max_points = 20_000
        device._laser_max_range_m = 12.0
        captured = {}
        device._bridge = SimpleNamespace(publish=lambda card, payload: captured.update({card: payload}))
        scan = SimpleNamespace(
            header=SimpleNamespace(frame_id="laser", stamp=SimpleNamespace(sec=10, nanosec=0)),
            ranges=[1.0, float("inf"), 2.0],
            angle_min=-1.0, angle_max=1.0, angle_increment=1.0,
            range_min=0.1, range_max=10.0,
        )
        front = "/lidar_front/scan"
        only_center = SimpleNamespace(**{**vars(scan), "ranges": [float("inf"), 3.0, float("nan")]})
        with patch.object(device_module.time, "monotonic", return_value=10.0) as clock:
            device._on_laser(scan, front)
            cloud_wire = core_bridge.encode_bridge_payload("laser_cloud", captured["laser_cloud"])
            self.assertEqual(struct.unpack_from("<II", cloud_wire["data"]), (16, 2))
            self.assertEqual(device.sensor_info("laser_scan")["cloud_frame"], "laser")
            scan_wire = core_bridge.encode_bridge_payload("laser_scan", captured["laser_scan"])
            clock.return_value = 10.1
            device._on_laser(only_center, front)
            self.assertEqual(struct.unpack("<IIffff", captured["laser_cloud"]), (16, 1, 3.0, 0.0, 0.0, 0.0))
            clock.return_value = 10.2
            device._on_laser(scan, "/scan")
            device._on_laser(only_center, front)
            self.assertEqual(device.sensor_info("laser_scan")["source_topic"], "/scan")
            self.assertEqual(captured["laser_cloud"], cloud_wire["data"])
            clock.return_value = 11.3
            device._on_laser(only_center, front)
            self.assertEqual(device.sensor_info("laser_scan")["source_topic"], front)
            self.assertEqual(device.sensor_info("laser_scan")["cloud_points"], 1)

        scan_json = json.loads(scan_wire["data"])
        self.assertEqual(scan_wire["kind"], "string")
        self.assertEqual(scan_json["source_topic"], "/lidar_front/scan")
        self.assertEqual(scan_json["ranges"], [1.0, None, 2.0])
        with self.assertRaisesRegex(ValueError, "not JPEG"):
            core_bridge.encode_bridge_payload(
                "camera_color", {"message_format": "jpeg", "data": b"not-jpeg"}
            )

    def test_removed_nav_and_ultrasonic_have_no_runtime_code(self):
        plugins = (DRIVER / "plugins.py").read_text()
        device = (DRIVER / "device.py").read_text()
        self.assertNotIn('"ultrasonic"', plugins)
        self.assertNotIn('"nav"', plugins)
        self.assertNotIn("NavigateToPose", device)
        self.assertNotIn("SetBool", device)

    def test_camera_is_canvas_lazy_and_uses_host_transient_service(self):
        camera = (DRIVER / "camera.py").read_text()
        plugins_text = (DRIVER / "plugins.py").read_text()
        service = (DRIVER / "deploy" / "service.yml").read_text()
        self.assertIn("systemd-run", camera)
        self.assertIn("panther_orbbec.service", camera)
        self.assertIn("ros2 launch orbbec_camera gemini2.launch.py", camera)
        self.assertIn("DriverBundle.start_all() runs at container startup", plugins_text)
        self.assertIn("pid: host", service)
        self.assertIn("privileged: true", service)

        class FakeDevice:
            topics = {"camera_color": {"topic": "/test/rgb", "format": "image/jpeg"}}

            def __init__(self):
                self.calls = []

            def camera(self, kind, args):
                self.calls.append((kind, args["action"]))
                return {"state": "running"}

        device = FakeDevice()
        plugin = plugins.CameraPlugin(device, "camera_color", "rgb", "test")
        self.assertEqual(plugin.get_tool()["camera_info"][0]["schema"], "motus.camera/1")
        plugin.start()
        self.assertEqual(device.calls, [])
        plugin.dispatch("start", {})
        self.assertEqual(device.calls, [("rgb", "start")])

    def test_camera_uses_one_fixed_transient_unit_and_dds_profile(self):
        runtime = object.__new__(camera.PantherCameraRuntime)
        runtime._node = SimpleNamespace(count_publishers=lambda _topic: 0)
        runtime._lock = threading.RLock()
        runtime._config = {
            "workspace": "/home/unix_ai/orbbec_ws",
            "cyclonedds_profile": "/home/unix_ai/config/cyclonedds.xml",
        }
        runtime._source_unit = None
        runtime._managed_unit = None
        calls = []

        def run_host(command, timeout=15.0, check=True):
            calls.append((command, timeout, check))
            if command[:2] == ["systemctl", "is-active"]:
                return SimpleNamespace(stdout="inactive\n", returncode=3)
            if command[:2] == ["systemctl", "cat"]:
                return SimpleNamespace(stdout="", stderr="", returncode=1)
            return SimpleNamespace(stdout="", stderr="", returncode=0)

        runtime._run_host = run_host
        runtime._ensure_source()
        commands = [call[0] for call in calls]
        launch = next(command for command in commands if command[0] == "systemd-run")
        unit = next(value.split("=", 1)[1] for value in launch if value.startswith("--unit="))
        self.assertEqual(unit, camera.PantherCameraRuntime.HOST_UNIT)
        launch_script = launch[-1]
        self.assertIn("CYCLONEDDS_URI=file:///home/unix_ai/config/cyclonedds.xml", launch_script)
        self.assertEqual(runtime._managed_unit, unit)
        self.assertEqual(runtime._source_unit, unit)

        external = object.__new__(camera.PantherCameraRuntime)
        external._node = SimpleNamespace(count_publishers=lambda _topic: 0)
        external._lock = threading.RLock()
        external._config = {}
        external._source_unit = None
        external._managed_unit = None
        external_calls = []

        def active_host(command, timeout=15.0, check=True):
            external_calls.append((command, timeout, check))
            return SimpleNamespace(stdout="active\n", returncode=0)

        external._run_host = active_host
        external._ensure_source()
        self.assertEqual(len(external_calls), 1)
        self.assertEqual(external._source_unit, camera.PantherCameraRuntime.HOST_UNIT)
        self.assertIsNone(external._managed_unit)

    def test_camera_start_requires_a_real_first_frame(self):
        runtime = object.__new__(camera.PantherCameraRuntime)
        runtime._node = SimpleNamespace(count_publishers=lambda _topic: 1)
        runtime._config = {"startup_timeout_s": 0.01, "first_frame_grace_s": 0.01}
        runtime._active = set()
        runtime._lock = threading.RLock()
        runtime._source_lock = threading.Lock()
        runtime._frames = {"rgb": None, "depth": None}
        runtime._last_frame_at = {"rgb": 0.0, "depth": 0.0}
        runtime._armed_at = 0.0
        runtime._watchdog_restarts = 0
        runtime._ensure_source = lambda: None
        with self.assertRaisesRegex(RuntimeError, "no frame was received"):
            runtime.start("rgb")
        self.assertNotIn("rgb", runtime._active)

    def test_camera_frame_failures_are_bounded_and_sampled(self):
        runtime = object.__new__(camera.PantherCameraRuntime)
        runtime._lock = threading.RLock()
        runtime._frame_failures = {"rgb": 0, "depth": 0}
        output = io.StringIO()
        noisy = ValueError("bad\nlayout " + "x" * 400)
        with contextlib.redirect_stdout(output):
            for _ in range(200):
                runtime._log_frame_failure("rgb", "camera_color", noisy)
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 3)
        self.assertIn("#1:", lines[0])
        self.assertIn("#100:", lines[1])
        self.assertIn("#200:", lines[2])
        self.assertTrue(all(len(line) < 300 for line in lines))

    def test_laser_range_and_points_are_bounded_and_keep_original_beam_angles(self):
        scan = SimpleNamespace(angle_min=0.0, angle_increment=math.pi / 2,
                               range_min=0.1, range_max=20.0,
                               ranges=[0.05, 2.0, 13.0, 3.0])
        points = laser_cloud.current_points(scan, max_range_m=12.0, max_points=1)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.0)
        self.assertEqual(points[0][1:], (2.0, 0.0))

    def test_container_dependency_contract_is_apt_only_and_explained(self):
        dockerfile = (DRIVER / "Dockerfile").read_text()
        readme = (DRIVER / "README.md").read_text()
        self.assertFalse((DRIVER / "requirements.txt").exists())
        self.assertIn("ros-humble-nav-msgs", dockerfile)
        self.assertRegex(dockerfile, r"(?m)^\s+systemd\s*\\$")
        self.assertIn("nav_msgs/msg/Odometry", readme)
        self.assertIn("systemctl`/`systemd-run` clients", readme)
        self.assertIn("dependencies are installed only through apt", readme)
        self.assertIn("second pip requirements layer", readme)

    def test_canvas_registration_identity_matches_known_good_release(self):
        config = (DRIVER / "config.yaml").read_text()
        manifest = (DRIVER / "driver.yaml").read_text()
        main = (DRIVER / "main.py").read_text()
        self.assertIn('name: "UniX AI Panther Bundle"', config)
        self.assertIn("name: UniX AI Panther Bundle", manifest)
        self.assertIn("camera_color", config)
        self.assertIn("camera_color", manifest)
        self.assertNotIn("camera_rgb", manifest)
        self.assertIn('"unixai-panther"', main)
        self.assertIn('"PANTHER_MCP_ADVERTISE_HOST"', main)
        self.assertLess(main.index("ThreadingHTTPServer("), main.index("start_registration("))

    def test_panther_advertise_host_does_not_change_other_drivers(self):
        runtime = (ROOT / "common" / "vendor_runtime.py").read_text()
        self.assertIn('advertise_host: str = "localhost"', runtime)
        self.assertNotIn('os.environ.get("MCP_ADVERTISE_HOST"', runtime)
        self.assertIn("failures % 12", runtime)
        self.assertIn("type(exc).__name__[:80]", runtime)
        self.assertNotIn('f"[register] failed: {exc}', runtime)

    def test_action_resources_follow_agent_core_conventions(self):
        plugins = (DRIVER / "plugins.py").read_text()
        self.assertIn('["arm_l", "arm_r"]', plugins)
        self.assertIn('["hand_l", "hand_r"]', plugins)
        self.assertIn('"waist"', plugins)
        self.assertNotIn('"dual_arms"', plugins)
        self.assertNotIn('"grippers"', plugins)

    def test_deployment_contract_is_explicit(self):
        config = (DRIVER / "config.yaml").read_text()
        manifest = (DRIVER / "driver.yaml").read_text()
        service = (DRIVER / "deploy" / "service.yml").read_text()
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertNotIn("commissioning_motion_enabled", config)
        for text in (config, manifest, dockerfile):
            self.assertIn("15792", text)
        self.assertIn("AGENT_CORE_ROS_DOMAIN_ID=42", service)
        self.assertIn(
            "PANTHER_MCP_ADVERTISE_HOST=${PANTHER_MCP_ADVERTISE_HOST:-localhost}",
            service,
        )
        self.assertNotIn("PANTHER_COMMISSIONING_MOTION_ENABLED", service)
        self.assertNotIn("PANTHER_COMMISSIONING_MOTION_AUTH_FILE", service)
        self.assertIn("PANTHER_ROS_DOMAIN_ID=0", service)
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml", service)
        self.assertIn(
            "/home/unix_ai/config/cyclonedds.xml:/etc/cyclonedds/panther.xml:ro",
            service,
        )
        self.assertIn(
            "CYCLONEDDS_URI=file:///etc/cyclonedds/panther.xml,file:///work/resource/cyclonedds-muzzle.xml",
            service,
        )
        muzzle = (DRIVER / "resource" / "cyclonedds-muzzle.xml").read_text()
        self.assertIn("<Verbosity>none</Verbosity>", muzzle)
        self.assertIn("<OutputFile>/dev/null</OutputFile>", muzzle)
        self.assertIn("COPY resource/cyclonedds-muzzle.xml", dockerfile)
        self.assertIn("network_mode: host", service)
        self.assertIn("stop_grace_period: 30s", service)

    def test_verified_sdk_limits_and_topics_are_configured(self):
        config = (DRIVER / "config.yaml").read_text()
        main = (DRIVER / "main.py").read_text()
        self.assertIn('cmd_topics: ["/cmd_vel", "/cmd_vel_teleop"]', config)
        self.assertIn("min_position_m: 0.4", config)
        self.assertIn("max_position_m: 0.65", config)
        self.assertIn("yaw_min_deg: -90.0", config)
        self.assertIn("pitch_min_deg: -86.0", config)
        self.assertIn("home_pose_deg: {yaw_deg: 0.0, pitch_deg: 0.0}", config)
        self.assertIn("executor.spin_once", main)

    def test_dockerfile_builds_observed_gripper_interface(self):
        dockerfile = (DRIVER / "Dockerfile").read_text()
        message = (DRIVER / "vendor" / "unix_msgs" / "msg" / "GripperCommand.msg").read_text()
        self.assertIn("colcon build --packages-select unix_msgs", dockerfile)
        self.assertIn("string mode", message)
        self.assertIn("sensor_msgs/JointState command", message)

    def test_switch_controller_supports_both_humble_field_generations(self):
        class Duration:
            sec = 0

        class NewRequest:
            activate_controllers = []
            deactivate_controllers = []
            strictness = 0
            activate_asap = False
            timeout = Duration()

        class OldRequest:
            start_controllers = []
            stop_controllers = []
            strictness = 0
            start_asap = False
            timeout = Duration()

        new, old = NewRequest(), OldRequest()
        contracts.populate_switch_request(new, ["a"], ["b"])
        contracts.populate_switch_request(old, ["a"], ["b"])
        self.assertEqual(new.activate_controllers, ["a"])
        self.assertEqual(new.deactivate_controllers, ["b"])
        self.assertTrue(new.activate_asap)
        self.assertEqual(old.start_controllers, ["a"])
        self.assertEqual(old.stop_controllers, ["b"])
        self.assertTrue(old.start_asap)


if __name__ == "__main__":
    unittest.main()

