from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import math
import re
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "unixai" / "panther"
sys.path.insert(0, str(ROOT))

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


class PantherDriverTest(unittest.TestCase):
    def test_agent_core_can_discover_all_cards_over_mcp(self):
        card_names = ("joints", "robot_state", "imu", "odom", "laser_scan",
                      "camera_color", "camera_depth")

        class FakeDevice:
            topics = {
                name: {"topic": f"/test/{name}",
                       "format": "image/jpeg" if name == "camera_color" else
                                 "image/depth-zlib" if name == "camera_depth" else
                                 "data/json"}
                for name in (*card_names, "laser_view")
            }

            def __getattr__(self, _name):
                return lambda *_args, **_kwargs: None

        config = {"plugins": {
            **{name: {"enabled": True} for name in card_names},
            "base_drive": {"enabled": True, "max_linear_mps": 0.5,
                           "max_angular_deg_s": 45.0, "max_duration_s": 10.0},
            "arm_control": {"enabled": True, "max_duration_s": 20.0},
            "gripper": {"enabled": True},
            "lifting": {"enabled": True, "max_position_m": 0.8},
            "head": {"enabled": True},
        }}
        bundle = DriverBundle(plugins.build_plugins(config, "test", FakeDevice()))
        tools = bundle.get_all_tools()
        json.dumps({"jsonrpc": "2.0", "id": 1, "result": {"tools": tools}},
                   allow_nan=False)
        discovered = {tool["name"] for tool in tools}
        expected = {*card_names, "model", "base_drive", "arm_control",
                    "gripper", "lifting", "head"}
        self.assertEqual(discovered, expected)

    def test_control_cards_keep_safety_gates_and_add_lifecycle(self):
        class FakeDevice:
            topics = {
                name: {"topic": f"/test/{name}", "format": "data/json"}
                for name in ("joints", "robot_state", "imu", "odom", "laser_scan",
                             "camera_color", "camera_depth")
            }

            def __getattr__(self, _name):
                return lambda *_args, **_kwargs: None

        config = {"plugins": {
            "base_drive": {"enabled": True, "max_linear_mps": 0.5,
                           "max_angular_deg_s": 45.0, "max_duration_s": 10.0},
            "arm_control": {"enabled": True, "max_duration_s": 20.0},
            "gripper": {"enabled": True},
            "lifting": {"enabled": True, "max_position_m": 0.8},
            "head": {"enabled": True},
        }}
        tools = {plugin.name: plugin.get_tool()
                 for plugin in plugins.build_plugins(config, "test", FakeDevice())}
        base = tools["base_drive"]["inputSchema"]
        self.assertEqual(base["properties"]["confirm_motion"]["const"], True)
        self.assertEqual(base["x-completion"]["actions"], ["move"])
        self.assertEqual(base["x-hooks"]["on_interrupt_motion"], "stop")
        self.assertEqual(base["properties"]["angular_z_deg_s"]["maximum"], 45.0)
        self.assertEqual(base["properties"]["duration_s"]["maximum"], 10.0)
        arm = tools["arm_control"]["inputSchema"]
        self.assertTrue({"move", "home", "stop"} <= set(arm["properties"]["action"]["enum"]))
        self.assertTrue({"la0_deg", "la7_deg", "ra0_deg", "ra7_deg"} <= set(arm["properties"]))
        self.assertEqual(arm["properties"]["confirm_motion"]["const"], True)
        gripper = tools["gripper"]["inputSchema"]
        self.assertEqual(gripper["properties"]["side"]["enum"], ["left", "right", "both"])
        self.assertTrue({"set", "open", "close"} <= set(gripper["properties"]["action"]["enum"]))
        self.assertNotIn("mode", gripper["properties"])
        head = tools["head"]["inputSchema"]
        self.assertTrue({"look", "home", "stop"} <= set(head["properties"]["action"]["enum"]))
        self.assertTrue({"yaw_deg", "pitch_deg"} <= set(head["properties"]))
        exposed = json.dumps(tools, ensure_ascii=False)
        self.assertIsNone(re.search(r"[一-鿿]", exposed))
        self.assertNotRegex(exposed, r'"(?:head[01]|la[0-7]|ra[0-7])_rad"')

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

    def test_bundled_model_and_standardized_streams_are_packaged(self):
        model = DRIVER / "resource" / "panther.urdf"
        self.assertTrue(model.exists())
        self.assertIn("joint_la0", model.read_text())
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

    def test_live_urdf_limits_are_authoritative(self):
        urdf = """<robot name='p'><joint name='joint_la0' type='revolute'>
          <limit lower='-1.0' upper='2.0' effort='1' velocity='1'/>
        </joint></robot>"""
        self.assertEqual(contracts.parse_joint_limits(urdf), {"joint_la0": (-1.0, 2.0)})

    def test_render_model_and_joint_indices_share_one_snapshot(self):
        device = (DRIVER / "device.py").read_text()
        self.assertIn("self._render_urdf = self._urdf", device)
        self.assertIn("self._render_joint_indices = dict(self._joint_indices)", device)
        self.assertIn("joint_indices = dict(self._render_joint_indices)", device)
        self.assertIn('"model_sha256": model_sha256', device)
        self.assertIn('"source": render_source', device)

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
            contracts.validate_joint_target(("joint_la0",), [0.0], {}, {"joint_la0": 0.0}, 0.5)

    def test_joint_target_rejects_large_commissioning_step(self):
        with self.assertRaisesRegex(ValueError, "commissioning gate"):
            contracts.validate_joint_target(
                ("joint_la0",), [0.8], {"joint_la0": (-1.0, 1.0)}, {"joint_la0": 0.0}, 0.35
            )

    def test_drive_requires_explicit_motion_confirmation(self):
        config = {"max_linear_mps": 0.5, "max_angular_deg_s": 45.0, "max_duration_s": 10.0}
        with self.assertRaisesRegex(ValueError, "confirm_motion"):
            contracts.validate_drive({"linear_x": 0.1, "duration_s": 1.0}, config)

    def test_drive_caps_and_finite_duration(self):
        config = {"max_linear_mps": 0.5, "max_angular_deg_s": 45.0, "max_duration_s": 10.0}
        command = contracts.validate_drive(
            {"linear_x": 0.5, "linear_y": -0.5, "angular_z_deg_s": 45.0,
             "duration_s": 10.0, "confirm_motion": True}, config
        )
        self.assertEqual(command[:2], (0.5, -0.5))
        self.assertAlmostEqual(command[2], math.radians(45.0))
        self.assertEqual(command[3], 10.0)
        with self.assertRaisesRegex(ValueError, "duration_s"):
            contracts.validate_drive(
                {"linear_x": 0.0, "duration_s": -1, "confirm_motion": True}, config
            )

    def test_manifest_lists_expected_cards(self):
        driver_text = (DRIVER / "driver.yaml").read_text()
        cards = set(re.findall(r"name:\s*([a-z_]+)", driver_text))
        expected = {"joints", "model", "robot_state", "imu", "odom", "laser_scan",
                    "camera_color", "camera_depth", "base_drive", "arm_control",
                    "gripper", "lifting", "head"}
        self.assertTrue(expected <= cards)
        self.assertFalse({"robot_status", "battery", "gripper_state", "lifting_state",
                          "ultrasonic", "nav", "servo"} & cards)

    def test_bridge_exposes_consolidated_state_merged_lidar_and_typed_media(self):
        topics = core_bridge.output_topics("test")
        self.assertEqual(topics["robot_state"]["format"], "data/json")
        self.assertEqual(topics["laser_scan"]["topic"], "/test/panther/laser_scan")
        self.assertEqual(topics["laser_view"]["format"], "image/jpeg")
        self.assertEqual(topics["camera_color"]["format"], "image/jpeg")
        self.assertEqual(topics["camera_depth"]["format"], "image/depth-zlib")
        self.assertFalse({"laser_front", "laser_rear", "laser_fused"} & set(topics))

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

    def test_camera_only_manages_its_unique_transient_unit(self):
        runtime = object.__new__(camera.PantherCameraRuntime)
        runtime._node = SimpleNamespace(count_publishers=lambda _topic: 0)
        runtime._lock = threading.RLock()
        runtime._source_unit = None
        runtime._managed_unit = None
        calls = []

        def run_host(command, timeout=15.0, check=True):
            calls.append((command, timeout, check))
            if command[:2] == ["systemctl", "is-active"]:
                return SimpleNamespace(stdout="inactive\n", returncode=3)
            return SimpleNamespace(stdout="", returncode=0)

        runtime._run_host = run_host
        runtime._ensure_source()
        commands = [call[0] for call in calls]
        self.assertFalse(any(command[:2] == ["systemctl", "start"] for command in commands))
        launch = next(command for command in commands if command[0] == "systemd-run")
        unit = next(value.split("=", 1)[1] for value in launch if value.startswith("--unit="))
        self.assertRegex(unit, r"^panther-orbbec-motus-[0-9a-f]{12}\.service$")
        self.assertEqual(runtime._managed_unit, unit)
        self.assertEqual(runtime._source_unit, unit)

        external = object.__new__(camera.PantherCameraRuntime)
        external._node = SimpleNamespace(count_publishers=lambda _topic: 0)
        external._lock = threading.RLock()
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

    def test_canvas_registration_identity_remains_backward_compatible(self):
        config = (DRIVER / "config.yaml").read_text()
        manifest = (DRIVER / "driver.yaml").read_text()
        main = (DRIVER / "main.py").read_text()
        self.assertIn('name: "UniX AI Panther"', config)
        self.assertIn("name: UniX AI Panther Driver", manifest)
        self.assertIn("camera_color", config)
        self.assertIn("camera_color", manifest)
        self.assertNotIn("camera_rgb", manifest)
        self.assertIn('"unixai-panther-device-bundle"', main)
        self.assertLess(main.index("ThreadingHTTPServer("), main.index("start_registration("))

    def test_action_resources_follow_agent_core_conventions(self):
        plugins = (DRIVER / "plugins.py").read_text()
        self.assertIn('["arm_l", "arm_r"]', plugins)
        self.assertIn('["hand_l", "hand_r"]', plugins)
        self.assertIn('"waist"', plugins)
        self.assertNotIn('"dual_arms"', plugins)
        self.assertNotIn('"grippers"', plugins)

    def test_deployment_contract_and_motion_gate_are_explicit(self):
        config = (DRIVER / "config.yaml").read_text()
        manifest = (DRIVER / "driver.yaml").read_text()
        service = (DRIVER / "deploy" / "service.yml").read_text()
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("commissioning_motion_enabled: false", config)
        for text in (config, manifest, dockerfile):
            self.assertIn("15792", text)
        self.assertIn("AGENT_CORE_ROS_DOMAIN_ID=42", service)
        self.assertIn("PANTHER_ROS_DOMAIN_ID=0", service)
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml", service)
        self.assertIn(
            "/home/unix_ai/config/cyclonedds.xml:/etc/cyclonedds/panther.xml:ro",
            service,
        )
        self.assertIn("CYCLONEDDS_URI=file:///etc/cyclonedds/panther.xml", service)
        self.assertIn("network_mode: host", service)
        self.assertIn("stop_grace_period: 30s", service)

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

