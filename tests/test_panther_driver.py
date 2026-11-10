from __future__ import annotations

import importlib.util
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "unixai" / "panther"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, DRIVER / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


contracts = _load("contracts")
core_bridge = _load("core_bridge")
plugins = _load("plugins")


class PantherDriverTest(unittest.TestCase):
    def test_live_urdf_limits_are_authoritative(self):
        urdf = """<robot name='p'><joint name='joint_la0' type='revolute'>
          <limit lower='-1.0' upper='2.0' effort='1' velocity='1'/>
        </joint></robot>"""
        self.assertEqual(contracts.parse_joint_limits(urdf), {"joint_la0": (-1.0, 2.0)})

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
        config = {"max_linear_mps": 0.2, "max_angular_radps": 0.3, "max_duration_s": 3.0}
        with self.assertRaisesRegex(ValueError, "confirm_motion"):
            contracts.validate_drive({"linear_x": 0.1, "duration_s": 1.0}, config)

    def test_drive_caps_and_finite_duration(self):
        config = {"max_linear_mps": 0.2, "max_angular_radps": 0.3, "max_duration_s": 3.0}
        self.assertEqual(contracts.validate_drive(
            {"linear_x": 0.1, "linear_y": -0.1, "angular_z": 0.2,
             "duration_s": 1.0, "confirm_motion": True}, config
        ), (0.1, -0.1, 0.2, 1.0))
        with self.assertRaisesRegex(ValueError, "duration_s"):
            contracts.validate_drive(
                {"linear_x": 0.0, "duration_s": -1, "confirm_motion": True}, config
            )

    def test_manifest_lists_expected_cards(self):
        driver_text = (DRIVER / "driver.yaml").read_text()
        cards = set(re.findall(r"name:\s*([a-z_]+)", driver_text))
        expected = {"joints", "model", "robot_state", "imu", "odom", "laser_scan",
                    "camera_rgb", "camera_depth", "base_drive", "arm_control",
                    "gripper", "lifting", "head"}
        self.assertTrue(expected <= cards)
        self.assertFalse({"robot_status", "battery", "gripper_state", "lifting_state",
                          "ultrasonic", "nav", "servo"} & cards)

    def test_bridge_exposes_consolidated_state_merged_lidar_and_typed_media(self):
        topics = core_bridge.output_topics("test")
        self.assertEqual(topics["robot_state"]["format"], "data/json")
        self.assertEqual(topics["laser_scan"]["topic"], "/test/panther/laser_scan")
        self.assertEqual(topics["camera_rgb"]["format"], "image/jpeg")
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
            topics = {"camera_rgb": {"topic": "/test/rgb", "format": "image/jpeg"}}

            def __init__(self):
                self.calls = []

            def camera(self, kind, args):
                self.calls.append((kind, args["action"]))
                return {"state": "running"}

        device = FakeDevice()
        plugin = plugins.CameraPlugin(device, "camera_rgb", "rgb", "test")
        plugin.start()
        self.assertEqual(device.calls, [])
        plugin.dispatch("start", {})
        self.assertEqual(device.calls, [("rgb", "start")])

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
        self.assertIn("network_mode: host", service)
        self.assertIn("stop_grace_period: 15s", service)

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

