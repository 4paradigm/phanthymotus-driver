"""Monitor contracts and lifecycle, with no ROS/SDK/CAN dependencies."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agilex.piper.monitor import PiperMonitorPlugin, monitor_frame, _RosPublisher, JOINT_NAMES, RESOURCE_DIR, load_model
from common.vendor_runtime import DriverBundle


def snapshot():
    return {"device_name": "机械臂 1", "connected": True, "feedback_time": 1000.,
            "feedback_fresh": True, "joints_deg": [0., 90., -90., 10., -20., 30.],
            "joint_limits_deg": [[-150, 150], [0, 180], [-170, 0], [-100, 100], [-70, 70], [-120, 120]],
            "control_mode": "CAN_CTRL(0x1)", "arm_status": "NORMAL(0x0)", "enabled": [True] * 6,
            "gripper": {"width_mm": 12.5, "enabled": False}, "motion_enabled": False,
            "error_code": 0, "motor_faults": [], "busy": False,
            "active_card": None, "completed_actions": {"arm_control": {"status": "completed"}}}


class Transport:
    def __init__(self, ros2, namespace, topics, period, callback):
        self.callback, self.period, self.topics = callback, period, topics
        self.messages = []
        self.closed = 0
        self.fail = False

    def publish(self, fmt, data):
        if self.fail:
            raise RuntimeError("DDS offline")
        # Strict serialization also rejects accidental NaN in nested data.
        self.messages.append((fmt, json.loads(json.dumps(data, allow_nan=False))))

    def close(self):
        self.closed += 1


class MonitorTests(unittest.TestCase):
    def test_new_skeleton_only_mode_never_declares_or_publishes_parameters(self):
        plugin = PiperMonitorPlugin(snapshot, namespace='arm1/piper', device_name='PiPER',
            ros2=object(), clock=lambda: 1000., publisher_factory=Transport,
            language='en', publish_parameters=False)
        self.addCleanup(plugin.stop)
        plugin.start()
        transport = plugin._transport
        self.assertEqual(transport.period, .1)
        self.assertEqual(transport.topics, [{'topic': '/arm1/piper/state/joints', 'format': 'sensor/skeleton'}])
        transport.callback()
        self.assertEqual([fmt for fmt, _ in transport.messages], ['sensor/skeleton'])
        self.assertEqual(transport.messages[0][1]['joints'][1]['q'], math.pi / 2)
        self.assertIn('Fault code', plugin.info()['parameters'])
        plugin.stop()
        transport.callback()
        self.assertEqual(len(transport.messages), 1)
        self.assertEqual(transport.closed, 1)

    def setUp(self):
        self.now = 1000.
        self.data = snapshot()
        self.reads = 0
        self.transports = []

        def read():
            self.reads += 1
            return deepcopy(self.data)

        def factory(*args):
            transport = Transport(*args)
            self.transports.append(transport)
            return transport

        self.plugin = PiperMonitorPlugin(read, namespace="agilex/piper", device_name="机械臂 1",
                                         ros2=object(), clock=lambda: self.now, publisher_factory=factory)
        self.addCleanup(self.plugin.stop)

    def test_deployed_passive_snapshot_names_without_mutation(self):
        data = snapshot()
        data["fresh"] = data.pop("feedback_fresh")
        data["faults"] = {key: data.pop(key) for key in ("arm_status", "error_code", "motor_faults")}
        data["last_actions"] = data.pop("completed_actions")
        original = deepcopy(data)
        skeleton, params, stamp = monitor_frame(data, now=1000., device_name="机械臂 1")
        self.assertIsNotNone(skeleton)
        self.assertEqual(params["反馈状态"], "新鲜")
        self.assertEqual(params["设备状态"], "NORMAL(0x0)")
        self.assertEqual(params["故障码"], 0)
        self.assertEqual(params["最近动作结果"], data["last_actions"])
        self.assertEqual(data, original)
        data["fresh"] = False
        self.assertIsNone(monitor_frame(data, now=1000., device_name="机械臂 1")[0])

    def test_explicit_canonical_fields_override_snapshot_aliases(self):
        data = snapshot()
        data.update(fresh=True, feedback_fresh=False,
                    faults={"error_code": 99, "arm_status": "ERROR", "motor_faults": ["J1"]},
                    last_actions={"other": "ignored"})
        skeleton, params, stamp = monitor_frame(data, now=1000., device_name="机械臂 1")
        self.assertIsNone(skeleton)
        self.assertEqual(params["故障码"], 0)
        self.assertEqual(params["设备状态"], "NORMAL(0x0)")
        self.assertEqual(params["最近动作结果"], data["completed_actions"])

    def test_bundle_contract_plain_model_and_no_construction_or_start_reads(self):
        bundle = DriverBundle([self.plugin])
        tools = bundle.get_all_tools()
        self.assertEqual([t["name"] for t in tools], ["piper_monitor", "model"])
        self.assertEqual([t["type"] for t in tools], ["sensor", "resource"])
        model = bundle.dispatch("model", {})
        self.assertEqual(set(model), {"urdf"})
        self.assertEqual(ET.fromstring(model["urdf"]).tag, "robot")
        bundle.start_all()
        self.assertEqual(self.reads, 0)
        self.assertEqual(self.transports[0].period, .1)
        self.assertEqual(bundle.dispatch("piper_monitor", {"action": "info"})["state"], "running")
        result = bundle.dispatch("piper_monitor", {"action": "read"})
        self.assertEqual(result["parameters"]["控制模式"], "电脑控制（CAN）")

    def test_measured_angles_units_names_parameters_and_no_snapshot_mutation(self):
        original = deepcopy(self.data)
        frame = self.plugin.read()
        self.assertEqual(tuple(j["name"] for j in frame["skeleton"]["joints"]), JOINT_NAMES)
        self.assertAlmostEqual(frame["skeleton"]["joints"][1]["q"], math.pi / 2)
        params = frame["parameters"]
        self.assertEqual(params["J2 角度（°）"], 90)
        self.assertEqual(params["J2 有效限位（°）"], [0, 180])
        self.assertEqual(params["J1 电机"], "已使能")
        self.assertEqual(params["夹爪开度（mm）"], 12.5)
        self.assertEqual(params["夹爪电机"], "已失能")
        self.assertEqual(params["动作许可"], "关闭")
        self.assertEqual(self.data, original)

    def test_stale_disconnect_and_recovery_keep_last_valid_time(self):
        self.plugin.start()
        transport = self.transports[0]
        transport.callback()
        good_time = self.plugin.info()["parameters"]["最后有效时间（UTC）"]
        self.now += .501
        transport.callback()
        self.assertEqual([fmt for fmt, _ in transport.messages], ["data/json", "sensor/skeleton", "data/json"])
        self.assertEqual(transport.messages[-1][1]["反馈状态"], "反馈过期")
        self.assertEqual(transport.messages[-1][1]["最后有效时间（UTC）"], good_time)
        self.data["connected"] = False
        transport.callback()
        self.assertEqual(transport.messages[-1][1]["反馈状态"], "未连接")
        self.data.update(connected=True, feedback_time=self.now)
        transport.callback()
        self.assertEqual(transport.messages[-1][0], "sensor/skeleton")

    def test_age_boundary_and_provider_freshness_are_both_required(self):
        for age, source_fresh, expected in ((.5, True, True), (.50001, True, False),
                                            (-.001, True, False), (0, False, False), (0, None, False)):
            with self.subTest(age=age, source_fresh=source_fresh):
                self.data["feedback_fresh"] = source_fresh
                self.now = 1000 + age
                self.assertEqual(self.plugin.read()["skeleton"] is not None, expected)

    def test_invalid_joint_vectors_never_publish_skeleton(self):
        for angles in ([0] * 5, [0] * 7, [0, 0, 0, 0, 0, float("nan")],
                       [0, 0, 0, 0, 0, float("inf")], [0, 0, 0, 0, 0, 10**1000],
                       [0, 0, 0, 0, 0, True], "000000", None):
            with self.subTest(angles=angles):
                self.data["joints_deg"] = angles
                frame = self.plugin.read()
                self.assertIsNone(frame["skeleton"])
                self.assertEqual(frame["parameters"]["J1 角度（°）"], "不可用")
                json.dumps(frame, allow_nan=False)

    def test_missing_fields_do_not_become_zero_or_disabled(self):
        self.data = {}
        frame = self.plugin.read()
        self.assertIsNone(frame["skeleton"])
        for key in ("J1 角度（°）", "J1 电机", "动作许可", "故障码", "夹爪开度（mm）", "夹爪电机"):
            self.assertEqual(frame["parameters"][key], "不可用")
        self.assertEqual(frame["parameters"]["设备名称"], "机械臂 1")

    def test_invalid_optional_values_do_not_escape_as_nan(self):
        self.data.update(joint_limits_deg=[[float("nan"), 0]] * 6,
                         completed_actions={"arm_control": {"result": float("inf")}},
                         gripper={"enabled": "false", "width_mm": float("nan")})
        frame = self.plugin.read()
        self.assertEqual(frame["parameters"]["J1 有效限位（°）"], "不可用")
        self.assertEqual(frame["parameters"]["夹爪电机"], "不可用")
        self.assertEqual(frame["parameters"]["夹爪开度（mm）"], "不可用")
        json.dumps(frame, allow_nan=False)

    def test_reader_failure_publishes_error_and_recovers_without_restart(self):
        self.plugin.start()
        transport = self.transports[0]
        transport.callback()
        with patch.object(self.plugin, "_read_snapshot", side_effect=RuntimeError("CAN disconnected")):
            transport.callback()
        self.assertEqual(transport.messages[-1][0], "data/json")
        self.assertEqual(transport.messages[-1][1]["反馈状态"], "读取失败")
        self.assertEqual(transport.messages[-1][1]["J1 角度（°）"], "不可用")
        transport.callback()
        self.assertEqual(transport.messages[-1][0], "sensor/skeleton")

    def test_idempotent_start_stop_and_old_timer_cannot_publish(self):
        self.plugin.start()
        self.plugin.start()
        self.assertEqual(len(self.transports), 1)
        old = self.transports[0]
        self.plugin.stop()
        self.plugin.stop()
        self.assertEqual(old.closed, 1)
        self.plugin.start()
        old.callback()
        self.assertEqual(self.reads, 0)
        self.transports[1].callback()
        self.assertEqual(self.reads, 1)
        self.assertEqual(self.plugin.stop()["state"], "idle")

    def test_stop_during_slow_snapshot_returns_and_suppresses_old_publish(self):
        entered, release = threading.Event(), threading.Event()

        def slow():
            entered.set()
            release.wait(2)
            return snapshot()

        self.plugin._read_snapshot = slow
        self.plugin.start()
        transport = self.transports[0]
        thread = threading.Thread(target=transport.callback)
        thread.start()
        self.assertTrue(entered.wait(1))
        try:
            self.assertEqual(self.plugin.stop()["state"], "idle")
            self.assertEqual(transport.closed, 1)
            with self.assertRaisesRegex(RuntimeError, "正忙"):
                self.plugin.read()
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(transport.messages, [])
        self.assertEqual(self.plugin.info()["parameters"], {})

    def test_publish_failure_is_visible_and_retry_recovers(self):
        self.plugin.start()
        transport = self.transports[0]
        transport.fail = True
        transport.callback()
        self.assertEqual(self.plugin.info()["state"], "error")
        self.assertIn("DDS offline", self.plugin.info()["publish_error"])
        transport.fail = False
        transport.callback()
        self.assertEqual(self.plugin.info()["state"], "running")
        transport.fail = True
        transport.callback()
        self.assertEqual(self.plugin.stop()["state"], "idle")

    def test_failed_transport_start_can_be_retried(self):
        original = self.plugin._publisher_factory
        with patch.object(self.plugin, "_publisher_factory", side_effect=RuntimeError("attach failed")):
            with self.assertRaisesRegex(RuntimeError, "attach failed"):
                self.plugin.start()
        self.assertEqual(self.plugin.info()["state"], "idle")
        self.assertFalse(self.plugin.info()["publishing"])
        self.plugin._publisher_factory = original
        self.plugin.start()
        self.assertTrue(self.plugin.info()["publishing"])

    def test_dual_device_topics_and_payloads_are_independent(self):
        other_data = snapshot()
        other_data.update(device_name="机械臂 2", joints_deg=[10.] * 6)
        other = PiperMonitorPlugin(lambda: other_data, namespace="piper2/piper", device_name="机械臂 2", clock=lambda: self.now)
        topics1 = {t["topic"] for t in self.plugin.info()["topic_out"]}
        topics2 = {t["topic"] for t in other.info()["topic_out"]}
        self.assertFalse(topics1 & topics2)
        self.assertNotEqual(self.plugin.read()["skeleton"], other.read()["skeleton"])
        self.assertEqual(other.read()["parameters"]["设备名称"], "机械臂 2")

    def test_no_control_object_is_available_to_monitor(self):
        class FakeHardware:
            def __init__(self):
                self.commands = []

            def read_monitor_snapshot(self):
                return snapshot()

            def command(self, *args):
                self.commands.append(args)

        hardware = FakeHardware()
        plugin = PiperMonitorPlugin(hardware.read_monitor_snapshot, namespace="piper", device_name="机械臂 1", clock=lambda: 1000)
        plugin.start()
        plugin.read()
        plugin.info()
        plugin.dispatch("model", {"_tool_name": "model"})
        plugin.stop()
        self.assertEqual(hardware.commands, [])
        for verb in ("move", "enable", "disable", "switch", "capture", "halt"):
            with self.assertRaises(ValueError):
                plugin.dispatch(verb, {"_tool_name": "piper_monitor"})
        with self.assertRaises(ValueError):
            plugin.dispatch("read", {"execute": True})

    def test_invalid_namespace_rate_and_broken_model_fail_before_ros(self):
        for namespace in ("", "a-b", "piper//state", "1piper"):
            with self.subTest(namespace=namespace), self.assertRaises(ValueError):
                PiperMonitorPlugin(snapshot, namespace=namespace, device_name="test")
        for rate in (0, 31, float("nan"), float("inf"), True):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                PiperMonitorPlugin(snapshot, namespace="piper", device_name="test", publish_hz=rate)
        with patch("agilex.piper.monitor.load_model", side_effect=ValueError("bad URDF")):
            with self.assertRaisesRegex(ValueError, "bad URDF"):
                PiperMonitorPlugin(snapshot, namespace="piper", device_name="test")


class RosTransportTests(unittest.TestCase):
    def setUp(self):
        self.nodes, self.added, self.removed = [], [], []
        self.fail_timer = False
        self.fail_attach = False
        owner = self

        class Node:
            def __init__(self, name, context):
                self.name, self.context, self.destroyed = name, context, 0
                self.publishers = []
                owner.nodes.append(self)

            def create_publisher(self, message_type, topic, qos):
                messages = []
                publisher = types.SimpleNamespace(publish=messages.append, topic=topic, qos=qos, messages=messages)
                self.publishers.append(publisher)
                return publisher

            def create_timer(self, period, callback):
                if owner.fail_timer:
                    raise RuntimeError("timer failed")
                self.period, self.callback = period, callback

            def destroy_node(self):
                self.destroyed += 1

        def add(node):
            if self.fail_attach:
                raise RuntimeError("attach failed")
            self.added.append(node)

        self.ros = types.SimpleNamespace(ctx_core=object(), executor_core=types.SimpleNamespace(
            add_node=add, remove_node=self.removed.append))
        modules = {
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "rclpy.qos": types.SimpleNamespace(
                DurabilityPolicy=types.SimpleNamespace(VOLATILE="volatile"),
                HistoryPolicy=types.SimpleNamespace(KEEP_LAST="last"),
                ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT="best_effort"),
                QoSProfile=lambda **kw: kw),
            "std_msgs.msg": types.SimpleNamespace(String=types.SimpleNamespace),
        }
        self.patcher = patch.dict(sys.modules, modules)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)

    def test_real_transport_uses_core_context_qos_and_releases_node_once(self):
        transport = _RosPublisher(self.ros, "agilex/piper", [{"format": "data/json", "topic": "/piper/parameters"}], .1, lambda: None)
        node = self.nodes[0]
        self.assertIs(node.context, self.ros.ctx_core)
        self.assertEqual(node.period, .1)
        self.assertEqual(node.publishers[0].qos, {"reliability": "best_effort", "history": "last", "depth": 1, "durability": "volatile"})
        transport.publish("data/json", {"设备名称": "机械臂 1"})
        self.assertEqual(json.loads(node.publishers[0].messages[0].data), {"设备名称": "机械臂 1"})
        transport.close()
        transport.close()
        self.assertEqual(self.added, [node])
        self.assertEqual(self.removed, [node])
        self.assertEqual(node.destroyed, 1)

    def test_partial_setup_failure_destroys_node(self):
        for fail in ("fail_timer", "fail_attach"):
            with self.subTest(fail=fail):
                setattr(self, fail, True)
                with self.assertRaises(RuntimeError):
                    _RosPublisher(self.ros, "piper", [{"format": "data/json", "topic": "/piper/parameters"}], .1, lambda: None)
                setattr(self, fail, False)
                self.assertEqual(self.nodes[-1].destroyed, 1)
        self.assertEqual(self.added, [])
        self.assertEqual(self.removed, [])


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rotation(axis, angle):
    # Independent homogeneous Rodrigues transform.
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y, 0],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x, 0],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c, 0], [0, 0, 0, 1]]


def forward(xml, angles, renderer=False):
    robot = ET.fromstring(xml)
    transforms = {"dummy_link": [[1 if i == j else 0 for j in range(4)] for i in range(4)]}
    remaining = list(robot.findall("joint"))
    while remaining:
        progress = False
        for joint in remaining[:]:
            parent = joint.find("parent").attrib["link"]
            if parent not in transforms:
                continue
            origin = joint.find("origin")
            xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
            r, p, y = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            rx, ry, rz = rotation((1, 0, 0), r), rotation((0, 1, 0), p), rotation((0, 0, 1), y)
            local = multiply(multiply(rx, ry), rz) if renderer else multiply(multiply(rz, ry), rx)
            for i in range(3):
                local[i][3] = xyz[i]
            if joint.attrib["type"] == "revolute":
                axis = [float(x) for x in joint.find("axis").attrib["xyz"].split()]
                local = multiply(local, rotation(axis, angles[JOINT_NAMES.index(joint.attrib["name"])]))
            transforms[joint.find("child").attrib["link"]] = multiply(transforms[parent], local)
            remaining.remove(joint)
            progress = True
        if not progress:
            raise AssertionError("Broken model tree")
    return transforms


class ModelTests(unittest.TestCase):
    def test_pinned_upstream_and_license_bytes_are_unchanged(self):
        for filename, digest in (
            ("upstream/piper_description.urdf", "c1a5c29ae9b91160557ed02bddd47365a71f0eb1653abb4997eb34f124021486"),
            ("LICENSE.piper_ros", "e32d78fd3761052ab0b2bb42fef8a7a9ad3f298d315d305333c349d2aff1f134"),
        ):
            with self.subTest(filename=filename):
                self.assertEqual(hashlib.sha256((RESOURCE_DIR / filename).read_bytes()).hexdigest(), digest)

    def test_renderer_chain_matches_official_urdf_at_zero_and_nonzero_poses(self):
        source = (RESOURCE_DIR / "upstream/piper_description.urdf").read_text()
        exported = load_model()
        for degrees in ([0] * 6, [15, 30, -40, 20, -10, 60], [-90, 100, -110, -50, 35, -80]):
            with self.subTest(degrees=degrees):
                angles = [math.radians(x) for x in degrees]
                official, actual = forward(source, angles), forward(exported, angles, renderer=True)
                for link in ("base_link", *(f"link{i}" for i in range(1, 9)), "gripper_base"):
                    for i in range(4):
                        for j in range(4):
                            self.assertAlmostEqual(actual[link][i][j], official[link][i][j], places=12)
        # Demonstrate that feeding the unadapted official URDF to this renderer
        # actually has a different joint2 transform (regression sensitivity).
        raw = forward(source, [0] * 6, renderer=True)
        ref = forward(source, [0] * 6)
        self.assertGreater(max(abs(raw["link2"][i][j] - ref["link2"][i][j]) for i in range(4) for j in range(4)), .1)

    def test_only_six_revolute_joints_and_no_mesh_dependencies(self):
        model = ET.fromstring(load_model())
        self.assertEqual([j.attrib["name"] for j in model.findall("joint") if j.attrib["type"] == "revolute"], list(JOINT_NAMES))
        self.assertFalse(model.findall(".//mesh"))
        self.assertFalse([j for j in model.findall("joint") if j.attrib["type"] == "prismatic"])

    def test_display_shell_branches_are_fixed_and_follow_the_original_links(self):
        model = load_model()
        root = ET.fromstring(model)
        display = [j for j in root.findall('joint') if j.get('name').startswith('display_')]
        self.assertGreater(len(display), 100)
        self.assertTrue(all(j.get('type') == 'fixed' for j in display))
        # A housing vertex retains its local offset at both measured poses;
        # the real joints/link transforms remain covered independently above.
        for degrees in ([0]*6, [20,40,-50,10,-15,60]):
            actual = forward(model, [math.radians(x) for x in degrees], renderer=True)
            parent = actual['link2']
            vertex = actual['display_link2_0_vertex0']
            expected = [sum(parent[row][k]*[.043,0,-.037,1][k] for k in range(4)) for row in range(4)]
            for row in range(4):
                self.assertAlmostEqual(vertex[row][3], expected[row], places=12)


if __name__ == "__main__":
    unittest.main()
