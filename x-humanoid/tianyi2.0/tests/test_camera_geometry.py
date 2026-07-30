import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


class _DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _DummyNode:
    def __init__(self, name, context=None):
        self.name = name
        self.context = context
        self.subscriptions = []

    def create_publisher(self, _message_type, _topic, _qos):
        return _DummyPublisher()

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription


class _DummyExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class _DummyRos2:
    def __init__(self):
        self.ctx_tianyi = object()
        self.ctx_core = object()
        self.executor_tianyi = _DummyExecutor()
        self.executor_core = _DummyExecutor()


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    qos = types.ModuleType("rclpy.qos")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _QoSProfile:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class _ReliabilityPolicy:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"

    class _HistoryPolicy:
        KEEP_LAST = "keep_last"

    class _DurabilityPolicy:
        VOLATILE = "volatile"
        TRANSIENT_LOCAL = "transient_local"

    class _String:
        def __init__(self, data=""):
            self.data = data

    class _Bool:
        def __init__(self, data=False):
            self.data = data

    node.Node = _DummyNode
    qos.QoSProfile = _QoSProfile
    qos.ReliabilityPolicy = _ReliabilityPolicy
    qos.HistoryPolicy = _HistoryPolicy
    qos.DurabilityPolicy = _DurabilityPolicy
    std_msgs_msg.String = _String
    std_msgs_msg.Bool = _Bool

    rclpy.node = node
    rclpy.qos = qos
    std_msgs.msg = std_msgs_msg
    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": node,
        "rclpy.qos": qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    })


def _load_device_module():
    _install_ros_stubs()
    device_path = Path(__file__).resolve().parents[1] / "device.py"
    spec = importlib.util.spec_from_file_location("tianyi2_device_geometry_test", device_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


DEVICE = _load_device_module()


def _header(frame_id, sec=123, nanosec=456_000_000):
    return SimpleNamespace(
        frame_id=frame_id,
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
    )


def _camera_info(frame_id, fx=600.0, fy=601.0, cx=320.0, cy=240.0):
    return SimpleNamespace(
        header=_header(frame_id),
        height=480,
        width=640,
        distortion_model="plumb_bob",
        d=[0.1, -0.2, 0.003, 0.004, 0.0],
        k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=2,
        binning_y=3,
        roi=SimpleNamespace(
            x_offset=4,
            y_offset=5,
            height=470,
            width=630,
            do_rectify=True,
        ),
    )


class CameraGeometryTests(unittest.TestCase):
    def setUp(self):
        self.plugin = DEVICE.CameraPlugin({}, "robot", _DummyRos2())

    def test_schema_exposes_camera_stream_and_geometry_resource(self):
        tools = {tool["name"]: tool for tool in self.plugin.get_tools()}

        self.assertEqual({"camera_head", "camera_geometry"}, set(tools))
        self.assertEqual("sensor", tools["camera_head"]["type"])
        self.assertEqual("resource", tools["camera_geometry"]["type"])
        self.assertNotIn("topic_out", tools["camera_geometry"])

    def test_camera_info_conversion_preserves_complete_geometry(self):
        converted = self.plugin._camera_info_to_dict(_camera_info("color_optical_frame"))

        self.assertEqual(640, converted["width"])
        self.assertEqual(480, converted["height"])
        self.assertEqual(
            {"fx": 600.0, "fy": 601.0, "cx": 320.0, "cy": 240.0},
            converted["intrinsics"],
        )
        self.assertEqual("plumb_bob", converted["distortion_model"])
        self.assertEqual([0.1, -0.2, 0.003, 0.004, 0.0], converted["d"])
        self.assertEqual(9, len(converted["k"]))
        self.assertEqual(9, len(converted["r"]))
        self.assertEqual(12, len(converted["p"]))
        self.assertEqual({"x": 2, "y": 3}, converted["binning"])
        self.assertEqual(
            {
                "x_offset": 4,
                "y_offset": 5,
                "height": 470,
                "width": 630,
                "do_rectify": True,
            },
            converted["roi"],
        )
        self.assertEqual("color_optical_frame", converted["frame_id"])
        self.assertEqual(
            {"sec": 123, "nanosec": 456_000_000},
            converted["timestamp"],
        )
        self.assertEqual(123_456, converted["timestamp_ms"])

    def test_missing_geometry_returns_availability_instead_of_error(self):
        snapshot = self.plugin.dispatch("camera_geometry", {})

        self.assertFalse(snapshot["available"])
        self.assertEqual(
            {"color": False, "depth": False, "depth_to_color": False},
            snapshot["availability"],
        )
        self.assertEqual(["color", "depth"], snapshot["missing"])
        self.assertIsNone(snapshot["color"])
        self.assertIsNone(snapshot["depth"])
        self.assertFalse(snapshot["depth_to_color"]["available"])
        self.assertIsNone(snapshot["depth_to_color"]["supported"])
        self.assertEqual("plugin_not_started", snapshot["depth_to_color"]["reason"])

    def test_complete_color_and_depth_geometry_is_available(self):
        self.plugin._on_camera_info("color", _camera_info("color_optical_frame"))
        self.plugin._on_camera_info(
            "depth",
            _camera_info("depth_optical_frame", fx=580.0, fy=581.0),
        )

        snapshot = self.plugin.dispatch("camera_geometry", {})

        self.assertTrue(snapshot["available"])
        self.assertTrue(snapshot["availability"]["color"])
        self.assertTrue(snapshot["availability"]["depth"])
        self.assertEqual([], snapshot["missing"])
        self.assertEqual("color_optical_frame", snapshot["color"]["frame_id"])
        self.assertEqual(580.0, snapshot["depth"]["intrinsics"]["fx"])

    def test_optional_depth_to_color_extrinsics_use_meters_and_header(self):
        extrinsics = SimpleNamespace(
            header=_header("depth_to_color_extrinsics", sec=321, nanosec=7),
            rotation=[float(i) for i in range(9)],
            translation=[0.01, -0.02, 0.03],
        )

        self.plugin._on_depth_to_color(extrinsics)
        snapshot = self.plugin.dispatch("camera_geometry", {})
        depth_to_color = snapshot["depth_to_color"]

        self.assertTrue(depth_to_color["supported"])
        self.assertTrue(depth_to_color["available"])
        self.assertEqual([float(i) for i in range(9)], depth_to_color["rotation"])
        self.assertEqual([0.01, -0.02, 0.03], depth_to_color["translation_m"])
        self.assertEqual("depth_to_color_extrinsics", depth_to_color["frame_id"])
        self.assertEqual({"sec": 321, "nanosec": 7}, depth_to_color["timestamp"])

    def test_extrinsics_subscription_uses_latched_vendor_qos(self):
        package = types.ModuleType("orbbec_camera_msgs")
        package.__path__ = []
        message_module = types.ModuleType("orbbec_camera_msgs.msg")
        message_module.Extrinsics = type("Extrinsics", (), {})
        package.msg = message_module
        sys.modules["orbbec_camera_msgs"] = package
        sys.modules["orbbec_camera_msgs.msg"] = message_module
        try:
            self.plugin._subscribe_depth_to_color()
        finally:
            sys.modules.pop("orbbec_camera_msgs.msg", None)
            sys.modules.pop("orbbec_camera_msgs", None)

        subscription = self.plugin._sub_node.subscriptions[-1]
        self.assertEqual("/ob_camera_head/depth_to_color", subscription.topic)
        self.assertEqual("reliable", subscription.qos.reliability)
        self.assertEqual(1, subscription.qos.depth)
        self.assertEqual("transient_local", subscription.qos.durability)

    def test_unavailable_optional_message_package_has_clear_degraded_state(self):
        self.plugin._set_extrinsics_unavailable(
            "orbbec_camera_msgs.msg.Extrinsics is unavailable"
        )

        snapshot = self.plugin.dispatch("camera_geometry", {})

        self.assertFalse(snapshot["depth_to_color"]["supported"])
        self.assertFalse(snapshot["depth_to_color"]["available"])
        self.assertEqual(
            "orbbec_camera_msgs.msg.Extrinsics is unavailable",
            snapshot["depth_to_color"]["reason"],
        )

    def test_vendored_extrinsics_interface_matches_orbbec_contract(self):
        tianyi_dir = Path(__file__).resolve().parents[1]
        definition = (
            tianyi_dir
            / "msgs"
            / "orbbec_camera_msgs"
            / "msg"
            / "Extrinsics.msg"
        ).read_text().splitlines()

        self.assertEqual(
            [
                "std_msgs/Header header",
                "float64[9] rotation",
                "float64[3] translation",
            ],
            definition,
        )


if __name__ == "__main__":
    unittest.main()
