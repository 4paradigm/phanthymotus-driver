"""Pure angle math and control law for the Bumi loco_destination card."""

import importlib.util
import sys
import types
from pathlib import Path


def _load_device(monkeypatch):
    """Load device.py with ROS message stubs (scaffold as in test_mic_card)."""
    class Node:
        def __init__(self, name):
            self.subscriptions = []
            self.published = []

        def create_subscription(self, message_type, topic, callback, qos):
            self.subscriptions.append((topic, callback))

        def create_publisher(self, message_type, topic, qos):
            return types.SimpleNamespace(publish=self.published.append)

    class String:
        def __init__(self, data=""):
            self.data = data

    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: object()
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    modules = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": qos,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
        "audio_msgs": types.ModuleType("audio_msgs"),
        "audio_msgs.msg": types.ModuleType("audio_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
    }
    modules["rclpy.node"].Node = Node
    modules["std_msgs.msg"].String = String

    class AudioChunk:
        def __init__(self):
            self.header = types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=0, nanosec=0))

    modules["audio_msgs.msg"].AudioChunk = AudioChunk
    modules["sensor_msgs.msg"].CompressedImage = type("CompressedImage", (), {})
    modules["sensor_msgs.msg"].Image = type("Image", (), {})
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "device.py"
    spec = importlib.util.spec_from_file_location("bumi_loco_destination_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, String


def test_wrap180_takes_shortest_path(monkeypatch):
    module, _ = _load_device(monkeypatch)
    assert module._wrap180(190) == -170
    assert module._wrap180(-190) == 170
    assert module._wrap180(350) == -10
    assert module._wrap180(0) == 0


def test_yaw_deg_from_sdk_quaternion(monkeypatch):
    # ori 顺序 [x, y, z, w]（SDK 序）；纯 yaw 四元数 (0,0,sin(θ/2),cos(θ/2))
    import math
    module, _ = _load_device(monkeypatch)
    for theta in (0.0, 30.0, -45.0, 170.0):
        half = math.radians(theta) / 2
        ori = [0.0, 0.0, math.sin(half), math.cos(half)]
        assert abs(module._yaw_deg_from_quat(ori) - theta) < 1e-6


def test_rotate_vyaw_uses_three_segment_profile(monkeypatch):
    module, _ = _load_device(monkeypatch)
    assert module._rotate_vyaw(60) == 0.6
    assert module._rotate_vyaw(-30) == -0.6
    assert module._rotate_vyaw(10) == 0.3        # P 段被段速封顶
    assert abs(module._rotate_vyaw(3) - 0.15) < 1e-9   # min(0.15, 0.15)
    assert abs(module._rotate_vyaw(-2.5) + 0.125) < 1e-9  # P 段未触顶


def test_rotate_timeout_s_stays_under_llm_budget(monkeypatch):
    module, _ = _load_device(monkeypatch)
    assert module._rotate_timeout_s(60) == 14.0
    assert module._rotate_timeout_s(360) == 44.0
    assert all(module._rotate_timeout_s(a) <= 45.0 for a in (0.5, 60, 180, 360))  # < agent-core 60s
