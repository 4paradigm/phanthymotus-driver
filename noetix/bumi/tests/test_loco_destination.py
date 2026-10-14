"""Angle math, control law, and closed-loop rotate for the Bumi loco_destination card."""

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


class FakeHighCtrl:
    """一阶植物：publish_cmd 的 z(vyaw) 按 GAIN 度/次推进 yaw。"""
    def __init__(self, mode=2, yaw0=0.0, gain_deg_per_tick=3.0):
        self.mode, self.yaw, self.gain = mode, yaw0, gain_deg_per_tick
        self.cmds = []          # (x, y, z) 每次 publish_cmd 记录
    def get_mode(self): return self.mode
    def get_imu_data(self):
        import math
        half = math.radians(self.yaw) / 2
        return types.SimpleNamespace(
            ori=[0.0, 0.0, math.sin(half), math.cos(half)],
            angular_vel=[0.0, 0.0, 0.0], linear_acc=[0.0, 0.0, 9.8])
    def publish_cmd(self, x, y, z, action_cmd, index=0):
        self.cmds.append((x, y, z))
        self.yaw += z * self.gain          # z 即 vyaw


def _rotate_plugin(module, monkeypatch, fake):
    """装配：绕开 highcontrol_py 懒加载 + 建 LocoPlugin（同上装配法）。"""
    monkeypatch.setattr(module, '_get_control_cmd', lambda name: name)
    return module.LocoPlugin(
        {}, 'robot', types.SimpleNamespace(add_node=lambda n: None), fake)


def test_loco_destination_tool_schema_contract(monkeypatch):
    module, _ = _load_device(monkeypatch)
    plugin = module.LocoPlugin({}, 'robot', types.SimpleNamespace(add_node=lambda n: None), object())
    tool = [t for t in plugin.get_tools() if t['name'] == 'loco_destination'][0]
    assert tool['type'] == 'actuator' and tool['multiInstance'] is False
    props = tool['inputSchema']['properties']
    assert set(props['action']['enum']) == {'rotate', 'stop_move', 'status'}
    assert props['angle_deg']['minimum'] == -360 and props['angle_deg']['maximum'] == 360
    assert tool['inputSchema']['x-hooks'] == {'on_interrupt_motion': {'action': 'stop_move'}}
    assert set(tool['inputSchema']['x-action-params']) == {'rotate', 'stop_move', 'status'}
    assert tool['inputSchema']['x-action-params']['rotate']['params'] == ['angle_deg']


def test_rotate_converges_within_tolerance_and_brakes(monkeypatch):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2.0
    assert abs(result['turned_deg'] - 60) < 3            # 植物实际转过约 60°
    assert fake.cmds[-1] == (0, 0, 0)                    # 退出必零速
    assert all(c == (0, 0, 0) for c in fake.cmds[-module._ROTATE_SETTLE_TICKS:])


def test_rotate_takes_shortest_path_across_yaw_wrap(monkeypatch):
    # fake yaw0=170，rotate +20 → 目标 -170；轨迹应过 ±180 递增，不得回摆
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(yaw0=170.0, gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 20})
    assert result['state'] == 'done' and abs(result['final_error_deg']) <= 2.0
    assert result['turned_deg'] > 0                      # 逆时针 +20，而非 -340
