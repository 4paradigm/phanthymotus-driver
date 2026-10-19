"""Angle math, control law, and closed-loop rotate for the Bumi loco_destination card."""

import __future__
import importlib.util
import sys
import threading
import time
import types
from pathlib import Path

import pytest
import yaml


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


@pytest.fixture(autouse=True)
def _stub_acp_notify(monkeypatch):
    """单测绝不对真实 Agent Core 发 HTTP 回调：默认桩掉每个新建 device 模块的
    _acp_notify（先例 unitree/g1/tests/test_vision_capture_race.py:34）。
    需要断言回调的测试随后自行 monkeypatch.setattr 覆盖。"""
    real_load = _load_device

    def _load_with_stub(mp):
        module, String = real_load(mp)
        mp.setattr(module, '_acp_notify', lambda *args, **kwargs: None)
        return module, String

    monkeypatch.setattr(sys.modules[__name__], '_load_device', _load_with_stub)


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


def test_pulse_width_tracks_measured_turn_response(monkeypatch):
    module, _ = _load_device(monkeypatch)
    pulse = module._pulse_duration_s
    assert pulse(12) == pytest.approx(0.30)
    assert pulse(5, measured_rate=10) == pytest.approx(0.30)
    assert pulse(5, measured_rate=20) == pytest.approx(0.175)
    assert pulse(-5, measured_rate=20) == pytest.approx(0.175)
    assert pulse(5, measured_rate=50) == pytest.approx(0.12)


def test_rotate_timeout_s_stays_under_llm_budget(monkeypatch):
    module, _ = _load_device(monkeypatch)
    assert module._rotate_timeout_s(60) == 22.0
    assert module._rotate_timeout_s(360) == 75.0
    assert all(module._rotate_timeout_s(a) <= 75.0 for a in (0.5, 60, 180, 360))  # 封顶 75s


class FakeHighCtrl:
    """双模植物：plant="ideal"（默认）逐帧 yaw += z*gain，与原行为完全一致；
    plant="realistic" 带 stiction 死区（|z|<0.4 → 目标速度归零）与一阶速度迟滞
    （目标 45*z °/s，τ=0.15s，dt=0.02s），yaw 按内部速度 wz_dps 推进。"""
    def __init__(self, mode=2, yaw0=0.0, gain_deg_per_tick=3.0, plant="ideal"):
        if plant not in ("ideal", "realistic"):
            raise ValueError(f"unknown plant: {plant!r}")
        self.plant = plant
        self.mode, self.yaw, self.gain = mode, yaw0, gain_deg_per_tick
        self.cmds = []          # (x, y, z) 每次 publish_cmd 记录
        self.wz_dps = 0.0       # realistic 内部速度状态（°/s）；ideal 恒 0
    def get_mode(self): return self.mode
    def get_imu_data(self):
        import math
        half = math.radians(self.yaw) / 2
        return types.SimpleNamespace(
            ori=[0.0, 0.0, math.sin(half), math.cos(half)],
            # ideal：wz_dps 恒 0 → 仍报 0.0；realistic：真速度的 rad/s
            angular_vel=[0.0, 0.0, math.radians(self.wz_dps)],
            linear_acc=[0.0, 0.0, 9.8])
    def publish_cmd(self, x, y, z, action_cmd, index=0):
        self.cmds.append((x, y, z))
        if self.plant == "realistic":
            target = 0.0 if abs(z) < 0.4 else 45.0 * z      # stiction 死区
            self.wz_dps += (target - self.wz_dps) * (0.02 / 0.15)  # τ=0.15s
            self.yaw += self.wz_dps * 0.02
        else:
            self.yaw += z * self.gain          # z 即 vyaw


def _rotate_plugin(module, monkeypatch, fake):
    """装配：绕开 highcontrol_py 懒加载 + 建 LocoPlugin（同上装配法）。"""
    monkeypatch.setattr(module, '_get_control_cmd', lambda name: name)
    return module.LocoPlugin(
        {}, 'robot', types.SimpleNamespace(add_node=lambda n: None), fake)


def wait_rotate(plugin, timeout_s=120):
    """轮询 status 至 idle，返回 last_result（ACP 异步后行为测试的取结果方式）。"""
    deadline = time.monotonic() + timeout_s
    while True:
        status = plugin.dispatch('status', {'_tool_name': 'loco_destination'})
        if status['state'] == 'idle':
            return status['last_result']
        if time.monotonic() >= deadline:
            raise AssertionError(
                f"rotate still {status['state']} after {timeout_s}s")
        time.sleep(0.05)


# ── 双模植物真实性钉死（ideal/realistic） ─────────────────────────────────────

def test_realistic_plant_has_stiction_deadband(monkeypatch):
    # plant="realistic"：stiction 死区 |z| < 0.4 —— 连发 z=0.3（死区内）100 帧，
    # 目标速度恒 0 → wz 与 yaw 一帧都不得动；0.39 仍在死区；恰在 0.4 边界必须开动。
    fake = FakeHighCtrl(yaw0=10.0, plant="realistic")
    for _ in range(100):
        fake.publish_cmd(0.0, 0.0, 0.3, "test")
    assert fake.yaw == 10.0                      # 死区内：yaw 精确不变
    assert fake.wz_dps == 0.0                    # 内部速度状态保持静止
    assert fake.get_imu_data().angular_vel[2] == 0.0
    assert len(fake.cmds) == 100                 # cmds 仍逐帧全量记录

    fake.publish_cmd(0.0, 0.0, 0.39, "test")
    assert fake.yaw == 10.0                      # 0.39 < 0.4 仍在死区

    fake.publish_cmd(0.0, 0.0, 0.4, "test")      # |z| < 0.4 不成立 → 必须开动
    assert fake.yaw > 10.0


def test_realistic_plant_velocity_lags(monkeypatch):
    # plant="realistic"：首帧 z=1.0 → 目标 45°/s，一阶迟滞（τ=0.15s, dt=0.02s）
    # 单帧只爬到 45*(0.02/0.15)；随后停令（只发 z=0 帧）→ 惯性使 yaw 继续增长，
    # 每帧增量按 (1 - 0.02/0.15) 单调衰减趋零，而非 ideal 停令后增量即刻归 0。
    import math
    fake = FakeHighCtrl(gain_deg_per_tick=3.0, plant="realistic")
    fake.publish_cmd(0.0, 0.0, 1.0, "test")
    assert fake.wz_dps == pytest.approx(45.0 * (0.02 / 0.15))  # 首帧仅爬升一截
    assert fake.wz_dps < 45.0                    # 迟滞：单帧到不了目标速度
    assert fake.yaw == pytest.approx(fake.wz_dps * 0.02)
    # Task 2 的 settle 逻辑读 angular_vel[2]：必须是真 wz 的 rad/s（非 0、非 °/s）
    wz_rad = fake.get_imu_data().angular_vel[2]
    assert wz_rad == pytest.approx(math.radians(fake.wz_dps))
    assert wz_rad > 0.0

    increments = [fake.yaw]                      # 首帧位移（yaw0=0.0）
    for _ in range(8):                           # 静默：此后只发零速帧
        prev = fake.yaw
        fake.publish_cmd(0.0, 0.0, 0.0, "test")
        assert fake.yaw > prev                   # 惯性：停令后仍在继续转
        increments.append(fake.yaw - prev)
    assert all(a > b for a, b in zip(increments, increments[1:]))  # 单调衰减
    decay = 1.0 - 0.02 / 0.15
    for prev_step, step in zip(increments, increments[1:]):
        assert step == pytest.approx(prev_step * decay)  # 精确一阶衰减
    assert increments[-1] > 0                    # 尚未停死 —— 迟滞而非瞬停


def test_ideal_plant_unchanged(monkeypatch):
    # plant="ideal"（默认值）：逐帧精确 yaw += z*gain —— 无死区、无迟滞、无速度
    # 状态，钉住既有 21 个测试赖以成立的旧行为；angular_vel[2] 恒 0.0。
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)           # 默认即 ideal
    explicit = FakeHighCtrl(gain_deg_per_tick=3.0, plant="ideal")
    expected = 0.0
    for z in (1.0, 0.5, -2.0, 0.0, 0.3):        # 含死区内的 0.3：ideal 不吃死区
        fake.publish_cmd(0.0, 0.0, z, "test")
        explicit.publish_cmd(0.0, 0.0, z, "test")
        expected += z * 3.0
        assert fake.yaw == expected              # 逐帧精确相等（非近似）
        assert explicit.yaw == expected          # 显式 plant="ideal" 同轨
    assert fake.wz_dps == 0.0 and explicit.wz_dps == 0.0  # ideal 无速度状态
    assert fake.get_imu_data().angular_vel[2] == 0.0      # ideal 恒报 0.0
    assert len(fake.cmds) == 5                   # cmds 全量记录不变


def test_loco_destination_tool_schema_contract(monkeypatch):
    module, _ = _load_device(monkeypatch)
    plugin = module.LocoPlugin({}, 'robot', types.SimpleNamespace(add_node=lambda n: None), object())
    tool = [t for t in plugin.get_tools() if t['name'] == 'loco_destination'][0]
    assert tool['type'] == 'actuator' and tool['multiInstance'] is False
    props = tool['inputSchema']['properties']
    assert set(props['action']['enum']) == {'rotate', 'stop_move', 'status'}
    assert props['angle_deg']['minimum'] == -360 and props['angle_deg']['maximum'] == 360
    assert tool['inputSchema']['x-hooks'] == {'on_interrupt_motion': {'action': 'stop_move'}}
    assert tool['inputSchema']['x-completion'] == {'actions': ['rotate'], 'timeout': 90}
    assert set(tool['inputSchema']['x-action-params']) == {'rotate', 'stop_move', 'status'}
    assert tool['inputSchema']['x-action-params']['rotate']['params'] == ['angle_deg']


def test_rotate_dispatch_returns_immediately_with_action_id(monkeypatch):
    # ACP 异步契约：dispatch('rotate') 立即返回 running + action_id + 预算，
    # 不得阻塞到动作结束；随后 wait_rotate 取终态。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    started_at = time.monotonic()
    started = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    elapsed = time.monotonic() - started_at
    assert started['state'] == 'running'
    assert started['action_id'].startswith('bumi_loco_destination_')
    assert started['expected_duration_s'] == module._rotate_timeout_s(60)
    assert started['requested_angle_deg'] == 60
    assert elapsed < 0.5                       # 返回远早于动作完成（~0.6s+）
    status = plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert status['state'] == 'running'        # 返回瞬间动作仍在进行
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2.0


def test_acp_terminal_state_fires_exactly_once(monkeypatch):
    # 每个 action 恰一次 ACP 终态回调：done → completed；stop → cancelled。
    # payload 与 last_result 同一 result dict（wait_rotate 返回即回调已发）。
    module, _ = _load_device(monkeypatch)
    calls = []
    monkeypatch.setattr(
        module, '_acp_notify',
        lambda action_id, status, result, tool: calls.append(
            (action_id, status, result, tool)))

    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    started = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert len(calls) == 1                     # done 路径恰 1 次
    action_id, status, payload, tool = calls[0]
    assert status == 'completed'
    assert action_id == started['action_id']
    assert tool == 'loco_destination'
    assert payload is result                    # ACP payload 与 last_result 同一 dict

    calls.clear()
    fake2 = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin2 = _rotate_plugin(module, monkeypatch, fake2)
    started2 = plugin2.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})
    stopped = plugin2.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    assert stopped == {'state': 'stopped'}
    result2 = wait_rotate(plugin2)
    assert result2['state'] == 'stopped'
    assert len(calls) == 1                     # stop 路径恰 1 次
    assert calls[0][1] == 'cancelled'
    assert calls[0][0] == started2['action_id']
    assert calls[0][2] is result2


def test_sigterm_stop_completes_pending_acp(monkeypatch):
    # SIGTERM 路径（bundle stop_all → plugin.stop()）：进行中的 rotate 以
    # cancelled 收尾（worker 自己完成回调），stop() 返回后状态回 idle。
    module, _ = _load_device(monkeypatch)
    calls = []
    monkeypatch.setattr(
        module, '_acp_notify',
        lambda action_id, status, result, tool: calls.append(
            (action_id, status, result, tool)))
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    started = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})
    assert started['state'] == 'running'
    deadline = time.monotonic() + 2.0          # 确认真在转（已有转速帧）
    while time.monotonic() < deadline and not any(c[2] != 0 for c in fake.cmds):
        time.sleep(0.01)
    assert any(c[2] != 0 for c in fake.cmds)

    plugin.stop()                              # SIGTERM/redeploy 收尾路径
    assert len(calls) == 1
    assert calls[0][1] == 'cancelled'
    assert calls[0][0] == started['action_id']
    status = plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert status['state'] == 'idle'
    assert status['last_result']['state'] == 'stopped'
    assert fake.cmds[-1] == (0, 0, 0)           # 退出必零速


def test_sigterm_during_dispatch_acceptance_still_cancels(monkeypatch):
    # I1 回归：stop() 落在 dispatch 受理窗口（preflight 阻塞、worker 尚未
    # 发布）时不得直接返回——必须等 worker 出现并 join，返回前回调已发。
    module, _ = _load_device(monkeypatch)
    calls = []
    monkeypatch.setattr(
        module, '_acp_notify',
        lambda action_id, status, result, tool: calls.append(
            (action_id, status, result, tool)))
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    entered, release = threading.Event(), threading.Event()

    def get_mode():
        entered.set()
        assert release.wait(5)
        return 2

    monkeypatch.setattr(fake, 'get_mode', get_mode)
    box = {}

    def _dispatch():
        box['result'] = plugin.dispatch(
            'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})

    dispatcher = threading.Thread(target=_dispatch, daemon=True)
    dispatcher.start()
    assert entered.wait(2)                     # dispatch 卡在 preflight（受理窗口内）

    stopper = threading.Thread(target=plugin.stop, daemon=True)
    stopper.start()
    deadline = time.monotonic() + 2.0          # 等 stop() 走过 _stop_move
    while time.monotonic() < deadline and not plugin._rotate_stop_event.is_set():
        time.sleep(0.01)
    assert plugin._rotate_stop_event.is_set()  # stop 已置位停止信号
    stopper.join(timeout=0.5)
    early_return = not stopper.is_alive()      # 受理窗口内就返回 = 漏 join
    assert dispatcher.is_alive()               # dispatch 仍在受理中

    release.set()
    dispatcher.join(5)
    stopper.join(5)
    assert not dispatcher.is_alive() and not stopper.is_alive()
    assert not early_return                     # I1：stop 必须覆盖受理窗口
    assert box['result']['state'] == 'running'
    assert len(calls) == 1                      # 恰一次 ACP 终态
    assert calls[0][1] == 'cancelled'
    status = plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert status['state'] == 'idle'
    assert status['last_result']['state'] == 'stopped'


def test_rotate_converges_within_tolerance_and_brakes(monkeypatch):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(plugin)
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
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 20})
    result = wait_rotate(plugin)
    assert result['state'] == 'done' and abs(result['final_error_deg']) <= 2.0
    assert result['turned_deg'] > 0                      # 逆时针 +20，而非 -340


def test_rotate_full_turn_actually_turns(monkeypatch):
    # 整圈：360° 拆成 ≤150° 子段链式执行，实际转过 ≈360（而非 wrap 到 0 不动）
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 360})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert 354 <= result['turned_deg'] <= 366
    assert abs(result['final_error_deg']) <= 2.0         # 整请求仍守 ±2° 容差
    assert 354 <= fake.yaw <= 366                        # 植物 yaw 真的推进了 ≈360，而非 0


def test_rotate_large_angle_turns_requested_direction_not_shortcut(monkeypatch):
    # +200 必须向请求方向转过 ≈200（两段 150+50），而非 wrap 最短路转 -160
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 200})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert 194 <= result['turned_deg'] <= 206
    assert result['turned_deg'] > 0                      # 正向，绝不 ≈ −160
    assert abs(result['final_error_deg']) <= 2.0         # 整请求仍守 ±2° 容差


# ── 失败路径：超时 / 打断 / 模式拒绝 / 与 loco move 互斥 ─────────────────────

def test_rotate_timeout_stops_and_reports_turned_degrees(monkeypatch):
    # 植物 gain=0（永远不动）+ 超时预算压到 0.05s → 必然走 timeout 支：
    # 报告实际转角（≈0）与末帧零速，而不是挂着或带着残速退出。
    module, _ = _load_device(monkeypatch)
    calls = []
    monkeypatch.setattr(module, '_acp_notify', lambda *args: calls.append(args))
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda a: 0.05)
    fake = FakeHighCtrl(gain_deg_per_tick=0.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(plugin)
    assert result['state'] == 'timeout'
    assert len(calls) == 1
    assert calls[0][1] == 'error'
    assert calls[0][2] is result
    assert abs(result['turned_deg']) < 1e-6             # 植物没动，实际转角为 0
    assert 'final_error_deg' in result                  # timeout 契约含误差报告
    assert abs(result['final_error_deg'] - 60.0) < 1e-6  # 仍差 60°，如实报告
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


def test_rotate_timeout_on_full_turn_reports_full_shortfall(monkeypatch):
    # final_error_deg（I1）必须报“请求还欠多少角”：360° 请求超时且完全没转时，
    # wrap180(target−end) 会把 360° 折叠成 0.0（谎报已对准）；修复后如实报 ≈360。
    module, _ = _load_device(monkeypatch)
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda a: 0.05)
    fake = FakeHighCtrl(gain_deg_per_tick=0.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 360})
    result = wait_rotate(plugin)
    assert result['state'] == 'timeout'
    assert abs(result['turned_deg']) < 1e-6              # 植物没动
    assert abs(result['final_error_deg'] - 360.0) < 1e-6  # 欠整整一圈，如实报 360（而非 0）
    assert fake.cmds[-1] == (0, 0, 0)                     # 退出必零速


@pytest.mark.parametrize('during_motion', [False, True])
@pytest.mark.parametrize('field, value', [
    ('ori', [0, 0, float('nan'), 1]),
    ('ori', [0, 0, float('inf'), 1]),
    ('ori', [0, 0, 0, 0]),
    ('angular_vel', [0, 0, float('nan')]),
    ('angular_vel', [0, 0, float('inf')]),
])
def test_invalid_imu_aborts_rotate(monkeypatch, during_motion, field, value):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    calls = []
    monkeypatch.setattr(module, '_acp_notify', lambda *args: calls.append(args))
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda angle: 0.15)
    read = fake.get_imu_data

    def get_imu():
        imu = read()
        if not during_motion or any(z != 0 for x, y, z in fake.cmds):
            setattr(imu, field, value)
        return imu

    monkeypatch.setattr(fake, 'get_imu_data', get_imu)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(plugin, timeout_s=2)
    assert result['state'] == 'error'
    assert 'IMU' in result['error']
    assert sum(z != 0 for x, y, z in fake.cmds) == int(during_motion)
    if during_motion:
        assert fake.cmds[-1] == (0, 0, 0)
    assert len(calls) == 1 and calls[0][1] == 'error'
    assert calls[0][2] is result


@pytest.mark.parametrize('mode', [1, 26, None])
def test_mode_loss_during_rotate_brakes_and_reports_error(monkeypatch, mode):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    calls = []
    monkeypatch.setattr(module, '_acp_notify', lambda *args: calls.append(args))
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda angle: 0.15)

    def get_mode():
        if any(z != 0 for x, y, z in fake.cmds):
            if mode is None:
                raise RuntimeError('mode feedback unavailable')
            return mode
        return 2

    monkeypatch.setattr(fake, 'get_mode', get_mode)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(plugin, timeout_s=2)
    assert result['state'] == 'error'
    assert sum(z != 0 for x, y, z in fake.cmds) == 1
    assert fake.cmds[-1] == (0, 0, 0)
    assert len(calls) == 1 and calls[0][1] == 'error'
    assert calls[0][2] is result


@pytest.mark.parametrize('failure', ['mode_check', 'thread_start'])
def test_rotate_startup_failure_releases_busy_state(monkeypatch, failure):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl()
    plugin = _rotate_plugin(module, monkeypatch, fake)
    calls = []
    monkeypatch.setattr(module, '_acp_notify', lambda *args: calls.append(args))

    def fail(*args):
        raise RuntimeError('startup unavailable')

    with monkeypatch.context() as startup:
        if failure == 'mode_check':
            startup.setattr(fake, 'get_mode', fail)
        else:
            startup.setattr(threading.Thread, 'start', fail)
        result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert result['state'] == 'error'
    assert 'startup unavailable' in result['error']
    assert 'action_id' not in result
    assert plugin._rotate_status() == {'state': 'idle', 'last_result': result}
    assert calls == []
    assert fake.cmds == []
    restarted = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 0})
    assert restarted['state'] == 'running'
    assert wait_rotate(plugin, timeout_s=2)['state'] == 'done'
    assert len(calls) == 1 and calls[0][1] == 'completed'


@pytest.mark.parametrize('angle, pending_send', [(60, 1), (10, 1), (10, 2)])
def test_rotate_cannot_publish_motion_after_stop_returns(monkeypatch, angle, pending_send):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    entered, release = threading.Event(), threading.Event()
    publish = plugin._publish_cmd
    nonzero_sends = 0

    def pending_publish(x, y, z, action_cmd, index=0, **kwargs):
        nonlocal nonzero_sends
        if z != 0:
            nonzero_sends += 1
            if nonzero_sends == pending_send:
                entered.set()
                assert release.wait(2)
        return publish(x, y, z, action_cmd, index, **kwargs)

    monkeypatch.setattr(plugin, '_publish_cmd', pending_publish)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': angle})
    try:
        assert entered.wait(2)
        stopped = plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
        assert stopped == {'state': 'stopped'}
        after_stop = len(fake.cmds)
        assert fake.cmds[-1] == (0, 0, 0)
    finally:
        release.set()
        result = wait_rotate(plugin, timeout_s=3)
    assert result['state'] == 'stopped'
    assert all(cmd == (0, 0, 0) for cmd in fake.cmds[after_stop:])


def test_stop_move_interrupts_running_rotate(monkeypatch):
    # rotate 在 bumi_rotate 工作线程里跑（150° / 小 gain 转不完），主线程 30ms 后
    # 下发 stop_move：rotate 必须经 _rotate_stop_event 以 stopped 收尾，且末帧零速。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    started = plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})
    assert started['state'] == 'running'                # dispatch 立即返回，不阻塞
    time.sleep(0.03)
    stopped = plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    result = wait_rotate(plugin)
    assert stopped == {'state': 'stopped'}
    assert result['state'] == 'stopped'
    assert 'final_error_deg' in result                  # stopped 契约含误差报告
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


def test_rotate_refuses_non_walking_and_protection_modes(monkeypatch):
    # 模式拒绝与 _do_move 逐字一致：26 → protection；非 2 → workmode + switch_mode 指引。
    # 两种情况都必须在发出任何指令之前就拒绝（fake.cmds 为空）。
    module, _ = _load_device(monkeypatch)

    fake_protection = FakeHighCtrl(mode=26)
    plugin_protection = _rotate_plugin(module, monkeypatch, fake_protection)
    result = plugin_protection.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert result['state'] == 'error'
    assert 'protection' in result['error']
    assert fake_protection.cmds == []                   # 未发任何指令

    fake_ready = FakeHighCtrl(mode=1)
    plugin_ready = _rotate_plugin(module, monkeypatch, fake_ready)
    result = plugin_ready.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert result['state'] == 'error'
    assert 'workmode' in result['error'] and 'switch_mode' in result['error']
    assert fake_ready.cmds == []                        # 未发任何指令


def test_rotate_stops_a_running_loco_move(monkeypatch):
    # loco move 线程运行中发起 rotate：互斥前导必须先停 move 线程再转角，
    # 全程 fake.cmds 只有一路指令流（rotate 首个转速帧之后不得再混入 move 的 x≠0 帧）。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    move = plugin.dispatch('move', {'_tool_name': 'loco', 'vx': 0.2, 'duration': 3})
    assert move['state'] == 'moving'
    time.sleep(0.03)                                    # 确保 move 线程真的在发指令
    assert any(c == (0.2, 0.0, 0.0) for c in fake.cmds) # move 流已开

    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2.0
    # move 线程已被互斥前导 join 停掉
    assert plugin._move_thread is not None and not plugin._move_thread.is_alive()
    # 只有一路指令流：rotate 发出转速后，move 的 (0.2, 0, 0) 不得再出现
    rotate_started = False
    for cmd in fake.cmds:
        if cmd[2] != 0:
            rotate_started = True
        elif rotate_started:
            assert cmd[0] == 0, f"move 指令流混入 rotate 之后: {cmd}"
    assert rotate_started
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


def test_concurrent_move_is_refused_while_rotate_runs(monkeypatch):
    # busy-guard（T2-2）：rotate 运行中（跑在 bumi_rotate 线程，非 _move_thread）并发
    # 下发 loco move 必须被拒绝且错误点名冲突——否则 move 前导 set→join(None)→clear
    # 会擦掉 rotate 待读的 stop 信号，move 帧还会与 rotate 帧交错。stop 不受限。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    started = plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})
    assert started['state'] == 'running'
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not any(c[2] != 0 for c in fake.cmds):
        time.sleep(0.01)
    assert any(c[2] != 0 for c in fake.cmds)         # rotate 已在发转速帧

    refused = plugin.dispatch('move', {'_tool_name': 'loco', 'vx': 0.2, 'duration': 3})
    assert refused['state'] == 'error'
    assert 'rotate in progress' in refused['error']  # 错误点名 rotate 冲突
    assert plugin._move_thread is None               # 拒绝发生在起 move 线程之前
    assert not any(c[0] != 0 or c[1] != 0 for c in fake.cmds)  # 无 move 帧混入
    status = plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert status['state'] == 'running'              # 拒 move 不得动 rotate 状态

    stopped = plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    result = wait_rotate(plugin)
    assert stopped == {'state': 'stopped'}           # busy-guard 不拦 stop_move
    assert result['state'] == 'stopped'
    assert fake.cmds[-1] == (0, 0, 0)                # 退出必零速


def test_concurrent_rotate_is_refused_while_rotate_runs(monkeypatch):
    # busy-guard（T4-3）：rotate 运行中重入第二个 rotate 必须被拒绝——否则第二个
    # 前导的 clear() 会擦掉第一个 rotate 待读的 stop 信号（显式 stop 被静默吞掉）。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    started = plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})
    assert started['state'] == 'running'
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not any(c[2] != 0 for c in fake.cmds):
        time.sleep(0.01)
    assert any(c[2] != 0 for c in fake.cmds)         # rotate 已在发转速帧

    refused = plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert refused['state'] == 'error'
    assert 'rotate in progress' in refused['error']  # 错误点名 rotate 冲突
    status = plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert status['state'] == 'running'              # 重入拒绝不得清掉 running

    stopped = plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    result = wait_rotate(plugin)
    assert stopped == {'state': 'stopped'}           # stop 信号未被重入擦掉
    assert result['state'] == 'stopped'
    assert fake.cmds[-1] == (0, 0, 0)                # 退出必零速


def test_stop_during_rotate_startup_is_preserved(monkeypatch):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    entered, release = threading.Event(), threading.Event()
    box = {}

    def get_mode():
        entered.set()
        assert release.wait(2)
        return 2

    monkeypatch.setattr(fake, 'get_mode', get_mode)
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda angle: 0.1)
    worker = threading.Thread(target=lambda: box.update(result=plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})), daemon=True)
    worker.start()
    try:
        assert entered.wait(2)
        assert plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'}) == {'state': 'stopped'}
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert box['result']['state'] == 'running'       # dispatch 立即返回 running
    result = wait_rotate(plugin)
    assert result['state'] == 'stopped'
    assert all(cmd == (0, 0, 0) for cmd in fake.cmds)


def test_move_startup_cannot_overlap_rotate(monkeypatch):
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    entered, release, rotating = threading.Event(), threading.Event(), threading.Event()
    box = {}

    def get_mode():
        if threading.current_thread().name == 'starting_move':
            entered.set()
            assert release.wait(2)
        return 2

    publish = fake.publish_cmd

    def publish_cmd(x, y, z, action_cmd, index=0):
        publish(x, y, z, action_cmd, index)
        if z != 0:
            rotating.set()

    monkeypatch.setattr(fake, 'get_mode', get_mode)
    monkeypatch.setattr(fake, 'publish_cmd', publish_cmd)
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda angle: 0.3)
    mover = threading.Thread(name='starting_move', target=lambda: box.update(move=plugin.dispatch(
        'move', {'_tool_name': 'loco', 'vx': 0.2, 'duration': 0.1})), daemon=True)
    rotator = threading.Thread(target=lambda: box.update(rotate=plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})), daemon=True)
    mover.start()
    try:
        assert entered.wait(2)
        rotator.start()
        rotating.wait(0.1)
    finally:
        release.set()
        mover.join(2)
        rotator.join(2)
    assert not mover.is_alive() and not rotator.is_alive()
    wait_rotate(plugin)                              # rotate 线程跑完（0.3s 超时收尾）
    plugin.dispatch('stop_move', {'_tool_name': 'loco'})
    first_rotate = next(i for i, cmd in enumerate(fake.cmds) if cmd[2] != 0)
    assert all(x == 0 and y == 0 for x, y, z in fake.cmds[first_rotate:])


def test_rotate_negative_angle_turns_opposite_direction(monkeypatch):
    # 负角与正角对称（copysign 对称性）：-60 → 实际转过 ≈ -60，仍在 ±2° 容差内收敛。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': -60})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert -66 <= result['turned_deg'] <= -54
    assert abs(result['final_error_deg']) <= 2.0
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


# ── I-A：posture/semantic/teaching 动作取消进行中的 rotate ───────────────────

def test_posture_action_cancels_inflight_rotate(monkeypatch):
    # I-A：realistic 植物上 rotate 转 360°（远未完成）时下发 lie_prone——
    # _prepare_workmode 必须先取消 rotate：worker 以 stopped 收尾、恰 1 次
    # ACP cancelled（对应该 action_id）、退出必零速；posture 路径照常继续
    # （STANDTOFALL 被接受 → running）；且 STANDTOFALL 边沿之后不得再出现
    # 任何 vyaw 转速帧——两路指令流不重叠。
    module, _ = _load_device(monkeypatch)
    acp_calls = []
    monkeypatch.setattr(
        module, '_acp_notify',
        lambda action_id, status, result, tool: acp_calls.append(
            (action_id, status, result, tool)))
    fake = FakeHighCtrl(plant="realistic")
    plugin = _rotate_plugin(module, monkeypatch, fake)

    frames = []                               # (x, y, z, action_cmd) 全量指令流
    publish = fake.publish_cmd

    def publish_cmd(x, y, z, action_cmd, index=0):
        publish(x, y, z, action_cmd, index)
        frames.append((x, y, z, action_cmd))
        if action_cmd == 'STANDTOFALL':
            fake.mode = 30                    # 反馈：卧倒动作被接受，posture 继续

    monkeypatch.setattr(fake, 'publish_cmd', publish_cmd)

    started = plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 360})
    assert started['state'] == 'running'
    deadline = time.monotonic() + 3.0         # 确认真在转（已有转速帧）
    while time.monotonic() < deadline and not any(c[2] != 0 for c in fake.cmds):
        time.sleep(0.01)
    assert any(c[2] != 0 for c in fake.cmds)

    posture = plugin.dispatch(
        'lie_prone', {'_tool_name': 'stand_up_lie_prone'})
    assert posture['state'] == 'running'      # posture 路径不受阻，照常继续
    assert posture['command_sent'] is True

    result = wait_rotate(plugin)
    assert result['state'] == 'stopped'       # I-A：被 posture 取消，而非转完
    assert len(acp_calls) == 1                # 恰一次 ACP 终态
    assert acp_calls[0][1] == 'cancelled'
    assert acp_calls[0][0] == started['action_id']
    assert fake.cmds[-1] == (0, 0, 0)         # 退出必零速
    first_edge = next(i for i, f in enumerate(frames) if f[3] == 'STANDTOFALL')
    last_stream = max(i for i, f in enumerate(frames) if f[2] != 0)
    assert last_stream < first_edge           # 边沿后无转速帧：两路流不重叠


def test_posture_cancel_sticks_during_rotate_dispatch_acceptance(monkeypatch):
    # I-A 跨步(straddle)回归：取消必须在 _rotate_lock 内置位——普通 set 可
    # 落在 dispatch 受理窗口的 busy-check→clear 之间，被 reservation 的
    # clear 擦掉。构造：dispatch 卡在 preflight（clear 已过、worker 未发布），
    # 此时取消必须阻塞在锁上且尚未置位；锁释放后置位不得再被擦除——受理
    # 完成的 rotate 帧 1 即见取消，一帧未发即以 stopped/cancelled 收尾。
    module, _ = _load_device(monkeypatch)
    acp_calls = []
    monkeypatch.setattr(
        module, '_acp_notify',
        lambda action_id, status, result, tool: acp_calls.append(
            (action_id, status, result, tool)))
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    entered, release = threading.Event(), threading.Event()

    def get_mode():
        entered.set()
        assert release.wait(5)
        return 2

    monkeypatch.setattr(fake, 'get_mode', get_mode)
    box = {}
    dispatcher = threading.Thread(
        target=lambda: box.update(result=plugin.dispatch(
            'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})),
        daemon=True)
    dispatcher.start()
    assert entered.wait(2)                    # 受理窗口内：clear 已过、worker 未发布

    cancel_box = {}
    canceler = threading.Thread(
        target=lambda: cancel_box.update(
            prepared=plugin._prepare_workmode(2, 'lie_prone')),
        daemon=True)
    with plugin._rotate_lock:                 # 模拟 reservation 持锁期间到达的取消
        canceler.start()
        canceler.join(0.3)
        assert canceler.is_alive()            # 取消阻塞在 _rotate_lock 上
        assert not plugin._rotate_stop_event.is_set()  # 锁内不得先行置位
    release.set()
    canceler.join(5)
    assert not canceler.is_alive()
    assert cancel_box['prepared']['state'] == 'completed'
    assert plugin._rotate_stop_event.is_set()  # 置位在 clear 之后，未被擦除
    dispatcher.join(5)
    assert not dispatcher.is_alive()
    assert box['result']['state'] == 'running'  # 受理照常完成
    result = wait_rotate(plugin)
    assert result['state'] == 'stopped'       # 受理后的 worker 帧 1 即见取消
    assert len(acp_calls) == 1
    assert acp_calls[0][1] == 'cancelled'
    assert all(cmd == (0, 0, 0) for cmd in fake.cmds)  # 一帧运动都未发


# ── realistic 植物症状回归：stiction 小角度 / 极限环 / 整圈契约 ───────────────

def _vyaw_sign_flips(cmds):
    """vyaw 非零命令序列的变号次数：零速滑行帧排除、不打断变号判定。"""
    flips, last = 0, 0.0
    for _, _, z in cmds:
        if z == 0:
            continue
        if last and (z > 0) != (last > 0):
            flips += 1
        last = z
    return flips


def _creep_clock(module, monkeypatch):
    clock = types.SimpleNamespace(now=0.0)
    monkeypatch.setattr(module, 'time', types.SimpleNamespace(
        monotonic=lambda: clock.now,
        sleep=lambda seconds: setattr(clock, 'now', clock.now + seconds)))
    return clock


def test_creep_waits_out_transient_target_crossing(monkeypatch):
    module, _ = _load_device(monkeypatch)
    clock = _creep_clock(module, monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    first_pulse = []
    read, publish = fake.get_imu_data, fake.publish_cmd

    def publish_cmd(x, y, z, action_cmd, index=0):
        publish(x, y, z, action_cmd, index)
        if z != 0 and not first_pulse:
            first_pulse.append(clock.now)

    def get_imu():
        if first_pulse:
            elapsed = clock.now - first_pulse[0]
            fake.yaw = 7.7 if 0.04 <= elapsed < 0.12 else 8.5
        return read()

    monkeypatch.setattr(fake, 'publish_cmd', publish_cmd)
    monkeypatch.setattr(fake, 'get_imu_data', get_imu)
    result = plugin._do_rotate_impl({'angle_deg': 10})
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2
    assert sum(z != 0 for x, y, z in fake.cmds) == 1
    assert clock.now - first_pulse[0] >= 0.3


@pytest.mark.parametrize('angle, yaw0', [(10, 179), (-10, -179)])
def test_creep_overcomes_step_startup_delay_without_tiny_retries(monkeypatch, angle, yaw0):
    module, _ = _load_device(monkeypatch)
    clock = _creep_clock(module, monkeypatch)
    fake = FakeHighCtrl(yaw0=yaw0, gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda angle: 3)
    pulses = []
    active = []
    publish = fake.publish_cmd

    def publish_cmd(x, y, z, action_cmd, index=0):
        publish(x, y, z, action_cmd, index)
        if z != 0 and not active:
            active.extend((clock.now, 1 if z > 0 else -1))
        elif z == 0 and active:
            duration = clock.now - active[0]
            # A gait needs 100 ms before a burst produces effective yaw.
            fake.yaw += active[1] * max(0, duration - 0.1) * 100
            pulses.append(duration)
            active.clear()

    monkeypatch.setattr(fake, 'publish_cmd', publish_cmd)
    result = plugin._do_rotate_impl({'angle_deg': angle})
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2 + 1e-9
    assert all(duration >= 0.12 - 1e-6 for duration in pulses)
    assert len(pulses) <= 4
    assert fake.cmds[-1] == (0, 0, 0)


def test_creep_stops_after_repeated_ineffective_pulses(monkeypatch):
    module, _ = _load_device(monkeypatch)
    clock = _creep_clock(module, monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin._do_rotate_impl({'angle_deg': 10})
    bursts = sum(z != 0 and (i == 0 or fake.cmds[i - 1][2] == 0)
                 for i, (x, y, z) in enumerate(fake.cmds))
    assert result['state'] == 'error'
    assert 'progress' in result['error']
    assert result['final_error_deg'] == pytest.approx(10)
    assert bursts == 3
    assert clock.now < 3
    assert fake.cmds[-1] == (0, 0, 0)


def test_small_angle_reaches_target_despite_stiction(monkeypatch):
    # 10° 在 realistic 植物上：旧 P 律给 |err|=10 → 0.3（<0.4 死区）→ 永不动、
    # 超时收场；三段式 creep 发 0.5 点动必须推过 stiction 并收敛到 ±2° 内。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(plant="realistic")
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 10})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2.0
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


def test_no_limit_cycle_on_20_and_90(monkeypatch):
    # 20°/90° 在 realistic 植物上：旧 P 律进 |err|<15 段给 0.3 死区卡住，叠加
    # 一阶迟滞惯性即真机的极限环抖动；三段式必须收敛且 vyaw 非零命令变号 ≤ 2。
    module, _ = _load_device(monkeypatch)
    for angle in (20, 90):
        fake = FakeHighCtrl(plant="realistic")
        plugin = _rotate_plugin(module, monkeypatch, fake)
        plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': angle})
        result = wait_rotate(plugin)
        assert result['state'] == 'done', f"{angle}°: {result}"
        assert abs(result['final_error_deg']) <= 2.0, f"{angle}°: {result}"
        flips = _vyaw_sign_flips(fake.cmds)
        assert flips <= 2, f"{angle}° 极限环：vyaw 非零命令变号 {flips} 次"


def test_realistic_plant_full_turn_contract(monkeypatch):
    # 360° 整圈在 realistic 植物上沿用既有契约：done、实际转过 ∈ [354,366]、
    # 整请求残差 ±2°（stiction/迟滞下旧 P 律在首段即卡死 → 超时）。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(plant="realistic")
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 360})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert 354 <= result['turned_deg'] <= 366
    assert abs(result['final_error_deg']) <= 2.0
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


def test_large_angle_subtargets_advance_in_cruise_band(monkeypatch):
    # 最小版插补：非末段子目标在巡航带内（±20°）即推进，中途不再 brake/creep
    # 爬行——指令序列必须是「巡航 0.8 段打头 + 刹停/爬行尾巴」的前缀形状。
    # 旧推进阈值（±2°）会把中间段的 creep 脉冲（0.5）和零速打进序列中部，
    # 0.8 不再是前缀 → 此断言必红。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(plant="realistic")
    plugin = _rotate_plugin(module, monkeypatch, fake)
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 360})
    result = wait_rotate(plugin)
    assert result['state'] == 'done'
    assert abs(result['final_error_deg']) <= 2.0        # 末段一次爬行收敛不变

    cruise_done = False
    for (x, y, z) in fake.cmds:
        if not cruise_done:
            if abs(z) == module._ROTATE_VYAW_FULL:
                continue                                # 巡航段：连续 0.8
            cruise_done = True                          # 首个非巡航令开始尾巴
        else:
            assert abs(z) != module._ROTATE_VYAW_FULL, (
                "cruise command reappeared after brake/creep began — "
                "intermediate sub-target crawled instead of advancing")


# ── status 真值 / driver.yaml 注册 ────────────────────────────────────────────

def test_status_reflects_last_rotate_result(monkeypatch):
    # status 必须给真值：idle 时带前次 rotate 的完整 result（缺失 last_result 即失败），
    # rotate 进行中时报 running（dispatch 返回的 action_id 流确认动作已启动）；
    # stopped 退出同样要记录进 last_result。
    module, _ = _load_device(monkeypatch)

    idle_plugin = _rotate_plugin(module, monkeypatch, FakeHighCtrl(gain_deg_per_tick=3.0))
    assert idle_plugin.dispatch(
        'status', {'_tool_name': 'loco_destination'}
    ) == {"state": "idle", "last_result": None}         # 初始：无前次结果

    idle_plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    result = wait_rotate(idle_plugin)
    assert result['state'] == 'done'
    assert idle_plugin.dispatch(
        'status', {'_tool_name': 'loco_destination'}
    ) == {"state": "idle", "last_result": result}       # 与前次 rotate 完全一致

    # dispatch 立即返回 action_id（150° / gain=1 转不完），进行中 status 报 running。
    run_plugin = _rotate_plugin(module, monkeypatch, FakeHighCtrl(gain_deg_per_tick=1.0))
    started = run_plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})
    assert started['state'] == 'running'                # action_id 流：立即返回 running
    status = run_plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert status['state'] == 'running'                 # 进行中必须报 running
    assert status['last_result'] is None                # 尚无完成记录

    run_plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    stopped = wait_rotate(run_plugin)
    assert stopped['state'] == 'stopped'
    final = run_plugin.dispatch('status', {'_tool_name': 'loco_destination'})
    assert final['state'] == 'idle'
    assert final['last_result'] is not None             # stopped 退出也记录 last_result
    assert final['last_result']['state'] == 'stopped'


def test_bundle_exposes_loco_destination_card(monkeypatch, tmp_path):
    # 按 test_direction_card 的 bundle 装配法（该文件不在本分支）：以 Task 1 的
    # _load_device 脚手架加载 device/main，BumiDeviceBundle 只启用 loco 插件。
    module, _ = _load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, 'device', module)  # main.py: from device import ...
    monkeypatch.setitem(sys.modules, 'rclpy.executors', types.ModuleType('rclpy.executors'))
    # 避免真实 common.logsafe.install() 在测试进程里替换 sys.stdout/stderr
    monkeypatch.setitem(sys.modules, 'common', types.ModuleType('common'))

    main_path = Path(__file__).resolve().parents[1] / 'main.py'
    spec = importlib.util.spec_from_file_location('bumi_main_loco_destination_test', main_path)
    main = importlib.util.module_from_spec(spec)
    # main.py 内联 `dict | None` 注解但无 __future__ annotations；本机 Python 3.9
    # 在类定义处会立刻求值并报 TypeError（机器人镜像是 3.10+）。按
    # tests/test_go2_remote_controller.py 的做法以 annotations 标志编译，运行时
    # 行为不变（注解仅延迟为字符串）。
    exec(compile(main_path.read_text(), str(main_path), 'exec',
                 flags=__future__.annotations.compiler_flag),
         main.__dict__)

    bundle = main.BumiDeviceBundle(
        {'plugins': {'loco': {'enabled': True}}},
        'robot', types.SimpleNamespace(add_node=lambda n: None),
        FakeHighCtrl(), media_ctrl=None)
    names = [t['name'] for t in bundle.get_all_tools()]
    assert 'loco_destination' in names                   # bundle 对外暴露该卡

    driver = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / 'driver.yaml').read_text())
    assert {'name': 'loco_destination', 'type': 'actuator'} in driver['cards']


def test_servo_sees_reserved_rotation_as_busy(monkeypatch):
    module, _ = _load_device(monkeypatch)
    plugin = module.LocoPlugin({}, 'robot', None, FakeHighCtrl())
    plugin._rotate_state['state'] = 'running'
    assert plugin.is_moving() is True


def test_rotate_waits_for_previous_move_handoff_and_releases_on_error(monkeypatch):
    module, _ = _load_device(monkeypatch)
    monkeypatch.setattr(module, '_get_default_cmd', lambda: 'walk')
    plugin = module.LocoPlugin({}, 'robot', None, FakeHighCtrl())

    class Servo:
        paused = False
        owned = False
        owners = []

        def pause_for_explicit_command(self, reason):
            self.paused = True
            return True

        def set_explicit_velocity_owner(self, owned):
            self.owned = owned
            self.owners.append(owned)

    servo = Servo()
    plugin.attach_servo(servo)
    previous_move_alive = [True]

    def finish_previous_move(timeout):
        servo.set_explicit_velocity_owner(False)
        previous_move_alive[0] = False

    plugin._move_thread = types.SimpleNamespace(
        is_alive=lambda: previous_move_alive[0], join=finish_previous_move)
    plugin._active_action_id = 'rotation'
    plugin._rotate_state['state'] = 'running'

    def read_imu():
        assert servo.paused and servo.owned
        raise RuntimeError('simulated IMU failure')

    monkeypatch.setattr(plugin, '_read_rotate_imu', read_imu)
    result = plugin._do_rotate_worker({'angle_deg': 30}, 'rotation')
    assert 'simulated IMU failure' in result['error']
    assert servo.owners == [False, True, False]
    assert servo.owned is False


def test_rotate_status_and_zero_do_not_pause_streaming_servo(monkeypatch):
    module, _ = _load_device(monkeypatch)
    plugin = module.LocoPlugin({}, 'robot', None, FakeHighCtrl())

    def refuse_pause(reason):
        raise AssertionError('read-only status and zero turn must not pause servo')

    plugin.attach_servo(types.SimpleNamespace(pause_for_explicit_command=refuse_pause))
    assert plugin.dispatch('status', {'_tool_name': 'loco_destination'})['state'] == 'idle'
    plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 0})
    assert wait_rotate(plugin)['state'] == 'done'


def test_rotation_refuses_if_previous_move_did_not_stop(monkeypatch):
    module, _ = _load_device(monkeypatch)
    monkeypatch.setattr(module, '_get_default_cmd', lambda: 'walk')
    controller = FakeHighCtrl()
    plugin = module.LocoPlugin({}, 'robot', None, controller)
    plugin._move_thread = types.SimpleNamespace(is_alive=lambda: True, join=lambda timeout: None)
    plugin._active_action_id = 'rotation'
    plugin._rotate_state['state'] = 'running'
    result = plugin._do_rotate_worker({'angle_deg': 30}, 'rotation')
    assert result['state'] == 'error'
    assert 'previous move' in result['error']
    assert plugin._move_stop_event.is_set()
    assert controller.cmds == []


def test_rotation_action_ids_stay_unique_with_frozen_wall_clock(monkeypatch):
    module, _ = _load_device(monkeypatch)
    monkeypatch.setattr(module.time, 'time_ns', lambda: 12345)
    plugin = module.LocoPlugin({}, 'robot', None, FakeHighCtrl())
    first = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 0})
    wait_rotate(plugin)
    second = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 0})
    wait_rotate(plugin)
    assert first['action_id'] != second['action_id']
