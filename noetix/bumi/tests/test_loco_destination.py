"""Angle math, control law, and closed-loop rotate for the Bumi loco_destination card."""

import __future__
import importlib.util
import sys
import threading
import time
import types
from pathlib import Path

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


def test_rotate_full_turn_actually_turns(monkeypatch):
    # 整圈：360° 拆成 ≤150° 子段链式执行，实际转过 ≈360（而非 wrap 到 0 不动）
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 360})
    assert result['state'] == 'done'
    assert 354 <= result['turned_deg'] <= 366
    assert abs(result['final_error_deg']) <= 2.0         # 整请求仍守 ±2° 容差
    assert 354 <= fake.yaw <= 366                        # 植物 yaw 真的推进了 ≈360，而非 0


def test_rotate_large_angle_turns_requested_direction_not_shortcut(monkeypatch):
    # +200 必须向请求方向转过 ≈200（两段 150+50），而非 wrap 最短路转 -160
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 200})
    assert result['state'] == 'done'
    assert 194 <= result['turned_deg'] <= 206
    assert result['turned_deg'] > 0                      # 正向，绝不 ≈ −160
    assert abs(result['final_error_deg']) <= 2.0         # 整请求仍守 ±2° 容差


# ── 失败路径：超时 / 打断 / 模式拒绝 / 与 loco move 互斥 ─────────────────────

def test_rotate_timeout_stops_and_reports_turned_degrees(monkeypatch):
    # 植物 gain=0（永远不动）+ 超时预算压到 0.05s → 必然走 timeout 支：
    # 报告实际转角（≈0）与末帧零速，而不是挂着或带着残速退出。
    module, _ = _load_device(monkeypatch)
    monkeypatch.setattr(module, '_rotate_timeout_s', lambda a: 0.05)
    fake = FakeHighCtrl(gain_deg_per_tick=0.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert result['state'] == 'timeout'
    assert abs(result['turned_deg']) < 1e-6             # 植物没动，实际转角为 0
    assert 'final_error_deg' in result                  # timeout 契约含误差报告
    assert abs(result['final_error_deg'] - 60.0) < 1e-6  # 仍差 60°，如实报告
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


def test_stop_move_interrupts_running_rotate(monkeypatch):
    # rotate 在工作线程里跑（150° / 小 gain 转不完），主线程 30ms 后下发 stop_move：
    # rotate 必须经 _move_stop_event 以 stopped 收尾，且末帧零速。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=1.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    box = {}

    def _run():
        box['result'] = plugin.dispatch(
            'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})

    worker = threading.Thread(target=_run, daemon=True)
    worker.start()
    time.sleep(0.03)
    stopped = plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    worker.join(timeout=5)
    assert not worker.is_alive()                        # rotate 确实被打断返回，而非挂死
    assert stopped == {'state': 'stopped'}
    result = box['result']
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

    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
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


def test_rotate_negative_angle_turns_opposite_direction(monkeypatch):
    # 负角与正角对称（copysign 对称性）：-60 → 实际转过 ≈ -60，仍在 ±2° 容差内收敛。
    module, _ = _load_device(monkeypatch)
    fake = FakeHighCtrl(gain_deg_per_tick=3.0)
    plugin = _rotate_plugin(module, monkeypatch, fake)
    result = plugin.dispatch('rotate', {'_tool_name': 'loco_destination', 'angle_deg': -60})
    assert result['state'] == 'done'
    assert -66 <= result['turned_deg'] <= -54
    assert abs(result['final_error_deg']) <= 2.0
    assert fake.cmds[-1] == (0, 0, 0)                   # 退出必零速


# ── status 真值 / driver.yaml 注册 ────────────────────────────────────────────

def test_status_reflects_last_rotate_result(monkeypatch):
    # status 必须给真值：idle 时带前次 rotate 的完整 result（缺失 last_result 即失败），
    # rotate 在另一线程进行中时报 running；stopped 退出同样要记录进 last_result。
    module, _ = _load_device(monkeypatch)

    idle_plugin = _rotate_plugin(module, monkeypatch, FakeHighCtrl(gain_deg_per_tick=3.0))
    assert idle_plugin.dispatch(
        'status', {'_tool_name': 'loco_destination'}
    ) == {"state": "idle", "last_result": None}         # 初始：无前次结果

    result = idle_plugin.dispatch(
        'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 60})
    assert result['state'] == 'done'
    assert idle_plugin.dispatch(
        'status', {'_tool_name': 'loco_destination'}
    ) == {"state": "idle", "last_result": result}       # 与前次 rotate 完全一致

    # 另一线程跑 150°（gain=1 转不完），主线程轮询到 running 再断言。
    run_plugin = _rotate_plugin(module, monkeypatch, FakeHighCtrl(gain_deg_per_tick=1.0))
    box = {}

    def _run():
        box['result'] = run_plugin.dispatch(
            'rotate', {'_tool_name': 'loco_destination', 'angle_deg': 150})

    worker = threading.Thread(target=_run, daemon=True)
    worker.start()
    status = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        status = run_plugin.dispatch('status', {'_tool_name': 'loco_destination'})
        if status['state'] == 'running':
            break
        time.sleep(0.01)
    assert status['state'] == 'running'                 # 进行中必须报 running
    assert status['last_result'] is None                # 尚无完成记录

    run_plugin.dispatch('stop_move', {'_tool_name': 'loco_destination'})
    worker.join(timeout=5)
    assert not worker.is_alive()
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
