#!/usr/bin/env python3
"""
x-humanoid/Q5/main.py — 星动纪元Q5轮式人形机器人 设备bundle统一入口。

读取config.yaml，按插件配置加载插件，聚合成MCP HTTP server对外暴露。
驱动启动时自动start所有插件，关闭时自动stop。

双Domain模式：
  - domain 211: 订阅/发布到Q5本体控制器
  - domain 42: 发布传感器数据给Agent Core

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml路径（默认同目录下）
"""

import json
import os
import re
import signal
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

import rclpy
import rclpy.executors
from rclpy.context import Context
from xbot_common_interfaces.srv import DynamicLaunch
from xbot_common_interfaces.action import SimpleActions
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


# ── Config ────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── Dual Domain ROS2 Init ────────────────────────────────────────────────────

class DualDomainROS2:
    """Manages two ROS2 contexts: hardware domain (Q5 default 211) and domain 42 (agent-core)."""

    def __init__(self, hardware_domain: int = 211):
        # Hardware domain: connect to Q5 body controller
        self.ctx_q5 = Context()
        rclpy.init(context=self.ctx_q5, domain_id=hardware_domain)
        self.executor_q5 = rclpy.executors.MultiThreadedExecutor(context=self.ctx_q5)

        # Domain 42: publish to agent-core
        self.ctx_core = Context()
        rclpy.init(context=self.ctx_core, domain_id=42)
        self.executor_core = rclpy.executors.MultiThreadedExecutor(context=self.ctx_core)

        # Backward-compat aliases
        self.ctx_hw = self.ctx_q5
        self.executor_hw = self.executor_q5

        self._spin_threads = []
        self._bundle = None  # set by Q5DeviceBundle

    def start_spin(self):
        """Start spinning both executors in background threads."""
        def _spin(executor, name):
            try:
                while rclpy.ok(context=executor.context):
                    executor.spin_once(timeout_sec=0.1)
            except Exception:
                pass
            print(f"[ros2] {name} spin exited")

        t1 = threading.Thread(target=_spin, args=(self.executor_q5, f"domain{self.executor_q5.context.get_domain_id()}"), daemon=True)
        t2 = threading.Thread(target=_spin, args=(self.executor_core, "domain42"), daemon=True)
        t1.start()
        t2.start()
        self._spin_threads = [t1, t2]

    def shutdown(self):
        self.executor_q5.shutdown()
        self.executor_core.shutdown()
        rclpy.shutdown(context=self.ctx_q5)
        rclpy.shutdown(context=self.ctx_core)


# ── Bundle ────────────────────────────────────────────────────────────────────

class Q5DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, ros2: DualDomainROS2):
        self.cfg = cfg
        self._plugins: list = []
        self.ros2 = ros2
        ros2._bundle = self
        self._stop_launch_cli = None  # 在 _init_joints 中设置
        plugins_cfg = cfg.get("plugins", {})

        # 系统状态类插件
        if plugins_cfg.get("system_state", {}).get("enabled", False):
            from device import SystemStatePlugin
            self._plugins.append(SystemStatePlugin(plugins_cfg["system_state"], namespace, ros2))
            print("[bundle] SystemStatePlugin loaded")

        if plugins_cfg.get("battery", {}).get("enabled", False):
            from device import BatteryPlugin
            self._plugins.append(BatteryPlugin(plugins_cfg["battery"], namespace, ros2))
            print("[bundle] BatteryPlugin loaded")

        if plugins_cfg.get("estop", {}).get("enabled", False):
            from device import EstopPlugin
            self._plugins.append(EstopPlugin(plugins_cfg["estop"], namespace, ros2))
            print("[bundle] EstopPlugin loaded")

        if plugins_cfg.get("fault", {}).get("enabled", False):
            from device import FaultPlugin
            self._plugins.append(FaultPlugin(plugins_cfg["fault"], namespace, ros2))
            print("[bundle] FaultPlugin loaded")

        if plugins_cfg.get("joints", {}).get("enabled", False):
            from device import JointsPlugin
            self._plugins.append(JointsPlugin(plugins_cfg["joints"], namespace, ros2))
            print("[bundle] JointsPlugin loaded (joints)")

        if plugins_cfg.get("imu", {}).get("enabled", False):
            from device import IMUPlugin
            self._plugins.append(IMUPlugin(plugins_cfg["imu"], namespace, ros2))
            print("[bundle] IMUPlugin loaded")

        if plugins_cfg.get("temperature", {}).get("enabled", False):
            from device import TemperaturePlugin
            self._plugins.append(TemperaturePlugin(plugins_cfg["temperature"], namespace, ros2))
            print("[bundle] TemperaturePlugin loaded")

        # 传感器类插件
        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, ros2))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("lidar", {}).get("enabled", False):
            from device import LidarPlugin
            self._plugins.append(LidarPlugin(plugins_cfg["lidar"], namespace, ros2))
            print("[bundle] LidarPlugin loaded")

        if plugins_cfg.get("mic", {}).get("enabled", False):
            from device import MicPlugin
            self._plugins.append(MicPlugin(plugins_cfg["mic"], namespace, ros2))
            print("[bundle] MicPlugin loaded")

        if plugins_cfg.get("asr", {}).get("enabled", False):
            from device import AsrPlugin
            self._plugins.append(AsrPlugin(plugins_cfg["asr"], namespace, ros2))
            print("[bundle] AsrPlugin loaded")

        # 运动控制类执行器插件
        if plugins_cfg.get("chassis", {}).get("enabled", False):
            from device import ChassisPlugin
            self._plugins.append(ChassisPlugin(plugins_cfg["chassis"], namespace, ros2))
            print("[bundle] ChassisPlugin loaded")

        if plugins_cfg.get("head", {}).get("enabled", False):
            from device import HeadPlugin
            self._plugins.append(HeadPlugin(plugins_cfg["head"], namespace, ros2))
            print("[bundle] HeadPlugin loaded")

        if plugins_cfg.get("arm", {}).get("enabled", False):
            from device import ArmPlugin
            self._plugins.append(ArmPlugin(plugins_cfg["arm"], namespace, ros2))
            print("[bundle] ArmPlugin loaded")

        if plugins_cfg.get("arm_servo", {}).get("enabled", False):
            from device import ArmServoPlugin
            self._plugins.append(ArmServoPlugin(plugins_cfg["arm_servo"], namespace, ros2))
            print("[bundle] ArmServoPlugin loaded")

        if plugins_cfg.get("waist", {}).get("enabled", False):
            from device import WaistPlugin
            self._plugins.append(WaistPlugin(plugins_cfg["waist"], namespace, ros2))
            print("[bundle] WaistPlugin loaded")

        if plugins_cfg.get("hand", {}).get("enabled", False):
            from device import HandPlugin
            self._plugins.append(HandPlugin(plugins_cfg["hand"], namespace, ros2))
            print("[bundle] HandPlugin loaded")

        if plugins_cfg.get("hand_low", {}).get("enabled", False):
            from device import HandLowlevelPlugin
            self._plugins.append(HandLowlevelPlugin(plugins_cfg["hand_low"], namespace, ros2))
            print("[bundle] HandLowlevelPlugin loaded")

        if plugins_cfg.get("hand_sensor", {}).get("enabled", False):
            from device import HandSensorPlugin
            self._plugins.append(HandSensorPlugin(plugins_cfg["hand_sensor"], namespace, ros2))
            print("[bundle] HandSensorPlugin loaded")

        # 交互类执行器插件
        if plugins_cfg.get("tts", {}).get("enabled", False):
            from device import TtsPlugin
            self._plugins.append(TtsPlugin(plugins_cfg["tts"], namespace, ros2))
            print("[bundle] TtsPlugin loaded")

        if plugins_cfg.get("audio_player", {}).get("enabled", False):
            from device import AudioPlayerPlugin
            self._plugins.append(AudioPlayerPlugin(plugins_cfg["audio_player"], namespace, ros2))
            print("[bundle] AudioPlayerPlugin loaded")

        if plugins_cfg.get("audio", {}).get("enabled", False):
            from device import AudioPlugin
            self._plugins.append(AudioPlugin(plugins_cfg["audio"], namespace, ros2))
            print("[bundle] AudioPlugin loaded")

        if plugins_cfg.get("led", {}).get("enabled", False):
            from device import LedPlugin
            self._plugins.append(LedPlugin(plugins_cfg["led"], namespace, ros2))
            print("[bundle] LedPlugin loaded")

        if plugins_cfg.get("chat", {}).get("enabled", False):
            from device import ChatPlugin
            self._plugins.append(ChatPlugin(plugins_cfg["chat"], namespace, ros2))
            print("[bundle] ChatPlugin loaded")

        if plugins_cfg.get("action_player", {}).get("enabled", False):
            from device import ActionPlayerPlugin
            self._plugins.append(ActionPlayerPlugin(plugins_cfg["action_player"], namespace, ros2))
            print("[bundle] ActionPlayerPlugin loaded")

        if plugins_cfg.get("gesture_player", {}).get("enabled", False):
            from device import GesturePlayerPlugin
            self._plugins.append(GesturePlayerPlugin(plugins_cfg["gesture_player"], namespace, ros2))
            print("[bundle] GesturePlayerPlugin loaded")

        if plugins_cfg.get("nav", {}).get("enabled", False):
            from device import NavPlugin
            self._plugins.append(NavPlugin(plugins_cfg["nav"], namespace, ros2))
            print("[bundle] NavPlugin loaded")

        # 资源类插件
        if plugins_cfg.get("bag_record", {}).get("enabled", False):
            from device import BagRecordPlugin
            self._plugins.append(BagRecordPlugin(plugins_cfg["bag_record"], namespace, ros2))
            print("[bundle] BagRecordPlugin loaded")

        if plugins_cfg.get("bag_playback", {}).get("enabled", False):
            from device import BagPlaybackPlugin
            self._plugins.append(BagPlaybackPlugin(plugins_cfg["bag_playback"], namespace, ros2))
            print("[bundle] BagPlaybackPlugin loaded")

        if plugins_cfg.get("mpc_controller", {}).get("enabled", False):
            from device import MpcControllerPlugin
            self._plugins.append(MpcControllerPlugin(plugins_cfg["mpc_controller"], namespace, ros2))
            print("[bundle] MpcControllerPlugin loaded")

        if plugins_cfg.get("model", {}).get("enabled", False):
            from device import ModelPlugin
            self._plugins.append(ModelPlugin(plugins_cfg["model"], namespace, ros2))
            print("[bundle] ModelPlugin loaded")

        # 新增卡片 — 系统诊断/里程计/遥操作/手柄/关节配置/运动/抱闸/TF/遥控/电源
        if plugins_cfg.get("odometry", {}).get("enabled", False):
            from device import OdometryPlugin
            self._plugins.append(OdometryPlugin(plugins_cfg["odometry"], namespace, ros2))
            print("[bundle] OdometryPlugin loaded")

        if plugins_cfg.get("diagnostics", {}).get("enabled", False):
            from device import DiagnosticsPlugin
            self._plugins.append(DiagnosticsPlugin(plugins_cfg["diagnostics"], namespace, ros2))
            print("[bundle] DiagnosticsPlugin loaded")

        if plugins_cfg.get("joystick", {}).get("enabled", False):
            from device import JoystickPlugin
            self._plugins.append(JoystickPlugin(plugins_cfg["joystick"], namespace, ros2))
            print("[bundle] JoystickPlugin loaded")

        if plugins_cfg.get("teleop", {}).get("enabled", False):
            from device import TeleopPlugin
            self._plugins.append(TeleopPlugin(plugins_cfg["teleop"], namespace, ros2))
            print("[bundle] TeleopPlugin loaded")

        if plugins_cfg.get("joint_config", {}).get("enabled", False):
            from device import JointConfigPlugin
            self._plugins.append(JointConfigPlugin(plugins_cfg["joint_config"], namespace, ros2))
            print("[bundle] JointConfigPlugin loaded")

        if plugins_cfg.get("motion", {}).get("enabled", False):
            from device import MotionPlugin
            self._plugins.append(MotionPlugin(plugins_cfg["motion"], namespace, ros2))
            print("[bundle] MotionPlugin loaded")

        if plugins_cfg.get("brake", {}).get("enabled", False):
            from device import BrakePlugin
            self._plugins.append(BrakePlugin(plugins_cfg["brake"], namespace, ros2))
            print("[bundle] BrakePlugin loaded")

        if plugins_cfg.get("tf", {}).get("enabled", False):
            from device import TfPlugin
            self._plugins.append(TfPlugin(plugins_cfg["tf"], namespace, ros2))
            print("[bundle] TfPlugin loaded")

        if plugins_cfg.get("remote_control", {}).get("enabled", False):
            from device import RemoteControlPlugin
            self._plugins.append(RemoteControlPlugin(plugins_cfg["remote_control"], namespace, ros2))
            print("[bundle] RemoteControlPlugin loaded")

        if plugins_cfg.get("power", {}).get("enabled", False):
            from device import PowerPlugin
            self._plugins.append(PowerPlugin(plugins_cfg["power"], namespace, ros2))
            print("[bundle] PowerPlugin loaded")

    def start_all(self) -> None:
        # 关节服务初始化流程（按官方顺序）
        self._init_joints()
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def _init_joints(self):
        """按官方SDK流程初始化关节服务。

        SDK手册：底层关节控制接口章节。
        步骤：
        0. 检查机器人硬件状态（EtherCAT连接、关节上电、绿色电源灯亮起）
        1. /dynamic_launch (有XHAND='pos', 无XHAND='no_hand_pos')
        2. /ready_service (初始化关节模组，约5秒)
        3. /simple_actions → initpose_handsdown (归位，约4秒; SDK接口名: /simple_trajectory)
        4. /activate_service (切到 Activate 状态)
        5. /simple_actions → lift_up (小臂抬起，SDK标准流程外，可选)
        """
        print("[bundle] Waiting for joint services...")
        print("[bundle] 前置检查: 请确认 EtherCAT 已连接、关节已上电、绿色电源灯亮起")
        import time as _time

        # 复用已有的 sub_node（domain 211，能访问本体服务）
        init_node = None
        for p in self._plugins:
            if hasattr(p, '_sub_node') and p._sub_node:
                init_node = p._sub_node
                break

        if not init_node:
            print("[bundle] WARN: no ROS2 node available for joint initialization")
            return

        # 检查是否配置了灵巧手，决定 dynamic_launch 模式
        plugins_cfg = self.cfg.get("plugins", {})
        _has_hand = (
            plugins_cfg.get("hand", {}).get("enabled", False) or
            plugins_cfg.get("hand_low", {}).get("enabled", False)
        )

        def _call_service_sync(client, request, timeout=15.0):
            future = client.call_async(request)
            start = _time.time()
            while not future.done() and (_time.time() - start) < timeout:
                _time.sleep(0.1)
            if not future.done():
                print(f"[bundle] WARN: service call timed out")
                return None
            return future.result()

        # 1. /dynamic_launch — 启动关节服务，按有无XHAND选择模式
        launch_mode = "pos" if _has_hand else "no_hand_pos"
        dyn_cli = init_node.create_client(DynamicLaunch, "/dynamic_launch")
        if not dyn_cli.wait_for_service(timeout_sec=10):
            print("[bundle] ERROR: /dynamic_launch service not available")
        else:
            req = DynamicLaunch.Request()
            req.app_name = ""
            req.sync_control = False
            req.launch_mode = launch_mode
            resp = _call_service_sync(dyn_cli, req)
            if resp and resp.success:
                print(f"[bundle] dynamic_launch OK — joint service started (mode={launch_mode})")
            else:
                print(f"[bundle] WARN: dynamic_launch failed: {resp}")

        # 2. /ready_service — 初始化关节模组（SDK手册：约5秒）
        ready_cli = init_node.create_client(Trigger, "/ready_service")
        if not ready_cli.wait_for_service(timeout_sec=10):
            print("[bundle] ERROR: /ready_service service not available")
        else:
            resp = _call_service_sync(ready_cli, Trigger.Request(), timeout=30.0)
            if resp and resp.success:
                print("[bundle] ready_service OK — joint modules initialized")
            else:
                print(f"[bundle] WARN: ready_service failed: {resp}")

        # 等待初始化完成（SDK手册：约5秒）
        _time.sleep(3)

        # 3. /simple_actions → initpose_handsdown — 归位（约4秒）
        #    注: SDK手册中此步骤接口名为 /simple_trajectory，实际ROS2 Action服务名为 /simple_actions
        action_cli = ActionClient(init_node, SimpleActions, "/simple_actions")
        if not action_cli.wait_for_server(timeout_sec=10):
            print("[bundle] ERROR: /simple_actions action server not available")
        else:
            goal = SimpleActions.Goal()
            goal.action_name = "initpose_handsdown"
            goal.time_cost = 4.0
            send_future = action_cli.send_goal_async(goal)
            _time.sleep(4)
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle and goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    _time.sleep(4)
                    if result_future.done():
                        print("[bundle] initpose_handsdown OK — joints homed")
                    else:
                        print("[bundle] WARN: initpose_handsdown result timeout")
                else:
                    print("[bundle] WARN: initpose_handsdown rejected")
            else:
                print("[bundle] WARN: initpose_handsdown send timeout")

        # 4. /activate_service — 切到 Activate 状态
        _time.sleep(1)
        act_cli = init_node.create_client(Trigger, "/activate_service")
        if not act_cli.wait_for_service(timeout_sec=10):
            print("[bundle] ERROR: /activate_service service not available")
        else:
            resp = _call_service_sync(act_cli, Trigger.Request())
            if resp and resp.success:
                print("[bundle] activate_service OK — joints activated")
            else:
                print(f"[bundle] WARN: activate_service failed: {resp}")

        # 5. /simple_actions → lift_up — 小臂抬起（SDK标准流程外，可选步骤）
        if action_cli is not None:
            goal = SimpleActions.Goal()
            goal.action_name = "lift_up"
            goal.time_cost = 4.0  # 手册示例: time_cost: 4.0
            send_future = action_cli.send_goal_async(goal)
            _time.sleep(4)
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle and goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    _time.sleep(4)
                    if result_future.done():
                        print("[bundle] lift_up OK — arms raised")
                    else:
                        print("[bundle] WARN: lift_up result timeout")
                else:
                    print("[bundle] WARN: lift_up rejected")
            else:
                print("[bundle] WARN: lift_up send timeout")

        # 保存 /stop_launch 客户端供 stop_all 使用
        stop_cli = init_node.create_client(Trigger, "/stop_launch")
        if stop_cli.wait_for_service(timeout_sec=3):
            self._stop_launch_cli = stop_cli
            print("[bundle] /stop_launch client registered for shutdown")
        else:
            self._stop_launch_cli = None
            print("[bundle] WARN: /stop_launch service not available")

        # 清理临时客户端
        for cli in (dyn_cli, ready_cli, act_cli):
            try:
                init_node.destroy_client(cli)
            except Exception:
                pass

    def stop_all(self) -> None:
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        print("[bundle] All plugins stopped")

        # 调用 /stop_launch 停止底层关节控制服务（手册流程）
        if hasattr(self, '_stop_launch_cli') and self._stop_launch_cli:
            try:
                from std_srvs.srv import Trigger
                if self._stop_launch_cli.service_is_ready():
                    import time as _time
                    future = self._stop_launch_cli.call_async(Trigger.Request())
                    start = _time.time()
                    while not future.done() and (_time.time() - start) < 5.0:
                        _time.sleep(0.1)
                    if future.done() and future.result() and future.result().success:
                        print("[bundle] /stop_launch OK — joint service stopped")
                    else:
                        print("[bundle] WARN: /stop_launch failed")
            except Exception as e:
                print(f"[bundle] WARN: /stop_launch error: {e}")

    def get_all_tools(self) -> list:
        tools = []
        for p in self._plugins:
            if hasattr(p, 'get_tools'):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, 'get_tools') else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args['_tool_name'] = tool_name
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: Q5DeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and '200' in msg:
                return
            print(f"[mcp] {self.address_string()} {msg}")

        def _send(self, status: int, body: str):
            encoded = body.encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(encoded)

        def do_GET(self):
            self.send_response(404)
            self.end_headers()

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                rpc = json.loads(raw)
            except Exception:
                self._send(400, json.dumps({"jsonrpc": "2.0", "id": None,
                                             "error": {"code": -32700, "message": "Parse error"}}))
                return

            rid    = rpc.get("id")
            method = rpc.get("method", "")
            params = rpc.get("params") or {}

            if rid is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}))

            def err(code, msg):
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid,
                                             "error": {"code": code, "message": msg}}))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "q5-device-bundle", "version": "1.0.0"},
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name   = params.get("name", "")
                    args   = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        tool_result = {
                            "content": [{
                                "type": "text",
                                "text": json.dumps(result),
                            }],
                        }
                        if (isinstance(result, dict)
                                and (result.get("state") == "error"
                                     or "error" in result)):
                            tool_result["isError"] = True
                        ok(tool_result)
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                import traceback
                traceback.print_exc()
                err(-32603, str(e))

    return Handler


# ── Entry point ───────────────────────────────────────────────────────────────


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url":  f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    _ctx = _ssl.create_default_context()
    _ctx.check_hostname = False
    _ctx.verify_mode = _ssl.CERT_NONE

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with _urllib.urlopen(req, timeout=3, context=_ctx):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)
    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle

    cfg       = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port  = int(cfg.get("mcp_port", 15708))
    hw_domain = int(cfg.get("ros_domain_id", 211))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} hw_domain={hw_domain}")

    # Dual-domain ROS2
    ros2 = DualDomainROS2(hardware_domain=hw_domain)
    ros2.start_spin()
    print(f"[bundle] Dual-domain ROS2 initialized (domain {hw_domain} + domain 42)")

    _bundle = Q5DeviceBundle(cfg, namespace, ros2)
    _bundle.start_all()

    _start_registration(mcp_port, cfg.get("name", "Xingdong Q5 Wheeled Humanoid"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        ros2.shutdown()


if __name__ == "__main__":
    main()