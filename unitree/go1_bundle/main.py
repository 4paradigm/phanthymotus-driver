#!/usr/bin/env python3
"""
go1_bundle/main.py — Unitree Go1 (EDU) 状态 + 基础控制驱动入口（原始 unitree_legged_sdk）。

一个驱动 = 一个 MCP server。卡片聚合在 4 个文件：
`sensors.py`（11 张状态/资源卡）/ `controllers.py`（5 张控制卡）/
`ext_devices.py`（4 张外部设备卡）/ `camera.py`（独立注册 RGB、深度、点云三张视觉卡）。
main.py 按 config.yaml 里启用的卡名**显式导入对应模块**并装配
（约定：config key == 模块内的 make_* 函数名）。
因此新增一张卡 = 在对应聚合文件中追加 `Plugin` + `make_<卡名>` +
在 config.yaml 打开它，**同时需在本文件的 `Go1Bundle.__init__` 中添加对应导入和创建逻辑**。

所有卡共享同一个 raw SDK client（唯一 UDP 收发线程 → snapshot()）；状态卡只读 snapshot 的
不同切片，控制卡经该 client 的下发原语发 HighCmd。装了 rclpy 时状态卡发 ROS2 topic 在画布
渲染，否则走 MCP action=info。固定 HIGHLEVEL（读 HighState / 下发 HighCmd）。
无 robot_interface / 无硬件时自动 STUB（server 仍能起、注册、列 tool）。

用法： python3 main.py [networkInterface]   环境变量： CONFIG_PATH / AGENT_CORE_URL
详见同目录 CONTRIBUTING.md。
"""

from __future__ import annotations

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

try:
    import rclpy
    import rclpy.executors
    _HAS_ROS2 = True
except Exception:
    _HAS_ROS2 = False


def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


class Go1Bundle:
    """按 config 装配启用的卡片；对外提供 tools 列表与 dispatch。

    装配规则：按 config.plugins 中 enabled 的卡名，从对应聚合模块调用 make_<卡名> 工厂函数。
    """

    def __init__(self, cfg, namespace, executor, client):
        self._plugins = []
        pc = cfg.get("plugins", {}) or {}

        # 传感卡 - 从 sensors.py 导入
        if pc.get("loco_state", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_loco_state(pc["loco_state"], namespace, executor, client))
            print("[bundle] loco_state loaded")

        if pc.get("battery", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_battery(pc["battery"], namespace, executor, client))
            print("[bundle] battery loaded")

        if pc.get("imu", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_imu(pc["imu"], namespace, executor, client))
            print("[bundle] imu loaded")

        if pc.get("feet", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_feet(pc["feet"], namespace, executor, client))
            print("[bundle] feet loaded")

        if pc.get("fall_alarm", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_fall_alarm(pc["fall_alarm"], namespace, executor, client))
            print("[bundle] fall_alarm loaded")

        if pc.get("odometry", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_odometry(pc["odometry"], namespace, executor, client))
            print("[bundle] odometry loaded")

        if pc.get("obstacle_range", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_obstacle_range(pc["obstacle_range"], namespace, executor, client))
            print("[bundle] obstacle_range loaded")

        if pc.get("udp_diagnostics", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_udp_diagnostics(pc["udp_diagnostics"], namespace, executor, client))
            print("[bundle] udp_diagnostics loaded")

        if pc.get("joints", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_joints(pc["joints"], namespace, executor, client))
            print("[bundle] joints loaded")

        if pc.get("remote_controller", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_remote_controller(pc["remote_controller"], namespace, executor, client))
            print("[bundle] remote_controller loaded")

        if pc.get("model", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_model(pc["model"], namespace, executor, client))
            print("[bundle] model loaded")

        if pc.get("activity_monitor", {}).get("enabled", False):
            import sensors
            self._plugins.append(sensors.make_activity_monitor(pc["activity_monitor"], namespace, executor, client))
            print("[bundle] activity_monitor loaded")

        # 控制卡 - 从 controllers.py 导入
        if pc.get("loco", {}).get("enabled", False):
            import controllers
            self._plugins.append(controllers.make_loco(pc["loco"], namespace, executor, client))
            print("[bundle] loco loaded")

        if pc.get("body_pose", {}).get("enabled", False):
            import controllers
            self._plugins.append(controllers.make_body_pose(pc["body_pose"], namespace, executor, client))
            print("[bundle] body_pose loaded")

        if pc.get("switch_gait", {}).get("enabled", False):
            import controllers
            self._plugins.append(controllers.make_switch_gait(pc["switch_gait"], namespace, executor, client))
            print("[bundle] switch_gait loaded")

        if pc.get("gesture", {}).get("enabled", False):
            import controllers
            self._plugins.append(controllers.make_gesture(pc["gesture"], namespace, executor, client))
            print("[bundle] gesture loaded")

        if pc.get("special_motion", {}).get("enabled", False):
            import controllers
            self._plugins.append(controllers.make_special_motion(pc["special_motion"], namespace, executor, client))
            print("[bundle] special_motion loaded")

        # 外部设备 - 从 ext_devices.py 导入
        if pc.get("beep", {}).get("enabled", False):
            import ext_devices
            self._plugins.append(ext_devices.make_beep(pc["beep"], namespace, executor, client))
            print("[bundle] beep loaded")

        if pc.get("speaker", {}).get("enabled", False):
            import ext_devices
            self._plugins.append(ext_devices.make_speaker(pc["speaker"], namespace, executor, client))
            print("[bundle] speaker loaded")

        if pc.get("face_light", {}).get("enabled", False):
            import ext_devices
            self._plugins.append(ext_devices.make_face_light(pc["face_light"], namespace, executor, client))
            print("[bundle] face_light loaded")

        if pc.get("system_health", {}).get("enabled", False):
            import ext_devices
            self._plugins.append(ext_devices.make_system_health(pc["system_health"], namespace, executor, client))
            print("[bundle] system_health loaded")

        # 相机 - 三张独立卡仍由同一个 camera.py 聚合文件提供。
        if pc.get("camera_rgb", {}).get("enabled", False):
            import camera
            self._plugins.append(camera.make_camera_rgb(pc["camera_rgb"], namespace, executor, client))
            print("[bundle] camera_rgb loaded")

        if pc.get("camera_depth", {}).get("enabled", False):
            import camera
            self._plugins.append(camera.make_camera_depth(pc["camera_depth"], namespace, executor, client))
            print("[bundle] camera_depth loaded")

        if pc.get("camera_pointcloud", {}).get("enabled", False):
            import camera
            self._plugins.append(camera.make_camera_pointcloud(pc["camera_pointcloud"], namespace, executor, client))
            print("[bundle] camera_pointcloud loaded")

        print(f"[bundle] {len(self._plugins)} plugins loaded", flush=True)

    def start_all(self):
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__module__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self):
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        print("[bundle] All plugins stopped")

    def get_all_tools(self):
        tools = []
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name, args):
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name
                    return p.dispatch(action, args)
        return None


_bundle = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and "200" in msg:
                return
            print(f"[mcp] {self.address_string()} {msg}")

        def _send(self, status, body):
            enc = body.encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(enc)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(enc)

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
            rid = rpc.get("id")
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
                    ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                        "serverInfo": {"name": "go1-bundle", "version": "1.0.0"}})
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    elif isinstance(result, dict) and "__mcp_content__" in result:
                        # 插件直接返回 MCP content（如图像），不包装为 text
                        ok({"content": result["__mcp_content__"]})
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result)}]})
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                err(-32603, str(e))

    return Handler


def _start_registration(mcp_port, name, category):
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    # 与 agent-core 同机 → localhost；跨机须设 MCP_ADVERTISE_HOST
    advertise_host = os.environ.get("MCP_ADVERTISE_HOST", "localhost")
    payload = json.dumps({"name": name, "url": f"http://{advertise_host}:{mcp_port}/mcp", "category": category}).encode()
    _ctx = _ssl.create_default_context()
    _ctx.check_hostname = False
    _ctx.verify_mode = _ssl.CERT_NONE

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(f"{agent_core_url}/api/mcp", data=payload,
                                      headers={"Content-Type": "application/json"}, method="POST")
                with _urllib.urlopen(req, timeout=3, context=_ctx):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)

    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle
    network_iface = sys.argv[1] if len(sys.argv) >= 2 else ""
    cfg = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15715))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} control_level=HIGHLEVEL")

    from sdk_proxy import SdkProxy
    client = SdkProxy(network_iface=network_iface)
    print(f"[bundle] SdkProxy subprocess started ({'live' if client.available else 'STUB'})")

    executor = None
    if _HAS_ROS2:
        try:
            rclpy.init()
            executor = rclpy.executors.MultiThreadedExecutor()
            print("[bundle] ROS2 executor ready")
        except Exception as e:
            print(f"[bundle] ROS2 init 失败，状态卡走 MCP 轮询: {e}")
    else:
        print("[bundle] 未检测到 rclpy，状态卡走 MCP 轮询")

    _bundle = Go1Bundle(cfg, namespace, executor, client)
    _bundle.start_all()

    if executor is not None:
        def _spin():
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        threading.Thread(target=_spin, daemon=True, name="bundle_spin").start()

    _start_registration(mcp_port, cfg.get("name", "Unitree Go1 (Bundle)"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        client.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        client.stop()
        if executor is not None:
            executor.shutdown()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
