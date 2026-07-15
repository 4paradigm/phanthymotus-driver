from __future__ import annotations

#!/usr/bin/env python3
"""
drivers/unitree/go1/main.py — Unitree Go1 四足机器狗 设备 bundle 统一入口。

结构对齐 go2(MCP HTTP server + 插件聚合 + 自动注册/心跳),但:
  • 去掉 DDS/RpcProxy,改用本地 Go1 高层客户端 Go1HighLevel(go1_hl.py,唯一硬件写入口)。
  • rclpy 全部隔离到 ros_bridge.py:有 rclpy 真发 topic,无 rclpy(开发 Mac)自动降级离线。
  • 插件模块化到 plugins/,硬件访问经 adapters/(real/fake 自动选择)。

用法:  python3 main.py <networkInterface>      # networkInterface 目前仅用于日志
环境变量:CONFIG_PATH(config.yaml 路径)、AGENT_CORE_URL(默认 https://localhost:15678)、
         ROS_MODE(auto|real|fake,覆盖 bridge 模式;默认 auto)。
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

from go1_hl import Go1HighLevel
from ros_bridge import RosBridge


# ── Config ────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    import yaml  # 惰性:开发机离线测试不走这里(smoke 直接喂 dict)
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── 插件注册表(config 驱动;供 main 与 smoke 复用) ───────────────────────────

def build_plugins(cfg: dict, namespace: str, bridge: RosBridge, hl: Go1HighLevel, ctrl=None) -> list:
    """只装配 3 张已实机验证的只读状态卡:loco_state / imu / feet(MT 状态卡)。
    需 mt.enabled 且 ctrl(Go1Control)就绪;单卡失败只跳过它,不拖垮 bundle。"""
    plugins = []
    mt = cfg.get("mt", {}) or {}
    if mt.get("enabled") and ctrl is not None:
        from plugins.mt_state import LocoStateCard, ImuCard, FeetCard
        for cardcls in (LocoStateCard, ImuCard, FeetCard):
            try:
                plugins.append(cardcls(mt.get(cardcls.CARD, {}) or {}, namespace, bridge, ctrl))
                print(f"[bundle] MT card '{cardcls.CARD}' loaded", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[bundle] MT card '{cardcls.CARD}' FAILED: {e}", flush=True)
    return plugins


# ── Bundle:聚合插件,提供 tools/list 与 dispatch ─────────────────────────────

class Go1DeviceBundle:
    def __init__(self, plugins: list):
        self._plugins = plugins

    def start_all(self):
        for p in self._plugins:
            try:
                p.start()
            except Exception as e:  # noqa: BLE001
                print(f"[bundle] {type(p).__name__}.start() FAILED: {e}", flush=True)
                import traceback; traceback.print_exc()
        print(f"[bundle] {len(self._plugins)} plugins started", flush=True)

    def stop_all(self):
        for p in self._plugins:
            try:
                p.stop()
            except Exception:  # noqa: BLE001
                pass
        print("[bundle] all plugins stopped", flush=True)

    def _model_tool(self):
        return {"name": "model", "type": "resource",
                "description": "Go1 quadruped robot URDF model for skeleton renderer",
                "inputSchema": {"type": "object", "properties": {}}}

    def get_all_tools(self):
        tools = [self._model_tool()]
        for p in self._plugins:
            tools.extend(p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()])
        return tools

    def dispatch(self, tool_name, args):
        if tool_name == "model":
            urdf_path = Path(__file__).parent / "resource" / "go1_model.urdf"
            return {"urdf": urdf_path.read_text()}
        for p in self._plugins:
            tool_defs = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for td in tool_defs:
                if td["name"] == tool_name:
                    if td["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name
                    return p.dispatch(action, args)
        return None


# ── MCP HTTP server(JSON-RPC 2.0,与 go2 一致) ───────────────────────────────

_bundle: "Go1DeviceBundle | None" = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and '200' in msg:
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
            self.send_response(404); self.end_headers()

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
            rid = rpc.get("id"); method = rpc.get("method", ""); params = rpc.get("params") or {}
            if rid is None:
                self.send_response(202); self.end_headers(); return

            def ok(result): self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}))
            def err(code, msg): self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid,
                                                            "error": {"code": code, "message": msg}}))
            try:
                if method == "initialize":
                    ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                        "serverInfo": {"name": "go1-hcl", "version": "1.0.0"}})
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", ""); a = params.get("arguments") or {}
                    result = _bundle.dispatch(name, a)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result)}]})
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:  # noqa: BLE001
                err(-32603, str(e))
    return Handler


# ── 向 Agent Core 注册 + 30s 心跳 ────────────────────────────────────────────

def _start_registration(mcp_port, name, category):
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({"name": name, "url": f"http://localhost:{mcp_port}/mcp",
                          "category": category}).encode()
    _ctx = _ssl.create_default_context(); _ctx.check_hostname = False; _ctx.verify_mode = _ssl.CERT_NONE

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(f"{agent_core_url}/api/mcp", data=payload,
                                      headers={"Content-Type": "application/json"}, method="POST")
                with _urllib.urlopen(req, timeout=3, context=_ctx):
                    pass
                _t.sleep(30)
            except Exception as e:  # noqa: BLE001
                print(f"[register] failed: {e}, retry in 5s")
                _t.sleep(5)
    threading.Thread(target=_run, daemon=True, name="register").start()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global _bundle
    network_iface = sys.argv[1] if len(sys.argv) >= 2 else ""
    cfg = _load_config()
    # 环境变量覆盖后端(便于测试/容器显式指定,如 GO1_BACKEND=subproc)
    _be = os.environ.get("GO1_BACKEND")
    if _be:
        cfg.setdefault("robot", {})["backend"] = _be
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15704))
    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} iface={network_iface or '(auto)'}")

    hl = Go1HighLevel(cfg.get("robot", {}))
    ctrl = None
    mt_on = bool((cfg.get("mt", {}) or {}).get("enabled"))
    # 高低层单核心互斥:mt 模式用 Go1Control(经典 SDK 14 卡),否则用 Go1HighLevel(proprietary)。
    # 两者都会开高层 UDP 通道,绝不同时 start(否则争用同一 Legged_sport 通道)。
    if mt_on:
        from go1_ctrl import Go1Control
        ctrl = Go1Control(cfg.get("mt", {}))
        ctrl.start()
        print(f"[bundle] MT 模式 control_level={ctrl.control_level}", flush=True)
    else:
        hl.start()

    bridge = RosBridge(force_mode=os.environ.get("ROS_MODE", "auto"))
    bridge.start()

    _bundle = Go1DeviceBundle(build_plugins(cfg, namespace, bridge, hl, ctrl))
    _bundle.start_all()
    bridge.spin_background()

    _start_registration(mcp_port, cfg.get("name", "Unitree Go1"), "driver")

    # accept 队列调大:默认 backlog=5,多客户端(ROS 转发器串行读 + core 轮询 + 人工 call)
    # 并发连接稍一叠加就溢出 → Connection refused。128 足够吸收突发。
    ThreadingHTTPServer.request_queue_size = 128
    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        if ctrl is not None:
            ctrl.stop()
        hl.stop()
        bridge.shutdown()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        if ctrl is not None:
            ctrl.stop()
        hl.stop()
        bridge.shutdown()


if __name__ == "__main__":
    main()
