#!/usr/bin/env python3
"""
engineai/t800/main.py — 众擎 T800 开发版 driver bundle 统一入口（MCP HTTP server）。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

双 domain 架构：
  - ctx_t800 (domain 69 / rmw_cyclonedds_cpp)：直连 T800 机器人话题
  - ctx_core (domain 42 / rmw_fastrtps_cpp)  ：与 agent-core / dashboard / perception 通信

启动流程：加载 config → 建 Ros2Contexts → 建 T800DeviceBundle →
bundle.start_all() → ros2.start() → HTTP server（config 的 mcp_port）→
向 agent-core 注册并每 30s 心跳。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH     — config.yaml 路径（默认同目录下）
    AGENT_CORE_URL  — agent-core 注册地址（默认 https://localhost:15678）

模块级只 import 标准库；yaml / rclpy 等在函数内延迟导入，
保证模块可在无 ROS2 环境下被纯 import 测试。
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


# ── Config ────────────────────────────────────────────────────────────────────


def _load_config() -> dict:
    import yaml
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle = None  # T800DeviceBundle，handler 通过闭包访问


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            # 抑制常规请求日志（心跳/info），只记录错误与工具调用
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
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {
                            "name": _bundle._cfg.get("name", "EngineAI T800 Bundle"),
                            "version": "1.0.0",
                        },
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        tool_result = {
                            "content": [{
                                "type": "text",
                                "text": json.dumps(result, ensure_ascii=False),
                            }],
                        }
                        # 运动状态前置检查等返回 {"state":"error",...} 时标记 isError，
                        # 让 agent-core / LLM 明确感知调用失败
                        if (isinstance(result, dict)
                                and (result.get("state") == "error" or "error" in result)):
                            tool_result["isError"] = True
                        ok(tool_result)
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                import traceback
                traceback.print_exc()
                err(-32603, str(e))

    return Handler


# ── agent-core 注册与心跳 ─────────────────────────────────────────────────────


def _start_registration(mcp_port: int, name: str, category: str):
    """向 agent-core 注册本 driver（后台线程），之后每 30s 心跳。"""
    import ssl as _ssl
    import urllib.request as _urllib
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "id": _bundle._cfg.get("id", "t800-driver"),
        "name": name,
        "url": f"http://localhost:{mcp_port}/mcp",
        "transport": "http",
        "category": category,
    }).encode()
    # agent-core 使用自签名证书，本地回环跳过校验
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
                    pass  # 心跳成功，抑制日志
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)

    threading.Thread(target=_run, daemon=True, name="register").start()


# ── Entry point ───────────────────────────────────────────────────────────────


def main():
    global _bundle

    cfg = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15708))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # 双域 ROS2：ctx_t800 (domain 69 / CycloneDDS) + ctx_core (domain 42 / FastDDS)
    from ros2 import Ros2Contexts
    ros2 = Ros2Contexts(namespace)
    ros2.start()
    print("[bundle] dual-domain ROS2 initialized "
          "(t800: domain 69/cyclonedds, core: domain 42/fastrtps)")

    from device import T800DeviceBundle
    _bundle = T800DeviceBundle(cfg, namespace, ros2)
    _bundle.start_all()

    _start_registration(mcp_port, cfg.get("name", "EngineAI T800 Bundle"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        ros2.shutdown()
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
