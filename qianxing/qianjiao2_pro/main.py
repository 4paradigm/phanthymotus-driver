#!/usr/bin/env python3
"""MCP HTTP entry point for the Qianjiao 2.0 Pro ROV driver."""
from __future__ import annotations
import json, os, signal, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
import yaml
from device import QianjiaoDevice

CFG = yaml.safe_load(open(os.environ.get("CONFIG_PATH", str(Path(__file__).with_name("config.yaml")))))
DEVICE = QianjiaoDevice(CFG.get("rov", {}))

def start_registration(port: int) -> None:
    """Register with Agent Core and refresh the lease periodically."""
    import ssl
    import urllib.request
    agent = os.environ.get("AGENT_CORE_URL", "http://127.0.0.1:15678").rstrip("/")
    payload = json.dumps({
        "name": CFG.get("name", "潜蛟 2.0 Pro ROV"),
        "url": f"http://localhost:{port}/mcp",
        "category": "driver",
    }).encode()
    def loop():
        while True:
            try:
                req = urllib.request.Request(
                    f"{agent}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST")
                context = ssl._create_unverified_context()
                with urllib.request.urlopen(req, timeout=5, context=context) as response:
                    response.read()
                print(f"[register] Agent Core <- {agent}/api/mcp", flush=True)
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}; retrying in 5s", flush=True)
                time.sleep(5)
    threading.Thread(target=loop, daemon=True, name="agent-core-registration").start()

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return
    def _send(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_POST(self):
        if urlparse(self.path).path != "/mcp": self._send(404, {}); return
        try: rpc = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        except Exception: self._send(400, {"jsonrpc":"2.0","id":None,"error":{"code":-32700,"message":"Parse error"}}); return
        rid, method, params = rpc.get("id"), rpc.get("method", ""), rpc.get("params") or {}
        try:
            if method == "initialize": result = {"protocolVersion":"2024-11-05","capabilities":{"tools":{}},"serverInfo":{"name":"qianjiao2-pro","version":"1.0.0"}}
            elif method == "tools/list": result = {"tools": DEVICE.get_tools()}
            elif method == "tools/call": result = {"content":[{"type":"text","text":json.dumps(DEVICE.dispatch(params.get("name", ""), params.get("arguments") or {}), ensure_ascii=False)}]}
            else: self._send(200, {"jsonrpc":"2.0","id":rid,"error":{"code":-32601,"message":"Method not found"}}); return
            self._send(200, {"jsonrpc":"2.0","id":rid,"result":result})
        except Exception as exc:
            self._send(200, {"jsonrpc":"2.0","id":rid,"error":{"code":-32000,"message":str(exc)}})

def main():
    DEVICE.start()
    port = int(CFG.get("mcp_port", 15719)); server = ThreadingHTTPServer(("", port), Handler)
    start_registration(port)
    def shutdown(*_): DEVICE.stop(); threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown)
    print(f"[bundle] Qianjiao MCP server -> http://localhost:{port}/mcp", flush=True); server.serve_forever()

if __name__ == "__main__": main()
