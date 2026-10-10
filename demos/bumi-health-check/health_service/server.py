"""Small stateless HTTP MCP endpoint, compatible with Agent Core."""
import json
import os
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from .service import HealthService
from .ros import RosReceiver

def handler(service):
    class Handler(BaseHTTPRequestHandler):
        def send(self, status, data=None):
            raw = b"" if data is None else json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == "/healthz":
                return self.send(200, service.info())
            return self.send(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/mcp":
                return self.send(404, {"error": "not found"})
            rid = None
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 65536:
                    return self.send(413, {"error": "invalid body size"})
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict) or body.get("jsonrpc") != "2.0":
                    raise ValueError("invalid JSON-RPC request")
                rid = body.get("id")
                method = body.get("method")
                if "id" not in body:
                    return self.send(202)
                params = body.get("params", {})
                if not isinstance(params, dict):
                    raise ValueError("params must be an object")
                if method == "initialize":
                    result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                              "serverInfo": {"name": "bumi-health-check", "version": "1.0.0"}}
                elif method == "ping":
                    result = {}
                elif method == "tools/list":
                    result = {"tools": [service.tool()]}
                elif method == "tools/call":
                    args = params.get("arguments", {})
                    if params.get("name") != "health_check" or not isinstance(args, dict):
                        raise ValueError("unknown tool or invalid arguments")
                    value = service.dispatch(args.get("action"))
                    result = {"content": [{"type": "text", "text": json.dumps(value, ensure_ascii=False)}]}
                else:
                    return self.send(200, {"jsonrpc": "2.0", "id": rid,
                                          "error": {"code": -32601, "message": "Method not found"}})
                self.send(200, {"jsonrpc": "2.0", "id": rid, "result": result})
            except (ValueError, TypeError) as exc:
                self.send(200, {"jsonrpc": "2.0", "id": rid,
                               "error": {"code": -32602, "message": str(exc)}})
    return Handler

def main():
    with open(os.environ.get("HEALTH_CONFIG", "/work/config.json"), encoding="utf-8") as f:
        config = json.load(f)
    service = HealthService(os.environ.get("BUMI_NAMESPACE", ""), config)
    receiver = RosReceiver(service)
    try:
        receiver.start()
    except Exception as exc:
        # Expose diagnostic reports even when ROS cannot initialize.
        service.transport_error = f"ROS 初始化失败：{exc}"
    http = ThreadingHTTPServer(("0.0.0.0", int(os.environ.get("HEALTH_PORT", "15741"))), handler(service))
    def stop(*_):
        threading.Thread(target=http.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(json.dumps(service.info(), ensure_ascii=False), flush=True)
    try:
        http.serve_forever()
    finally:
        http.server_close()
        receiver.close()

if __name__ == "__main__":
    main()
