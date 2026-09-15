#!/usr/bin/env python3
import json, os, re, signal, socket, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import yaml
import rclpy
import rclpy.executors
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from rpc_proxy import RpcProxy

def load_config():
    return yaml.safe_load(open(os.environ.get("CONFIG_PATH", Path(__file__).with_name("config.yaml"))))

class Bundle:
    def __init__(self, cfg, namespace, executor, proxy):
        from device import StatePlugin, LocoPlugin
        from lidar import LidarPlugin
        p = cfg.get("plugins", {})
        self.plugins = []
        if p.get("state", {}).get("enabled", True): self.plugins.append(StatePlugin(p.get("state", {}), namespace, executor))
        if p.get("loco", {}).get("enabled", True): self.plugins.append(LocoPlugin(p.get("loco", {}), namespace, executor, proxy))
        if p.get("lidar", {}).get("enabled", True): self.plugins.append(LidarPlugin(p.get("lidar", {}), namespace, executor))
    def start_all(self):
        for plugin in self.plugins: plugin.start()
    def stop_all(self):
        for plugin in self.plugins: plugin.stop()
    def tools(self):
        out = [
            {"name": "model", "type": "resource", "description": "Unitree AS2 quadruped URDF model", "inputSchema": {"type": "object", "properties": {}}},
            {"name": "joints", "type": "resource", "description": "AS2 joint skeleton mapping for model visualization", "inputSchema": {"type": "object", "properties": {}}},
        ]
        for plugin in self.plugins: out.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return out
    def call(self, name, args):
        if name == "model":
            return {"path": str(Path(__file__).with_name("resource") / "as2_model.urdf"), "format": "urdf"}
        if name == "joints":
            return {"format": "sensor/skeleton", "joint_names": [f"{side}_{joint}_joint" for side in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")], "model": "as2_model.urdf"}
        for plugin in self.plugins:
            defs = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            if any(item["name"] == name for item in defs):
                return plugin.dispatch(args.pop("action", name), {**args, "_tool_name": name})
        return None

def handler(bundle):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def do_POST(self):
            try: request = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            except Exception: self.send_error(400); return
            method, params, rid = request.get("method", ""), request.get("params") or {}, request.get("id")
            if rid is None: self.send_response(202); self.end_headers(); return
            if method == "initialize": result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}}, "serverInfo": {"name": "as2-driver", "version": "1.0"}}
            elif method == "tools/list": result = {"tools": bundle.tools()}
            elif method == "tools/call":
                result = {"content": [{"type": "text", "text": json.dumps(bundle.call(params.get("name", ""), params.get("arguments") or {}))}]}
            else: self.send_response(200); self.end_headers(); return
            body = json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}).encode(); self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
    return Handler

def main():
    cfg, interface = load_config(), sys.argv[1] if len(sys.argv) > 1 else os.environ.get("NETWORK_INTERFACE", "")
    try: ChannelFactoryInitialize(0, interface)
    except Exception as exc: print(f"[as2] DDS init failed: {exc}")
    namespace = re.sub(r"[^a-zA-Z0-9_]", "_", cfg.get("ros_namespace") or socket.gethostname())
    proxy = RpcProxy(interface); rclpy.init(); executor = rclpy.executors.MultiThreadedExecutor(); bundle = Bundle(cfg, namespace, executor, proxy); bundle.start_all()
    threading.Thread(target=lambda: executor.spin(), daemon=True).start()
    server = ThreadingHTTPServer(("", int(cfg.get("mcp_port", 15704))), handler(bundle))
    def shutdown(*_):
        bundle.stop_all(); proxy.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown); server.serve_forever()

if __name__ == "__main__": main()
