#!/usr/bin/env python3
try:
    from common import logsafe
    logsafe.install()
except ImportError:
    pass

import json, os, re, signal, socket, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import yaml
import rclpy
import rclpy.executors
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from rpc_proxy import RpcProxy


class _UnavailableProxy:
    """Preserve MCP availability when the robot DDS interface is absent."""
    def __getattr__(self, name):
        return lambda *args: (3104, {}) if name == "GetState" else 3104

def load_config():
    return yaml.safe_load(open(os.environ.get("CONFIG_PATH", Path(__file__).with_name("config.yaml"))))

class Bundle:
    def __init__(self, cfg, namespace, executor, proxy, interface, dds_ready=True):
        from device import StatePlugin, LocoPlugin, SpecialActionPlugin
        from lidar import LidarPlugin
        from controlled_spatial import ControlledSpatialPlugin
        from motion_tools import MotionExecutor, MotionRecorderPlugin, TrajectoryMotionPlugin
        p = cfg.get("plugins", {})
        self.plugins = []
        if dds_ready and p.get("state", {}).get("enabled", True): self.plugins.append(StatePlugin(p.get("state", {}), namespace, executor))
        motion_executor = MotionExecutor(proxy)
        loco = LocoPlugin(p.get("loco", {}), namespace, executor, proxy)
        loco.set_external_motion_stop(motion_executor.stop)
        if p.get("loco", {}).get("enabled", True): self.plugins.append(loco)
        def stop_loco():
            motion_executor.stop()
            loco.interrupt_motion()
        if p.get("trajectory_motion", {}).get("enabled", True):
            self.plugins.append(TrajectoryMotionPlugin(p.get("trajectory_motion", {}), proxy, motion_executor, stop_loco))
        if p.get("motion_recorder", {}).get("enabled", True):
            self.plugins.append(MotionRecorderPlugin(p.get("motion_recorder", {}), proxy, motion_executor, stop_loco))
        if p.get("special_action", {}).get("enabled", True): self.plugins.append(SpecialActionPlugin(p.get("special_action", {}), namespace, executor, proxy, stop_loco))
        if dds_ready and p.get("lidar", {}).get("enabled", True): self.plugins.append(LidarPlugin(p.get("lidar", {}), namespace, executor))
        if dds_ready and p.get("controlled_spatial", {}).get("enabled", True): self.plugins.append(ControlledSpatialPlugin(p.get("controlled_spatial", {}), namespace, executor, interface))
    def start_all(self):
        for plugin in self.plugins: plugin.start()
    def stop_all(self):
        for plugin in self.plugins: plugin.stop()
    def tools(self):
        out = [
            {"name": "model", "type": "resource", "description": "Unitree AS2W wheel-legged robot URDF model", "inputSchema": {"type": "object", "properties": {}}},
        ]
        for plugin in self.plugins: out.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return out
    def call(self, name, args):
        if name == "model":
            return {"urdf": (Path(__file__).with_name("resource") / "as2w.urdf").read_text()}
        for plugin in self.plugins:
            defs = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            if any(item["name"] == name for item in defs):
                return plugin.dispatch(args.pop("action", name), {**args, "_tool_name": name})
        return None

def handler(bundle):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            message = fmt % args
            if '"POST /mcp' in message and "200" in message:
                return
            safe = message.encode("unicode_escape").decode("ascii")[:200]
            print(f"[mcp] {self.address_string()} {safe}", flush=True)
        def do_POST(self):
            try: request = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            except Exception: self.send_error(400); return
            method, params, rid = request.get("method", ""), request.get("params") or {}, request.get("id")
            if rid is None: self.send_response(202); self.end_headers(); return
            if method == "initialize": result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}}, "serverInfo": {"name": "as2w-driver", "version": "1.0"}}
            elif method == "tools/list": result = {"tools": bundle.tools()}
            elif method == "tools/call":
                result = {"content": [{"type": "text", "text": json.dumps(bundle.call(params.get("name", ""), params.get("arguments") or {}))}]}
            else: self.send_response(200); self.end_headers(); return
            body = json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}).encode(); self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
    return Handler


def _start_registration(mcp_port, name, category):
    import ssl
    import urllib.request
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({"name": name, "url": f"http://localhost:{mcp_port}/mcp", "category": category}).encode()
    context = ssl._create_unverified_context()
    def run():
        import time
        while True:
            try:
                request = urllib.request.Request(f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=3, context=context):
                    pass
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}; retrying in 5s", flush=True)
                time.sleep(5)
    threading.Thread(target=run, daemon=True, name="agent-core-registration").start()

def main():
    cfg, interface = load_config(), sys.argv[1] if len(sys.argv) > 1 else os.environ.get("NETWORK_INTERFACE", "")
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if os.environ.get("ROS_DOMAIN_ID") != "42" or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        print("[as2w] WARNING: ROS2 is not configured for agent-core Domain 42/FastDDS", flush=True)
    elif not profile or not os.path.isfile(profile):
        print(f"[as2w] WARNING: FastDDS profile is missing: {profile or '(unset)'}", flush=True)
    else:
        print(f"[as2w] ROS2 isolation profile: {profile} (Domain 42, FastDDS); Unitree SDK: CycloneDDS Domain 0 on {interface or '(auto)'}", flush=True)
    dds_ready = False
    candidates = [interface] if interface else []
    try:
        candidates.extend(name for name in os.listdir("/sys/class/net") if name not in candidates and name != "lo")
    except OSError:
        pass
    candidates.append("")
    for candidate in candidates:
        try:
            ChannelFactoryInitialize(0, candidate or None)
            dds_ready = True
        except Exception as exc:
            print(f"[as2w] DDS init failed on {candidate or '(auto)'}: {exc}", flush=True)
            dds_ready = False
        if dds_ready:
            interface = candidate
            print(f"[as2w] Unitree DDS initialized on {interface or '(auto)'}", flush=True)
            break
    if not dds_ready:
        print("[as2w] WARNING: Unitree DDS unavailable; starting MCP in degraded mode", flush=True)
    namespace = re.sub(r"[^a-zA-Z0-9_]", "_", cfg.get("ros_namespace") or socket.gethostname())
    proxy = RpcProxy(interface) if dds_ready else _UnavailableProxy()
    rclpy.init(); executor = rclpy.executors.MultiThreadedExecutor(); bundle = Bundle(cfg, namespace, executor, proxy, interface, dds_ready); bundle.start_all()
    threading.Thread(target=lambda: executor.spin(), daemon=True).start()
    mcp_port = int(cfg.get("mcp_port", 15709))
    server = ThreadingHTTPServer(("", mcp_port), handler(bundle))
    _start_registration(mcp_port, "Unitree AS2W Bundle", "driver")
    def shutdown(*_):
        bundle.stop_all()
        if hasattr(proxy, "stop"): proxy.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown); server.serve_forever()

if __name__ == "__main__": main()
