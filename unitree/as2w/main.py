#!/usr/bin/env python3
try:
    from common import logsafe
    logsafe.install()
except ImportError:
    pass

import ipaddress, json, os, re, signal, socket, struct, sys, threading, time, uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
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


_UNITREE_NETWORK = ipaddress.ip_network("192.168.123.0/24")
_VIRTUAL_INTERFACE_PREFIXES = (
    "br-", "cni", "docker", "flannel", "podman", "veth", "virbr",
)


def _network_interfaces():
    """Inspect Linux interfaces without route guessing or external commands."""
    import fcntl

    interfaces = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _, name in socket.if_nameindex():
            request = struct.pack("256s", name[:15].encode("ascii", "ignore"))
            try:
                flags_data = fcntl.ioctl(sock.fileno(), 0x8913, request)  # SIOCGIFFLAGS
                address_data = fcntl.ioctl(sock.fileno(), 0x8915, request)  # SIOCGIFADDR
            except OSError:
                continue
            sysfs = Path("/sys/class/net") / name
            try:
                virtual = "/virtual/net/" in str(sysfs.resolve())
            except OSError:
                virtual = False
            interfaces.append({
                "name": name,
                "ipv4": socket.inet_ntoa(address_data[20:24]),
                "up": bool(struct.unpack_from("H", flags_data, 16)[0] & 0x1),
                "wireless": (sysfs / "wireless").exists() or name.lower().startswith(("wl", "wlan")),
                "virtual": virtual or (sysfs / "bridge").exists(),
            })
    finally:
        sock.close()
    return interfaces


def _configured_interface(cfg):
    """Return the highest-priority setting; None means safe auto-detection."""
    positional = str(sys.argv[1]).strip() if len(sys.argv) > 1 else ""
    if positional:
        value = positional
    elif "NETWORK_INTERFACE" in os.environ:
        value = os.environ["NETWORK_INTERFACE"]
    else:
        value = cfg.get("robot_interface", "auto")
    value = str(value or "").strip()
    return None if not value or value.lower() == "auto" else value


def resolve_robot_interface(cfg):
    """Resolve an explicit interface or uniquely identify Unitree's wired LAN."""
    configured = _configured_interface(cfg)
    if configured:
        return configured

    try:
        interfaces = _network_interfaces()
    except Exception as exc:
        print(f"[as2w] Failed to inspect network interfaces: {exc}", flush=True)
        interfaces = []

    candidates = []
    for interface in interfaces:
        name = interface["name"]
        lowered = name.lower()
        if (not interface["up"] or lowered == "lo" or interface["wireless"]
                or interface["virtual"]
                or lowered.startswith(_VIRTUAL_INTERFACE_PREFIXES)):
            continue
        try:
            address = ipaddress.ip_address(interface["ipv4"])
        except ValueError:
            continue
        if address in _UNITREE_NETWORK:
            candidates.append((name, str(address)))

    if len(candidates) == 1:
        name, address = candidates[0]
        print(f"[as2w] Auto-detected Unitree interface: {name} ({address})", flush=True)
        return name
    if not candidates:
        print("[as2w] No Unitree robot interface found on 192.168.123.0/24", flush=True)
        print("[as2w] Set NETWORK_INTERFACE explicitly if the robot uses another subnet", flush=True)
        return None
    choices = ", ".join(f"{name} ({address})" for name, address in candidates)
    print(f"[as2w] Multiple Unitree network candidates found: {choices}", flush=True)
    print("[as2w] Set NETWORK_INTERFACE explicitly", flush=True)
    return None

class Bundle:
    def __init__(self, cfg, namespace, executor, proxy, interface, dds_ready=True):
        from device import LedPlugin, StatePlugin, LocoPlugin, SpecialMotionPlugin
        from multimedia import CameraPlugin, MicPlugin, SpeakerPlugin
        from lidar import LidarPlugin
        from controlled_spatial import ControlledSpatialPlugin
        from slam_mapping import SlamMappingPlugin
        p = cfg.get("plugins", {})
        self.plugins = []
        self._servo = None
        state_plugin = None
        if dds_ready and p.get("state", {}).get("enabled", True):
            state_plugin = StatePlugin(p.get("state", {}), namespace, executor)
            self.plugins.append(state_plugin)
        if p.get("loco", {}).get("enabled", True):
            self.plugins.append(LocoPlugin(p.get("loco", {}), namespace, executor, proxy,
                motion_snapshot=state_plugin.motion_snapshot if state_plugin else None))
        if p.get("special_motion", {}).get("enabled", True): self.plugins.append(SpecialMotionPlugin(p.get("special_motion", {}), namespace, executor, proxy))
        if dds_ready and p.get("mic", {}).get("enabled", True): self.plugins.append(MicPlugin(p.get("mic", {}), namespace, executor, interface))
        if dds_ready and p.get("speaker", {}).get("enabled", True): self.plugins.append(SpeakerPlugin(p.get("speaker", {}), namespace, executor, interface))
        if p.get("led", {}).get("enabled", True): self.plugins.append(LedPlugin(p.get("led", {}), namespace, executor, proxy))
        if dds_ready and p.get("camera", {}).get("enabled", True): self.plugins.append(CameraPlugin(p.get("camera", {}), namespace, executor, interface))
        if dds_ready and p.get("lidar", {}).get("enabled", True):
            self.plugins.append(LidarPlugin(
                p.get("lidar", {}), namespace, executor, interface=interface))
        if dds_ready and p.get("controlled_spatial", {}).get("enabled", True):
            spatial = ControlledSpatialPlugin(p.get("controlled_spatial", {}), namespace, executor, interface)
            spatial.set_chassis_guard(proxy)
            self.plugins.append(spatial)
        if dds_ready and p.get("slam_mapping", {}).get("enabled", True): self.plugins.append(SlamMappingPlugin(p.get("slam_mapping", {}), namespace, executor))
        if dds_ready and p.get("loco_servo", {}).get("enabled", False):
            from loco_servo import LocoServoPlugin
            loco = next((item for item in self.plugins if isinstance(item, LocoPlugin)), None)
            state = next((item for item in self.plugins if isinstance(item, StatePlugin)), None)
            special = next((item for item in self.plugins if isinstance(item, SpecialMotionPlugin)), None)
            client = proxy.create_control_client()
            def conflict():
                if loco is not None and loco.is_moving():
                    return "loco motion is active; stop it before resuming navigation"
                if special is not None and special.is_moving():
                    return "special motion is active; exit it before navigation"
                return None
            client.conflict_check = conflict
            self._servo = LocoServoPlugin(
                p.get("loco_servo", {}), namespace, executor, client,
                loco_plugin=loco,
                odom_provider=state.odom_snapshot if state is not None else None)
            self.plugins.append(self._servo)
    def start_all(self):
        for plugin in self.plugins: plugin.start()
    def stop_all(self):
        if self._servo is not None:
            self._servo.stop()
        for plugin in self.plugins:
            if plugin is not self._servo:
                plugin.stop()
    def tools(self):
        out = [
            {"name": "model", "type": "resource", "description": "Unitree As2W wheel-legged robot URDF model", "inputSchema": {"type": "object", "properties": {}}},
        ]
        for plugin in self.plugins: out.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return out
    def call(self, name, args):
        if name == "model":
            return {"urdf": (Path(__file__).with_name("resource") / "as2w.urdf").read_text()}
        for plugin in self.plugins:
            defs = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            if any(item["name"] == name for item in defs):
                action = args.get("action", name)
                if (self._servo is not None
                        and name in ("loco", "special_motion", "controlled_spatial")
                        and action not in ("start", "info", "get_state", "config")):
                    halted = self._servo.pause_for_explicit_command(f"{name}.{action}")
                    if not halted.get("ok"):
                        return {"ret": 3104, "accepted": False,
                                "error": "Navigation has not acknowledged stopping",
                                "navigation_stop": halted}
                return plugin.dispatch(action, {**{k: v for k, v in args.items() if k != "action"}, "_tool_name": name})
        return None

def handler(bundle):
    sse_sessions, sse_lock = {}, threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            message = fmt % args
            if '"POST /mcp' in message and "200" in message:
                return
            safe = message.encode("unicode_escape").decode("ascii")[:200]
            print(f"[mcp] {self.address_string()} {safe}", flush=True)
        def _send_json(self, status, payload):
            body = json.dumps(payload).encode()
            try:
                self.send_response(status); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers(); self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        def _send_sse(self, event, data):
            try:
                self.wfile.write(f"event: {event}\\ndata: {data}\\n\\n".encode())
                self.wfile.flush()
                return True
            except (BrokenPipeError, ConnectionResetError, OSError):
                return False
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path != "/mcp/sse":
                self._send_json(404, {"error": "not found"})
                return
            session_id = uuid.uuid4().hex
            self.send_response(200); self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache"); self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers()
            with sse_lock: sse_sessions[session_id] = self
            try:
                if not self._send_sse("endpoint", f"/mcp/messages?session_id={session_id}"): return
                while self._send_sse("ping", "{}"):
                    time.sleep(15)
            finally:
                with sse_lock: sse_sessions.pop(session_id, None)
        def do_OPTIONS(self):
            self.send_response(204); self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept"); self.end_headers()
        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path not in ("/mcp", "/mcp/messages"):
                self._send_json(404, {"error": "not found"}); return
            try: request = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            except Exception:
                self._send_json(400, {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}}); return
            if not isinstance(request, dict):
                self._send_json(400, {"jsonrpc": "2.0", "id": None,
                                      "error": {"code": -32600, "message": "Invalid Request"}})
                return
            method = request.get("method", "")
            params = request.get("params") or {}
            rid = request.get("id")
            if not isinstance(method, str) or not isinstance(params, dict):
                self._send_json(400, {"jsonrpc": "2.0", "id": rid,
                                      "error": {"code": -32600, "message": "Invalid Request"}})
                return
            if rid is None: self.send_response(202); self.end_headers(); return
            session_id = parse_qs(parsed.query).get("session_id", [""])[0]
            with sse_lock: sse_client = sse_sessions.get(session_id)
            def respond(payload):
                if sse_client is None:
                    self._send_json(200, payload); return
                self.send_response(202); self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers()
                if not sse_client._send_sse("message", json.dumps(payload)):
                    with sse_lock: sse_sessions.pop(session_id, None)
            if method == "initialize": result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}}, "serverInfo": {"name": "as2w-driver", "version": "1.0"}}
            elif method == "tools/list": result = {"tools": bundle.tools()}
            elif method == "tools/call":
                result = {"content": [{"type": "text", "text": json.dumps(bundle.call(params.get("name", ""), params.get("arguments") or {}))}]}
            else:
                respond({"jsonrpc": "2.0", "id": rid, "error": {"code": -32601, "message": f"Method not found: {method}"}}); return
            respond({"jsonrpc": "2.0", "id": rid, "result": result})
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
    cfg = load_config()
    interface = resolve_robot_interface(cfg)
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if os.environ.get("ROS_DOMAIN_ID") != "42" or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        print("[as2w] WARNING: ROS2 is not configured for agent-core Domain 42/FastDDS", flush=True)
    elif not profile or not os.path.isfile(profile):
        print(f"[as2w] WARNING: FastDDS profile is missing: {profile or '(unset)'}", flush=True)
    else:
        print(f"[as2w] ROS2 isolation profile: {profile} (Domain 42, FastDDS); Unitree SDK: CycloneDDS Domain 0 on {interface or '(unavailable)'}", flush=True)
    dds_ready = False
    # A body DDS participant must never silently bind to the office Wi-Fi.
    # The resolver returns only an explicit setting or a unique wired adapter
    # on Unitree's 192.168.123.0/24 network.
    candidates = [interface] if interface else []
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
    _start_registration(mcp_port, "Unitree As2W Bundle", "driver")
    def shutdown(*_):
        bundle.stop_all()
        if hasattr(proxy, "stop"): proxy.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown); server.serve_forever()

if __name__ == "__main__": main()
