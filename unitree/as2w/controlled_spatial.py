"""AS2W adapter for Unitree's documented ``slam_operate`` DDS service.

The SDK currently publishes no AS2-specific SLAM wrapper. The service itself
uses Unitree's common RPC protocol, so this module owns the documented client.
"""
import json
import multiprocessing
import threading

def _install_logsafe():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass

_SERVICE = "slam_operate"
_VERSION = "1.0.0.1"
_APIS = {"start_mapping": 1801, "stop_mapping": 1802, "init_pose": 1804,
         "navigate_to": 1102, "pause_navigation": 1201,
         "resume_navigation": 1202, "shutdown": 1901}


class _SlamClient:
    def __init__(self):
        from unitree_sdk2py.rpc.client import Client
        self._client = Client(_SERVICE)
        self._client._SetApiVerson(_VERSION)
        for api_id in _APIS.values():
            self._client._RegistApi(api_id, 0)
        self._client.SetTimeout(10.0)

    def call(self, action, data):
        return self._client._Call(_APIS[action], json.dumps({"data": data}))


def _worker(commands, results, interface):
    _install_logsafe()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0, interface)
    client = _SlamClient()
    while True:
        command = commands.get()
        if command is None:
            return
        try:
            code, response = client.call(command["action"], command["data"])
            results.put({"code": code, "response": response})
        except Exception as exc:
            results.put({"code": 3104, "response": str(exc)})


class _SpatialRpcProxy:
    def __init__(self, interface):
        context = multiprocessing.get_context("spawn")
        self._commands, self._results = context.Queue(), context.Queue()
        self._process = context.Process(target=_worker, args=(self._commands, self._results, interface), daemon=True)
        self._process.start()
        self._lock = threading.Lock()

    def call(self, action, data):
        with self._lock:
            self._commands.put({"action": action, "data": data})
            try:
                return self._results.get(timeout=20)
            except Exception:
                return {"code": 3104, "response": "SLAM service timeout"}

    def stop(self):
        self._commands.put(None)
        self._process.join(timeout=3)


class ControlledSpatialPlugin:
    """Low-level map, relocalization, and navigation backed by vendor SLAM."""
    PREFIX = "controlled_spatial"

    def __init__(self, config, namespace, executor, interface):
        self._client = _SpatialRpcProxy(interface)

    def get_tool(self):
        return {"name": "controlled_spatial", "type": "actuator", "multiInstance": False,
                "description": "AS2W SLAM map, relocalization, and point-goal navigation. Requires the vendor unitree_slam service already running.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": list(_APIS)},
                    "address": {"type": "string", "description": "Absolute PCD path for stop_mapping or init_pose."},
                    "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                    "q_x": {"type": "number"}, "q_y": {"type": "number"}, "q_z": {"type": "number"}, "q_w": {"type": "number"},
                    "speed": {"type": "number", "minimum": 0.2, "maximum": 1.5},
                    "mode": {"type": "integer", "enum": [0, 1]}}, "required": ["action"],
                "x-action-params": {
                    "start_mapping": {"params": [], "description": "Start indoor SLAM mapping."},
                    "stop_mapping": {"params": ["address"], "description": "Stop mapping and save PCD."},
                    "init_pose": {"params": ["address", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"], "description": "Load map and initialize pose."},
                    "navigate_to": {"params": ["x", "y", "z", "q_x", "q_y", "q_z", "q_w", "speed", "mode"], "description": "Navigate to a target pose."},
                    "pause_navigation": {"params": [], "description": "Pause navigation."},
                    "resume_navigation": {"params": [], "description": "Resume navigation."},
                    "shutdown": {"params": [], "description": "Close vendor SLAM service."}}}}

    def start(self): pass
    def stop(self): self._client.stop()

    @staticmethod
    def _pose(args):
        return {key: float(args.get(key, default)) for key, default in {
            "x": 0, "y": 0, "z": 0, "q_x": 0, "q_y": 0, "q_z": 0, "q_w": 1}.items()}

    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action not in _APIS: return None
        if action in ("stop_mapping", "init_pose") and not args.get("address"):
            return {"error": "address is required for this action"}
        if action == "start_mapping": data = {"slam_type": "indoor"}
        elif action == "stop_mapping": data = {"address": args["address"]}
        elif action == "init_pose": data = {**self._pose(args), "address": args["address"]}
        elif action == "navigate_to":
            data = {"targetPose": self._pose(args), "mode": int(args.get("mode", 1)), "speed": float(args.get("speed", 0.5))}
        else: data = {}
        result = self._client.call(action, data)
        response = result["response"]
        try: response = json.loads(response) if isinstance(response, str) else response
        except json.JSONDecodeError: pass
        return {"ret": result["code"], "response": response}
