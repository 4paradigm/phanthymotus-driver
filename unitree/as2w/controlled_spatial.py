"""As2W adapter for Unitree's documented ``slam_operate`` DDS service.

The SDK currently publishes no AS2-specific SLAM wrapper. The service itself
uses Unitree's common RPC protocol, so this module owns the documented client.
"""
import json
import multiprocessing
import threading
import time
from uuid import uuid4

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


def _acp_notify(action_id, status, result):
    """Report asynchronous navigation completion to Agent Core."""
    import os
    import ssl
    import urllib.request
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": "controlled_spatial",
                          "ts": time.time()}).encode()
    try:
        request = urllib.request.Request(
            f"{os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')}/api/acp/complete",
            data=payload, headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(request, timeout=5, context=ssl._create_unverified_context())
    except Exception as exc:
        print(f"[ACP] callback failed for {action_id}: {exc}", flush=True)


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
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface or None)
        client = _SlamClient()
        results.put({"ready": True})
    except Exception as exc:
        results.put({"startup_error": str(exc)})
        return
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
        self._startup_error = None
        try:
            result = self._results.get(timeout=5)
            self._startup_error = result.get("startup_error")
        except Exception:
            self._startup_error = "SLAM worker did not become ready"

    def call(self, action, data):
        if self._startup_error:
            return {"code": 3104, "response": self._startup_error}
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
        self._nav_done = threading.Event()
        self._nav_result = None
        self._nav_action_id = None
        self._nav_lock = threading.Lock()
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            self._nav_sub = ChannelSubscriber("rt/slam_key_info", String_)
            self._nav_sub.Init(self._on_slam_key_info, 10)
        except Exception as exc:
            self._nav_sub = None
            print(f"[controlled_spatial] SLAM completion topic unavailable: {exc}", flush=True)

    def get_tool(self):
        return {"name": "controlled_spatial", "type": "actuator", "multiInstance": False,
                "description": "As2W SLAM map, relocalization, and point-goal navigation. Requires the vendor unitree_slam service already running.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": list(_APIS)},
                    "address": {"type": "string", "description": "Absolute PCD path for stop_mapping or init_pose."},
                    "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                    "q_x": {"type": "number"}, "q_y": {"type": "number"}, "q_z": {"type": "number"}, "q_w": {"type": "number"},
                    "speed": {"type": "number", "minimum": 0.2, "maximum": 1.5},
                    "mode": {"type": "integer", "enum": [0, 1]}}, "required": ["action"],
                "x-completion": {"actions": ["navigate_to"], "timeout": 180},
                "x-action-params": {
                    "start_mapping": {"params": [], "description": "Start indoor SLAM mapping."},
                    "stop_mapping": {"params": ["address"], "description": "Stop mapping and save PCD."},
                    "init_pose": {"params": ["address", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"], "description": "Load map and initialize pose."},
                    "navigate_to": {"params": ["x", "y", "z", "q_x", "q_y", "q_z", "q_w", "speed", "mode"], "description": "Navigate to a target pose."},
                    "pause_navigation": {"params": [], "description": "Pause navigation."},
                    "resume_navigation": {"params": [], "description": "Resume navigation."},
                    "shutdown": {"params": [], "description": "Close vendor SLAM service."}}}}

    def start(self): pass
    def stop(self):
        with self._nav_lock:
            action_id, self._nav_action_id = self._nav_action_id, None
        if action_id:
            self._client.call("pause_navigation", {})
            _acp_notify(action_id, "cancelled", {"reason": "card stopped"})
        if self._nav_sub is not None:
            try:
                self._nav_sub.Close()
            except Exception:
                pass
        self._client.stop()

    def _on_slam_key_info(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, AttributeError):
            return
        if payload.get("type") == "task_result":
            self._nav_result = payload
            self._nav_done.set()

    def _wait_for_navigation(self, action_id, target):
        completed = self._nav_done.wait(timeout=180)
        with self._nav_lock:
            if self._nav_action_id != action_id:
                return
        if completed:
            result = self._nav_result or {}
            arrived = bool(result.get("data", {}).get("is_arrived", False))
            status = "completed" if arrived and result.get("errorCode", 0) == 0 else "error"
            _acp_notify(action_id, status, {"target": target, "response": result})
        else:
            self._client.call("pause_navigation", {})
            _acp_notify(action_id, "error", {"target": target, "error": "navigation timed out after 180 seconds"})
        with self._nav_lock:
            if self._nav_action_id == action_id:
                self._nav_action_id = None

    @staticmethod
    def _pose(args):
        return {key: float(args.get(key, default)) for key, default in {
            "x": 0, "y": 0, "z": 0, "q_x": 0, "q_y": 0, "q_z": 0, "q_w": 1}.items()}

    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action not in _APIS: return None
        if action in ("stop_mapping", "init_pose") and not args.get("address"):
            return {"error": "address is required for this action"}
        if action == "start_mapping": data = {"slam_type": "indoor"}
        elif action == "stop_mapping": data = {"address": args["address"]}
        elif action == "init_pose": data = {**self._pose(args), "address": args["address"]}
        elif action == "navigate_to":
            data = {"targetPose": self._pose(args), "mode": int(args.get("mode", 1)), "speed": float(args.get("speed", 0.5))}
        else: data = {}
        action_id = None
        previous = None
        if action == "navigate_to":
            # Arm completion state before the RPC. The vendor can publish a very
            # fast task_result before _Call returns, so clearing the event after
            # the call loses that completion and leaves ACP waiting for 180s.
            with self._nav_lock:
                previous = self._nav_action_id
                action_id = f"as2w_nav_{uuid4().hex[:8]}"
                self._nav_action_id = action_id
                self._nav_done.clear()
                self._nav_result = None
            if previous:
                _acp_notify(previous, "cancelled", {"reason": "superseded by new navigation request"})

        result = self._client.call(action, data)
        response = result["response"]
        try: response = json.loads(response) if isinstance(response, str) else response
        except json.JSONDecodeError: pass
        if action != "navigate_to" or result["code"] != 0:
            if action == "navigate_to":
                with self._nav_lock:
                    if self._nav_action_id == action_id:
                        self._nav_action_id = None
                        self._nav_done.clear()
            return {"ret": result["code"], "response": response}
        threading.Thread(target=self._wait_for_navigation,
                         args=(action_id, data["targetPose"]), daemon=True).start()
        return {"ret": 0, "status": "navigating", "action_id": action_id,
                "target_pose": data["targetPose"], "response": response}
