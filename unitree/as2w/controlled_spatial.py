"""As2W adapter for Unitree's documented ``slam_operate`` DDS service.

The SDK currently publishes no AS2-specific SLAM wrapper. The service itself
uses Unitree's common RPC protocol, so this module owns the documented client.
"""
import json
from contextlib import nullcontext
import multiprocessing
import queue
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
        if not isinstance(interface, str) or not interface.strip() or interface.strip().lower() == "auto":
            raise ValueError("SLAM requires an explicitly resolved robot interface")
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface.strip())
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
            results.put({"request_id": command["request_id"], "code": code, "response": response})
        except Exception as exc:
            results.put({"request_id": command["request_id"], "code": 3104, "response": str(exc)})


class _SpatialRpcProxy:
    def __init__(self, interface):
        context = multiprocessing.get_context("spawn")
        self._commands, self._results = context.Queue(), context.Queue()
        self._process = context.Process(target=_worker, args=(self._commands, self._results, interface), daemon=True)
        self._process.start()
        self._lock = threading.Lock()
        self._startup_error = None
        self._stopped = False
        self._next_request_id = 0
        try:
            result = self._results.get(timeout=5)
            self._startup_error = result.get("startup_error")
        except Exception:
            self._startup_error = "SLAM worker did not become ready"

    def call(self, action, data):
        with self._lock:
            if self._stopped or self._startup_error:
                return {"code": 3104, "response": self._startup_error or "SLAM worker is stopped"}
            self._next_request_id += 1
            request_id = self._next_request_id
            self._commands.put({"request_id": request_id, "action": action, "data": data})
            deadline = time.monotonic() + 20
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return {"code": 3104, "response": "SLAM service timeout"}
                try:
                    result = self._results.get(timeout=remaining)
                except queue.Empty:
                    return {"code": 3104, "response": "SLAM service timeout"}
                # A timed-out navigation reply is not an acknowledgement of a
                # later pause. Only the matching command may release ownership.
                if isinstance(result, dict) and result.get("request_id") == request_id:
                    return result

    def stop(self):
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._commands.put(None)
            self._process.join(timeout=3)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1)


class ControlledSpatialPlugin:
    """Low-level map, relocalization, and navigation backed by vendor SLAM."""
    PREFIX = "controlled_spatial"

    def __init__(self, config, namespace, executor, interface):
        self._interface = interface
        self._client = None
        self._nav_sub = None
        self._running = False
        self._subscription_generation = 0
        self._nav_done = threading.Event()
        self._nav_result = None
        self._nav_action_id = None
        self._nav_target = None
        self._paused_target = None
        self._nav_lock = threading.Lock()
        self._chassis_guard = None
        self._chassis_reserved = False
        self._last_navigation_result = None
        self._last_stop_acknowledged = None
        self._operation_lock = threading.RLock()
        self.start()

    def _open_completion_subscription(self):
        self._subscription_generation += 1
        generation = self._subscription_generation
        subscriber = None
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            subscriber = ChannelSubscriber("rt/slam_key_info", String_)
            # A queued DDS callback from the previous subscription must not
            # complete a navigation request started after a stop/start cycle.
            def on_message(message):
                self._on_slam_key_info(message, generation)
            subscriber.Init(on_message, 10)
            self._nav_sub = subscriber
        except Exception as exc:
            if subscriber is not None:
                try:
                    subscriber.Close()
                except Exception:
                    pass
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
                # The 180-second navigation deadline leaves time for the
                # bounded terminal pause RPC before ACP itself times out.
                "x-completion": {"actions": ["navigate_to", "resume_navigation"], "timeout": 210},
                "x-action-params": {
                    "start_mapping": {"params": [], "description": "Start indoor SLAM mapping."},
                    "stop_mapping": {"params": ["address"], "description": "Stop mapping and save PCD."},
                    "init_pose": {"params": ["address", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"], "description": "Load map and initialize pose."},
                    "navigate_to": {"params": ["x", "y", "z", "q_x", "q_y", "q_z", "q_w", "speed", "mode"], "description": "Navigate to a target pose."},
                    "pause_navigation": {"params": [], "description": "Pause navigation."},
                    "resume_navigation": {"params": [], "description": "Resume a goal paused by this card, with a new tracked action and timeout."},
                    "shutdown": {"params": [], "description": "Close vendor SLAM service."}}}}

    def set_chassis_guard(self, guard):
        self._chassis_guard = guard
        if not hasattr(self, "_operation_lock"):
            self._operation_lock = threading.RLock()

    def _release_chassis(self):
        guard = getattr(self, "_chassis_guard", None)
        if guard is not None:
            guard.release_external_navigation()
        self._chassis_reserved = False

    def _call_vendor(self, action, data):
        try:
            result = self._client.call(action, data)
            if not isinstance(result, dict):
                raise ValueError("invalid vendor RPC result")
            return result
        except Exception as exc:
            return {"code": 3104, "response": f"{type(exc).__name__}: {exc}"}

    @staticmethod
    def _accepted(result):
        code = result.get("code")
        return isinstance(code, int) and not isinstance(code, bool) and code == 0

    def _acknowledge_stop(self, action="pause_navigation"):
        result = self._call_vendor(action, {})
        self._last_stop_acknowledged = self._accepted(result)
        if self._last_stop_acknowledged:
            self._release_chassis()
        return {"stop_acknowledged": self._last_stop_acknowledged,
                "chassis_reserved": getattr(self, "_chassis_reserved", False),
                "vendor_stop": result}

    def start(self):
        with self._operation_lock:
            if self._client is None:
                self._client = _SpatialRpcProxy(self._interface)
            self._running = True
            if self._nav_sub is None:
                self._open_completion_subscription()
            return self._lifecycle_info()

    def _lifecycle_info(self):
        return {"state": "ready" if self._running else "error" if self._chassis_reserved else "idle",
                "completion_available": self._nav_sub is not None,
                "chassis_reserved": self._chassis_reserved,
                "stop_acknowledged": self._last_stop_acknowledged,
                "navigation_state": "navigating" if self._nav_action_id else
                    "reserved_without_active_action" if self._chassis_reserved else "idle",
                "requires_vendor_pause": bool(self._chassis_reserved and not self._nav_action_id),
                "can_resume": self._paused_target is not None,
                "last_navigation_result": self._last_navigation_result,
                "completion_policy": "task_result triggers vendor pause; arrival is not goal-correlated"}

    def stop(self):
        with self._operation_lock:
            return self._stop()

    def _stop(self):
        self._running = False
        with self._nav_lock:
            action_id, self._nav_action_id = self._nav_action_id, None
            self._nav_target = self._paused_target = None
            self._subscription_generation += 1
            self._nav_done.set()
        # An RPC timeout or terminal result may have cleared the action id
        # while the vendor still owns the chassis. Teardown is not a stop ACK.
        stop_result = None
        if action_id or getattr(self, "_chassis_reserved", False):
            if self._client is None:
                try:
                    self._client = _SpatialRpcProxy(self._interface)
                except Exception:
                    # _call_vendor reports the unavailable client and keeps
                    # the reservation instead of pretending teardown stopped it.
                    pass
            stop_result = self._acknowledge_stop()
        if action_id:
            _acp_notify(action_id, "cancelled" if stop_result["stop_acknowledged"] else "error",
                        {"reason": "card stopped", **stop_result})
        if self._nav_sub is not None:
            subscriber, self._nav_sub = self._nav_sub, None
            try:
                subscriber.Close()
            except Exception:
                pass
        if self._client is not None:
            client, self._client = self._client, None
            client.stop()
        return {**self._lifecycle_info(), "ok": not self._chassis_reserved,
                **({"error": "vendor stop unconfirmed; chassis reservation retained"}
                   if self._chassis_reserved else {}), **(stop_result or {})}

    def _on_slam_key_info(self, message, generation=None):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, AttributeError):
            return
        if isinstance(payload, dict) and payload.get("type") == "task_result" and isinstance(payload.get("data"), dict):
            with self._nav_lock:
                if generation is not None and generation != self._subscription_generation:
                    return
                if self._nav_action_id is not None:
                    self._nav_result = payload
                    self._nav_done.set()

    def _wait_for_navigation(self, action_id, target, done=None):
        completed = (done if done is not None else self._nav_done).wait(timeout=180)
        with getattr(self, "_operation_lock", nullcontext()):
            with self._nav_lock:
                if self._nav_action_id != action_id:
                    return
            if completed:
                result = self._nav_result or {}
                stopped = self._acknowledge_stop()
                # The observed vendor message has no goal identity. It can
                # request a conservative stop, but cannot certify this target's
                # arrival. Report the limit instead of completing the wrong goal.
                report = {"target": target, "response": result,
                          "arrival_reported": result.get("data", {}).get("is_arrived") is True,
                          "arrival_verified": False, **stopped,
                          "reason": "uncorrelated_vendor_task_result",
                          "error": "task_result has no verified goal identity; arrival unverified"}
                if not stopped["stop_acknowledged"]:
                    report["error"] += "; vendor stop unconfirmed and chassis remains reserved"
            else:
                report = {"target": target, "error": "navigation timed out after 180 seconds",
                          **self._acknowledge_stop()}
            with self._nav_lock:
                self._nav_action_id = None
                self._nav_target = None
            self._last_navigation_result = report
            _acp_notify(action_id, "error", report)

    @staticmethod
    def _pose(args):
        return {key: float(args.get(key, default)) for key, default in {
            "x": 0, "y": 0, "z": 0, "q_x": 0, "q_y": 0, "q_z": 0, "q_w": 1}.items()}

    def dispatch(self, action, args):
        # Serialize reservations and their RPC outcomes as one transaction.
        # A concurrent resume must not sit behind an acknowledged pause while
        # that pause has already released the chassis to the visual navigator.
        with getattr(self, "_operation_lock", nullcontext()):
            return self._dispatch(action, args)

    def _dispatch(self, action, args):
        if action == "start": return self.start()
        if action == "info": return self._lifecycle_info()
        if action == "stop":
            return self.stop()
        if action not in _APIS: return None
        if not getattr(self, "_running", True):
            return {"ret": 3104, "accepted": False,
                    "error": "card stopped; call start before using vendor SLAM"}
        if action == "resume_navigation" and getattr(self, "_paused_target", None) is None:
            return {"ret": 3104, "accepted": False,
                    "error": "no tracked paused goal; call navigate_to with a target"}
        if action in ("navigate_to", "resume_navigation") and hasattr(self, "_nav_sub") and self._nav_sub is None:
            return {"ret": 3104, "accepted": False,
                    "error": "SLAM completion topic unavailable; call start to retry subscription"}
        if action in ("stop_mapping", "init_pose") and not args.get("address"):
            return {"error": "address is required for this action"}
        if action == "start_mapping": data = {"slam_type": "indoor"}
        elif action == "stop_mapping": data = {"address": args["address"]}
        elif action == "init_pose": data = {**self._pose(args), "address": args["address"]}
        elif action == "navigate_to":
            data = {"targetPose": self._pose(args), "mode": int(args.get("mode", 1)), "speed": float(args.get("speed", 0.5))}
        else: data = {}
        if action in ("navigate_to", "resume_navigation"):
            guard = getattr(self, "_chassis_guard", None)
            if guard is not None and not guard.reserve_external_navigation():
                return {"ret": 3104, "accepted": False,
                        "error": "another controller owns or is moving the chassis; stop it first"}
            self._chassis_reserved = True
            self._last_stop_acknowledged = None
        action_id = None
        previous = None
        if action in ("navigate_to", "resume_navigation"):
            # Arm completion state before the RPC. The vendor can publish a very
            # fast task_result before _Call returns, so clearing the event after
            # the call loses that completion and leaves ACP waiting for 180s.
            with self._nav_lock:
                previous = self._nav_action_id
                action_id = f"as2w_nav_{uuid4().hex[:8]}"
                self._nav_action_id = action_id
                target = data["targetPose"] if action == "navigate_to" else dict(self._paused_target)
                self._nav_target = target
                self._paused_target = None
                self._nav_done.set()
                self._nav_done = threading.Event()
                self._nav_result = None
            if previous:
                _acp_notify(previous, "cancelled", {"reason": "superseded by new navigation request"})

        if action in ("pause_navigation", "shutdown"):
            stop_state = self._acknowledge_stop(action)
            result = stop_state["vendor_stop"]
        else:
            result = self._call_vendor(action, data)
        # Keep ownership on timeout/error: the vendor may have accepted an RPC
        # whose reply was lost. Only an explicit accepted pause/shutdown clears
        # it; an uncorrelated completion topic cannot safely release a new goal.
        if action in ("pause_navigation", "shutdown") and self._accepted(result):
            with self._nav_lock:
                cancelled, self._nav_action_id = self._nav_action_id, None
                if action == "shutdown":
                    self._paused_target = None
                elif cancelled and self._nav_target is not None:
                    self._paused_target = dict(self._nav_target)
                self._nav_target = None
                self._nav_done.set()
            if cancelled:
                _acp_notify(cancelled, "cancelled", {"reason": action,
                            "stop_acknowledged": True, "chassis_reserved": False})
        response = result.get("response")
        try: response = json.loads(response) if isinstance(response, str) else response
        except json.JSONDecodeError: pass
        if action not in ("navigate_to", "resume_navigation") or not self._accepted(result):
            if action in ("navigate_to", "resume_navigation"):
                with self._nav_lock:
                    if self._nav_action_id == action_id:
                        self._nav_action_id = None
                        self._nav_target = None
                        self._nav_done.clear()
            return {"ret": result.get("code", 3104), "response": response,
                    "chassis_reserved": getattr(self, "_chassis_reserved", False),
                    "stop_acknowledged": getattr(self, "_last_stop_acknowledged", None)}
        threading.Thread(target=self._wait_for_navigation,
                         args=(action_id, target, self._nav_done), daemon=True).start()
        return {"ret": 0, "status": "navigating", "action_id": action_id,
                "target_pose": target, "response": response,
                "chassis_reserved": True, "arrival_verified": False,
                "completion_policy": "task_result triggers vendor pause; arrival is not goal-correlated"}
