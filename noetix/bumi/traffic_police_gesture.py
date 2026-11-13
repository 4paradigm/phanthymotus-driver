"""Two semantic labels over LocoPlugin's existing teaching-action execution.

This module owns no joint trajectory or motor publisher. PLAYTEACH and WALK
remain in _do_teaching_action. The SDK confirms modes, not physical completion;
retain the body's reservation until explicit cancel/exit and stationary checks.
"""
from collections import OrderedDict, deque
from copy import deepcopy
import logging
import math
import threading
import time
import uuid

log = logging.getLogger(__name__)
DEFAULT_PLAYBACK_TIMEOUT_S = 60.0


class _NotificationDelivery:
    """One lazy daemon, bounded snapshots, and no access to robot safety state."""
    MAX_PENDING = 16
    MAX_RECORDS = 32

    def __init__(self, sender):
        self.sender = sender
        self.condition = threading.Condition()
        self.queue = deque()
        self.records = OrderedDict()
        self.worker = None
        self.in_flight = None
        self.closed = False

    def info(self):
        with self.condition:
            return {"queued": len(self.queue), "in_flight": self.in_flight,
                    "closed": self.closed, "records": deepcopy(list(self.records.values()))}

    def schedule(self, action_id, status, result, tool):
        work = (action_id, status, deepcopy(result), tool)
        with self.condition:
            record = {"action_id": action_id, "status": status, "state": "pending", "error": None}
            self.records[action_id] = record
            if self.closed or len(self.queue) >= self.MAX_PENDING:
                record.update(state="failed", error="Notification delivery closed" if self.closed
                              else "Notification queue capacity reached")
                log.warning("ACP delivery not scheduled for %s: %s", action_id, record["error"])
            else:
                self.queue.append(work)
                if self.worker is None:
                    try:
                        worker = threading.Thread(target=self._run, daemon=True,
                                                  name="bumi_gesture_acp_delivery")
                        worker.start()
                        self.worker = worker
                    except Exception as exc:
                        while self.queue:
                            pending_id = self.queue.popleft()[0]
                            self.records[pending_id].update(state="failed", error=str(exc))
                        log.exception("Gesture ACP delivery worker could not start")
                self.condition.notify()
            # Pending/in-flight records are never evicted. At most 17 exist;
            # the remainder is a bounded diagnostic tail, not a pending ledger.
            while len(self.records) > self.MAX_RECORDS:
                key = next(key for key, item in self.records.items()
                           if item["state"] in ("sent", "failed"))
                del self.records[key]

    def _run(self):
        while True:
            with self.condition:
                while not self.queue:
                    if self.closed:
                        return
                    self.condition.wait()
                action_id, status, result, tool = self.queue.popleft()
                self.in_flight = action_id
                self.records[action_id]["state"] = "sending"
            # No delivery lock, gesture lock, or operation lock surrounds I/O.
            error = None
            try:
                receipt = self.sender(action_id, status, result, tool)
                if receipt is False or (isinstance(receipt, dict) and receipt.get("state") == "failed"):
                    error = (receipt.get("error") if isinstance(receipt, dict) else None) or "ACP HTTP delivery failed"
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            with self.condition:
                self.records[action_id].update(state="failed" if error else "sent", error=error)
                self.in_flight = None
                self.condition.notify_all()
            if error:
                log.warning("Gesture ACP delivery failed for %s: %s", action_id, error)

    def close(self):
        # Drain already queued work while this process lives, but never join
        # network I/O during safety shutdown. Process exit remains best-effort.
        with self.condition:
            if self.closed:
                return
            self.closed = True
            pending = len(self.queue) + (self.in_flight is not None)
            self.condition.notify_all()
        if pending:
            log.warning("Gesture shutdown leaves %s ACP notification(s) best-effort; not waiting for network", pending)


class TrafficPoliceGesture:
    NAME = "traffic_police_gesture"

    def __init__(self, owner, config, notify):
        self.owner, self.notify = owner, notify
        self.config = deepcopy(config) if isinstance(config, dict) else {}
        self.enabled = self.config.get("enabled", True) is not False
        self.lock = owner._rotate_lock
        self.operation_lock = threading.Lock()
        self.active = None
        self.last_result = None
        self.closed = False
        self.worker = None
        # Lifecycle readiness is independent of a firmware playback operation.
        # Guard both fields with the existing chassis RLock.
        self._started = False
        self._stop_requests = 0
        self._delivery = _NotificationDelivery(notify)

    def get_tool(self):
        return {"name": self.NAME, "type": "actuator", "multiInstance": False,
                "description": "Bumi交警手势：stop=停车手势、go=放行手势，播放配置的示教编号。"
                    "业务动作须execute=true；不带execute的stop仅用于生命周期取消，绝不是停车手势。"
                    "SDK没有物理完成事件，须人工观察并cancel退出；不允许底盘运动。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["stop", "go", "info", "cancel"]},
                    "execute": {"type": "boolean", "const": True,
                                "description": "显式业务动作标记；stop/go必须true，避免项目停止触发停车手势"}},
                    "required": ["action"],
                    "x-action-params": {
                        "stop": {"params": ["execute"], "required": ["execute"],
                                 "description": "execute=true播放配置的停车手势，不是底盘stop_move"},
                        "go": {"params": ["execute"], "required": ["execute"],
                               "description": "execute=true播放配置的放行手势；必须明确目标方向和通行语义"},
                        "info": {"params": [], "description": "只读查看配置与活动手势，不能证明物理动作完成"},
                        "cancel": {"params": [], "description": "复用stop_playback退出，核对walking及停稳后释放身体；不表示手势物理完成"}},
                    "x-hooks": {"on_interrupt_motion": {"action": "cancel"},
                                "on_interrupt_all": {"action": "cancel"}},
                    "x-resource": ["base", "arm"],
                    "x-stop": {"pending_flags": {"busy": True}, "failed_states": ["locked"]},
                    # A physical-completion event does not exist. Background ACP
                    # permits info/cancel. The declared timeout is metadata:
                    # Core background pending expiry is a separate platform
                    # issue; it is not a reliable callback-loss fallback.
                    "x-completion": {"actions": ["stop", "go"], "background": True,
                                     "timeout": (self._timeout() or DEFAULT_PLAYBACK_TIMEOUT_S)+15,
                                     "silent_completion": True}},
                "topic_out": []}

    def _timeout(self):
        value = self.config.get("playback_timeout_s", DEFAULT_PLAYBACK_TIMEOUT_S)
        return float(value) if (type(value) in (int, float) and math.isfinite(value)
                                and 1 <= value <= 300) else None

    @staticmethod
    def _error(reason, message, **extra):
        return {"state": "error", "reason": reason, "error": message,
                "command_sent": False, **extra}

    def _recording(self, gesture):
        key = gesture + "_recording_id"
        value = self.config.get(key)
        if value is None:
            return None, self._error("gesture_not_configured", f"Missing {key}",
                                      gesture=gesture, missing_config=[key])
        if type(value) is not int or not 0 <= value <= 65535:
            return None, self._error("invalid_recording_id", f"{key} must be an integer within 0..65535",
                                      gesture=gesture, config_field=key)
        return value, None

    def info(self):
        with self.lock:
            active = self.active
            gesture_state = active["state"] if active else "idle"
            error = active.get("error") if active else None
            if not self.enabled:
                state, error = "error", "traffic_police_gesture is disabled"
            elif gesture_state == "locked":
                state, error = "error", error or "Playback exit remains unconfirmed"
            elif self._stop_requests:
                state = "stopping"
            else:
                state = "ready" if self._started and not self.closed else "idle"
            return {"state": state, "started": self._started, "closed": self.closed,
                    "gesture_state": gesture_state, "busy": active is not None,
                    "stop_configured": self._recording("stop")[1] is None,
                    "go_configured": self._recording("go")[1] is None,
                    "active_gesture": active["gesture"] if active else None,
                    "action_id": active["action_id"] if active else None,
                    "recording_id": active["recording_id"] if active else None,
                    "last_result": deepcopy(self.last_result),
                    "physical_completion_available": False, "physical_completion_confirmed": False,
                    "completion_note": "SDK only reports workmode. Observe the motion, then cancel to exit; never infer physical completion from running or mode changes.",
                    "recording_validation": "ID type/range only; the SDK has no recording catalog or trajectory validation API",
                    "error": error,
                    "last_error": error or (self.last_result or {}).get("error"),
                    "acp_notifications": self._delivery.info()}

    def start(self):
        """Declare an on-demand Card ready; never inspect or command the SDK."""
        with self.lock:
            if not self.enabled:
                return self._error("card_disabled", "traffic_police_gesture is disabled")
            if self.closed:
                return self._error("shutting_down", "Gesture controller is shutting down")
            if self._stop_requests:
                return self._error("lifecycle_stopping", "Wait for lifecycle stop to finish")
            if self.active is not None and self.active["state"] == "locked":
                return self.info()
            self._started = True
            return self.info()

    def stop(self):
        """Lifecycle stop disarms readiness and reuses the existing cancel path."""
        with self.lock:
            self._started = False
            self._stop_requests += 1
        try:
            return self.cancel()
        finally:
            with self.lock:
                self._stop_requests -= 1

    def dispatch(self, action, args):
        if action == "info":
            return self.info()
        if action == "start":
            return self.start()
        if action == "stop" and "execute" not in args:
            return self.stop()
        if action == "cancel":
            return self.cancel()
        if action in ("stop", "go"):
            if args.get("execute") is not True:
                return self._error("execute_required", "Business stop/go requires execute=true", gesture=action)
            return self.play(action)
        return None

    def _preflight(self):
        owner = self.owner
        if not self.enabled:
            return self._error("card_disabled", "traffic_police_gesture is disabled")
        if self.closed:
            return self._error("shutting_down", "Gesture controller is shutting down")
        if self._stop_requests:
            return self._error("lifecycle_stopping", "Lifecycle stop is in progress")
        if self.active is not None:
            return self._error("gesture_busy", "Cancel the existing gesture before another playback",
                               active_action_id=self.active["action_id"])
        if owner._rotate_state["state"] == "running":
            return self._error("rotation_busy", "Rotation has not stopped; gesture was not sent")
        if owner._move_thread and owner._move_thread.is_alive():
            return self._error("chassis_busy", "Chassis translation is active")
        if owner._servo is not None and owner._servo._info().get("state") not in ("idle", "paused"):
            return self._error("servo_conflict", "Stop or pause the existing servo policy first")
        try:
            # Reuse the existing mode/fault/IMU/joint stationary checks. This
            # cannot certify foot contact or recording contents; verify on site.
            owner._observation._snapshot()
        except Exception as exc:
            return self._error("unsafe_robot_state", str(exc))
        return None

    def play(self, gesture):
        recording, refusal = self._recording(gesture)
        if refusal:
            return refusal
        timeout = self._timeout()
        if timeout is None:
            return self._error("invalid_timeout", "playback_timeout_s must be finite within 1..300")
        # Do not queue a second gesture behind playback startup or cancellation.
        if not self.operation_lock.acquire(blocking=False):
            return self._error("gesture_busy", "A gesture operation is already in progress")
        operation = None
        try:
            operation = {"gesture": gesture, "recording_id": recording,
                         "action_id": "bumi_traffic_police_gesture_"+uuid.uuid4().hex,
                         "owner": self.NAME+"."+gesture, "state": "starting",
                         "cancel": threading.Event(), "notified": False, "async": False,
                         "error": None}

            def reserved():
                self.active = operation
                # Preserve direct business calls: successful admission can also
                # lazily ready the Card, without bypassing the body preflight.
                self._started = True

            try:
                receipt = self.owner._execute_body_action(
                    self.NAME, gesture,
                    lambda: self.owner._do_teaching_action("play_recording", {"recording_id": recording}),
                    retain=True, preflight=self._preflight, on_reserved=reserved)
            except Exception as exc:
                receipt = {"state": "error", "error": str(exc), "command_sent": None}
            with self.lock:
                if self.active is not operation:
                    return {**receipt, "gesture": gesture, "recording_id": recording}
                operation["receipt"] = deepcopy(receipt)
                if receipt.get("state") not in ("running", "accepted"):
                    if receipt.get("command_sent") is False:
                        self.owner._release_body_action(operation["owner"])
                        self.active = None
                        self.last_result = {**receipt, "gesture": gesture, "recording_id": recording}
                        return deepcopy(self.last_result)
                    operation["error"] = receipt.get("error") or "playback start failed"
                else:
                    operation["state"] = "stopping" if operation["cancel"].is_set() else receipt["state"]
            if operation["error"]:
                return self._cancel_locked(operation, "error", operation["error"])
            try:
                self.worker = threading.Thread(target=self._watch, args=(operation, float(timeout)),
                                               name="bumi_traffic_gesture_watchdog", daemon=True)
                operation["worker"] = self.worker
                operation["async"] = True
                self.worker.start()
            except Exception as exc:
                operation["async"] = False
                return self._cancel_locked(operation, "error", "watchdog failed: "+str(exc))
            return {**receipt, "state": operation["state"], "gesture": gesture,
                    "recording_id": recording, "action_id": operation["action_id"],
                    "physical_completion_confirmed": False, "playback_timeout_s": timeout,
                    "next_action": "Inspect info; visually verify the gesture, then cancel to exit playback. No automatic physical-completed event exists."}
        finally:
            self.operation_lock.release()
            if operation is not None and "terminal_result" in operation:
                self._notify_once(operation)

    def _notify_once(self, operation):
        with self.lock:
            if operation["notified"]:
                return
            operation["notified"] = True
            # A synchronous startup error has no pending ACP registration.
            # Core registers ANY returned action_id after the MCP response;
            # an early terminal callback followed by that ID would orphan it.
            if not operation["async"]:
                return
            status, result = operation["terminal_status"], deepcopy(operation["terminal_result"])
        self._delivery.schedule(operation["action_id"], status, result, self.NAME)

    def _cancel_locked(self, operation, terminal, reason):
        operation["cancel"].set()
        exited = False
        try:
            receipt = self.owner._execute_body_action(
                self.NAME, "cancel", lambda: self.owner._do_teaching_action("stop_playback", {}),
                retain=True, owned_by=operation["owner"])
            exited = receipt.get("state") == "completed" and receipt.get("confirmed") is True
            stationary, detail = self.owner._observation._settled() if exited else (False, "Playback exit was not confirmed")
        except Exception as exc:
            receipt, stationary, detail = {"state": "error", "error": str(exc)}, False, str(exc)
        confirmed = exited and stationary
        result = {"state": "idle" if confirmed and terminal == "cancelled" else "error" if confirmed else "locked",
                  "gesture": operation["gesture"], "recording_id": operation["recording_id"],
                  "action_id": operation["action_id"], "playback_exit_confirmed": confirmed,
                  "physical_completion_confirmed": False, "reason": reason,
                  "playback_result": receipt, "error": None if confirmed and terminal == "cancelled" else detail or reason}
        with self.lock:
            if confirmed and self.owner._release_body_action(operation["owner"]):
                if self.active is operation:
                    self.active = None
            else:
                confirmed = False
                result.update(state="locked", error=result["error"] or "Body ownership release was not confirmed")
            operation.update(state=result["state"], error=result["error"])
            self.last_result = deepcopy(result)
            # A later safe-exit retry may change local locked/idle state, but
            # cannot replace this action's already committed ACP terminal.
            if "terminal_result" not in operation:
                operation["terminal_status"] = terminal if confirmed else "error"
                operation["terminal_result"] = deepcopy(result)
        if not operation["async"]:
            result.pop("action_id", None)
        return result

    def cancel(self, *, expected=None, terminal="cancelled", reason="playback_exit_requested"):
        with self.lock:
            operation = self.active
            if operation is None or (expected is not None and operation is not expected):
                return {"state": "idle", "busy": False, "active_gesture": None}
            operation["cancel"].set()
            operation["state"] = "stopping"
        with self.operation_lock:
            with self.lock:
                if self.active is not operation:
                    return {"state": "idle", "busy": False, "active_gesture": None}
            result = self._cancel_locked(operation, terminal, reason)
        self._notify_once(operation)
        worker = operation.get("worker")
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            worker.join(timeout=1)
        return result

    def _watch(self, operation, timeout):
        deadline = time.monotonic()+timeout
        try:
            while not operation["cancel"].wait(min(.1, max(0., deadline-time.monotonic()))):
                if time.monotonic() >= deadline:
                    self.cancel(expected=operation, terminal="error", reason="playback_timeout")
                    return
                mode = int(self.owner._high_ctrl.get_mode())
                if mode not in (2, 23):
                    raise RuntimeError(f"Unexpected playback workmode={mode}")
                if mode == 2:
                    with self.lock:
                        if self.active is operation and not operation["cancel"].is_set():
                            operation["state"] = "awaiting_verification"
        except Exception as exc:
            self.cancel(expected=operation, terminal="error", reason="playback_feedback_failed: "+str(exc))

    def shutdown(self):
        with self.lock:
            self.closed = True
            self._started = False
        result = self.cancel(reason="shutdown")
        worker = self.worker
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            worker.join(timeout=1)
        self._delivery.close()
        return result
