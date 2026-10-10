"""Ordered locomotion writes with deadlines independent of RPC response waits."""
import os
import math
import json
import threading
import time
from collections import OrderedDict
from uuid import uuid4


def trace_event(stage, **data):
    try:
        record = {"stage": stage, "mono_s": time.monotonic(),
                  "pid": os.getpid(), "thread": threading.current_thread().name, **data}
        print(f"[LocoTrace] {json.dumps(record)}", flush=True)
    except Exception:
        pass


class TimedMotion:
    def __init__(self, client, *, timing=None, on_stop=None,
                 clock=time.monotonic, timer_factory=threading.Timer):
        self.client = client
        self.timing = timing
        self.on_stop = on_stop
        self.clock = clock
        self.timer_factory = timer_factory
        self.lock = threading.RLock()
        self.active = None
        self.actions = OrderedDict()

    def _log(self, stage, **data):
        if self.timing:
            self.timing.emit(stage, **data)

    def _send_log(self, method, action_id, started, ended):
        # Log local SDK send-call boundaries, never response arrival times.
        action = self.actions.get(action_id)
        record = {"action_id": action_id, "method": method,
                  "send_start_s": started, "send_end_s": ended,
                  "send_call_elapsed_s": ended - started}
        if action:
            initial = action.get("initial_move_send")
            if method == "Move" and initial is None:
                action["initial_move_send"] = (started, ended)
                initial = action["initial_move_send"]
            if initial is not None:
                record["from_initial_move_start_s"] = started - initial[0]
                record["from_initial_move_end_s"] = ended - initial[1]
                # Bounds on the gap between writes occurring inside each call.
                if method == "StopMove":
                    record["write_gap_lower_s"] = started - initial[1]
                    record["write_gap_upper_s"] = ended - initial[0]
            record["requested_duration_s"] = action["duration"]
            record["deadline_s"] = action["deadline"]
            if method == "StopMove":
                record["reason"] = action["reason"]
                record["stop_send_attempt"] = action.get("stop_send_attempt", 0) + 1
                action["stop_send_attempt"] = record["stop_send_attempt"]
                if action["deadline"] is not None:
                    record["stop_start_vs_deadline_s"] = started - action["deadline"]
                if action.get("timer_fired_s") is not None:
                    record["timer_lateness_s"] = action["timer_fired_s"] - action["deadline"]
                    record["timer_to_stop_call_s"] = started - action["timer_fired_s"]
        try:
            print(f"[LocoSend] {json.dumps(record)}", flush=True)
        except Exception:
            pass

    def _begin(self, method, action_id, *args):
        started = self.clock()
        self._log("rpc_enter", method=method, timing_id=action_id, call_start_s=started)
        try:
            wait = getattr(self.client, "Begin" + method)(*args)
        except Exception as exc:
            self._log("rpc_exception", method=method, timing_id=action_id,
                      exception=repr(exc), elapsed_s=self.clock() - started)
            raise
        request_id = getattr(wait, "request_id", None)
        sent = self.clock()
        self._send_log(method, action_id, started, sent)
        trace_event("rpc_bound", action_id=action_id, method=method, request_id=request_id)
        self._log("rpc_sent", method=method, timing_id=action_id,
                  send_elapsed_s=sent - started)

        def response():
            try:
                ret = wait()
                trace_event("motion_wait_return", action_id=action_id, method=method,
                            request_id=request_id, ret=ret)
            except Exception as exc:
                self._log("rpc_exception", method=method, timing_id=action_id,
                          exception=repr(exc), elapsed_s=self.clock() - started)
                raise
            self._log("rpc_return", method=method, timing_id=action_id,
                      ret=ret, elapsed_s=self.clock() - started)
            return ret
        return response

    def _outcome(self, action):
        if "outcome" in action:
            return {**action["outcome"], "result": dict(action["outcome"]["result"])}
        if action["status"] is None:
            return {"action_id": action["id"], "status": "pending"}
        return {"action_id": action["id"], "status": action["status"],
                "result": {"reason": action["reason"], "move_ret": action["move_ret"],
                           "stop_ret": action["stop_ret"], "update_ret": action["update_ret"],
                           "stop_acknowledged": action["stop_ret"] == 0,
                           "error": action.get("error")}}

    def _finish(self, action):
        if (action["status"] is not None or not action["move_done"] or
                not action["stop_done"] or action["updates_pending"]):
            return
        if (action["move_ret"] != 0 or action["stop_ret"] != 0 or
                action["reason"] == "move_failed" or action.get("update_error")):
            action["status"] = "error"
            action.setdefault("error", f"Motion failed: move_ret={action['move_ret']}, stop_ret={action['stop_ret']}")
        else:
            action["status"] = "completed" if action["reason"] == "duration_expired" else "cancelled"
        action["outcome"] = self._outcome(action)
        trace_event("outcome_ready", action_id=action["id"], status=action["status"])

    def _stopped(self, action):
        if self.on_stop and self.active is action:
            try:
                self.on_stop({"action_id": action["id"], "ret": action["stop_ret"],
                              "reason": action["reason"], "error": action.get("error")})
            except Exception as exc:
                self._log("stop_callback_exception", timing_id=action["id"], exception=repr(exc))

    def start(self, vx, vy, vyaw, duration, action_id=None):
        if not all(math.isfinite(v) for v in (vx, vy, vyaw, duration)):
            return {"error": "Motion parameters must be finite", "state": "idle"}
        action_id = action_id or f"g1_move_{uuid4().hex[:8]}"
        with self.lock:
            if action_id in self.actions:
                return {"error": "Duplicate motion action_id", "action_id": action_id}
            # Bound history, but keep unfinished actions until their replies arrive.
            for key in list(self.actions):
                if len(self.actions) < 256:
                    break
                if self.actions[key]["status"] is not None and self.actions[key] is not self.active:
                    del self.actions[key]
            if len(self.actions) >= 256:
                return {"error": "Too many retained motion actions", "action_id": action_id}
            old = self.active
            if old and old["status"] is None:
                if old["timer"]:
                    old["timer"].cancel()
                old["status"], old["reason"] = "cancelled", "replaced"
                old["outcome"] = self._outcome(old)
            started = self.clock()
            action = {"id": action_id, "duration": duration, "deadline": started + duration if duration > 0 else None,
                      "timer": None, "move_ret": None, "move_done": False,
                      "updates_pending": 0, "update_ret": None,
                      "stop_ret": None, "stop_done": False, "stopping": False,
                      "stop_event": threading.Event(), "status": None, "reason": None}
            self.active = action
            self.actions[action_id] = action
            self._log("handle_move_enter", timing_id=action_id, duration=duration,
                      requested_deadline_s=action["deadline"])
            try:
                # Arm before sending. The same lock prevents a timer/stop/new move
                # from overtaking this send, but is released before response wait.
                if duration > 0:
                    timer = self.timer_factory(max(0., action["deadline"] - self.clock()),
                                               self._expire, args=(action_id, action["deadline"]))
                    timer.daemon = True
                    action["timer"] = timer
                    timer.start()
                    self._log("timer_armed", timing_id=action_id,
                              timer_deadline_s=action["deadline"])
                wait = self._begin("Move", action_id, vx, vy, vyaw, True)
            except Exception as exc:
                wait = None
                action["error"] = repr(exc)
        ret = None
        try:
            if wait:
                ret = wait()
        except Exception as exc:
            action["error"] = repr(exc)
        with self.lock:
            action["move_ret"], action["move_done"] = ret, True
            self._finish(action)
        if ret != 0:
            # A timeout is ambiguous: the robot may have executed the command.
            self.stop("move_failed", expected_id=action_id)
        with self.lock:
            result = {"ret": ret, "action_id": action_id, "vx": vx, "vy": vy,
                      "vyaw": vyaw, "duration": duration,
                      "state": "superseded" if self.active is not action else
                      "moving" if not action["stopping"] else
                      "idle" if action["stop_ret"] == 0 else
                      "stop_failed" if action["stop_done"] else "stop_pending"}
            if ret != 0:
                result["error"] = action.get("error") or f"Move failed: code={ret}"
            if action["status"] is not None:
                result["motion_status"] = action["status"]
            return result

    def _expire(self, action_id, deadline):
        fired = self.clock()
        self._log("timer_fired", timing_id=action_id, timer_deadline_s=deadline,
                  fired_s=fired, lateness_s=fired - deadline)
        self.stop("duration_expired", expected_id=action_id, timer_fired_s=fired)

    def stop(self, reason="command", expected_id=None, retry=False, timer_fired_s=None):
        with self.lock:
            action = self.active
            if expected_id is not None and (action is None or action["id"] != expected_id):
                return {"state": "superseded", "action_id": expected_id}
            if action and timer_fired_s is not None:
                action["timer_fired_s"] = timer_fired_s
            owner = not action or not action["stopping"]
            # Explicit stops and guarded safety repeats send a fresh zero command.
            if action and action["stop_done"] and (expected_id is None or retry):
                owner = True
                action["stop_event"] = threading.Event()
                action["stop_done"] = False
            event = action["stop_event"] if action else None
            wait = None
            if owner:
                if action:
                    action["stopping"] = True
                    if action["status"] is None:
                        action["reason"] = reason
                    if action["timer"]:
                        action["timer"].cancel()
                try:
                    wait = self._begin("StopMove", action["id"] if action else None)
                except Exception as exc:
                    if action:
                        action["error"] = repr(exc)
        if not owner:
            event.wait()
            ret = action["stop_ret"]
        else:
            ret = None
            try:
                if wait:
                    ret = wait()
            except Exception as exc:
                if action:
                    action["error"] = repr(exc)
            with self.lock:
                if action:
                    action["stop_ret"], action["stop_done"] = ret, True
                    event.set()
                    self._finish(action)
                    self._stopped(action)
        with self.lock:
            state = "superseded" if self.active is not action else "idle" if ret == 0 else "stop_failed"
        result = {"ret": ret, "state": state, "reason": reason}
        if action:
            result["action_id"] = action["id"]
        if ret != 0:
            result["error"] = f"StopMove failed or unconfirmed: code={ret}"
        return result

    def update(self, action_id, vx, vy, vyaw):
        if not all(math.isfinite(v) for v in (vx, vy, vyaw)):
            return {"error": "Motion parameters must be finite"}
        with self.lock:
            action = self.active
            if not action or action["id"] != action_id or action["stopping"]:
                return {"state": "superseded"}
            expired = action["deadline"] is not None and self.clock() >= action["deadline"]
            wait = None
            if not expired:
                action["updates_pending"] += 1
                try:
                    wait = self._begin("Move", action_id, vx, vy, vyaw, True)
                except Exception as exc:
                    action["error"] = repr(exc)
        if expired:
            return self.stop("duration_expired", expected_id=action_id)
        ret = None
        try:
            if wait:
                ret = wait()
        except Exception as exc:
            action["error"] = repr(exc)
        with self.lock:
            action["updates_pending"] -= 1
            action["update_ret"] = ret
            if ret != 0:
                action["update_error"] = True
                action.setdefault("error", f"Speed update failed: code={ret}")
            self._finish(action)
        if ret != 0:
            self.stop("move_failed", expected_id=action_id)
        return {"ret": ret}

    def get_result(self, action_id):
        with self.lock:
            action = self.actions.get(action_id)
            return self._outcome(action) if action else {"action_id": action_id, "status": "error",
                                                       "result": {"error": "Unknown motion action"}}
