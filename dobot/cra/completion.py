"""Bounded joint-motion monitoring and Agent Core ACP completion delivery."""
from __future__ import annotations

import json
import os
import ssl
import threading
import time
from urllib.request import Request, urlopen
from uuid import uuid4

if __package__:
    from .feedback import positive_number
else:
    from feedback import positive_number


class MotionCompletion:
    def __init__(self, control, config, notify=None):
        self.control = control
        settings = config.get("joint_move", {})
        self.timeout = positive_number(settings.get("motion_timeout_s", 300), "joint_move.motion_timeout_s")
        self.tolerance = positive_number(settings.get("target_tolerance_deg", 0.5), "joint_move.target_tolerance_deg")
        self.poll_interval = 0.05
        self._lock = threading.Lock()
        self._active = None
        self._jobs = {}
        self._last_target = None
        self.last_completion = None
        self._notify = notify
        # Includes preparation, each bounded SDK request, and callback delivery.
        self.contract_timeout = self.timeout + 8 * control.enable_timeout + 10 * control.receive_timeout + control.connect_timeout + 30

    @staticmethod
    def _callback_context():
        ca = os.environ.get("AGENT_CORE_CA_CERT")
        if not ca:
            raise RuntimeError("AGENT_CORE_CA_CERT is required for motion completion callbacks")
        return ssl.create_default_context(cafile=ca)

    def start(self, action, args):
        if args.get("confirm_move") is not True:
            raise ValueError("confirm_move=true is required for each motion request")
        # Fail before transmitting motion when callback TLS is not configured.
        if action not in ("move_j", "rel_joint_mov_j", "resume", "pause", "stop_motion"):
            raise ValueError(f"unsupported joint motion action: {action}")
        try:
            context = None if self._notify is not None else self._callback_context()
        except Exception:
            if action != "stop_motion":
                raise
            # Explicit Stop must remain available even with broken callback TLS.
            context = None
        with self._lock:
            interrupting = self._active is not None
            if self._active is not None:
                if action not in ("pause", "stop_motion"):
                    raise RuntimeError("another joint_move action is active")
                self._active["cancel"].set()
            action_id = f"dobot_cra_{action}_{uuid4().hex}"
            active = {"action_id": action_id, "cancel": threading.Event(), "interrupting": interrupting}
            self._active = active
            self._jobs[action_id] = active
            thread = threading.Thread(target=self._run, args=(active, action, dict(args), context), daemon=True)
            active["thread"] = thread
            try:
                thread.start()
            except Exception:
                self._active = None
                self._jobs.pop(action_id, None)
                raise
        return {"state": "running", "action_id": action_id}

    def interrupt(self):
        with self._lock:
            if self._active is not None:
                self._active["cancel"].set()

    def shutdown(self):
        with self._lock:
            jobs = list(self._jobs.values())
            for job in jobs:
                job["cancel"].set()
        for job in jobs:
            job["thread"].join(timeout=self.contract_timeout)

    def _run(self, active, action, args, context):
        status, result = "error", {"reason": "unknown"}
        submitted = False
        try:
            response = self.control.execute_motion(action, args, cancel=active["cancel"])
            submitted = True
            sample = self.control.feedback.snapshot()
            baseline = sample.get("packets_received")
            target = response["target_joint_angles_deg"] if action in ("move_j", "rel_joint_mov_j") else self._last_target
            if action in ("move_j", "rel_joint_mov_j"):
                self._last_target = target
            if action in ("pause", "stop_motion"):
                result = self._monitor_control(action, active["cancel"], baseline)
                if action == "stop_motion":
                    self._last_target = None
            else:
                result = self._monitor(active["cancel"], target, response.get("result_id"), baseline)
                self._last_target = None
            status = "completed"
        except Exception as exc:
            status = "cancelled" if active["cancel"].is_set() else "error"
            result = {"reason": str(exc)}
            must_stop = (submitted and action in ("move_j", "rel_joint_mov_j", "resume")) or (action == "pause" and active["interrupting"])
            if status == "error" and must_stop:
                try:
                    self.control.execute_motion("stop_motion", {"confirm_move": True})
                except Exception as stop_error:
                    result["stop_error"] = str(stop_error)
        finally:
            with self._lock:
                if active["cancel"].is_set():
                    status, result = "cancelled", {"reason": "motion interrupted"}
                if self._active is active:
                    self._active = None
            try:
                self._complete(active["action_id"], status, result, context)
            finally:
                with self._lock:
                    self._jobs.pop(active["action_id"], None)

    def _monitor_control(self, action, cancel, baseline):
        deadline = time.monotonic() + self.timeout
        last_packet = baseline
        stable_packets = 0
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("motion interrupted")
            sample = self.control.feedback.snapshot()
            state = sample.get("data")
            if not sample.get("connected") or sample.get("stale") or state is None:
                raise RuntimeError("command sent but fresh feedback cannot confirm pause/stop")
            packet = sample.get("packets_received")
            if packet is not None and last_packet is not None and packet > last_packet:
                last_packet = packet
                velocities = state["joint_state"]["qd_actual"]
                stopped = (len(velocities) == 6 and all(abs(v) <= 0.1 for v in velocities)
                           and not state["running_status"] and not state["jog_status"]
                           and state["robot_mode"] in (3, 4, 5, 9, 10, 11))
                paused = state["robot_mode"] == 10 and bool(state["pause_cmd_flag"])
                confirmed = stopped and (paused if action == "pause" else not state["run_queued_cmd"])
                stable_packets = stable_packets + 1 if confirmed else 0
                if stable_packets >= 2:
                    return {"reason": "paused" if action == "pause" else "stopped", "robot_mode": state["robot_mode"]}
            cancel.wait(self.poll_interval)
        raise TimeoutError("pause/stop confirmation deadline exceeded")

    def _monitor(self, cancel, target, queue_id, baseline):
        deadline = time.monotonic() + self.timeout
        observed_running = False
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("motion interrupted")
            sample = self.control.feedback.snapshot()
            state = sample.get("data")
            if not sample.get("connected") or sample.get("stale") or state is None:
                raise RuntimeError("feedback unavailable during motion")
            if (state["robot_mode"] in (9, 11) or state["error_status"] or state["collision_state"] or
                    state["drag_status"] or any(state["brake_released"]) or
                    any(state[f"{part}_approach_state"] for part in ("arm", "j4", "j5", "j6"))):
                raise RuntimeError("motion interrupted by protection state")
            running = bool(state["running_status"] or state["robot_mode"] in (7, 8))
            observed_running |= running
            fresh_packet = baseline is not None and sample.get("packets_received", baseline) > baseline
            idle = (state["robot_mode"] == 5 and state["enable_status"] and not running
                    and not state["jog_status"] and not state["pause_cmd_flag"]
                    and all(abs(v) <= 0.1 for v in state["joint_state"]["qd_actual"]))
            # Queue ID must be observed, or the submitted motion must have been
            # seen running. Old idle/target data never proves completion.
            correlated = (queue_id is not None and state.get("current_command_id") == queue_id) or observed_running
            if fresh_packet and idle and correlated:
                if target is None:
                    return {"reason": "queue_completed", "result_id": queue_id}
                actual = state["joint_state"]["q_actual"]
                error = max(abs(a - b) for a, b in zip(actual, target))
                if error <= self.tolerance:
                    return {"reason": "target_reached", "max_error_deg": error, "result_id": queue_id}
                raise RuntimeError("motion ended before reaching the requested joint target")
            cancel.wait(self.poll_interval)
        raise TimeoutError("motion completion deadline exceeded")

    def _complete(self, action_id, status, result, context):
        record = {"action_id": action_id, "status": status, "result": result, "callback": "sending"}
        self.last_completion = record
        for attempt in range(3):
            try:
                self._post_complete(action_id, status, result, context)
                record["callback"] = "accepted"
                return
            except Exception as exc:
                record["callback_error"] = str(exc)
                if attempt < 2:
                    time.sleep(0.2)
        record["callback"] = "failed"
        print(f"[dobot ACP] {action_id}: callback failed: {record['callback_error']}", flush=True)

    def _post_complete(self, action_id, status, result, context):
        if self._notify is not None:
            self._notify(action_id, status, result)
            return
        if context is None:
            raise RuntimeError("AGENT_CORE_CA_CERT is required for completion callbacks")
        payload = {"action_id": action_id, "status": status, "result": result,
                   "tool": "joint_move", "ts": time.time()}
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        request = Request(url + "/api/acp/complete", data=json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json"}, method="POST")
        with urlopen(request, timeout=5, context=context) as response:
            acknowledgement = json.load(response)
        if acknowledgement.get("ok") is not True or acknowledgement.get("action_id") != action_id:
            raise RuntimeError("Agent Core did not acknowledge this action_id")
