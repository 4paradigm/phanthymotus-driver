"""Go2 execution adapter for validated Nav2 velocity proposals."""

from __future__ import annotations

import json
import math
import queue
import threading
import time

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from velocity_proposal import ProposalLimits, VelocityProposalGate


_PROPOSAL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class Go2VelocityProposalController(Node):
    """Own the Go2 proposal subscription, lease, watchdog and StopMove path."""

    def __init__(self, config: dict, namespace: str, rpc_proxy):
        super().__init__("go2_velocity_proposal")
        self._rpc = rpc_proxy
        self._topic = str(
            config.get(
                "velocity_proposal_topic",
                "/ubuntu/navigation/nav2/velocity_proposal",
            )
        ).strip()
        if not self._topic.startswith("/"):
            self._topic = f"/{self._topic}"
        self._state_topic = f"/{namespace}/loco/state"
        self._limits = ProposalLimits(
            max_ttl_ms=int(config.get("velocity_proposal_max_ttl_ms", 250)),
            min_x=float(config.get("velocity_proposal_min_x", -1.0)),
            max_x=float(config.get("velocity_proposal_max_x", 1.0)),
            max_abs_y=float(config.get("velocity_proposal_max_abs_y", 1.0)),
            max_abs_yaw=float(config.get("velocity_proposal_max_abs_yaw", 2.0)),
            max_planar_speed=float(
                config.get("velocity_proposal_max_planar_speed", math.sqrt(2.0))
            ),
        )
        self._gate = VelocityProposalGate(self._limits)
        self._lock = threading.RLock()
        self._rpc_lock = threading.Lock()
        self._state_condition = threading.Condition()
        self._last_state_monotonic = 0.0
        self._last_velocity = (float("inf"), float("inf"), float("inf"))
        self._stop_timeout = max(
            0.2, float(config.get("velocity_proposal_stop_confirm_timeout", 1.0))
        )
        self._linear_epsilon = max(
            0.0, float(config.get("velocity_proposal_stop_linear_epsilon", 0.04))
        )
        self._yaw_epsilon = max(
            0.0, float(config.get("velocity_proposal_stop_yaw_epsilon", 0.08))
        )
        self._proposal_sub = None
        self._state_sub = self.create_subscription(
            String, self._state_topic, self._on_state, _STATE_QOS
        )
        self._commands: queue.Queue = queue.Queue(maxsize=1)
        self._shutdown = threading.Event()
        self._stop_transition = threading.Event()
        self._last_stop_attempt_monotonic = 0.0
        self._counters = {
            "received": 0,
            "accepted": 0,
            "applied": 0,
            "rejected": 0,
            "coalesced": 0,
            "rejections_by_reason": {},
            "watchdog_faults_by_reason": {},
        }
        self._last_rpc = {
            "ret": None,
            "duration_ms": None,
            "error": None,
            "nav_id": None,
            "sequence": None,
        }
        self._worker = threading.Thread(
            target=self._command_loop,
            name="go2_velocity_proposal_apply",
            daemon=True,
        )
        self._watchdog = threading.Thread(
            target=self._watchdog_loop,
            name="go2_velocity_proposal_watchdog",
            daemon=True,
        )
        self._worker.start()
        self._watchdog.start()

    @property
    def topic(self) -> str:
        return self._topic

    def _record_rejection(self, reason: str) -> None:
        self._counters["rejected"] += 1
        reasons = self._counters["rejections_by_reason"]
        reasons[reason] = reasons.get(reason, 0) + 1

    def _record_watchdog(self, reason: str) -> None:
        reasons = self._counters["watchdog_faults_by_reason"]
        reasons[reason] = reasons.get(reason, 0) + 1

    def _on_state(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
            velocity = payload["velocity"]
            sample = (
                float(velocity[0]),
                float(velocity[1]),
                float(payload["yaw_speed"]),
            )
        except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._state_condition:
            self._last_velocity = sample
            self._last_state_monotonic = time.monotonic()
            self._state_condition.notify_all()

    def _put_latest(self, command: dict) -> None:
        try:
            self._commands.put_nowait(command)
            return
        except queue.Full:
            pass
        try:
            self._commands.get_nowait()
        except queue.Empty:
            pass
        else:
            self._counters["coalesced"] += 1
        self._commands.put_nowait(command)

    def _enqueue_stop(self, reason: str) -> None:
        if self._stop_transition.is_set():
            return
        self._stop_transition.set()
        self._put_latest({"kind": "stop", "reason": reason})

    def _on_proposal(self, msg) -> None:
        now = time.monotonic()
        now_unix_ms = time.time() * 1000.0
        with self._lock:
            self._counters["received"] += 1
            if self._stop_transition.is_set():
                self._record_rejection("stop_transition")
                return
            try:
                payload = json.loads(msg.data)
            except (AttributeError, TypeError, json.JSONDecodeError):
                self._record_rejection("invalid_json")
                if self._gate.armed:
                    self._gate.disarm("invalid_json")
                    self._enqueue_stop("invalid_json")
                return
            was_armed = self._gate.armed
            decision = self._gate.accept(payload, now, now_unix_ms=now_unix_ms)
            if decision.stop:
                if decision.reason != "proposal_zero":
                    self._record_rejection(decision.reason)
                if was_armed or self._gate.armed or self._gate.terminal_pending_stop:
                    self._enqueue_stop(decision.reason)
                return
            if not decision.execute or decision.proposal is None:
                self._record_rejection(decision.reason or "proposal_not_executable")
                return
            proposal = decision.proposal
            self._counters["accepted"] += 1
            self._put_latest(
                {
                    "kind": "move",
                    "nav_id": proposal.nav_id,
                    "sequence": proposal.sequence,
                    "vx": proposal.x,
                    "vy": proposal.y,
                    "vyaw": proposal.yaw,
                    "deadline": self._gate.deadline_monotonic,
                }
            )

    def _apply_move(self, command: dict) -> None:
        with self._lock:
            if (
                self._stop_transition.is_set()
                or not self._gate.armed
                or command["nav_id"] != self._gate.expected_nav_id
                or command["sequence"] != self._gate.last_sequence
            ):
                return
            if time.monotonic() >= command["deadline"]:
                self._record_rejection("proposal_ttl_expired")
                self._gate.watchdog(time.monotonic())
                self._record_watchdog("proposal_ttl_expired")
                self._enqueue_stop("proposal_ttl_expired")
                return

        started = time.monotonic()
        error = None
        try:
            with self._rpc_lock:
                if time.monotonic() >= command["deadline"]:
                    ret = None
                    error = "proposal_ttl_expired_before_rpc"
                else:
                    ret = self._rpc.Move(
                        command["vx"], command["vy"], command["vyaw"]
                    )
                    if ret == 0 and time.monotonic() >= command["deadline"]:
                        error = "proposal_ttl_expired_after_rpc"
        except Exception as exc:  # hardware boundary: fail closed
            ret = None
            error = str(exc)
        duration_ms = round((time.monotonic() - started) * 1000.0, 1)
        with self._lock:
            self._last_rpc = {
                "ret": ret,
                "duration_ms": duration_ms,
                "error": error,
                "nav_id": command["nav_id"],
                "sequence": command["sequence"],
            }
            if error or ret != 0:
                reason = (
                    "proposal_ttl_expired"
                    if error in {
                        "proposal_ttl_expired_before_rpc",
                        "proposal_ttl_expired_after_rpc",
                    }
                    else "set_velocity_failed"
                )
                self._record_rejection(reason)
                if reason == "proposal_ttl_expired":
                    self._gate.request_ttl_stop(time.monotonic())
                else:
                    self._gate.disarm(reason)
                self._enqueue_stop(reason)
                return
            self._counters["applied"] += 1

    def _stop_and_confirm(self) -> tuple[int | None, bool, str | None]:
        for _attempt in range(2):
            boundary = time.monotonic()
            try:
                with self._rpc_lock:
                    ret = self._rpc.StopMove()
            except Exception as exc:  # hardware boundary: fail closed
                return None, False, str(exc)
            if ret != 0:
                return ret, False, None
            deadline = time.monotonic() + self._stop_timeout
            with self._state_condition:
                while time.monotonic() < deadline:
                    vx, vy, vyaw = self._last_velocity
                    if (
                        self._last_state_monotonic >= boundary
                        and math.hypot(vx, vy) <= self._linear_epsilon
                        and abs(vyaw) <= self._yaw_epsilon
                    ):
                        return ret, True, None
                    self._state_condition.wait(
                        timeout=min(0.05, max(0.0, deadline - time.monotonic()))
                    )
        return ret, False, None

    def _process_stop(self, reason: str) -> None:
        self._last_stop_attempt_monotonic = time.monotonic()
        ret, confirmed, error = self._stop_and_confirm()
        with self._lock:
            terminal = self._gate.terminal_pending_stop
            if terminal:
                self._gate.record_terminal_stop_result(confirmed)
            elif confirmed and reason == "proposal_ttl_expired" and self._gate.armed:
                self._gate.hold_after_confirmed_stop("proposal_ttl_expired")
            elif not confirmed and self._gate.armed:
                self._gate.disarm("stop_unconfirmed")
            self._last_rpc.update(
                {
                    "stop_ret": ret,
                    "stop_confirmed": confirmed,
                    "stop_error": error,
                }
            )
            self._stop_transition.clear()

    def _command_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                command = self._commands.get(timeout=0.1)
            except queue.Empty:
                continue
            if command is None:
                return
            if command["kind"] == "stop":
                self._process_stop(command["reason"])
            else:
                self._apply_move(command)

    def _watchdog_loop(self) -> None:
        while not self._shutdown.wait(0.025):
            with self._lock:
                if self._stop_transition.is_set():
                    continue
                if self._gate.terminal_pending_stop:
                    if time.monotonic() - self._last_stop_attempt_monotonic >= 0.25:
                        self._enqueue_stop("terminal_retry")
                    continue
                decision = self._gate.watchdog(time.monotonic())
                if decision.stop:
                    self._record_watchdog(decision.reason)
                    self._enqueue_stop(decision.reason)

    def connect(self, topic: str, expected_nav_id: str | None = None) -> dict:
        if topic != self._topic:
            return {"error": "unexpected_velocity_proposal_topic", **self.status()}
        with self._lock:
            if self._proposal_sub is not None:
                if (
                    expected_nav_id is not None
                    and self._gate.expected_nav_id
                    and self._gate.expected_nav_id != expected_nav_id
                ):
                    return {"error": "proposal_lease_already_connected", **self.status()}
                return self.status()
            self._stop_transition.set()
        self._clear_commands()
        ret, confirmed, error = self._stop_and_confirm()
        if not confirmed:
            with self._lock:
                self._stop_transition.clear()
                self._last_rpc.update(
                    {"stop_ret": ret, "stop_confirmed": False, "stop_error": error}
                )
            return {
                "error": error or "StopMove was not confirmed before proposal bind",
                **self.status(),
            }
        try:
            proposal_sub = self.create_subscription(
                String, topic, self._on_proposal, _PROPOSAL_QOS
            )
            with self._lock:
                self._gate.bind(topic, expected_nav_id)
                self._proposal_sub = proposal_sub
        except Exception as exc:
            if "proposal_sub" in locals():
                self.destroy_subscription(proposal_sub)
            with self._lock:
                self._gate.unbind("proposal_bind_failed")
                self._stop_transition.clear()
            return {"error": str(exc), **self.status()}
        with self._lock:
            self._stop_transition.clear()
        return self.status()

    def _clear_commands(self) -> None:
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    def disconnect(self, reason: str = "canvas_stop") -> dict:
        self._stop_transition.set()
        self._clear_commands()
        ret, confirmed, error = self._stop_and_confirm()
        with self._lock:
            sub = self._proposal_sub
            self._proposal_sub = None
            if sub is not None:
                self.destroy_subscription(sub)
            self._gate.unbind(reason)
            self._stop_transition.clear()
            self._last_rpc.update(
                {"stop_ret": ret, "stop_confirmed": confirmed, "stop_error": error}
            )
            result = self.status()
        if not confirmed:
            result["error"] = error or "StopMove was not confirmed"
        return result

    def manual_override(self) -> bool:
        with self._lock:
            if not self._gate.connected_topic:
                return True
            self._gate.disarm("manual_override")
            self._stop_transition.set()
        self._clear_commands()
        ret, confirmed, error = self._stop_and_confirm()
        with self._lock:
            self._last_rpc.update(
                {"stop_ret": ret, "stop_confirmed": confirmed, "stop_error": error}
            )
            self._stop_transition.clear()
        return confirmed

    def status(self) -> dict:
        with self._lock:
            snapshot = self._gate.snapshot(time.monotonic())
            state_age_ms = (
                round((time.monotonic() - self._last_state_monotonic) * 1000.0)
                if self._last_state_monotonic > 0.0
                else None
            )
            return {
                **snapshot,
                "state": "ready" if snapshot["connected"] else "idle",
                "canvas_running": snapshot["connected"],
                "driver_authorized": snapshot["armed"],
                "proposal_execution": {
                    **self._counters,
                    "last_rpc": dict(self._last_rpc),
                },
                "loco_state_age_ms": state_age_ms,
                "stop_transition_active": self._stop_transition.is_set(),
            }

    def close(self) -> None:
        self.disconnect("driver_stop")
        self._shutdown.set()
        try:
            self._commands.put_nowait(None)
        except queue.Full:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                pass
            self._commands.put_nowait(None)
        self._worker.join(timeout=2.0)
        self._watchdog.join(timeout=2.0)
