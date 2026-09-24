"""Bounded DDS operator requests, independent of the latest-pose slot."""

from __future__ import annotations
import asyncio
import copy
import hashlib
import time
from collections import OrderedDict


class OperatorCommands:
    ACTIONS = {
        "start": "begin",
        "begin": "begin",
        "finish": "finish",
        "stop": "stop",
        "calibrate": "calibrate",
    }

    def __init__(self, manager, runtime, publish):
        self.manager, self.runtime, self.publish = manager, runtime, publish
        self.receipts = OrderedDict()
        self.pending = OrderedDict()
        self._seen = bytearray(8192)
        self.status = {"state": "unbound", "armed": False, "error": None}
        self.bound = False
        self._feedback_at = None
        self._server_epoch = None
        self._task = None

    def bind(self, enabled):
        self.bound = bool(enabled)
        if not self.bound:
            self.status = {"state": "unbound", "armed": False, "error": None}
            for key in list(self.pending):
                self._fail(key, "device_collection_stopped")
        elif self._task is None or self._task.done():
            self._task = asyncio.create_task(self._pump())

    def _fail(self, key, error):
        item = self.pending.pop(key, None)
        receipt = self.receipts[key]
        receipt.update(state="failed", error=error, execution_outcome="unknown")
        self._notify(receipt, item.get("connection") if item else None)

    def _notify(self, receipt, expected_connection):
        connection = self.manager._connection
        if connection is not None and connection is expected_connection:
            # A receipt never blocks newer input or a stop. The WSS queue is
            # bounded by CaptureConnection; latest state can be queried again.
            try:
                connection.events.put_nowait(copy.deepcopy(receipt))
            except asyncio.QueueFull:
                pass

    async def submit(self, connection, message):
        if (
            self.manager._connection is not connection
            or self.manager.presence_expired(connection)
            or message.get("connection_id") != connection.connection_id
        ):
            raise ValueError("operator_connection_changed")
        rid, requested = message.get("request_id"), message.get("action")
        if (
            not isinstance(rid, str)
            or not 1 <= len(rid) <= 80
            or requested not in (*self.ACTIONS, "query")
        ):
            raise ValueError("operator_request_invalid")
        key = (connection.capture_id, rid)
        if key in self.receipts:
            old = self.receipts[key]
            if requested != "query" and old["action"] != requested:
                raise ValueError("operator_request_conflict")
            return copy.deepcopy(old)
        if requested == "query":
            raise ValueError("operator_request_unknown")
        if not self.bound:
            raise ValueError("teleop_not_bound")
        if requested != "stop" and (
            self._feedback_at is None or time.monotonic() - self._feedback_at > 1
        ):
            raise ValueError("teleop_feedback_unavailable")
        if requested != "stop" and self.pending:
            raise ValueError("operator_busy")
        digest = hashlib.sha256((connection.capture_id + "\0" + rid).encode()).digest()
        indices = [int.from_bytes(digest[i : i + 2], "big") for i in (0, 2, 4)]
        if requested != "stop" and all(
            self._seen[i // 8] & (1 << (i % 8)) for i in indices
        ):
            raise ValueError("operator_request_retired")
        # Stop is idempotent and must never be rejected by a probabilistic
        # tombstone collision or by capacity occupied by ordinary operations.
        # Exact retained receipts above still deduplicate repeated stop IDs.
        if requested == "stop":
            for other in list(self.pending):
                self._fail(other, "superseded_by_stop")
        while len(self.receipts) >= 256:
            old_key = next((k for k in self.receipts if k not in self.pending), None)
            if old_key is None:
                raise ValueError("operator_request_history_full")
            self.receipts.pop(old_key)
        identity = self.runtime.command_identity(require_capture=requested != "stop")
        packet = {
            **identity,
            "schema": "motus.teleop.command/1",
            "kind": "operation",
            "request_id": rid,
            "action": self.ACTIONS[requested],
            "expires_monotonic_ns": identity["received_monotonic_ns"] + 5_000_000_000,
        }
        receipt = {
            "type": "operator_result",
            "request_id": rid,
            "action": requested,
            "state": "accepted",
        }
        self.receipts[key] = receipt
        self.pending[key] = {
            "packet": packet,
            "generation": identity["connection_epoch"],
            "connection": connection,
            "accepted": False,
            "created": time.monotonic(),
            "last_send": 0.0,
        }
        for i in indices:
            self._seen[i // 8] |= 1 << (i % 8)
        # Emit immediately, before another pose can take the output slot.
        try:
            self.publish(copy.deepcopy(packet))
        except Exception:
            # Keep the admitted request and original deadline for bounded
            # retry; a full middleware queue must not kill the operation loop.
            pass
        self.pending[key]["last_send"] = time.monotonic()
        return copy.deepcopy(receipt)

    def feedback(self, value, *, update_status=True):
        epoch = value["server_epoch"]
        if self._server_epoch is not None and self._server_epoch != epoch:
            for key in list(self.pending):
                self._fail(key, "teleop_control_restarted")
        self._server_epoch = epoch
        if update_status:
            self._feedback_at = time.monotonic()
            execution = value.get("execution") or {}
            self.status = {
                "state": value["state"],
                "armed": bool(execution.get("armed", False)),
                "started": bool(
                    execution.get("started", value.get("operator_session_id"))
                ),
                "mode": execution.get("mode", "shadow"),
                "error": value.get("reason"),
                "session_id": value.get("operator_session_id"),
                "observed_monotonic_ns": time.monotonic_ns(),
            }
        for incoming in value.get("receipts", []):
            for key, item in list(self.pending.items()):
                packet = item["packet"]
                if (
                    incoming["request_id"] != key[1]
                    or incoming["action"] != packet["action"]
                ):
                    continue
                if any(
                    incoming.get(field) != packet[field]
                    for field in ("device_id", "connection_epoch", "space_epoch")
                ):
                    continue
                if incoming["status"] == "accepted":
                    item["accepted"] = True
                else:
                    receipt = self.receipts[key]
                    receipt["state"] = incoming["status"]
                    if incoming.get("error"):
                        receipt["error"] = str(incoming["error"])[:160]
                    # Never forward leases, authentication or private robot data.
                    if isinstance(incoming.get("result"), dict):
                        receipt["result"] = {
                            k: v
                            for k, v in incoming["result"].items()
                            if k in ("state", "reason", "started")
                            and isinstance(v, (str, bool, type(None)))
                        }
                    self.pending.pop(key)
                    self._notify(receipt, item["connection"])

    async def _pump(self):
        while self.bound:
            now = time.monotonic()
            if self._feedback_at is not None and now - self._feedback_at > 1:
                self.status.update(
                    state="unavailable",
                    armed=False,
                    error="teleop_feedback_unavailable",
                )
            for key, item in list(self.pending.items()):
                if not self.runtime.generation_matches(item["generation"]):
                    self._fail(key, "operator_connection_changed")
                elif (
                    not item["accepted"]
                    and self.runtime.clock_ns() > item["packet"]["expires_monotonic_ns"]
                ):
                    self._fail(key, "operator_acceptance_timeout")
                elif now - item["created"] > 60:
                    # Unknown final outcome is surfaced; it never causes a new
                    # begin or an automatic release/restart of the robot.
                    self._fail(key, "operator_completion_unknown")
                elif not item["accepted"] and now - item["last_send"] >= 0.1:
                    try:
                        self.publish(copy.deepcopy(item["packet"]))
                    except Exception:
                        pass
                    item["last_send"] = now
            await asyncio.sleep(0.025)

    async def close(self):
        self.bind(False)
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
