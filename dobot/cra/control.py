"""Explicit, serialized safe-skin settings using the official V4 SDK.

No connection, control takeover, enable, error clearing or motion on startup.
Write requests are never replayed after a network error. Queued acknowledgements
are reported as submissions, not as verified configuration readback.
"""

from __future__ import annotations

import re
import threading
import time

from dobot_sdk.api.robot_control import RobotControl
from dobot_sdk.core.connection import DobotConnection

if __package__:
    from .feedback import disconnect, endpoint, positive_number
else:
    from feedback import disconnect, endpoint, positive_number


PARTS = {3: "arm", 4: "j4", 5: "j5", 6: "j6"}


class DashboardConnection(DobotConnection):
    """Retain SDK command formatting, with complete framed send/receive."""

    def send_receive_text(self, text, recv_size=4096):
        with self._lock:
            if not self.is_connected or self._socket is None:
                raise ConnectionError("Dashboard is not connected")
            self._socket.sendall((text.rstrip("\n") + "\n").encode("ascii"))
            data = bytearray()
            while b";" not in data:
                chunk = self._socket.recv(recv_size)
                if not chunk:
                    raise ConnectionError("Dashboard closed before acknowledging the command")
                data.extend(chunk)
                if len(data) > 65536:
                    raise ValueError("Dashboard response exceeds 64 KiB")
                if b"occupied" in data or b"Control Mode Is Not Tcp" in data:
                    raise RuntimeError(data.decode("utf-8", errors="replace").strip())
            if data.split(b";", 1)[1].strip():
                raise ValueError("unexpected extra Dashboard response")
            return data.decode("utf-8").strip()


class CommandRejected(RuntimeError):
    pass


def parse_reply(reply, command):
    match = re.fullmatch(r"\s*(-?\d+),\{([^{}]*)\},([A-Za-z][A-Za-z0-9_]*)\(.*\);\s*", reply, re.S)
    if match is None or match[3] != command:
        raise ValueError(f"invalid {command} acknowledgement: {reply!r}")
    code = int(match[1])
    if code:
        raise CommandRejected(f"{command} rejected with ErrorID={code}: {reply}")
    identifier = match[2].strip()
    if identifier and not identifier.isdecimal():
        raise ValueError(f"invalid queue identifier: {identifier!r}")
    return int(identifier) if identifier else None


class SafeSkinController:
    def __init__(self, feedback, config, connection_factory=DashboardConnection):
        self.feedback = feedback
        settings = config.get("safe_skin_control", {})
        self.connect_timeout = positive_number(settings.get("connect_timeout_s", 2), "safe_skin_control.connect_timeout_s")
        self.receive_timeout = positive_number(settings.get("receive_timeout_s", 2), "safe_skin_control.receive_timeout_s")
        self._factory = connection_factory
        self._command_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._connection = None
        self._api = None
        self._address = None
        self._status = {"dashboard_connected": False, "tcp_control_requested": False,
                        "last_request": None, "last_error": None}

    def snapshot(self):
        import copy
        with self._state_lock:
            return copy.deepcopy(self._status)

    def _update(self, **values):
        with self._state_lock:
            self._status.update(values)

    def _close(self):
        connection, self._connection = self._connection, None
        self._api = None
        self._address = None
        self._update(dashboard_connected=False, tcp_control_requested=False)
        if connection is not None:
            try:
                disconnect(connection)
            except Exception as exc:
                self._update(last_cleanup_error=str(exc))

    def close(self):
        with self._command_lock:
            self._close()

    def reconfigure(self, address, port):
        address, port = endpoint(address, port)
        with self._command_lock:
            if (address, port) != (self.feedback.address, self.feedback.port):
                self._close()
                self._update(last_request=None, last_error=None)
                self.feedback.configure(address, port)

    def _check_state(self, action):
        sample = self.feedback.snapshot()
        state = sample.get("data")
        if not sample.get("connected") or sample.get("stale") or state is None:
            raise RuntimeError("fresh feedback is required before changing safe-skin settings")
        if (state["robot_mode"] not in (3, 4, 5) or state["running_status"] or
                state["error_status"] or state["collision_state"] or state["drag_status"] or
                state["pause_cmd_flag"] or any(state["brake_released"]) or
                any(state[key] for key in ("arm_approach_state", "j4_approach_state", "j5_approach_state", "j6_approach_state"))):
            raise RuntimeError("robot must be stopped, without alarm, collision, skin trigger, drag or released brakes")
        if action == "request_control" and (state["robot_mode"] not in (3, 4) or state["auto_manual_mode"] != 0):
            raise RuntimeError("TCP takeover requires power-off/disabled state and hand/auto switching disabled")
        return sample

    def execute(self, action, args):
        # Validate everything before opening a Dashboard connection.
        if args.get("confirm_change") is not True:
            raise ValueError("confirm_change=true is required for each control request")
        if action == "set_enabled":
            if not isinstance(args.get("enabled"), bool):
                raise ValueError("enabled must be a boolean")
            command, parameters = "EnableSafeSkin", (int(args["enabled"]),)
            requested = {"enabled": args["enabled"]}
        elif action == "set_sensitivity":
            part, level = args.get("part"), args.get("level")
            if type(part) is not int or part not in PARTS:
                raise ValueError("part must be 3 (arm), 4, 5 or 6")
            if type(level) is not int or level not in (0, 1, 2, 3):
                raise ValueError("level must be 0=off, 1=low, 2=middle or 3=high")
            command, parameters = "SetSafeSkin", (part, level)
            requested = {"part": part, "level": level}
        elif action == "request_control":
            command, parameters, requested = "RequestControl", (), {}
        else:
            raise ValueError(f"unsupported safe-skin action: {action}")
        with self._command_lock:
            sample = self._check_state(action)
            request = {"action": action, **requested, "timestamp_ms": int(time.time() * 1000),
                       "controller_ip": sample["controller_ip"], "configuration_verified": False}
            self._update(last_request={**request, "state": "sending"}, last_error=None)
            try:
                if self._connection is None or self._address != sample["controller_ip"]:
                    self._close()
                    self._address = sample["controller_ip"]
                    self._connection = self._factory(self._address, 29999)
                    self._connection.set_timeout(self.connect_timeout, self.receive_timeout)
                    self._connection.connect()
                    self._api = RobotControl(self._connection)
                    self._update(dashboard_connected=True)
                # Connecting can take long enough for feedback/state to change.
                self._check_state(action)
                reply = getattr(self._api, command)(*parameters)
                queue_id = parse_reply(reply, command)
                request.update(state="submitted" if queue_id is not None else "accepted_unverified",
                               result_id=queue_id, response=reply)
                if action == "request_control":
                    request["state"] = "tcp_control_requested"
                    self._update(tcp_control_requested=True)
                self._update(last_request=request)
                return request
            except Exception as exc:
                # A timeout may happen after transmission; never replay the write.
                request.update(state="rejected" if isinstance(exc, CommandRejected) else "failed_or_outcome_unknown", error=str(exc))
                self._update(last_request=request, last_error=str(exc))
                self._close()
                raise
