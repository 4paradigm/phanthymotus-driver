"""Serialized Dashboard transport for the system control card.

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


def parse_response(reply, command):
    match = re.fullmatch(r"\s*(-?\d+),\{([^{}]*)\},([A-Za-z][A-Za-z0-9_]*)\(.*\);\s*", reply, re.S)
    if match is None or match[3] != command:
        raise ValueError(f"invalid {command} acknowledgement: {reply!r}")
    code = int(match[1])
    if code:
        raise CommandRejected(f"{command} rejected with ErrorID={code}: {reply}")
    return match[2].strip()


def parse_reply(reply, command):
    identifier = parse_response(reply, command)
    if identifier and not identifier.isdecimal():
        raise ValueError(f"invalid queue identifier: {identifier!r}")
    return int(identifier) if identifier else None


class RobotController:
    def __init__(self, feedback, config, connection_factory=DashboardConnection):
        self.feedback = feedback
        settings = config.get("systemcontrol", {})
        self.connect_timeout = positive_number(settings.get("connect_timeout_s", 2), "systemcontrol.connect_timeout_s")
        self.receive_timeout = positive_number(settings.get("receive_timeout_s", 2), "systemcontrol.receive_timeout_s")
        self._factory = connection_factory
        self._command_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._connection = None
        self._api = None
        self._address = None
        self._status = {"dashboard_connected": False, "last_request": None, "last_error": None}

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
        self._update(dashboard_connected=False)
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

    def system_status(self):
        sample = self.feedback.snapshot()
        state = sample.get("data")
        available = bool(sample.get("connected") and not sample.get("stale") and state is not None)
        keys = ("robot_mode", "robot_mode_name", "enable_status", "running_status", "error_status",
                "collision_state", "drag_status", "pause_cmd_flag", "brake_released",
                "speed_scaling", "safety_state", "safe_state", "current_command_id", "load")
        # Alarm/initialization modes can mask the underlying power state.
        mode = None if not available else state["robot_mode"]
        powered_on = False if mode == 3 else True if mode in (2, 4, 5, 6, 7, 8, 10, 11) else None
        return {"available": available, "timestamp_ms": int(time.time() * 1000),
                "powered_on": powered_on,
                "enabled": None if not available else bool(state["enable_status"]),
                "running": None if not available else bool(state["running_status"] or mode in (7, 8)),
                "paused": None if not available else bool(state["pause_cmd_flag"] or mode == 10),
                "dragging": None if not available else bool(state["drag_status"] or mode == 6),
                "collision": None if not available else bool(state["collision_state"] or mode == 11),
                "has_error": None if not available else bool(state["error_status"] or mode == 9),
                "speed_scaling": None if not available else state["speed_scaling"],
                "robot_status": None if not available else {key: state[key] for key in keys},
                **self.snapshot()}

    def _check_system_state(self, action):
        sample = self.feedback.snapshot()
        # Explicit disable and exit-drag remain possible without fresh feedback.
        if action in ("disable", "stop_drag"):
            if not self.feedback.address:
                raise RuntimeError("configure the controller IP before sending commands")
            return {**sample, "controller_ip": self.feedback.address}
        state = sample.get("data")
        if not sample.get("connected") or sample.get("stale") or state is None:
            raise RuntimeError("fresh feedback is required for this system control action")
        mode = state["robot_mode"]
        if action == "clear_error":
            if (mode not in (3, 4, 5, 9, 11) or state["running_status"] or state["jog_status"] or
                    state["drag_status"] or any(state["brake_released"])):
                raise RuntimeError("stop motion and drag before clearing alarms")
            return sample
        allowed = {"enable": (4,), "start_drag": (4, 5)}
        if mode not in allowed[action]:
            raise RuntimeError(f"{action} requires robot mode {allowed[action]}, current mode is {mode}")
        if (state["error_status"] or state["collision_state"] or state["drag_status"] or
                any(state["brake_released"]) or
                any(state[f"{part}_approach_state"] for part in ("arm", "j4", "j5", "j6"))):
            raise RuntimeError("clear the alarm/collision/skin trigger and lock brakes before this action")
        if state["running_status"] or state["jog_status"]:
            raise RuntimeError("robot must be stopped before this action")
        if state["pause_cmd_flag"]:
            raise RuntimeError("robot is paused; this action does not resume it")
        return sample

    def execute_system(self, action, args):
        commands = {"enable": "EnableRobot", "disable": "DisableRobot", "clear_error": "ClearError",
                    "start_drag": "StartDrag", "stop_drag": "StopDrag"}
        if action not in commands:
            raise ValueError(f"unsupported system control action: {action}")
        if args.get("confirm_change") is not True:
            raise ValueError("confirm_change=true is required for each control request")
        return self._execute(action, commands[action], self._check_system_state)

    def _execute(self, action, command, check_state):
        with self._command_lock:
            sample = check_state(action)
            request = {"action": action, "timestamp_ms": int(time.time() * 1000),
                       "controller_ip": sample["controller_ip"], "read_only": False, "effect_verified": False}
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
                check_state(action)
                reply = getattr(self._api, command)()
                queue_id = parse_reply(reply, command)
                request.update(state="submitted" if queue_id is not None else "accepted_unverified",
                               result_id=queue_id, response=reply)
                self._update(last_request=request)
                return request
            except Exception as exc:
                # A timeout may happen after transmission; never replay the write.
                request.update(state="rejected" if isinstance(exc, CommandRejected) else "failed_or_outcome_unknown", error=str(exc))
                self._update(last_request=request, last_error=str(exc))
                self._close()
                raise
