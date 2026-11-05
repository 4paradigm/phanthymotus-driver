"""Serialized Dashboard transport for the joint_move card.

No connection, control takeover, enable, error clearing or motion on startup.
Write requests are never replayed after a network error. Queued acknowledgements
are reported as submissions, not as verified configuration readback.
"""

from __future__ import annotations

import re
import math
import threading
import time

from dobot_sdk.api.robot_control import RobotControl
from dobot_sdk.api.motion import Motion, CoordinateType
from dobot_sdk.core.connection import DobotConnection

if __package__:
    from .feedback import disconnect, endpoint, positive_number
    from .model import JOINT_NAMES, resolve_model
else:
    from feedback import disconnect, endpoint, positive_number
    from model import JOINT_NAMES, resolve_model


# CRA hardware manual V3.2, Appendix A, CR3A (degrees). The visualization
# URDF rounds radians and is not used as a controller safety configuration.
JOINT_LIMITS_DEG = ((-360, 360), (-360, 360), (-155, 155),
                    (-360, 360), (-360, 360), (-360, 360))


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
        settings = config.get("joint_move", {})
        self.connect_timeout = positive_number(settings.get("connect_timeout_s", 2), "joint_move.connect_timeout_s")
        self.receive_timeout = positive_number(settings.get("receive_timeout_s", 2), "joint_move.receive_timeout_s")
        self.enable_timeout = positive_number(settings.get("enable_timeout_s", 5), "joint_move.enable_timeout_s")
        self._factory = connection_factory
        self._command_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._connection = None
        self._api = None
        self._motion_api = None
        self._address = None
        self._status = {"dashboard_connected": False}
        self._requests = {card: {"last_request": None, "last_error": None}
                          for card in ("joint_move",)}

    def snapshot(self, card="joint_move"):
        import copy
        with self._state_lock:
            return copy.deepcopy({**self._status, **self._requests[card]})

    def _update(self, card="joint_move", **values):
        with self._state_lock:
            for key, value in values.items():
                target = self._requests[card] if key in ("last_request", "last_error") else self._status
                target[key] = value

    def _close(self):
        connection, self._connection = self._connection, None
        self._api = None
        self._motion_api = None
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
                for card in self._requests:
                    self._update(card, last_request=None, last_error=None)
                self.feedback.configure(address, port)

    def _check_preparation_state(self, action):
        """Validate preparation without requiring the final idle/protection state."""
        sample = self.feedback.snapshot()
        if not self.feedback.address:
            raise RuntimeError("configure the controller IP before sending commands")
        state = sample.get("data")
        if not sample.get("connected") or sample.get("stale") or state is None:
            raise RuntimeError("fresh feedback is required for joint motion")
        if sample.get("robot_type_code") != 113:
            raise RuntimeError("joint motion currently requires controller-identified CR3A (type 113)")
        resolve_model(sample, self.feedback.model_name)
        # ClearError must not interrupt active motion/drag or operate with released brakes.
        # Alarm/collision/approach flags are intentionally checked only after preparation.
        if (state["robot_mode"] not in (4, 5, 9, 10, 11) or state["running_status"] or
                state["jog_status"] or state["drag_status"] or any(state["brake_released"])):
            raise RuntimeError("robot must be stationary with locked brakes before preparing motion")
        if action == "move_j" and state["robot_mode"] not in (9, 11) and (
                state["pause_cmd_flag"] or state["run_queued_cmd"]):
            raise RuntimeError("move_j requires an idle robot without a paused or running queue")
        return sample

    def _check_motion_state(self, action):
        sample = self.feedback.snapshot()
        if not self.feedback.address:
            raise RuntimeError("configure the controller IP before sending commands")
        # Explicit stopping must remain possible when feedback is lost.
        if action == "stop_motion":
            return {**sample, "controller_ip": self.feedback.address}
        state = sample.get("data")
        if not sample.get("connected") or sample.get("stale") or state is None:
            raise RuntimeError("fresh feedback is required for joint motion")
        mode = state["robot_mode"]
        if action == "pause":
            if mode != 7:
                raise RuntimeError("pause requires a running queue/program (mode 7); use stop_motion for jogging")
            return sample
        if sample.get("robot_type_code") != 113:
            raise RuntimeError("joint motion currently requires controller-identified CR3A (type 113)")
        resolve_model(sample, self.feedback.model_name)
        required_mode = 5 if action == "move_j" else 10
        if mode != required_mode or not state["enable_status"]:
            raise RuntimeError(f"{action} requires an enabled robot in mode {required_mode}; current mode is {mode}")
        if (state["error_status"] or state["collision_state"] or state["drag_status"] or
                any(state["brake_released"]) or
                any(state[f"{part}_approach_state"] for part in ("arm", "j4", "j5", "j6"))):
            raise RuntimeError("joint motion is blocked by alarm, collision, skin trigger, drag or released brakes")
        if state["running_status"] or state["jog_status"]:
            raise RuntimeError("robot must be stopped before move_j or resume")
        if action == "move_j" and (state["pause_cmd_flag"] or state["run_queued_cmd"]):
            raise RuntimeError("move_j requires an idle robot without a paused or running queue")
        return sample

    def execute_motion(self, action, args, *, cancel=None):
        commands = {"move_j": "MovJ", "stop_motion": "Stop", "pause": "Pause", "resume": "Continue"}
        if action not in commands:
            raise ValueError(f"unsupported joint motion action: {action}")
        if args.get("confirm_move") is not True:
            raise ValueError("confirm_move=true is required for each motion request")
        parameters, options, requested = (), {}, {}
        if action == "move_j":
            angles = []
            for joint, (lower, upper) in zip(JOINT_NAMES, JOINT_LIMITS_DEG):
                value = args.get(f"{joint}_deg")
                if type(value) not in (int, float) or not lower <= value <= upper or not math.isfinite(value):
                    raise ValueError(f"{joint}_deg must be a finite number in [{lower}, {upper}] degrees")
                angles.append(float(value))
            for key in ("velocity", "acceleration"):
                value = args.get(key, 10)
                if type(value) is not int or not 1 <= value <= 100:
                    raise ValueError(f"{key} must be an integer from 1 to 100 percent")
            parameters = (angles, CoordinateType.JOINT)
            options = {"v": args.get("velocity", 10), "a": args.get("acceleration", 10), "cp": 0}
            requested = {f"{joint}_deg": angle for joint, angle in zip(JOINT_NAMES, angles)}
            requested.update(velocity=options["v"], acceleration=options["a"])
        def check_state(current_action):
            if cancel is not None and cancel.is_set():
                raise RuntimeError("motion interrupted")
            if current_action in ("move_j", "resume"):
                return self._check_preparation_state(current_action)
            return self._check_motion_state(current_action)
        return self._execute(action, commands[action], check_state, card="joint_move",
                             parameters=parameters, options=options, requested=requested)

    def _execute(self, action, command, check_state, *, card="joint_move", parameters=(), options=None, requested=None):
        with self._command_lock:
            sample = check_state(action)
            request = {"action": action, **(requested or {}), "timestamp_ms": int(time.time() * 1000),
                       "controller_ip": sample["controller_ip"], "read_only": False, "effect_verified": False}
            self._update(card, last_request={**request, "state": "sending"}, last_error=None)
            try:
                if self._connection is None or self._address != sample["controller_ip"]:
                    self._close()
                    self._address = sample["controller_ip"]
                    self._connection = self._factory(self._address, 29999)
                    self._connection.set_timeout(self.connect_timeout, self.receive_timeout)
                    self._connection.connect()
                    self._api = RobotControl(self._connection)
                    self._motion_api = Motion(self._connection)
                    self._update(dashboard_connected=True)
                # Connecting can take long enough for feedback/state to change.
                check_state(action)
                if action in ("move_j", "resume"):
                    sample = check_state(action)
                    if not sample["data"]["enable_status"]:
                        enable_reply = self._api.EnableRobot()
                        parse_response(enable_reply, "EnableRobot")
                        request["enable_response"] = enable_reply
                        deadline = time.monotonic() + self.enable_timeout
                        while True:
                            sample = check_state(action)
                            if sample["data"]["enable_status"]:
                                break
                            if time.monotonic() >= deadline:
                                raise TimeoutError("EnableRobot acknowledged but enabled feedback was not confirmed")
                            time.sleep(0.02)
                if card == "joint_move" and action in ("move_j", "resume"):
                    skin_reply = self._api.EnableSafeSkin(0)
                    parse_response(skin_reply, "EnableSafeSkin")
                    request["safe_skin_disable_response"] = skin_reply
                    check_state(action)
                if command == "MovJ":
                    # Keep clear + state recheck + move serialized as one request.
                    # A failed/uncertain clear must never be followed by motion.
                    clear_reply = self._api.ClearError()
                    parse_response(clear_reply, "ClearError")
                    request["clear_error_response"] = clear_reply
                    check_state(action)
                if action in ("move_j", "resume"):
                    # ACKs can precede the feedback packet reporting the effect.
                    # Wait for final motion conditions only after all preparation.
                    deadline = time.monotonic() + self.enable_timeout
                    while True:
                        check_state(action)
                        try:
                            self._check_motion_state(action)
                            break
                        except RuntimeError:
                            if time.monotonic() >= deadline:
                                raise
                            time.sleep(0.02)
                api = self._motion_api if command == "MovJ" else self._api
                reply = getattr(api, command)(*parameters, **(options or {}))
                queue_id = parse_reply(reply, command)
                request.update(state="submitted" if queue_id is not None else "accepted_unverified",
                               result_id=queue_id, response=reply)
                self._update(card, last_request=request)
                return request
            except Exception as exc:
                # A timeout may happen after transmission; never replay the write.
                request.update(state="rejected" if isinstance(exc, CommandRejected) else "failed_or_outcome_unknown", error=str(exc))
                self._update(card, last_request=request, last_error=str(exc))
                self._close()
                raise
