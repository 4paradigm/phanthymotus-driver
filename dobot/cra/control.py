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
    from .ownership import ArmOwnership
else:
    from feedback import disconnect, endpoint, positive_number
    from model import JOINT_NAMES, resolve_model
    from ownership import ArmOwnership


# CRA hardware manual V3.2, Appendix A, CR3A (degrees). The visualization
# URDF rounds radians and is not used as a controller safety configuration.
JOINT_LIMITS_DEG = ((-360, 360), (-360, 360), (-155, 155),
                    (-360, 360), (-360, 360), (-360, 360))
# ServoJ follow parameters (SDK defaults). Field-tested on CR3A at a 20 ms period:
# about 90 ms constant lag, 0.03 deg RMS tracking error after removing that lag.
SERVO_AHEAD_TIME = 50.0
SERVO_GAIN = 500.0


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
        self.ownership = ArmOwnership()
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
                          for card in ("joint_move", "teach")}

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
                if self.ownership.owner is not None:
                    raise RuntimeError("stop the active arm operation before changing its address or feedback port")
                self._close()
                for card in self._requests:
                    self._update(card, last_request=None, last_error=None)
                self.feedback.configure(address, port)

    def _check_preparation_state(self, action, *, allow_own_queue=False):
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
        allowed_modes = (4, 5, 7, 9, 10, 11) if allow_own_queue else (4, 5, 9, 10, 11)
        if (state["robot_mode"] not in allowed_modes or state["running_status"] or
                state["jog_status"] or state["drag_status"] or any(state["brake_released"])):
            raise RuntimeError("robot must be stationary with locked brakes before preparing motion")
        if action in ("move_j", "rel_joint_mov_j", "start_trajectory") and state["robot_mode"] == 10:
            raise RuntimeError("stop the paused queue before preparing a new motion or recording")
        if action in ("move_j", "rel_joint_mov_j", "start_trajectory") and state["robot_mode"] not in (9, 11) and (
                state["pause_cmd_flag"] or (state["run_queued_cmd"] and not allow_own_queue)):
            raise RuntimeError("move_j requires an idle robot without a paused or running queue")
        return sample

    def _wait_preparation_queue(self, action, check_state, queue_id, baseline):
        deadline = time.monotonic() + self.enable_timeout
        while True:
            sample = check_state(action, allow_own_queue=True)
            state = sample["data"]
            fresh = baseline is None or sample.get("packets_received", baseline) > baseline
            queue_seen = queue_id is None or state.get("current_command_id") == queue_id
            if fresh and queue_seen and state["robot_mode"] != 7 and not state["run_queued_cmd"]:
                return sample
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"preparation queue did not finish: submitted_id={queue_id}, "
                    f"current_id={state.get('current_command_id')}, mode={state['robot_mode']}, "
                    f"run_queued_cmd={state['run_queued_cmd']}, running_status={state['running_status']}")
            time.sleep(0.02)

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
        required_mode = 5 if action in ("move_j", "rel_joint_mov_j") else 10
        if mode != required_mode or not state["enable_status"]:
            raise RuntimeError(f"{action} requires an enabled robot in mode {required_mode}; current mode is {mode}")
        if (state["error_status"] or state["collision_state"] or state["drag_status"] or
                any(state["brake_released"]) or
                any(state[f"{part}_approach_state"] for part in ("arm", "j4", "j5", "j6"))):
            raise RuntimeError("joint motion is blocked by alarm, collision, skin trigger, drag or released brakes")
        if state["running_status"] or state["jog_status"]:
            raise RuntimeError("robot must be stopped before move_j or resume")
        if action in ("move_j", "rel_joint_mov_j") and (state["pause_cmd_flag"] or state["run_queued_cmd"]):
            raise RuntimeError("move_j requires an idle robot without a paused or running queue")
        return sample

    def execute_motion(self, action, args, *, cancel=None, owner="joint_move"):
        if action not in ("stop_motion", "pause"):
            self.ownership.check(owner)
        commands = {"move_j": "MovJ", "rel_joint_mov_j": "RelJointMovJ", "stop_motion": "Stop", "pause": "Pause", "resume": "Continue"}
        if action not in commands:
            raise ValueError(f"unsupported joint motion action: {action}")
        if args.get("confirm_move") is not True:
            raise ValueError("confirm_move=true is required for each motion request")
        parameters, options, requested = (), {}, {}
        if action in ("move_j", "rel_joint_mov_j"):
            detect_collision = args.get("detect_collision", False)
            if type(detect_collision) is not bool:
                raise ValueError("detect_collision must be a boolean true or false")
            requested["detect_collision"] = detect_collision
        if action in ("move_j", "rel_joint_mov_j"):
            values = []
            provided = 0
            relative = action == "rel_joint_mov_j"
            for joint, (lower, upper) in zip(JOINT_NAMES, JOINT_LIMITS_DEG):
                field = f"{joint}_offset_deg" if relative else f"{joint}_deg"
                if relative:
                    span = upper - lower
                    lower, upper = -span, span
                value = args.get(field)
                if value is None or (isinstance(value, str) and not value.strip()):
                    values.append(0.0 if relative else None)
                    continue
                provided += 1
                if type(value) not in (int, float) or not lower <= value <= upper or not math.isfinite(value):
                    raise ValueError(f"{field} must be a finite number in [{lower}, {upper}] degrees")
                values.append(float(value))
            if not provided:
                raise ValueError("provide at least one joint angle or offset")
            for key in ("velocity", "acceleration"):
                value = args.get(key, 10)
                if type(value) is not int or not 1 <= value <= 100:
                    raise ValueError(f"{key} must be an integer from 1 to 100 percent")
            parameters = tuple(values) if relative else (values, CoordinateType.JOINT)
            options = {"v": args.get("velocity", 10), "a": args.get("acceleration", 10), "cp": 0}
            requested.update({f"{joint}_offset_deg" if relative else f"{joint}_deg": value
                              for joint, value in zip(JOINT_NAMES, values)})
            requested.update(velocity=options["v"], acceleration=options["a"])
        def check_state(current_action, *, allow_own_queue=False):
            if cancel is not None and cancel.is_set():
                raise RuntimeError("motion interrupted")
            if current_action in ("move_j", "rel_joint_mov_j"):
                return self._check_preparation_state(current_action, allow_own_queue=allow_own_queue)
            return self._check_motion_state(current_action)
        return self._execute(action, commands[action], check_state, card="joint_move",
                             parameters=parameters, options=options, requested=requested)

    def _connect(self, address):
        if self._connection is None or self._address != address:
            self._close()
            self._address = address
            self._connection = self._factory(address, 29999)
            self._connection.set_timeout(self.connect_timeout, self.receive_timeout)
            self._connection.connect()
            self._api = RobotControl(self._connection)
            self._motion_api = Motion(self._connection)
            self._update(dashboard_connected=True)

    def prepare_record(self, *, cancel=None):
        """ClearError/EnableRobot and safe skin off (dragging the arm would trigger it);
        start_drag follows once recording is subscribed."""
        self.ownership.check("teach")

        def check(current_action, *, allow_own_queue=False):
            if cancel is not None and cancel.is_set():
                raise RuntimeError("teaching interrupted")
            return self._check_preparation_state(current_action, allow_own_queue=allow_own_queue)

        return self._execute("start_trajectory", None, check, card="teach",
                             requested={"detect_collision": False})

    def _immediate(self, command, cancel=None, *, require_idle):
        """One immediate Dashboard command, serialized with every other command.
        A cancelled request is dropped under the command lock, so it cannot follow a later stop."""
        self.ownership.check("teach")
        with self._command_lock:
            if cancel is not None and cancel.is_set():
                raise RuntimeError("teaching interrupted")
            if not self.feedback.address:
                raise RuntimeError("configure the controller IP before sending commands")
            if require_idle:
                address = self._check_motion_state("move_j")["controller_ip"]
            else:
                address = self.feedback.address
            request = {"action": command, "timestamp_ms": int(time.time() * 1000), "controller_ip": address,
                       "read_only": False, "effect_verified": False}
            try:
                self._connect(address)
                reply = getattr(self._api, command)()
                parse_response(reply, command)
                self._update("teach", last_request={**request, "state": "accepted", "response": reply},
                             last_error=None)
            except Exception as exc:
                exc.dobot_command_attempted = command
                self._update("teach", last_request={**request, "state": "failed_or_outcome_unknown",
                                                    "error": str(exc)}, last_error=str(exc))
                self._close()
                raise

    def start_drag(self, cancel):
        """Enter drag from rest (enabled, mode 5, no alarm); the effect is read from feedback."""
        self._immediate("StartDrag", cancel, require_idle=True)

    def stop_drag(self):
        """Leave drag; callers send this only while fresh feedback shows the drag we started."""
        self._immediate("StopDrag", require_idle=False)

    def prepare_playback(self, start_joints, args, *, cancel=None):
        """Same preparation as move_j (ClearError, enable, skin, ClearError), then
        confirm the arm is at the trajectory start. Sends no motion."""
        self.ownership.check("teach")
        if args.get("confirm_move") is not True:
            raise ValueError("confirm_move=true is required")
        skin = args.get("detect_collision", False)
        if type(skin) is not bool:
            raise ValueError("detect_collision must be a boolean")

        def check(action, *, allow_own_queue=False):
            if cancel is not None and cancel.is_set():
                raise RuntimeError("teaching interrupted")
            return self._check_preparation_state(action, allow_own_queue=allow_own_queue)

        return self._execute("move_j", "ServoJ", check, card="teach",
                             requested={"detect_collision": skin, "expected_start": list(start_joints),
                                        "start_tolerance_deg": args.get("start_tolerance_deg", 0.5)})

    def servo_j(self, joints, period_s, cancel):
        """One ServoJ follow target. Checked against cancel under the command lock,
        so no target can be transmitted after a Stop that follows the cancellation."""
        self.ownership.check("teach")
        with self._command_lock:
            if cancel.is_set():
                raise RuntimeError("teaching interrupted")
            try:
                self._connect(self.feedback.address)
                reply = self._motion_api.ServoJ(list(joints), t=period_s, aheadtime=SERVO_AHEAD_TIME, gain=SERVO_GAIN)
                return parse_reply(reply, "ServoJ")
            except Exception as exc:
                exc.dobot_command_attempted = "ServoJ"
                self._update("teach", last_error=str(exc))
                self._close()
                raise

    @staticmethod
    def _absolute_target(sample, requested):
        current = sample["data"]["joint_state"]["q_actual"]
        if len(current) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in current):
            raise RuntimeError("six finite actual joint angles are required to hold omitted joints")
        target = [float(actual if wanted is None else wanted) for actual, wanted in zip(current, requested)]
        for joint, angle, (lower, upper) in zip(JOINT_NAMES, target, JOINT_LIMITS_DEG):
            if not lower <= angle <= upper:
                raise ValueError(f"absolute target for {joint} is {angle} degrees, outside [{lower}, {upper}]")
        return target

    @staticmethod
    def _relative_target(sample, offsets):
        current = sample["data"]["joint_state"]["q_actual"]
        if len(current) != 6 or any(type(value) not in (int, float) or not math.isfinite(value) for value in current):
            raise RuntimeError("six finite actual joint angles are required for relative motion")
        target = [float(angle + offset) for angle, offset in zip(current, offsets)]
        for joint, angle, (lower, upper) in zip(JOINT_NAMES, target, JOINT_LIMITS_DEG):
            if not lower <= angle <= upper:
                raise ValueError(f"relative target for {joint} is {angle} degrees, outside [{lower}, {upper}]")
        return target

    def _execute(self, action, command, check_state, *, card="joint_move", parameters=(), options=None, requested=None):
        with self._command_lock:
            sample = check_state(action)
            if command == "RelJointMovJ":
                self._relative_target(sample, parameters)
            elif command == "MovJ":
                self._absolute_target(sample, parameters[0])
            request = {"action": action, **(requested or {}), "timestamp_ms": int(time.time() * 1000),
                       "controller_ip": sample["controller_ip"], "read_only": False, "effect_verified": False}
            self._update(card, last_request={**request, "state": "sending"}, last_error=None)
            try:
                self._connect(sample["controller_ip"])
                # Connecting can take long enough for feedback/state to change.
                check_state(action)
                if action in ("move_j", "rel_joint_mov_j", "start_trajectory"):
                    pre_enable_clear = self._api.ClearError()
                    parse_response(pre_enable_clear, "ClearError")
                    request["pre_enable_clear_error_response"] = pre_enable_clear
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
                if action in ("move_j", "rel_joint_mov_j", "start_trajectory"):
                    baseline = check_state(action).get("packets_received")
                    detect_collision = request["detect_collision"]
                    skin_reply = self._api.EnableSafeSkin(int(detect_collision))
                    skin_queue_id = parse_reply(skin_reply, "EnableSafeSkin")
                    response_key = "safe_skin_enable_response" if detect_collision else "safe_skin_disable_response"
                    request[response_key] = skin_reply
                    self._wait_preparation_queue(action, check_state, skin_queue_id, baseline)
                    if detect_collision:
                        request["safe_skin_sensitivity_responses"] = []
                        for part in (3, 4, 5, 6):
                            baseline = check_state(action).get("packets_received")
                            sensitivity_reply = self._api.SetSafeSkin(part, 1)
                            skin_queue_id = parse_reply(sensitivity_reply, "SetSafeSkin")
                            request["safe_skin_sensitivity_responses"].append(sensitivity_reply)
                            self._wait_preparation_queue(action, check_state, skin_queue_id, baseline)
                if command in ("MovJ", "RelJointMovJ", "ServoJ"):
                    # Keep clear + state recheck + move serialized as one request.
                    # A failed/uncertain clear must never be followed by motion.
                    baseline = check_state(action).get("packets_received")
                    clear_reply = self._api.ClearError()
                    parse_response(clear_reply, "ClearError")
                    request["clear_error_response"] = clear_reply
                    self._wait_preparation_queue(action, check_state, skin_queue_id, baseline)
                if action in ("move_j", "rel_joint_mov_j", "start_trajectory"):
                    # ACKs can precede the feedback packet reporting the effect.
                    # Wait for final motion conditions only after all preparation.
                    deadline = time.monotonic() + self.enable_timeout
                    while True:
                        check_state(action)
                        try:
                            self._check_motion_state("move_j" if action == "start_trajectory" else action)
                            break
                        except RuntimeError:
                            if time.monotonic() >= deadline:
                                raise
                            time.sleep(0.02)
                if command is None:
                    request["state"] = "prepared"
                    self._update(card, last_request=request)
                    return request
                if command in ("MovJ", "RelJointMovJ"):
                    final_sample = self._check_motion_state(action)
                    request["start_joint_angles_deg"] = list(final_sample["data"]["joint_state"]["q_actual"])
                    target = (self._relative_target(final_sample, parameters) if command == "RelJointMovJ"
                              else self._absolute_target(final_sample, parameters[0]))
                    request["target_joint_angles_deg"] = target
                    if command == "MovJ":
                        parameters = (target, CoordinateType.JOINT)
                        request.update({f"{joint}_deg": value for joint, value in zip(JOINT_NAMES, target)})
                if action == "resume":
                    paused_sample = self._check_motion_state(action)
                    request["resumed_command_id"] = paused_sample["data"].get("current_command_id")
                if command == "ServoJ":
                    current = self._check_motion_state("move_j")["data"]["joint_state"]["q_actual"]
                    if max(abs(a - b) for a, b in zip(current, request["expected_start"])) > request["start_tolerance_deg"]:
                        raise RuntimeError("robot is not at the trajectory start; play moves to the start first")
                    # The ServoJ stream itself is sent by the teach card, target by target.
                    request["state"] = "prepared"
                    self._update(card, last_request=request)
                    return request
                request["submission_baseline"] = self.feedback.snapshot().get("packets_received")
                api = self._motion_api if command in ("MovJ", "RelJointMovJ") else self._api
                request["command_attempted"] = command
                reply = getattr(api, command)(*parameters, **(options or {}))
                queue_id = parse_reply(reply, command)
                request.update(state="submitted" if queue_id is not None else "accepted_unverified",
                               result_id=queue_id, response=reply)
                self._update(card, last_request=request)
                return request
            except Exception as exc:
                exc.dobot_command_attempted = request.get("command_attempted")
                # A timeout may happen after transmission; never replay the write.
                request.update(state="rejected" if isinstance(exc, CommandRejected) else "failed_or_outcome_unknown", error=str(exc))
                self._update(card, last_request=request, last_error=str(exc))
                self._close()
                raise
