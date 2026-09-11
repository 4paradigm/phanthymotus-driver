"""PNDbotics Adam reinforcement-learning gRPC adapter for PhanthyMotus."""

from __future__ import annotations

import grpc
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "proto"))
import robot_control_pb2 as pb2
import robot_control_pb2_grpc as pb2_grpc


class AdamGrpcClient:
    """Wrapper for the ``pnd.robot`` service on port 50051.

    The service exposes a few reserved calls (currently SetVelocity and
    SetHeight).  Those calls intentionally return an explicit unsupported
    result instead of pretending that a command reached the robot.
    """

    def __init__(self, host: str = "10.10.20.127", port: int = 50051,
                 timeout: float = 5.0):
        self._addr = f"{host}:{port}"
        self._timeout = float(timeout)
        self._channel = None
        self._stub = None

    def connect(self):
        self._channel = grpc.insecure_channel(self._addr)
        self._stub = pb2_grpc.RobotControlStub(self._channel)

    def close(self):
        if self._channel:
            self._channel.close()
        self._channel = None
        self._stub = None

    def _ensure_connected(self):
        if self._stub is None:
            self.connect()

    def _call(self, method, request):
        self._ensure_connected()
        try:
            return getattr(self._stub, method)(request, timeout=self._timeout)
        except grpc.RpcError as exc:
            return self._rpc_error(exc)

    @staticmethod
    def _rpc_error(exc):
        return {"success": False, "error": exc.details(), "code": exc.code().name}

    @staticmethod
    def _unsupported(message):
        return {"success": False, "code": "UNSUPPORTED_IN_RL", "message": message}

    @staticmethod
    def _invalid(message):
        return {"success": False, "code": "INVALID_ARGUMENT", "message": message}

    @staticmethod
    def _response(response, fields=()):
        if isinstance(response, dict):
            return response
        result = {
            "success": bool(response.success),
            "message": response.message,
        }
        for field in fields:
            result[field] = getattr(response, field)
        return result

    def set_mode(self, mode) -> dict:
        if not isinstance(mode, str) or not mode.strip():
            return self._unsupported(
                "RL set_mode requires a state name returned by switchable_states"
            )
        response = self._call(
            "SetMode", pb2.SetModeRequest(target_state=mode.strip()))
        return self._response(response, ("current_state",))

    def set_speed(self, vx: float, vy: float, vyaw: float) -> dict:
        values = (vx, vy, vyaw)
        try:
            if any(isinstance(value, bool) or not math.isfinite(float(value))
                   for value in values):
                return self._invalid("velocity values must be finite numbers")
            if any(abs(float(value)) > 1.0 for value in values):
                return self._invalid("velocity values must be in [-1.0, 1.0]")
        except (TypeError, ValueError):
            return self._invalid("velocity values must be numbers")
        return self._unsupported(
            "PNDbotics RL SetVelocity is reserved; use a supported policy/input path"
        )

    # Name used by the RL tool; keep set_speed for the existing card contract.
    set_velocity = set_speed

    def set_height(self, height: float) -> dict:
        try:
            if isinstance(height, bool) or not math.isfinite(float(height)):
                return self._invalid("height must be a finite number")
            if abs(float(height)) > 1.0:
                return self._invalid("height must be in [-1.0, 1.0]")
        except (TypeError, ValueError):
            return self._invalid("height must be a number")
        return self._unsupported(
            "PNDbotics RL SetHeight is reserved and currently not connected"
        )

    def set_motion(self, command: str, motion_file: str = "") -> dict:
        command = str(command).upper()
        if command not in ("PLAY", "STOP"):
            return self._invalid("command must be PLAY or STOP")
        if command == "PLAY" and (
                not isinstance(motion_file, str)
                or not motion_file.endswith(".txt")):
            return self._invalid(
                "motion_file must be a robot-side .txt path when playing")
        enum_value = (
            pb2.SetMotionRequest.PLAY
            if command == "PLAY" else pb2.SetMotionRequest.STOP
        )
        response = self._call(
            "SetMotion",
            pb2.SetMotionRequest(command=enum_value, motion_file=motion_file),
        )
        return self._response(response, ("current_motion", "is_playing"))

    def set_tracking_motion(self, motion_file: str) -> dict:
        if not isinstance(motion_file, str) or not motion_file.endswith(".txt"):
            return self._invalid("motion_file must be a robot-side .txt path")
        response = self._call(
            "SetTrackingMotion",
            pb2.SetTrackingMotionRequest(motion_file=motion_file),
        )
        return self._response(response, ("current_tracking_motion",))

    def get_robot_state(self) -> dict:
        response = self._call("GetRobotState", pb2.GetRobotStateRequest())
        return self._response(response, (
            "fsm_state", "vx", "vy", "vyaw", "height",
            "current_motion_file", "motion_playing",
            "current_tracking_motion", "tracking_playing",
            "switchable_states", "available_actions",
        ))

    def get_stand_list(self) -> dict:
        state = self.get_robot_state()
        if not state.get("success"):
            return state
        return {
            "success": True,
            "switchable_states": state["switchable_states"],
            "available_actions": state["available_actions"],
        }

    def set_stand_motion(self, motion_id: int) -> dict:
        return self._unsupported(
            "RL uses SetMotion with a motion file path, not a traditional motion ID"
        )

    def set_stand_action(self, action_id: int) -> dict:
        return self._unsupported("RL does not support traditional stand action IDs")

    def set_stand_dynamic(self, **kwargs) -> dict:
        return self._unsupported(
            "PNDbotics RL SetHeight is reserved and has no pitch/roll/yaw equivalent"
        )

    def set_error_clear(self) -> dict:
        return self._unsupported("RL protocol has no SetErrorClear RPC")

    def set_carry_box(self, enable: bool) -> dict:
        return self._unsupported("RL protocol has no carry-box RPC")

    def set_control_mode(self, domain_id: int) -> dict:
        if isinstance(domain_id, bool) or domain_id not in (0, 1):
            return self._invalid(
                "domain_id must be 0 (Traditional) or 1 (RL)")
        response = self._call(
            "SetControlMode", pb2.SetControlModeRequest(domain_id=domain_id))
        return self._response(response)

    def get_control_state(self) -> dict:
        response = self._call(
            "GetControlState", pb2.GetControlStateRequest())
        return self._response(response, ("domain_id",))

    def shutdown(self, force: bool = False) -> dict:
        response = self._call(
            "Shutdown", pb2.ShutdownRequest(force=bool(force)))
        return self._response(response)
