"""PNDbotics Adam reinforcement-learning gRPC adapter for PhanthyMotus."""

import grpc
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "proto"))
import robot_control_pb2 as pb2
import robot_control_pb2_grpc as pb2_grpc


class AdamGrpcClient:
    """Compatibility wrapper for the pnd.robot service on port 50051."""

    def __init__(self, host: str = "10.10.20.127", port: int = 50051):
        self._addr = f"{host}:{port}"
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

    @staticmethod
    def _rpc_error(exc):
        return {"success": False, "error": exc.details(), "code": exc.code().name}

    @staticmethod
    def _unsupported(message):
        return {"success": False, "code": "UNSUPPORTED_IN_RL", "message": message}

    def set_mode(self, mode) -> dict:
        self._ensure_connected()
        if not isinstance(mode, str) or not mode.strip():
            return self._unsupported(
                "RL set_mode requires a state name returned by switchable_states"
            )
        try:
            resp = self._stub.SetMode(
                pb2.SetModeRequest(target_state=mode.strip()), timeout=5
            )
            return {
                "success": resp.success,
                "message": resp.message,
                "current_state": resp.current_state,
            }
        except grpc.RpcError as exc:
            return self._rpc_error(exc)

    def set_speed(self, vx: float, vy: float, vyaw: float) -> dict:
        return self._unsupported(
            "PNDbotics RL SetVelocity is reserved; use a supported policy/input path"
        )

    def get_robot_state(self) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.GetRobotState(pb2.GetRobotStateRequest(), timeout=5)
            return {
                "success": resp.success,
                "fsm_state": resp.fsm_state,
                "vx": resp.vx,
                "vy": resp.vy,
                "vyaw": resp.vyaw,
                "height": resp.height,
                "current_motion_file": resp.current_motion_file,
                "motion_playing": resp.motion_playing,
                "current_tracking_motion": resp.current_tracking_motion,
                "tracking_playing": resp.tracking_playing,
                "switchable_states": list(resp.switchable_states),
                "available_actions": list(resp.available_actions),
            }
        except grpc.RpcError as exc:
            return self._rpc_error(exc)

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
