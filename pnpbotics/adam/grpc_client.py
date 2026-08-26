"""gRPC client wrapper for PNPbotics Adam locomotion control."""

import grpc
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "proto"))
import adam_control_pb2 as pb2
import adam_control_pb2_grpc as pb2_grpc


class AdamGrpcClient:
    """Wraps gRPC calls to Adam's RobotControl service on port 6666."""

    def __init__(self, host: str = "localhost", port: int = 6666):
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

    def set_mode(self, mode: int) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetMode(pb2.SetModeRequest(mode=mode), timeout=5)
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def set_speed(self, vx: float, vy: float, vyaw: float) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetSpeed(
                pb2.SetSpeedRequest(vx=vx, vy=vy, vyaw=vyaw), timeout=5
            )
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def set_stand_motion(self, motion_id: int) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetStandMotion(
                pb2.SetStandMotionRequest(motion_id=motion_id), timeout=5
            )
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def set_stand_action(self, action_id: int) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetStandAction(
                pb2.SetActionRequest(action_id=action_id), timeout=5
            )
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def set_stand_dynamic(self, pitch: float = 0, roll: float = 0,
                          yaw: float = 0, height: float = 0) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetStandDynamic(
                pb2.SetDynamicStandRequest(
                    pitch=pitch, roll=roll, yaw=yaw, height=height
                ),
                timeout=5,
            )
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def get_robot_state(self) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.GetRobotState(pb2.GetRobotStateRequest(), timeout=5)
            return {
                "code": resp.code,
                "message": resp.message,
                "mode": resp.mode,
                "gait_state": resp.gait_state,
                "battery_percentage": resp.battery_percentage,
            }
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def get_stand_list(self) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.GetStandList(pb2.GetStandListRequest(), timeout=5)
            return {
                "code": resp.code,
                "message": resp.message,
                "motions": [{"id": m.id, "name": m.name} for m in resp.motions],
                "actions": [{"id": a.id, "name": a.name} for a in resp.actions],
            }
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def set_error_clear(self) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetErrorClear(pb2.SetErrorClearRequest(), timeout=5)
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}

    def set_carry_box(self, enable: bool) -> dict:
        self._ensure_connected()
        try:
            resp = self._stub.SetStandCarryBox(
                pb2.SetCarryBoxRequest(enable=enable), timeout=5
            )
            return {"code": resp.code, "message": resp.message}
        except grpc.RpcError as e:
            return {"error": str(e.details()), "code": e.code().name}
