"""
Unitree SLAM Navigate Client — DDS RPC wrapper for slam_operate service.

Service: slam_operate v1.0.0.1
API IDs:
  1801 — 开始建图
  1802 — 结束建图（保存 pcd）
  1804 — 初始化位姿（加载地图 + 重定位）
  1102 — 位姿导航
  1201 — 暂停导航
  1202 — 恢复导航
  1901 — 关闭 SLAM
"""

import json

from ...rpc.client import Client


SLAM_SERVICE_NAME = "slam_operate"
SLAM_API_VERSION = "1.0.0.1"

SLAM_API_START_MAPPING = 1801
SLAM_API_STOP_MAPPING = 1802
SLAM_API_INIT_POSE = 1804
SLAM_API_NAVIGATE_TO = 1102
SLAM_API_PAUSE_NAV = 1201
SLAM_API_RESUME_NAV = 1202
SLAM_API_SHUTDOWN = 1901


class SlamClient(Client):
    def __init__(self):
        super().__init__(SLAM_SERVICE_NAME)

    def Init(self):
        self._SetApiVerson(SLAM_API_VERSION)
        self._RegistApi(SLAM_API_START_MAPPING, 0)
        self._RegistApi(SLAM_API_STOP_MAPPING, 0)
        self._RegistApi(SLAM_API_INIT_POSE, 0)
        self._RegistApi(SLAM_API_NAVIGATE_TO, 0)
        self._RegistApi(SLAM_API_PAUSE_NAV, 0)
        self._RegistApi(SLAM_API_RESUME_NAV, 0)
        self._RegistApi(SLAM_API_SHUTDOWN, 0)
        self.SetTimeout(5.0)

    def StartMapping(self) -> tuple:
        """开始建图。返回 (code, response_json_str)。"""
        param = json.dumps({"data": {"slam_type": "indoor"}})
        return self._Call(SLAM_API_START_MAPPING, param)

    def StopMapping(self, address: str) -> tuple:
        """结束建图并保存 pcd 到指定路径。超时设为 10 秒（保存 PCD 耗时较长）。"""
        param = json.dumps({"data": {"address": address}})
        self.SetTimeout(10.0)
        try:
            return self._Call(SLAM_API_STOP_MAPPING, param)
        finally:
            self.SetTimeout(5.0)

    def InitPose(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 q_x: float = 0.0, q_y: float = 0.0, q_z: float = 0.0, q_w: float = 1.0,
                 address: str = "") -> tuple:
        """加载地图并初始化位姿（重定位）。"""
        param = json.dumps({"data": {
            "x": x, "y": y, "z": z,
            "q_x": q_x, "q_y": q_y, "q_z": q_z, "q_w": q_w,
            "address": address,
        }})
        return self._Call(SLAM_API_INIT_POSE, param)

    def NavigateTo(self, x: float, y: float, z: float = 0.0,
                   q_x: float = 0.0, q_y: float = 0.0, q_z: float = 0.0, q_w: float = 1.0,
                   speed: float = 0.5, mode: int = 1) -> tuple:
        """导航到目标位姿（距离不超过 10m）。speed: 0.2~0.8 m/s (Go2), mode: 1=停障, 0=绕障"""
        param = json.dumps({"data": {
            "targetPose": {
                "x": x, "y": y, "z": z,
                "q_x": q_x, "q_y": q_y, "q_z": q_z, "q_w": q_w,
            },
            "mode": mode,
            "speed": speed,
        }})
        return self._Call(SLAM_API_NAVIGATE_TO, param)

    def PauseNav(self) -> tuple:
        """暂停导航。"""
        param = json.dumps({"data": {}})
        return self._Call(SLAM_API_PAUSE_NAV, param)

    def ResumeNav(self) -> tuple:
        """恢复导航。"""
        param = json.dumps({"data": {}})
        return self._Call(SLAM_API_RESUME_NAV, param)

    def Shutdown(self) -> tuple:
        """关闭 SLAM 服务。"""
        param = json.dumps({"data": {}})
        return self._Call(SLAM_API_SHUTDOWN, param)
