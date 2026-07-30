#!/usr/bin/env python3
"""
x-humanoid/tianyi2.0/nav_client.py — Slamtec 底盘 HTTP REST API 客户端。

通过 HTTP 调用 Slamtec 底盘的 RESTful API 实现导航控制。
API文档: https://docs.slamtec.com (Swagger UI)

底盘地址默认: http://192.168.11.1:1448
"""

import json
import urllib.request
import urllib.error
from typing import Optional

_TIMEOUT = 5  # seconds


class SlamtecClient:
    """Synchronous HTTP client for Slamtec chassis REST API."""

    def __init__(self, base_url: str = "http://192.168.11.1:1448"):
        self._base = base_url.rstrip("/")

    def _get(self, path: str) -> dict:
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, method="GET")
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                return json.loads(resp.read())
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _post(self, path: str, body: dict) -> dict:
        url = f"{self._base}{path}"
        data = json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                return json.loads(resp.read())
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _put(self, path: str, body: dict) -> dict:
        url = f"{self._base}{path}"
        data = json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, method="PUT",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                return json.loads(resp.read())
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _delete(self, path: str) -> dict:
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, method="DELETE")
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                return json.loads(resp.read())
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    # ── SLAM / Localization ───────────────────────────────────────────────────

    def get_pose(self) -> dict:
        """获取机器人位姿 {x, y, z, yaw, pitch, roll}"""
        return self._get("/api/core/slam/v1/localization/pose")

    def get_localization_quality(self) -> dict:
        """获取定位质量"""
        return self._get("/api/core/slam/v1/localization/quality")

    # ── Motion ────────────────────────────────────────────────────────────────

    def get_current_action(self) -> dict:
        """获取当前运动行为"""
        return self._get("/api/core/motion/v1/actions/:current")

    def cancel_current_action(self) -> dict:
        """终止当前运动行为"""
        return self._delete("/api/core/motion/v1/actions/:current")

    def get_speed(self) -> dict:
        """获取当前运动速度"""
        return self._get("/api/core/motion/v1/speed")

    def get_action_status(self, action_id: str) -> dict:
        """查询Action状态 {status: 0-4, result: 0/-1/-2}"""
        return self._get(f"/api/core/motion/v1/actions/{action_id}")

    def move_to(self, x: float, y: float, yaw: Optional[float] = None,
                speed_ratio: Optional[float] = None) -> dict:
        """
        自主导航移动到目标点。
        action_name: slamtec.agent.actions.MoveToAction
        """
        options: dict = {"target": {"x": x, "y": y}}
        move_opts: dict = {"mode": 0}  # 自由导航
        flags = []
        if yaw is not None:
            move_opts["yaw"] = yaw
            flags.append("with_yaw")
        if speed_ratio is not None:
            move_opts["speed_ratio"] = speed_ratio
        if flags:
            move_opts["flags"] = flags
        options["move_options"] = move_opts
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": options,
        })

    def move_by(self, direction: int, duration: int = 500) -> dict:
        """
        遥控方向移动 (不避障)。
        direction: 0=前进, 1=后退, 2=右转, 3=左转
        duration: 持续时间(ms), 默认500ms
        """
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.MoveByAction",
            "options": {"direction": direction, "duration": duration},
        })

    def rotate(self, angle_rad: float) -> dict:
        """
        原地旋转指定角度。
        angle_rad: 弧度, 正数=逆时针, 负数=顺时针
        """
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.RotateAction",
            "options": {"angle": angle_rad},
        })

    def rotate_to(self, angle_rad: float) -> dict:
        """
        原地旋转到指定绝对角度。
        angle_rad: 目标yaw值(弧度)
        """
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.RotateToAction",
            "options": {"angle": angle_rad},
        })

    def go_home(self) -> dict:
        """自主回桩充电"""
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.GoHomeAction",
            "options": {"gohome_options": {"flags": "dock"}},
        })

    # ── System ────────────────────────────────────────────────────────────────

    def get_power_status(self) -> dict:
        """获取底盘电源状态"""
        return self._get("/api/core/system/v1/power/status")

    def get_robot_health(self) -> dict:
        """获取底盘健康状态"""
        return self._get("/api/core/system/v1/robot/health")

    def get_robot_info(self) -> dict:
        """获取底盘设备信息"""
        return self._get("/api/core/system/v1/robot/info")
