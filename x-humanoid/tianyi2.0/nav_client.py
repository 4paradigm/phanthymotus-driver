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
_UPLOAD_TIMEOUT = 30  # seconds for map upload/download


class SlamtecClient:
    """Synchronous HTTP client for Slamtec chassis REST API."""

    def __init__(self, base_url: str = "http://192.168.11.1:1448"):
        self._base = base_url.rstrip("/")

    def _get(self, path: str) -> dict:
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, method="GET")
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                resp_body = resp.read()
                if not resp_body:
                    return {}
                result = json.loads(resp_body)
                if isinstance(result, dict):
                    return result
                if isinstance(result, list):
                    return {"raw": result}
                # BooleanResponse / IntegerResponse: raw boolean or integer
                return {"raw": result}
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _post(self, path: str, body) -> dict:
        url = f"{self._base}{path}"
        data = json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                resp_body = resp.read()
                if not resp_body:
                    return {}
                result = json.loads(resp_body)
                if isinstance(result, dict):
                    return result
                if isinstance(result, list):
                    return {"raw": result}
                return {"raw": result}
        except urllib.error.HTTPError as e:
            # Extract response body for detailed error info (e.g. 400 Bad Request)
            detail = ""
            try:
                detail = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            return {"error": f"HTTP {e.code}: {e.reason}", "detail": detail}
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _put(self, path: str, body) -> dict:
        url = f"{self._base}{path}"
        data = json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, method="PUT",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                resp_body = resp.read()
                if not resp_body:
                    return {}
                result = json.loads(resp_body)
                if isinstance(result, dict):
                    return result
                if isinstance(result, list):
                    return {"raw": result}
                return {"raw": result}
        except urllib.error.HTTPError as e:
            detail = ""
            try:
                detail = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            return {"error": f"HTTP {e.code}: {e.reason}", "detail": detail}
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _delete(self, path: str) -> dict:
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, method="DELETE")
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                body = resp.read()
                if not body:
                    return {}
                result = json.loads(body)
                if isinstance(result, dict):
                    return result
                if isinstance(result, list):
                    return {"raw": result}
                return {"raw": result}
        except urllib.error.HTTPError as e:
            detail = ""
            try:
                detail = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            return {"error": f"HTTP {e.code}: {e.reason}", "detail": detail}
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def _download_binary(self, path: str) -> tuple[bytes | None, str | None]:
        """Download binary data from path. Returns (data, error)."""
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, method="GET")
        try:
            with urllib.request.urlopen(req, timeout=_UPLOAD_TIMEOUT) as resp:
                return resp.read(), None
        except Exception as e:
            return None, str(e)

    def _upload_binary(self, path: str, data: bytes, content_type: str = "application/octet-stream") -> dict:
        """Upload binary data to path via PUT."""
        url = f"{self._base}{path}"
        req = urllib.request.Request(url, data=data, method="PUT",
                                     headers={"Content-Type": content_type})
        try:
            with urllib.request.urlopen(req, timeout=_UPLOAD_TIMEOUT) as resp:
                body = resp.read()
                if not body:
                    return {}
                result = json.loads(body)
                if isinstance(result, dict):
                    return result
                return {"raw": result}
        except urllib.error.HTTPError as e:
            detail = ""
            try:
                detail = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            return {"error": f"HTTP {e.code}: {e.reason}", "detail": detail}
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

    def set_max_moving_speed(self, speed: float) -> dict:
        """设置最大移动速度 (m/s)"""
        return self._post("/api/core/motion/v1/max_moving_speed", {"speed": speed})

    def set_max_angular_speed(self, speed: float) -> dict:
        """设置最大旋转速度 (rad/s)"""
        return self._post("/api/core/motion/v1/max_angular_speed", {"speed": speed})

    def get_safety_status(self) -> dict:
        """获取安全状态 (碰撞/跌落/虚拟墙/急停)"""
        return self._get("/api/core/safety/v1/status")

    def get_action_status(self, action_id: str) -> dict:
        """查询Action状态 {status: 0-4, result: 0/-1/-2}"""
        return self._get(f"/api/core/motion/v1/actions/{action_id}")

    def move_to(self, x: float, y: float, yaw: Optional[float] = None,
                speed_ratio: Optional[float] = None, mode: Optional[int] = None,
                fail_retry_count: Optional[int] = None,
                acceptable_precision: Optional[float] = None,
                strategy: Optional[str] = None,
                ignore_dynamic_obstacles: Optional[bool] = None,
                precise: Optional[bool] = None) -> dict:
        """
        自主导航移动到目标点。
        action_name: slamtec.agent.actions.MoveToAction
        mode: 0=free navigation (default), 1=strict-track, 2=track-priority
        """
        options: dict = {"target": {"x": x, "y": y, "z": 0}}
        move_opts: dict = {"mode": mode if mode is not None else 0}
        flags = []
        if yaw is not None:
            move_opts["yaw"] = yaw
            flags.append("with_yaw")
        if speed_ratio is not None:
            move_opts["speed_ratio"] = speed_ratio
        if fail_retry_count is not None:
            move_opts["fail_retry_count"] = fail_retry_count
            flags.append("fail_retry_count")
        if acceptable_precision is not None:
            move_opts["acceptable_precision"] = acceptable_precision
        if ignore_dynamic_obstacles:
            flags.append("find_path_ignoring_dynamic_obstacles")
        if precise:
            flags.append("precise")
        if flags:
            move_opts["flags"] = flags
        options["move_options"] = move_opts
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": options,
        })

    def set_motion_strategy(self, strategy: str) -> dict:
        """
        设置运动策略。
        strategy: default, depot, inventory, delivery, low_speed
        """
        return self._put("/api/core/motion/v1/strategies/:current", {"strategy": strategy})

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

    def get_laser_scan(self) -> dict:
        """获取当前激光观测帧"""
        return self._get("/api/core/system/v1/laserscan")

    # ── Mapping (建图) ─────────────────────────────────────────────────────────

    def start_mapping(self) -> dict:
        """开始建图（切换到建图模式）"""
        return self._put("/api/core/slam/v1/mapping/:enable", {"enable": True})

    def stop_mapping(self) -> dict:
        """停止建图（切换到定位模式）"""
        return self._put("/api/core/slam/v1/mapping/:enable", {"enable": False})

    def get_mapping_status(self) -> dict:
        """获取建图状态：true=建图模式，false=定位模式"""
        return self._get("/api/core/slam/v1/mapping/:enable")

    def recover_localization(self) -> dict:
        """重定位（使用RecoverLocalizationAction运动行为）"""
        return self._post("/api/core/motion/v1/actions", {
            "action_name": "slamtec.agent.actions.RecoverLocalizationAction",
            "options": {
                "relocalization_options": {
                    "recover_movement_type": "NoMove"
                }
            },
        })

    def set_pose_init(self, x: float, y: float, yaw: float) -> dict:
        """设置机器人位姿 (Pose3D: x, y, z=0, yaw, pitch=0, roll=0)"""
        return self._put("/api/core/slam/v1/localization/pose", {"x": x, "y": y, "z": 0, "yaw": yaw, "pitch": 0, "roll": 0})

    def clear_map(self) -> dict:
        """清空地图"""
        return self._delete("/api/core/slam/v1/maps")

    def get_current_map(self) -> tuple[bytes | None, str | None]:
        """获取STCM复合地图（二进制数据，包含完整地图信息）。Returns (data, error)."""
        return self._download_binary("/api/core/slam/v1/maps/stcm")

    def upload_map(self, map_data: bytes) -> dict:
        """上传STCM复合地图数据到底盘。PUT /api/core/slam/v1/maps/stcm
        上传后机器人位姿会被重置到原点，需调用 set_pose_init / recover_localization。"""
        return self._upload_binary("/api/core/slam/v1/maps/stcm", map_data)

    def set_map_update(self, enabled: bool) -> dict:
        """启用/禁用地图更新（建图模式开关）"""
        return self._put("/api/core/slam/v1/mapping/:enable", {"enable": enabled})

    # ── Navigation Status ──────────────────────────────────────────────────────

    def get_nav_status(self) -> dict:
        """获取当前导航/运动动作状态。

        API 返回 ActionInfo: {action_id, action_name, stage, state: {status, result, reason}}
        本方法将 state 展开到顶层，返回 {action_state, result, reason, action_id, ...}，
        action_state=-1 表示无活跃action（区别于HTTP错误）。
        """
        url = f"{self._base}/api/core/motion/v1/actions/:current"
        req = urllib.request.Request(url, method="GET")
        try:
            with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
                body = resp.read()
                if not body:
                    return {"action_state": -1}
                try:
                    result = json.loads(body)
                except (json.JSONDecodeError, ValueError):
                    return {"action_state": -1}
                if not isinstance(result, dict):
                    return {"action_state": -1}
                # Unwrap nested state: {state: {status, result, reason}} → flat top-level
                state = result.pop("state", None)
                if isinstance(state, dict):
                    result["action_state"] = state.get("status")
                    result["result"] = state.get("result")
                    result["reason"] = state.get("reason", "")
                else:
                    result["action_state"] = result.get("action_state")
                    result["result"] = result.get("result")
                return result
        except urllib.error.HTTPError as e:
            # 404 means no active action — not an error
            if e.code == 404:
                return {"action_state": -1}
            return {"error": str(e)}
        except urllib.error.URLError as e:
            return {"error": str(e)}
        except Exception as e:
            return {"error": str(e)}

    def cancel_action(self) -> dict:
        """取消当前导航/运动动作"""
        return self._delete("/api/core/motion/v1/actions/:current")

    # ── Artifacts (虚拟墙/轨道/区域) ────────────────────────────────────────────

    def get_lines(self, usage: str) -> dict:
        """获取虚拟线段。usage: 'tracks' 虚拟轨道, 'walls' 虚拟墙"""
        return self._get(f"/api/core/artifact/v1/lines/{usage}")

    def add_lines(self, usage: str, lines: list[dict]) -> dict:
        """添加虚拟线段。usage: 'tracks'/'walls', lines: [{start:{x,y}, end:{x,y}, metadata:{}}]"""
        return self._post(f"/api/core/artifact/v1/lines/{usage}", lines)

    def modify_lines(self, usage: str, lines: list[dict]) -> dict:
        """修改虚拟线段。lines 需包含有效 id"""
        return self._put(f"/api/core/artifact/v1/lines/{usage}", lines)

    def clear_lines(self, usage: str) -> dict:
        """清空某一类虚拟线段"""
        return self._delete(f"/api/core/artifact/v1/lines/{usage}")

    def remove_line(self, usage: str, line_id: int) -> dict:
        """删除指定虚拟线段"""
        return self._delete(f"/api/core/artifact/v1/lines/{usage}/{line_id}")

    def get_rectangle_areas(self, usage: str) -> dict:
        """获取矩形区域。usage: forbidden_area/elevator_area/dangerous_area/coverage_area/maintenance_area/sensor_disable_area/restricted_area"""
        return self._get(f"/api/core/artifact/v1/rectangle-areas/{usage}")

    def add_rectangle_area(self, usage: str, area: dict, metadata: dict | None = None) -> dict:
        """添加矩形区域。area: {start:{x,y}, end:{x,y}, half_width:float}
        metadata: 区域元数据，API要求必须提供（可为空对象{}）"""
        body: dict = {"area": area, "metadata": metadata if metadata is not None else {}}
        return self._post(f"/api/core/artifact/v1/rectangle-areas/{usage}", body)

    def edit_rectangle_area(self, usage: str, area_id: int, area: dict | None = None, metadata: dict | None = None) -> dict:
        """编辑矩形区域"""
        body: dict = {}
        if area is not None:
            body["area"] = area
        if metadata is not None:
            body["metadata"] = metadata
        return self._put(f"/api/core/artifact/v1/rectangle-areas/{usage}/{area_id}", body)

    def clear_rectangle_areas(self, usage: str) -> dict:
        """清空某一类矩形区域"""
        return self._delete(f"/api/core/artifact/v1/rectangle-areas/{usage}")

    def remove_rectangle_area(self, usage: str, area_id: int) -> dict:
        """删除指定矩形区域"""
        return self._delete(f"/api/core/artifact/v1/rectangle-areas/{usage}/{area_id}")

    # ── Artifacts — POI (地图兴趣点) ────────────────────────────────────────────

    def get_pois(self) -> dict:
        """获取当前地图中的所有POI"""
        return self._get("/api/core/artifact/v1/pois")

    def add_poi(self, poi: dict) -> dict:
        """添加POI。poi: {id: uuid, pose?: {x,y,yaw}, metadata: {display_name, type, ...}}
        建图时建议不包含pose，底盘会用机器人当前位置创建POI并记录传感器观测信息，闭环后自动调整位姿。"""
        return self._post("/api/core/artifact/v1/pois", poi)

    def adjust_pois(self) -> dict:
        """优化POI位姿（建图闭环后调用，减少位姿调整误差）"""
        return self._post("/api/core/artifact/v1/pois/:adjust", {})

    def get_poi(self, poi_id: str) -> dict:
        """根据ID查找POI"""
        return self._get(f"/api/core/artifact/v1/pois/{poi_id}")

    def modify_poi(self, poi_id: str, pose: dict | None = None, metadata: dict | None = None) -> dict:
        """修改POI。pose使用Pose3D格式(x,y,z,yaw,pitch,roll)，z/pitch/roll默认0。"""
        body: dict = {}
        if pose is not None:
            # API requires Pose3D (x,y,z,yaw,pitch,roll); fill defaults for 2D input
            p3d = {"x": pose.get("x", 0), "y": pose.get("y", 0), "z": pose.get("z", 0),
                    "yaw": pose.get("yaw", 0), "pitch": pose.get("pitch", 0), "roll": pose.get("roll", 0)}
            body["pose"] = p3d
        if metadata is not None:
            body["metadata"] = metadata
        return self._put(f"/api/core/artifact/v1/pois/{poi_id}", body)

    def delete_poi(self, poi_id: str) -> dict:
        """删除POI"""
        return self._delete(f"/api/core/artifact/v1/pois/{poi_id}")

    def clear_pois(self) -> dict:
        """清空所有POI"""
        return self._delete("/api/core/artifact/v1/pois")
