"""Read-only R1 health report built from the existing StatePlugin samples."""

import math
import time
from datetime import datetime, timezone


# Three missed publication intervals count as missing data.
SOURCE_INTERVALS = {"battery": 1.0, "imu": 0.05, "joints": 0.1, "mainboard": 2.0}


def _temperature_values(source: str, data: dict) -> list[float]:
    if source == "joints":
        raw = [value for joint in data.get("joints", []) for value in joint.get("temp", [])]
    else:
        raw = data.get("temperature", [])
        if not isinstance(raw, list):
            raw = [raw]
    return [float(value) for value in raw
            if isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value)]


def _timestamp(seconds: float) -> str:
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def evaluate_health(samples: dict, config: dict, *, now_monotonic: float | None = None,
                    now_wall: float | None = None) -> dict:
    """Evaluate a copy of cached samples without touching the robot or its SDK."""
    now_monotonic = time.monotonic() if now_monotonic is None else now_monotonic
    now_wall = time.time() if now_wall is None else now_wall
    battery_min = config.get("battery_min_soc", 20)
    temperature_limits = config.get("temperature_limits") or {}
    checks = {}

    for source, interval in SOURCE_INTERVALS.items():
        sample = samples.get(source)
        check = {
            "status": "数据不足",
            "data": sample["data"] if sample else None,
            "received_at": _timestamp(sample["received_at"]) if sample else None,
            "age_seconds": None,
            "reasons": [],
            "temperature": {"status": "未判定", "maximum": None,
                            "limit": temperature_limits.get(source)},
        }
        checks[source] = check
        if sample is None:
            check["reasons"].append("尚未收到数据")
            if temperature_limits.get(source) is not None:
                check["temperature"]["status"] = "数据不足"
            continue

        age = max(0.0, now_monotonic - sample["received_monotonic"])
        check["age_seconds"] = round(age, 3)
        if age > interval * 3:
            check["reasons"].append(f"数据已过期（超过 {interval * 3:g} 秒）")
            if temperature_limits.get(source) is not None:
                check["temperature"]["status"] = "数据不足"
            continue

        data = sample["data"]
        if not isinstance(data, dict):
            check["reasons"].append("数据格式无效")
            if temperature_limits.get(source) is not None:
                check["temperature"]["status"] = "数据不足"
            continue
        if source == "battery":
            soc = data.get("soc")
            if not isinstance(soc, (int, float)) or isinstance(soc, bool) or not math.isfinite(soc) or not 0 <= soc <= 100:
                check["reasons"].append("电量读数无效")
                if temperature_limits.get(source) is not None:
                    check["temperature"]["status"] = "数据不足"
                continue
            if soc < battery_min:
                check["status"] = "异常"
                check["reasons"].append(f"电量 {soc}% 低于 {battery_min}%")
        elif source == "joints" and not data.get("joints"):
            check["reasons"].append("关节读数为空")
            if temperature_limits.get(source) is not None:
                check["temperature"]["status"] = "数据不足"
            continue
        elif source == "imu" and not data.get("rpy"):
            check["reasons"].append("IMU 姿态读数为空")
            if temperature_limits.get(source) is not None:
                check["temperature"]["status"] = "数据不足"
            continue

        if check["status"] != "异常":
            check["status"] = "正常"

        temperatures = _temperature_values(source, data)
        if temperatures:
            check["temperature"]["maximum"] = max(temperatures)
        limit = temperature_limits.get(source)
        if limit is None:
            check["temperature"]["reason"] = "温度界限未配置"
        elif not temperatures:
            check["temperature"]["status"] = "数据不足"
            check["temperature"]["reason"] = "温度读数为空"
            if check["status"] == "正常":
                check["status"] = "数据不足"
            check["reasons"].append("温度读数为空")
        elif max(temperatures) > limit:
            check["temperature"]["status"] = "异常"
            check["status"] = "异常"
            check["reasons"].append(f"最高温度 {max(temperatures):g} 超过 {limit:g}")
        else:
            check["temperature"]["status"] = "正常"

    statuses = {check["status"] for check in checks.values()}
    overall = "异常" if "异常" in statuses else "数据不足" if "数据不足" in statuses else "正常"
    unassessed = [name for name, check in checks.items()
                  if check["temperature"]["status"] == "未判定"]
    summary = {
        "异常": "已发现预设异常；请查看各项原因。",
        "数据不足": "状态数据不完整或已过期，无法完成检查。",
        "正常": "已启用的检查项未发现异常；这不是运动安全许可。",
    }[overall]
    if unassessed:
        summary += " 未配置温度界限的项目未作温度安全判断。"
    return {
        "status": overall,
        "summary": summary,
        "checked_at": _timestamp(now_wall),
        "checks": checks,
        "temperature_unassessed": unassessed,
    }


class HealthCheckPlugin:
    PREFIX = "health_check"

    def __init__(self, plugin_config: dict, state_plugin=None):
        self._config = plugin_config
        self._state_plugin = state_plugin
        minimum = plugin_config.get("battery_min_soc", 20)
        if not isinstance(minimum, (int, float)) or isinstance(minimum, bool) or not math.isfinite(minimum) or not 0 <= minimum <= 100:
            raise ValueError("health_check.battery_min_soc must be between 0 and 100")
        limits = plugin_config.get("temperature_limits") or {}
        if not isinstance(limits, dict) or any(
            key not in SOURCE_INTERVALS or not isinstance(value, (int, float))
            or isinstance(value, bool) or not math.isfinite(value)
            for key, value in limits.items()
        ):
            raise ValueError("health_check.temperature_limits must contain finite numeric limits for known sources")

    def get_tool(self) -> dict:
        return {
            "name": "health_check",
            "type": "actuator",
            "multiInstance": False,
            "description": "Read-only R1 pre-operation health report: battery, IMU, joints and mainboard. Does not move or authorize movement.",
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["check"],
                                          "description": "Generate a health report"}},
                "required": ["action"],
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}
        if action == "check":
            samples = self._state_plugin.health_snapshot() if self._state_plugin else {}
            return evaluate_health(samples, self._config)
        return None
