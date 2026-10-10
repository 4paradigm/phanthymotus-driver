"""Read-only pre-operation report for Noetix Bumi."""

import math
import time
from datetime import datetime, timezone


SOURCE_INTERVALS = {"battery": 1.0, "imu": 0.05, "joints": 0.1}


def valid_quaternion(value):
    return (isinstance(value, list) and len(value) == 4
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in value)
            and any(v != 0 for v in value))


def valid_joints(value):
    return (isinstance(value, list) and len(value) == 21
            and all(isinstance(j, dict) and type(j.get("idx")) is int for j in value)
            and {j["idx"] for j in value} == set(range(21)))


def _timestamp(seconds: float) -> str:
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def _temperatures(source: str, data: dict) -> list[float]:
    if source == "joints":
        raw = [joint.get("temp") for joint in data.get("joints", [])]
    elif source == "battery":
        raw = [data.get("temperature")]
    else:
        return []
    return [float(value) for value in raw
            if isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value)]


def evaluate_health(samples: dict, config: dict, *, motion_interval: float = 0.5,
                    now_monotonic: float | None = None, now_wall: float | None = None) -> dict:
    now_monotonic = time.monotonic() if now_monotonic is None else now_monotonic
    now_wall = time.time() if now_wall is None else now_wall
    limits = config.get("temperature_limits") or {}
    minimum = config.get("battery_min_soc", 20)
    checks = {}

    intervals = {**SOURCE_INTERVALS, "motion_state": motion_interval, **config.get("intervals", {})}
    for source, interval in intervals.items():
        sample = samples.get(source)
        limit = limits.get(source)
        item = {
            "status": "数据不足",
            "data": sample["data"] if sample else None,
            "received_at": _timestamp(sample["received_at"]) if sample else None,
            "age_seconds": None,
            "reasons": [],
            "temperature": {"status": "未判定" if limit is None else "数据不足",
                            "maximum": None, "limit": limit},
        }
        checks[source] = item
        if sample is None:
            item["reasons"].append("尚未收到数据")
            continue

        age = max(0.0, now_monotonic - sample["received_monotonic"])
        item["age_seconds"] = round(age, 3)
        if age > interval * 3:
            item["reasons"].append(f"数据已过期（超过 {interval * 3:g} 秒）")
            continue

        data = sample["data"]
        if not isinstance(data, dict):
            item["reasons"].append("数据格式无效")
            continue
        item["status"] = "正常"

        if source == "battery":
            soc = data.get("soc")
            if not isinstance(soc, (int, float)) or isinstance(soc, bool) or not math.isfinite(soc) or not 0 <= soc <= 100:
                item["status"] = "数据不足"
                item["reasons"].append("电量读数无效")
                continue
            if soc < minimum:
                item["status"] = "异常"
                item["reasons"].append(f"电量 {soc}% 低于 {minimum}%")
            alarm = data.get("alarm")
            if isinstance(alarm, int) and not isinstance(alarm, bool) and alarm != 0:
                item["status"] = "异常"
                item["reasons"].append(f"电池报警码 {alarm}")
        elif source == "imu" and not valid_quaternion(data.get("quaternion")):
            item["status"] = "数据不足"
            item["reasons"].append("IMU 四元数无效")
            continue
        elif source == "joints" and not valid_joints(data.get("joints")):
            item["status"] = "数据不足"
            item["reasons"].append("关节读数未包含全部 21 个关节")
            continue
        elif source == "motion_state":
            workmode = data.get("workmode") or {}
            if (data.get("fresh") is not True or not isinstance(workmode, dict)
                    or not isinstance(workmode.get("protection"), bool)
                    or not isinstance(data.get("motor_faults"), list)):
                item["status"] = "数据不足"
                item["reasons"].append("运动状态读数无效")
                continue
            if workmode.get("protection"):
                item["status"] = "异常"
                item["reasons"].append("机器人处于保护模式")
            faults = data.get("motor_faults") or []
            if faults:
                item["status"] = "异常"
                item["reasons"].append(f"检测到 {len(faults)} 项已识别的电机故障")
            joint_states = data.get("joint_states")
            if not isinstance(joint_states, list) or len(joint_states) != 21:
                if item["status"] == "正常":
                    item["status"] = "数据不足"
                item["reasons"].append("缺少完整的 21 个原始电机状态，无法核对未收录错误码")
            else:
                unknown = [joint for joint in joint_states
                           if (isinstance(joint, dict)
                               and type(joint.get("error")) is int
                               and joint["error"] != 0
                               and joint.get("error_documented") is not True)]
                if unknown:
                    if item["status"] == "正常":
                        item["status"] = "数据不足"
                    details = sorted({f'{joint.get("joint", joint.get("motor_id", "?"))}:'
                                      f'{joint["error"]}' for joint in unknown})
                    item["reasons"].append(
                        f"{len(unknown)} 个电机报告未收录错误码（"
                        + ", ".join(details) + "）；含义待核实")

        values = _temperatures(source, data)
        if values:
            item["temperature"]["maximum"] = max(values)
        if source not in limits:
            if source in ("battery", "joints"):
                item["temperature"]["reason"] = "温度界限未配置"
            else:
                item["temperature"]["reason"] = "此数据源不提供温度判断"
            continue
        if not values:
            item["temperature"]["status"] = "数据不足"
            item["status"] = "数据不足" if item["status"] == "正常" else item["status"]
            item["reasons"].append("温度读数为空")
            continue
        maximum = max(values)
        if maximum > limit:
            item["temperature"]["status"] = "异常"
            item["status"] = "异常"
            item["reasons"].append(f"最高温度 {maximum:g} 超过 {limit:g}")
        else:
            item["temperature"]["status"] = "正常"

    statuses = {item["status"] for item in checks.values()}
    overall = "异常" if "异常" in statuses else "数据不足" if "数据不足" in statuses else "正常"
    summary = {
        "异常": "已发现预设异常；请查看各项原因。",
        "数据不足": "状态数据不完整、已过期或存在未判定错误码，无法完成检查。",
        "正常": "已启用的检查项未发现异常；这不是运动安全许可。",
    }[overall]
    unassessed = [source for source in ("battery", "joints")
                  if checks[source]["temperature"]["status"] == "未判定"]
    if unassessed:
        summary += " 未配置温度界限的项目未作温度安全判断。"
    return {
        "status": overall,
        "summary": summary,
        "checked_at": _timestamp(now_wall),
        "checks": checks,
        "temperature_unassessed": unassessed,
        "unsupported": {"mainboard": "Bumi 驱动未提供独立主板状态数据"},
    }
