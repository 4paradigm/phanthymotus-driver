"""Add monitoring to an existing bundle without replacing its control code."""
from copy import deepcopy
from dataclasses import asdict
import math
import time

from agilex.piper.monitor import PiperMonitorPlugin


def read_passive_snapshot(runtime):
    callback = getattr(runtime, "read_monitor_snapshot", None)
    if callable(callback):
        return callback()
    # Compatibility for arm 2's preserved ten-card image. Read its existing
    # SDK cache only; never call connected(), status(), read_state() or a gate.
    result = dict(device_name=runtime.config.get("name", "PiPER"),
                  connected=runtime.driver is not None and not runtime.closed,
                  feedback_time=None, fresh=False, joints_deg=None, enabled=None,
                  joint_limits_deg=None, gripper=None, faults=None,
                  motion_enabled=runtime.motion_enabled, busy=runtime.gate.locked(),
                  active_card=runtime.active_card, last_actions=deepcopy(dict(runtime.last_actions)), reasons={})
    if not runtime.connection_lock.acquire(blocking=False):
        result["reasons"]["connection"] = "连接正在变更，本次不等待"
        return result
    try:
        driver = runtime.driver
        if driver is None or runtime.closed:
            result["reasons"]["connection"] = "尚未连接；监控不会另开 CAN"
            return result
        try:
            state = driver.get_state()
            result["feedback_time"] = state.feedback_time
            if (not math.isfinite(state.feedback_time) or not 0 <= time.time()-state.feedback_time <= .5
                    or len(state.joints_deg) != 6 or not all(math.isfinite(v) for v in state.joints_deg)):
                raise ValueError("六轴反馈无效或过期")
            result.update(fresh=True, joints_deg=list(state.joints_deg), enabled=list(state.enabled),
                          control_mode=state.control_mode,
                          faults=dict(arm_status=state.arm_status, error_code=state.error_code,
                                      motor_faults=list(state.motor_faults)))
        except Exception as exc:
            result["reasons"]["feedback"] = str(exc)
        try:
            if hasattr(driver, "joint_bounds_deg"):
                result["joint_limits_deg"] = [list(pair) for pair in driver.joint_bounds_deg()]
            else:
                limits = []
                for i in range(6):
                    low, high = (round(math.degrees(v), 3) for v in driver.arm.GetSDKJointLimitParam(f"j{i+1}"))
                    deployed = runtime.config.get("joint_limits_deg")
                    if deployed is not None:
                        low, high = max(low, deployed[i][0]), min(high, deployed[i][1])
                    if not all(math.isfinite(v) for v in (low, high)) or low >= high:
                        raise ValueError("SDK 与部署限位交集无效")
                    limits.append([low, high])
                result["joint_limits_deg"] = limits
        except Exception as exc:
            result["reasons"]["joint_limits_deg"] = str(exc)
        try:
            grip = driver.get_gripper()
            if (not all(math.isfinite(v) for v in (grip.width_mm, grip.effort_nm, grip.feedback_time))
                    or not 0 <= time.time()-grip.feedback_time <= .5):
                raise ValueError("夹爪反馈无效或过期")
            result["gripper"] = asdict(grip)
        except Exception as exc:
            result["reasons"]["gripper"] = str(exc)
        return result
    finally:
        runtime.connection_lock.release()


def add_monitor(plugins, config, ros2):
    plugins = list(plugins)
    runtimes = [p for p in plugins if hasattr(p, "connection_lock") and hasattr(p, "driver") and hasattr(p, "topic")]
    if len(runtimes) != 1:
        raise ValueError("监控必须绑定唯一的现有 PiPER runtime")
    runtime = runtimes[0]
    if any(t.get("name") in {"piper_monitor", "model"} for p in plugins for t in p.get_tools()):
        raise ValueError("监控或模型工具已注册；拒绝重复添加")
    monitor = PiperMonitorPlugin(lambda: read_passive_snapshot(runtime),
                                 namespace=runtime.topic.rsplit("/", 1)[0],
                                 device_name=config.get("name", "PiPER"), ros2=ros2)
    return [*plugins, monitor]
