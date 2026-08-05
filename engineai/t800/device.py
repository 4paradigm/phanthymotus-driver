#!/usr/bin/env python3
"""
engineai/t800/device.py — 众擎 T800 开发版设备插件聚合（bundle）。

数据流：
  - 传感器类插件（motion_state / joints / imu / power / gamepad）：
      订阅 domain 69 机器人话题 → 缓存 → 定时 JSON 发布到 domain 42 的
      /{ns}/xxx（std_msgs/String，format=data/json）
  - 控制类插件（motion_switcher / loco / arm / joint_bridge）：
      发布 domain 69 控制话题；dispatch 前检查运动状态机前置
      （不在所需模式时返回 {"state":"error","error":"WRONG_MOTION_STATE",...}）
  - 外设类插件（led / mic / speaker）：
      led 发布 domain 69；mic/speaker 走 audio_msgs/AudioChunk 音频协议
      （format=audio/pcm-16k，16kHz 单声道，chunk >= 1024 字节）

插件实现位于 plugins/ 下（motion.py / sensors.py / peripherals.py），
本文件只做 import 与聚合，模块级不 import 任何第三方/ROS2 库，
保证可在无 ROS2 环境下被纯 import 测试。
"""

from __future__ import annotations

import importlib

# 插件键（config.yaml plugins 键） → (模块, 类名)
# 固定类名由并行插件 agent 实现，构造签名统一：
#   __init__(self, plugin_config: dict, namespace: str, ros2: Ros2Contexts)
_PLUGIN_TABLE = {
    "motion_state":    ("plugins.motion", "MotionStatePlugin"),
    "motion_switcher": ("plugins.motion", "MotionSwitcherPlugin"),
    "loco":            ("plugins.motion", "LocoPlugin"),
    "arm":             ("plugins.motion", "ArmMotionPlugin"),
    "joint_bridge":    ("plugins.motion", "JointBridgePlugin"),
    "joints":          ("plugins.sensors", "JointsStatePlugin"),
    "imu":             ("plugins.sensors", "ImuPlugin"),
    "power":           ("plugins.sensors", "PowerPlugin"),
    "gamepad":         ("plugins.sensors", "GamepadPlugin"),
    "led":             ("plugins.peripherals", "LedPlugin"),
    "mic":             ("plugins.peripherals", "MicPlugin"),
    "speaker":         ("plugins.peripherals", "SpeakerPlugin"),
}


class T800DeviceBundle:
    """按 config.yaml 的 plugins 键逐个加载启用的插件并聚合工具。"""

    def __init__(self, cfg: dict, namespace: str, ros2):
        self._cfg = cfg
        self._namespace = namespace
        self._plugins: list = []
        self._module_cache: dict = {}

        plugins_cfg = cfg.get("plugins", {})
        for key, (module_name, class_name) in _PLUGIN_TABLE.items():
            plugin_cfg = plugins_cfg.get(key)
            if not plugin_cfg or not plugin_cfg.get("enabled", False):
                continue
            try:
                module = self._module_cache.get(module_name)
                if module is None:
                    module = importlib.import_module(module_name)
                    self._module_cache[module_name] = module
                cls = getattr(module, class_name)
                plugin = cls(plugin_cfg, namespace, ros2)
                self._plugins.append(plugin)
                print(f"[bundle] {class_name} loaded", flush=True)
            except Exception as e:
                # 插件文件可能由并行 agent 尚未写完，失败不影响其余插件
                print(f"[bundle] {key} ({class_name}) load FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()

        self._warn_duplicate_tool_names()

    # ── 工具名冲突检查 ────────────────────────────────────────────────────

    def _warn_duplicate_tool_names(self) -> None:
        """启动时打印重复工具名警告（画布上工具名必须唯一）。"""
        seen: dict = {}
        for p in self._plugins:
            tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            owner = type(p).__name__
            for tool in tools or []:
                name = (tool or {}).get("name", "")
                if not name:
                    continue
                if name in seen:
                    print(
                        f"[bundle] WARNING: 工具名 '{name}' 重复，"
                        f"来自 {seen[name]} 与 {owner}",
                        flush=True,
                    )
                else:
                    seen[name] = owner

    # ── 生命周期 ───────────────────────────────────────────────────────────

    def start_all(self) -> None:
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self) -> None:
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        print("[bundle] All plugins stopped", flush=True)

    # ── 工具聚合与分发 ─────────────────────────────────────────────────────

    def get_all_tools(self) -> list:
        tools = []
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        """分发工具调用（逻辑同 g1）：
        - resource 类型直接转发（不 pop action）
        - 其余类型 args.pop("action", tool_name) 并注入 _tool_name
        返回纯 dict（MCP content 包装由 HTTP handler 完成）。
        """
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name  # 多工具插件据此识别被调用的工具
                    result = p.dispatch(action, args)
                    return result
        return None
