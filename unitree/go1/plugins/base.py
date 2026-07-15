from __future__ import annotations

#!/usr/bin/env python3
"""
plugins/base.py — 插件公共基类与 helper。

提供:
  • BasePlugin:统一 start/stop 空实现 + 生命周期动作(start/stop/info)默认处理。
  • clamp():参数范围保护。
子类覆写 get_tool()/get_tools() 与 dispatch();dispatch 里先调 self._lifecycle(action),
非生命周期动作(返回 None)再走自己的业务分支。
"""


def clamp(v, lo, hi, default=0.0):
    try:
        return max(lo, min(hi, float(v)))
    except (TypeError, ValueError):
        return default


class BasePlugin:
    #: 子类可覆写:info/start 返回的 state 文案
    RUNNING_STATE = "running"

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def _lifecycle(self, action, extra=None):
        """处理 start/stop/info 通用动作;不是这三者返回 None。extra: info 附加字段(dict)。"""
        if action == "start":
            return {"state": self.RUNNING_STATE}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            out = {"state": self.RUNNING_STATE}
            if extra:
                out.update(extra)
            return out
        return None
