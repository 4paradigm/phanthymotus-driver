"""
model.py — Go1 骨骼模型卡（resource）：返回 resource/go1_model.urdf 供 dashboard skeleton 渲染。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
这是 `type=resource` 卡：dashboard 的 skeleton 渲染器会调用名为 `model` 的工具取回 URDF
（返回 {"urdf": "<xml>"} → 走 URDF 分支渲染四足狗；取不到则退回默认人形骨架）。
joints 卡的骨架就靠这张卡的 URDF + 每关节 name/q 来摆姿态。
"""

from __future__ import annotations

from pathlib import Path

CARD = "model"
TYPE = "resource"
_URDF_PATH = Path(__file__).parent / "resource" / "go1_model.urdf"


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        pass

    def get_tool(self):
        return {"name": CARD, "type": TYPE,
                "description": "Go1 quadruped URDF model for skeleton renderer",
                "inputSchema": {"type": "object", "properties": {}}}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, tool_name, args):
        try:
            return {"urdf": _URDF_PATH.read_text()}
        except Exception as e:  # noqa: BLE001
            return {"error": f"go1_model.urdf not found: {e}"}


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
