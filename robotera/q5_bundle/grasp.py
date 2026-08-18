"""Q5 grasp card — XHand Lite gentle-grasp preset and release.

The grasp values mirror the vendor SDK ``control_xhand_lite_grasp`` (thumb half
bent, four fingers closed). The card delegates execution to ``hand_control`` so
both cards share one direct publisher and one in-process command lease.
"""

from __future__ import annotations

from hand_control import Plugin as HandControlPlugin

CARD = "grasp"
TYPE = "actuator"
DESC = "Q5 抓取：XHand Lite 轻柔抓取预设 + 松开，复用 hand_control 命令通道"


def _paired(left):
    return left, {name.replace("left_", "right_", 1): value for name, value in left.items()}


def _preset(left_values):
    left, right = _paired(left_values)
    return {"left": left, "right": right}


# 轻柔抓取：拇指半弯(0.5)，其余四指闭合(1.0)。对齐 vendor SDK control_xhand_lite_grasp。
GRASP = _preset({
    "left_hand_thumb_bend_joint": 0.5, "left_hand_thumb_rota_joint1": 0.5,
    "left_hand_index_joint1": 1.0, "left_hand_mid_joint1": 1.0,
    "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
})
# 松开：全 0（全张开）
RELEASE = _preset({
    "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.0,
    "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
    "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 0.0,
})


def _side_pose(left_values, right_values, side):
    positions = {}
    if side in ("left", "both"):
        positions.update(left_values)
    if side in ("right", "both"):
        positions.update(right_values)
    return positions


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        # Reuse the complete hand-control card's safety validation and command path.
        control_config = dict(plugin_config)
        control_config.setdefault("max_step_rad", 0.04)
        control_config.setdefault("min_position_rad", 0.0)
        control_config.setdefault("max_position_rad", 1.0)
        control_config.setdefault("hold_repetitions", 3)
        self._control = HandControlPlugin(control_config, namespace, executor, client)

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "grasp", "release", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        {"const": "grasp", "title": "轻柔抓取"},
                        {"const": "release", "title": "松开"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "side": {"type": "string", "title": "执行侧", "enum": ["left", "right", "both"], "oneOf": [
                        {"const": "left", "title": "左手"}, {"const": "right", "title": "右手"},
                        {"const": "both", "title": "双手"},
                    ], "default": "both"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    "grasp": {"params": ["side"], "description": "对指定手执行轻柔抓取（拇指半弯、四指闭合），对齐 vendor SDK control_xhand_lite_grasp。"},
                    "release": {"params": ["side"], "description": "对指定手执行松开（全部张开）。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return self._control.dispatch(action, args)
        if action not in ("grasp", "release"):
            return None
        side = args.get("side", "both")
        if side not in ("left", "right", "both"):
            return {"ok": False, "code": "INVALID_ARGUMENT",
                    "message": "side must be left, right, or both", "details": {}}
        preset = GRASP if action == "grasp" else RELEASE
        positions = _side_pose(preset["left"], preset["right"], side)
        command_args = {"targets": [{"joint_name": name, "position_rad": position}
                                    for name, position in positions.items()]}
        result = self._control.dispatch("set", command_args)
        if result.get("ok"):
            result["grasp_action"] = action
            result["side"] = side
            result["preset_vendor_certified"] = True
        return result

    def stop(self):
        self._control.stop()


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
