"""Direct XHand Lite preset-gesture card.

Gesture values are deployment presets, not vendor-certified grasp limits. The
card delegates execution to ``hand_control`` so both cards share one direct
publisher and one in-process command lease.
"""

from __future__ import annotations

from hand_control import Plugin as HandControlPlugin

CARD = "hand_gesture"
TYPE = "actuator"
DESC = "Q5 XHand Lite 预设手势：张手、握拳、指向、捏取及常用手势"


def _side_pose(left_values, right_values, side):
    positions = {}
    if side in ("left", "both"):
        positions.update(left_values)
    if side in ("right", "both"):
        positions.update(right_values)
    return positions


def _paired(left):
    return left, {name.replace("left_", "right_", 1): value for name, value in left.items()}


def _preset(left_values):
    left, right = _paired(left_values)
    return {"left": left, "right": right}


PRESETS = {
    "open_hand": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.0,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 0.0,
    }),
    "light_grip": _preset({
        "left_hand_thumb_bend_joint": 0.35, "left_hand_thumb_rota_joint1": 0.35,
        "left_hand_index_joint1": 0.45, "left_hand_mid_joint1": 0.45,
        "left_hand_ring_joint1": 0.45, "left_hand_pinky_joint1": 0.45,
    }),
    "closed_fist": _preset({
        "left_hand_thumb_bend_joint": 1.0, "left_hand_thumb_rota_joint1": 1.0,
        "left_hand_index_joint1": 1.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "point": _preset({
        "left_hand_thumb_bend_joint": 0.60, "left_hand_thumb_rota_joint1": 0.60,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "pinch": _preset({
        "left_hand_thumb_bend_joint": 0.75, "left_hand_thumb_rota_joint1": 0.75,
        "left_hand_index_joint1": 0.75, "left_hand_mid_joint1": 0.10,
        "left_hand_ring_joint1": 0.10, "left_hand_pinky_joint1": 0.10,
    }),
    "victory": _preset({
        "left_hand_thumb_bend_joint": 0.30, "left_hand_thumb_rota_joint1": 0.30,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "thumbs_up": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.0,
        "left_hand_index_joint1": 1.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "ok_sign": _preset({
        "left_hand_thumb_bend_joint": 0.75, "left_hand_thumb_rota_joint1": 0.75,
        "left_hand_index_joint1": 0.75, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 0.0,
    }),
    "three": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.30,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 1.0,
    }),
    "rock": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.30,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 0.0,
    }),
}
GESTURE_LABELS = {
    "open_hand": "张手", "light_grip": "轻握", "closed_fist": "完全握拳",
    "point": "指向", "pinch": "捏取", "victory": "胜利手势",
    "thumbs_up": "点赞", "ok_sign": "OK 手势", "three": "比三", "rock": "摇滚手势",
}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        # Reuse the complete command card's safety validation and command path.
        control_config = dict(plugin_config)
        control_config.setdefault("max_step_rad", 0.04)
        control_config.setdefault("min_position_rad", 0.0)
        control_config.setdefault("max_position_rad", 1.0)
        control_config.setdefault("hold_repetitions", 3)
        self._control = HandControlPlugin(control_config, namespace, executor, client)

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *PRESETS, "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": name, "title": label} for name, label in GESTURE_LABELS.items()],
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "side": {"type": "string", "title": "执行侧", "enum": ["left", "right", "both"], "oneOf": [
                        {"const": "left", "title": "左手"}, {"const": "right", "title": "右手"},
                        {"const": "both", "title": "双手"},
                    ], "default": "both"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{name: {"params": ["side"], "description": f"对指定手执行{GESTURE_LABELS[name]}预设。"}
                       for name in PRESETS},
                    "cancel": {"params": [], "description": "取消当前手势，并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return self._control.dispatch(action, args)
        if action in ("cancel", "stop"):
            return self._control.dispatch("stop", args)
        if action not in PRESETS:
            return None
        gesture, side = action, args.get("side", "both")
        if side not in ("left", "right", "both"):
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "side must be left, right, or both", "details": {}}
        positions = _side_pose(PRESETS[gesture]["left"], PRESETS[gesture]["right"], side)
        command_args = {"targets": [{"joint_name": name, "position_rad": position} for name, position in positions.items()]}
        result = self._control.dispatch("set", command_args)
        if result.get("ok"):
            result["gesture"] = gesture
            result["side"] = side
            result["preset_vendor_certified"] = False
        return result

    def stop(self):
        self._control.stop()


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
