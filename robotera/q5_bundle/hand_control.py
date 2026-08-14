"""Direct, joint-level XHand Lite control card.

The card accepts one or more named finger targets, interpolated from the live
joint state. It intentionally does not expose gain, torque, or velocity arrays.
"""

from __future__ import annotations

import math
import threading
import time

from hand_command import HAND_JOINTS, failure, finite_number, get_router
from control_contract import q5_active_status, q5_is_control_ready

CARD = "hand_control"
TYPE = "actuator"
DESC = "Q5 XHand Lite 完整关节控制：可同时设置任意左右手手指目标"

HAND_SIDES = ("left", "right", "both")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
FINGER_LABELS = {
    "thumb": "拇指",
    "index": "食指", "middle": "中指", "ring": "无名指", "pinky": "小指",
}
FINGER_JOINT_SUFFIXES = {
    "thumb": ("hand_thumb_bend_joint", "hand_thumb_rota_joint1"),
    "index": ("hand_index_joint1",),
    "middle": ("hand_mid_joint1",),
    "ring": ("hand_ring_joint1",),
    "pinky": ("hand_pinky_joint1",),
}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._router = get_router(client, executor)
        self._min_position = float(plugin_config.get("min_position_rad", 0.0))
        self._max_position = float(plugin_config.get("max_position_rad", 1.0))
        self._max_step = float(plugin_config.get("max_step_rad", 0.04))
        self._max_duration = float(plugin_config.get("max_duration_s", 2.0))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if not (self._min_position < self._max_position and min(self._max_step, self._max_duration, self._publish_rate) > 0 and self._hold_repetitions >= 1):
            raise ValueError("hand_control limits must be positive and position bounds ordered")
        self._lock = threading.Lock()
        self._motion_stop = None
        self._motion_thread = None
        self._active_command = None

    def get_tool(self):
        target = {"type": "object", "properties": {
            "joint_name": {"type": "string", "enum": list(HAND_JOINTS), "description": "XHand Lite 执行关节名称"},
            "position_rad": {"type": "number", "minimum": self._min_position, "maximum": self._max_position,
                             "description": f"范围[{self._min_position:g},{self._max_position:g}]rad"},
        }, "required": ["joint_name", "position_rad"], "additionalProperties": False}
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "open_hand", "close_hand", "set_hand", "set_finger", "set", "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        {"const": "open_hand", "title": "张开整手"},
                        {"const": "close_hand", "title": "合拢整手"},
                        {"const": "set_hand", "title": "设置整手弯曲程度"},
                        {"const": "set_finger", "title": "设置单指弯曲程度"},
                        {"const": "set", "title": "高级：指定关节目标"},
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "side": {"type": "string", "title": "执行侧", "enum": list(HAND_SIDES), "oneOf": [
                        {"const": "left", "title": "左手"}, {"const": "right", "title": "右手"},
                        {"const": "both", "title": "双手"},
                    ], "default": "both", "description": "先选左/右/双手。"},
                    "finger": {"type": "string", "title": "手指", "enum": list(FINGERS), "oneOf": [
                        {"const": name, "title": FINGER_LABELS[name]} for name in FINGERS
                    ], "description": "选thumb可同时设弯曲和旋转。"},
                    "curl_rad": {"type": "number", "title": "弯曲角度(rad)", "minimum": self._min_position,
                                 "maximum": self._max_position, "multipleOf": 0.01,
                                 "default": min(0.20, self._max_position),
                                 "description": f"范围[{self._min_position:g},{self._max_position:g}]rad"},
                    "rotation_rad": {"type": "number", "title": "拇指旋转(rad)",
                                     "minimum": self._min_position, "maximum": self._max_position,
                                     "multipleOf": 0.01,
                                     "description": f"范围[{self._min_position:g},{self._max_position:g}]rad"},
                    "targets": {"type": "array", "title": "关节目标", "items": target,
                                "minItems": 1, "maxItems": len(HAND_JOINTS), "x-widget": "json",
                                "x-example": '[{"joint_name":"left_hand_index_joint1","position_rad":0.10}]',
                                "description": "每项：关节名+目标角度。"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    "open_hand": {"params": ["side"], "description": "将选中手完整张开到 0 rad。"},
                    "close_hand": {"params": ["side"], "description": f"将选中手完整合拢到 {self._max_position:g} rad。"},
                    "set_hand": {"params": ["side", "curl_rad"], "description": "为整只手设定统一的弯曲程度。"},
                    "set_finger": {"params": ["side", "finger", "curl_rad", "rotation_rad"], "description": "thumb可填弯曲+旋转"},
                    "set": {"params": ["targets"], "description": "高级模式：用 JSON 指定多个关节的绝对目标。"},
                    "cancel": {"params": [], "description": "取消当前插补，并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def _safety(self):
        status = self._router.status()
        status.update({"control_mode": "direct_joint_position",
                       "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
                       "lifecycle_state": self._client.get_lifecycle_state(),
                       "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)),
                       "q5_fsm": q5_active_status(self._client),
                       "limits": {"min_position_rad": self._min_position, "max_position_rad": self._max_position,
                                  "max_step_rad": self._max_step, "max_duration_s": self._max_duration,
                                  "vendor_certified": False}})
        return status

    def _allowed(self, args):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return failure("ROS_UNAVAILABLE", "Q5 hand command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active":
            return failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before hand control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before hand control",
                           status={**status, "q5_fsm": q5_status})
        if not status["joint_state_fresh"]:
            return failure("JOINT_STATE_UNAVAILABLE", "Refusing hand control without fresh /joint_states", status=status)
        return status

    def _targets(self, args):
        raw_targets = args.get("targets")
        if not isinstance(raw_targets, list) or not raw_targets:
            return failure("INVALID_ARGUMENT", "targets must be a non-empty list")
        targets = {}
        try:
            for item in raw_targets:
                if not isinstance(item, dict):
                    raise ValueError("each target must be an object")
                name = item.get("joint_name")
                if name not in HAND_JOINTS:
                    raise ValueError("joint_name is not an allowed XHand Lite joint")
                if name in targets:
                    raise ValueError(f"duplicate target for {name}")
                value = finite_number(item.get("position_rad"), "position_rad")
                if not self._min_position <= value <= self._max_position:
                    raise ValueError(f"position_rad for {name} is outside the configured guardrail")
                targets[name] = value
            duration_value = args.get("duration_s")
            duration = finite_number(
                min(0.5, self._max_duration) if duration_value is None else duration_value,
                "duration_s",
            )
        except ValueError as e:
            return failure("INVALID_ARGUMENT", str(e))
        if not 0.0 < duration <= self._max_duration:
            return failure("INVALID_ARGUMENT", "duration_s is outside the configured safe interval", max_duration_s=self._max_duration)
        current = self._client.snapshot().get("joints", {})
        missing = [name for name in targets if name not in current]
        if missing:
            return failure("JOINT_UNAVAILABLE", "Requested hand joint is absent from /joint_states", missing_joints=missing)
        # Large target changes are safe to accept here because _run() always
        # interpolates them into max_step_rad-sized position updates.
        return {"current": {name: float(current[name]) for name in targets}, "targets": targets, "duration_s": duration}

    def _profile_targets(self, action, args):
        side = args.get("side", "both")
        if side not in HAND_SIDES:
            return failure("INVALID_ARGUMENT", "side must be left, right, or both")
        if action in ("open_hand", "close_hand"):
            # Named open/close operations target the complete hand range. The
            # worker interpolates the change into bounded position steps.
            curl = None
        else:
            try:
                curl = finite_number(args.get("curl_rad"), "curl_rad")
            except ValueError as e:
                return failure("INVALID_ARGUMENT", str(e))
            if not self._min_position <= curl <= self._max_position:
                return failure("INVALID_ARGUMENT", "curl_rad is outside the configured guardrail")

        fingers = FINGERS
        if action == "set_finger":
            finger = args.get("finger")
            if finger not in FINGERS:
                return failure("INVALID_ARGUMENT", "finger无效")
            fingers = (finger,)
        current = self._client.snapshot().get("joints", {})
        targets = []
        for hand in ("left", "right"):
            if side not in (hand, "both"):
                continue
            for finger in fingers:
                suffixes = FINGER_JOINT_SUFFIXES[finger]
                if action == "set_finger" and finger == "thumb":
                    rotation_value = args.get("rotation_rad", current.get(f"{hand}_hand_thumb_rota_joint1"))
                    try:
                        rotation = finite_number(rotation_value, "rotation_rad")
                    except ValueError as e:
                        return failure("INVALID_ARGUMENT", str(e))
                    if not self._min_position <= rotation <= self._max_position:
                        return failure("INVALID_ARGUMENT", "旋转超范围")
                    suffixes = ("hand_thumb_bend_joint", "hand_thumb_rota_joint1")
                for index, suffix in enumerate(suffixes):
                    name = f"{hand}_{suffix}"
                    if curl is None:
                        if name not in current:
                            return failure("JOINT_UNAVAILABLE", "Requested hand joint is absent from /joint_states",
                                           missing_joints=[name])
                        value = self._min_position if action == "open_hand" else self._max_position
                    else:
                        value = curl
                        if action == "set_finger" and finger == "thumb" and index == 1:
                            value = rotation
                    targets.append({"joint_name": name, "position_rad": value})
        return self._targets({"targets": targets, "duration_s": args.get("duration_s")})

    def _start_motion(self, command, source):
        if not self._router.acquire(CARD):
            return failure("COMMAND_IN_PROGRESS", "Another Q5 hand card currently owns the command publisher", status=self._router.status())
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                self._router.release(CARD)
                return failure("MOTION_IN_PROGRESS", "A hand command is already active; call stop before another command")
            event = threading.Event()
            self._motion_stop = event
            self._active_command = {"source": source, "targets_rad": dict(command["targets"]),
                                    "duration_s": command["duration_s"], "started_at_ms": int(time.time() * 1000)}
            self._motion_thread = threading.Thread(target=self._run, args=(event, command), daemon=True, name="q5_hand_control")
            self._motion_thread.start()
        return {"ok": True, "state": "moving", "command": dict(self._active_command), "stops_by_holding_current_position": True}

    def _run(self, stop_event, command):
        current, targets, duration = command["current"], command["targets"], command["duration_s"]
        steps = max(int(math.ceil(duration * self._publish_rate)),
                    max(int(math.ceil(abs(targets[name] - current[name]) / self._max_step)) for name in targets), 1)
        try:
            for index in range(1, steps + 1):
                if stop_event.is_set():
                    break
                position = {name: current[name] + (targets[name] - current[name]) * index / steps for name in targets}
                self._router.publish(position)
                stop_event.wait(duration / steps)
        finally:
            self._router.release(CARD)
            with self._lock:
                if self._motion_stop is stop_event:
                    self._motion_stop = self._motion_thread = self._active_command = None

    def _hold_current(self):
        snap = self._client.snapshot()
        positions = snap.get("joints", {})
        if not snap.get("fresh") or any(name not in positions for name in HAND_JOINTS):
            return False
        if not self._router.acquire(CARD):
            return False
        try:
            published = False
            for _ in range(self._hold_repetitions):
                published = self._router.publish({name: float(positions[name]) for name in HAND_JOINTS}) or published
                time.sleep(1.0 / self._publish_rate)
            return published
        finally:
            self._router.release(CARD)

    def _stop(self, reason):
        with self._lock:
            event, thread, active = self._motion_stop, self._motion_thread, self._active_command
        if event is not None:
            event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        held = self._hold_current() if active else False
        return {"ok": True, "state": "stopped", "reason": reason, "hold_current_published": held}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable", "safety": self._safety()}
        if action == "info":
            with self._lock:
                active = dict(self._active_command) if self._active_command else None
            return {"ok": True, "state": "moving" if active else "idle", "active_command": active, "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._stop("command")
        if action == "hold":
            allowed = self._allowed(args)
            if allowed.get("ok") is False:
                return allowed
            return {"ok": self._hold_current(), "state": "held"}
        if action not in ("set", "open_hand", "close_hand", "set_hand", "set_finger"):
            return None
        allowed = self._allowed(args)
        if allowed.get("ok") is False:
            return allowed
        command = self._targets(args) if action == "set" else self._profile_targets(action, args)
        if command.get("ok") is False:
            return command
        return self._start_motion(command, action)

    def stop(self):
        self._stop("driver_shutdown")


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
