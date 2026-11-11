"""Panther motion cards: the five actuators, each with a stop.

Nothing here imports ROS. Every ROS call goes through `io` (`RosMotionIO` in device.py in
production, a recorder in the tests) and everything the checks need to know about the robot
comes from `state` (the `Hub`). That keeps the rules that decide whether a command is sent --
fresh state, ranges, step limits, stop, completion -- checkable without a robot.

There is no confirmation field and no motion switch: with intelligent control on, a call moves
the robot. What still stands between a call and the robot is not a switch but conditions: fresh
`/joint_states`, a fresh emergency-stop diagnostic at level 0, the matching controller active,
and every value inside the range the card shows next to its box (speed and duration caps, URDF
joint limits, and a step limit from where the joint is now).

Ported from the Panther driver proposed in PR #520 and extended where its review found gaps:

* every actuator answers `stop`, and `on_interrupt_motion` / `on_interrupt_all` call it;
* `arm_control`, `base_drive` and the head's `home` run in a worker and return an `action_id`
  at once, then POST completion to Agent Core (README_dev "ACP / x-completion").

What `stop` can and cannot do. `base_drive`, `arm_control` and the head's `home` have something in
flight and stop it: zero velocity repeated, the trajectory goal cancelled and the controllers
restored, or the head's stepping halted where it is. `gripper`, `lifting` and the head's `look`
are one position command each; once published the robot's controller owns the motion and this
driver has no halt to call (the vendor halt services are deliberately not used). For those,
`stop` guarantees only that this driver sends nothing more.
"""
from __future__ import annotations

import json
import math
import os
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from contracts import (
    HEAD_JOINTS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, finite_number, optional_number,
    validate_drive, validate_joint_target,
)
from panther_bridge import safe_text

DRIVE_CONTROLLER = "movebase3_drive_controller"
ARM_FORWARD_CONTROLLER = "dual_arm_forward_position_controller"
ARM_P2P_CONTROLLERS = {"left": "left_arm_joint_trajectory_position_controller",
                       "right": "right_arm_joint_trajectory_position_controller"}
HEAD_CONTROLLER = "head_forward_position_controller"
LIFT_CONTROLLER = "lifting_controller"
GRIPPER_CONTROLLER = "{side}_unix_gripper3_controller"
# The robot has one two-finger parallel gripper per arm. The `mode` field exists in the vendor
# message, but only `parallel` is shown in the vendor SDK and the operator could see no difference
# between the three on the robot, so the driver sends this one and offers no choice.
GRIPPER_PARALLEL = "parallel"

MOTION_CARDS = ("base_drive", "arm_control", "gripper", "lifting", "head")


# ── completion callback to Agent Core ────────────────────────────────────────

def post_acp_callback(action_id: str, status: str, result: dict, tool: str) -> None:
    """POST `/api/acp/complete`; three attempts. Raises if all fail (the caller logs it)."""
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
    if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(("https://localhost:", "https://127.0.0.1:")):
        # Agent Core's certificate is self-signed with no subjectAltName (README_dev "ACP"):
        # hostname checking can never pass, and the endpoint only accepts loopback callers.
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                          "tool": tool, "ts": time.time()}, allow_nan=False).encode()
    request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    last_error = None
    for attempt in range(1, 4):
        try:
            with urllib.request.urlopen(request, timeout=3, context=context) as response:
                if not 200 <= response.status < 300:
                    raise RuntimeError(f"Agent Core returned HTTP {response.status}")
            print(f"[ACP] {tool} action_id={action_id} status={status} attempt={attempt}", flush=True)
            return
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            if attempt < 3:
                time.sleep(0.2 * attempt)
    raise RuntimeError(f"ACP callback failed after 3 attempts: {last_error}")


def notify_async(action_id: str, status: str, result: dict, tool: str) -> None:
    """Deliver completion off the motion worker, and log a failure -- it is silent otherwise."""
    def deliver():
        try:
            post_acp_callback(action_id, status, result, tool)
        except Exception as exc:  # noqa: BLE001
            print(f"[ACP] callback failed action_id={action_id} status={status}: "
                  f"{str(exc)[:180]}", flush=True)
    threading.Thread(target=deliver, daemon=True, name=f"panther-acp-{tool}").start()


def _spawn(target, name: str) -> None:
    threading.Thread(target=target, daemon=True, name=name).start()


# ── tool schemas ─────────────────────────────────────────────────────────────

# The arm and head boxes are named after the robot's joints (`la0` is `joint_la0`), so a value can
# be read off the `joints` card and typed straight in.
ARM_FIELDS = {f"la{i}": f"joint_la{i}" for i in range(8)} | {f"ra{i}": f"joint_ra{i}" for i in range(8)}
HEAD_FIELDS = {"head0_rad": "joint_head0", "head1_rad": "joint_head1"}
INTERRUPT_HOOKS = {"on_interrupt_motion": {"action": "stop"}, "on_interrupt_all": {"action": "stop"}}


def _fmt(value) -> str:
    return f"{float(value):g}"


def _number(title: str, unit: str, low, high, detail: str, default=None) -> dict:
    """A number box whose shown range is the one enforced: in the title, the description and the
    schema bounds, so whichever the canvas displays, it is the same numbers."""
    field = {"type": "number", "minimum": float(low), "maximum": float(high),
             "title": f"{title}（{unit}，{_fmt(low)} ~ {_fmt(high)}）",
             "description": f"{title}：范围 {_fmt(low)} ~ {_fmt(high)} {unit}。{detail}".rstrip()}
    if default is not None:
        field["default"] = default
    return field


def _choice(title: str, options: list, default: str) -> dict:
    return {"type": "string", "enum": [value for value, _ in options], "default": default,
            "title": title, "oneOf": [{"const": value, "title": label} for value, label in options]}


def _actions(options: list, default: str) -> dict:
    return _choice("操作", options, default)


def motion_tools(config: dict, limits: dict) -> list:
    """Tool definitions for the enabled motion cards (type `actuator`).

    `limits` are the joint limits the boxes show: the driver's bundled URDF, which is what the
    robot published when it was captured. The commands are checked against the robot's live
    `/robot_description` once it has arrived; a difference between the two is logged at start-up.
    """
    plugins = (config or {}).get("plugins", {})
    tools = []

    def on(name):
        return bool((plugins.get(name) or {}).get("enabled", False))

    def schema(properties, actions=None, params=None, resource=None, completion=None, hooks=True):
        body = {"type": "object", "properties": dict(properties), "additionalProperties": False}
        if actions:
            body["properties"] = {"action": actions, **body["properties"]}
            body["required"] = ["action"]
            body["x-action-params"] = {name: {"params": list(fields), "description": text}
                                       for name, (fields, text) in params.items()}
        if resource:
            body["x-resource"] = resource
        if completion:
            body["x-completion"] = completion
        if hooks:
            body["x-hooks"] = dict(INTERRUPT_HOOKS)
        return body

    if on("base_drive"):
        cap = plugins["base_drive"]
        lin, ang, longest = cap["max_linear_mps"], cap["max_angular_radps"], cap["max_duration_s"]
        tools.append({
            "name": "base_drive", "type": "actuator", "multiInstance": False,
            "description": f"Panther 全向底盘定时运动。按给定速度走 duration_s 秒后自动停车；stop 立即停车。"
                           f"速度上限 {_fmt(lin)} m/s、{_fmt(ang)} rad/s，单次最长 {_fmt(longest)} 秒。"
                           "发车前选一个有订阅者的话题（/cmd_vel 优先），结果里给出用的是哪一个",
            "inputSchema": schema(
                {"linear_x": _number("前后速度", "m/s", -lin, lin, "正为前进，留空为 0", 0.0),
                 "linear_y": _number("左右速度", "m/s", -lin, lin, "正为向左，留空为 0", 0.0),
                 "angular_z": _number("转向角速度", "rad/s", -ang, ang, "正为逆时针（向左转），留空为 0", 0.0),
                 "duration_s": _number("持续时间", "秒", 0.1, longest, "到时自动停车", 1.0)},
                _actions([("move", "移动"), ("stop", "停车")], "move"),
                {"move": (["linear_x", "linear_y", "angular_z", "duration_s"], "按给定速度移动 duration_s 秒后自动停车"),
                 "stop": ([], "立即停车")},
                "base", {"actions": ["move"], "timeout": int(longest) + 10})})
    if on("arm_control"):
        cap = plugins["arm_control"]
        longest = cap["max_duration_s"]
        speed = _fmt(cap["speed_rad_s"])
        joints = {}
        for field, joint in ARM_FIELDS.items():
            low, high = limits.get(joint, (-math.pi, math.pi))
            side = "左臂" if field.startswith("la") else "右臂"
            joints[field] = _number(f"{side} {field}", "rad", low, high, "留空则保持当前位置")
        tools.append({
            "name": "arm_control", "type": "actuator", "multiInstance": False,
            "description": "Panther 双臂关节运动（FollowJointTrajectory）。move：只填要动的关节（la0~la7 左臂，ra0~ra7 右臂），"
                           "留空的保持当前位置，填了就在 URDF 限位内直接到位；运动时间不用填，按最远关节约 "
                           f"{speed} rad/s 自动计算（1 ~ {_fmt(longest)} 秒）。home：所选手臂全部关节回到 0 rad，"
                           "时间按平均速度不超过 "
                           f"{_fmt(cap['home_speed_rad_s'])} rad/s 自动拉长。运动期间临时切换控制器，结束或 stop 后恢复",
            "inputSchema": schema(
                {**joints,
                 "side": _choice("回零的手臂", [("both", "双臂"), ("left", "左臂"), ("right", "右臂")], "both")},
                _actions([("move", "移动关节"), ("home", "回零（0 rad）"), ("stop", "停止")], "move"),
                {"move": ([*ARM_FIELDS], "把填写的关节移动到目标角度，留空的关节保持不动"),
                 "home": (["side"], "所选手臂全部关节回到 0 rad"),
                 "stop": ([], "取消正在进行的运动并恢复控制器")},
                ["arm_l", "arm_r"], {"actions": ["move", "home"], "timeout": int(longest) + 15})})
    if on("gripper"):
        tools.append({
            "name": "gripper", "type": "actuator", "multiInstance": False,
            "description": "Panther 左右夹爪。position 0 = 张开，1 = 闭合（操作员实测确认，厂商 SDK 写的是相反的）；"
                           "open/close 分别发 0 和 1。硬件是两指平行夹爪，驱动固定以 parallel 模式下发"
                           "（另两种模式在实机上没有可辨差别，不再作为选项）。单次指令，下发后由机器人控制器执行；"
                           "stop 仅保证本驱动不再下发",
            "inputSchema": schema(
                {"side": _choice("夹爪", [("both", "双手"), ("left", "左手"), ("right", "右手")], "both"),
                 "position": _number("夹爪位置", "0=张开，1=闭合", 0.0, 1.0, "")},
                _actions([("set", "设置位置"), ("open", "张开"), ("close", "闭合")], "set"),
                {"set": (["side", "position"], "把夹爪移动到 position（0=张开，1=闭合）"),
                 "open": (["side"], "张开（position 0）"),
                 "close": (["side"], "闭合（position 1）")},
                ["hand_l", "hand_r"], hooks=False)})
    if on("lifting"):
        cap = plugins["lifting"]
        tools.append({
            "name": "lifting", "type": "actuator", "multiInstance": False,
            "description": f"Panther 升降位置指令（单位未经厂商确认，读数约 0.61）。范围 0 ~ {_fmt(cap['max_position'])}，"
                           "范围内直接到位；stop 仅保证本驱动不再下发",
            "inputSchema": schema(
                {"position": _number("升降位置", "单位未确认", 0.0, cap["max_position"], "")},
                resource="waist", hooks=False)})
    if on("head"):
        cap = plugins.get("head") or {}
        boxes = {}
        for field, joint, axis in (("head0_rad", "joint_head0", "俯仰（绕 y 轴）"),
                                   ("head1_rad", "joint_head1", "水平转动（绕 z 轴）")):
            low, high = limits.get(joint, (-math.pi, math.pi))
            boxes[field] = _number(f"头部 {field[:5]} {axis}", "rad", low, high, "留空则保持当前位置")
        tools.append({
            "name": "head", "type": "actuator", "multiInstance": False,
            "description": "Panther 头部两个关节。按 URDF：joint_head0 绕 y（俯仰），joint_head1 绕 z（水平转动）。"
                           "look 在限位内直接到位；home 回到 0 rad，按平均速度不超过 "
                           f"{_fmt(cap.get('home_speed_rad_s', 0.35))} rad/s 逐步移动，stop 可中途停下",
            "inputSchema": schema(
                boxes,
                _actions([("look", "转到"), ("home", "回正（0 rad）"), ("stop", "停止")], "look"),
                {"look": (list(HEAD_FIELDS), "把头转到给定角度，留空的关节保持不动"),
                 "home": ([], "两个关节逐步回到 0 rad"),
                 "stop": ([], "停止正在进行的回正")},
                "head", {"actions": ["home"], "timeout": 30})})
    return tools


# ── the motion logic ─────────────────────────────────────────────────────────

class Motion:
    def __init__(self, config: dict, io, state, *, spawn=_spawn, notify=notify_async,
                 sleep=time.sleep, monotonic=time.monotonic):
        self._cfg = config or {}
        self._plugins = self._cfg.get("plugins", {})
        self._io = io
        self._state = state
        self._spawn = spawn
        self._notify = notify
        self._sleep = sleep
        self._mono = monotonic
        self._drive_lock = threading.Lock()
        self._drive_generation = 0
        self._arm_lock = threading.Lock()
        self._arm_running = False
        self._arm_cancel = threading.Event()
        self._arm_done = threading.Event()
        self._arm_done.set()
        self._arm_began = False                              # has the worker of the current move run yet?
        # Bumped by every stop. A move remembers the value it started with and refuses to go on if it
        # has changed: that is how a stop that arrives while the move is still being checked (before
        # it is marked as running) still cancels it, instead of being answered "idle" and then
        # followed by the whole move.
        self._arm_stop_generation = 0
        self._head_lock = threading.Lock()
        self._head_generation = 0                            # bumped by stop/look: supersedes a home in progress
        self._stopped: set = set()

    # ── gate ─────────────────────────────────────────────────────────────────

    def _gate(self) -> None:
        """The robot's state must allow motion: fresh joints and a fresh, normal emergency-stop
        diagnostic. This is a condition, not a switch: it clears by itself when the robot is fine."""
        ready, reasons = self._state.motion_ready()
        if not ready:
            raise RuntimeError("motion refused: " + "; ".join(reasons))

    def _require_active(self, name: str) -> dict:
        states = self._io.list_controllers()
        if states.get(name) != "active":
            raise RuntimeError(f"controller {name} is {states.get(name)!r}, expected 'active'")
        return states

    # ── dispatch ─────────────────────────────────────────────────────────────

    DEFAULT_ACTIONS = {"base_drive": "move", "arm_control": "move", "gripper": "set",
                       "lifting": "set_position", "head": "look"}

    def dispatch(self, card: str, action: str, args: dict) -> dict:
        handler = {"base_drive": self.drive, "arm_control": self.arm, "gripper": self.gripper,
                   "lifting": self.lifting, "head": self.head}[card]
        # The bundle passes the card's own name when a call carries no `action` (README_dev: "if
        # there is no action field, it equals the tool name"). `lifting` has no action box at all,
        # and an LLM or hook may omit it on the others: that means the card's one motion.
        if action in (None, "", card):
            action = self.DEFAULT_ACTIONS[card]
        payload = dict(args)
        payload["action"] = action
        if action not in ("stop", "info"):
            self._stopped.discard(card)          # any other action means the card is in use again
        try:
            return handler(payload)
        except (ValueError, RuntimeError, TimeoutError) as exc:
            return {"state": "error", "message": str(exc)}

    def start(self, card: str) -> dict:
        """The lifecycle marker: the card is up and answering. It says nothing about whether a
        command would be accepted -- that is `ready_for_motion` / `accepts_motion` in the same
        result -- so a card on a robot that is not ready still starts."""
        self._stopped.discard(card)
        return self.info(card)

    def info(self, card: str) -> dict:
        """Never raises: `start` must succeed on a robot that is not ready (agent-core rolls the
        whole project back on one failed start), so what is wrong is reported here.

        `state` is the card's lifecycle (`ready` until it is stopped, `idle` after), kept apart
        from whether motion would be accepted right now."""
        ready, reasons = self._state.motion_ready()
        result = {"state": "idle" if card in self._stopped else "ready",
                  "ready_for_motion": ready, "not_ready_reasons": reasons,
                  "accepts_motion": ready}
        needed = {"base_drive": [DRIVE_CONTROLLER],
                  "arm_control": [ARM_FORWARD_CONTROLLER, *ARM_P2P_CONTROLLERS.values()],
                  "gripper": [GRIPPER_CONTROLLER.format(side=s) for s in ("left", "right")],
                  "lifting": [LIFT_CONTROLLER], "head": [HEAD_CONTROLLER]}[card]
        try:
            states = self._io.list_controllers()
            result["controllers"] = {name: states.get(name, "missing") for name in needed}
        except Exception as exc:  # noqa: BLE001
            result["controllers_error"] = f"{type(exc).__name__}: {exc}"
        if card == "arm_control":
            result["busy"] = self._arm_running
        if card == "base_drive":
            try:
                result["cmd_topics"] = self._io.twist_candidates(self._drive_candidates())
            except Exception as exc:  # noqa: BLE001
                result["cmd_topics_error"] = f"{type(exc).__name__}: {exc}"
        return result

    def stop_all(self) -> None:
        for card in ("base_drive", "arm_control", "head"):
            try:
                self.stop(card)
            except Exception as exc:  # noqa: BLE001
                print(f"[panther] stopping {card} failed: {type(exc).__name__}: {exc}", flush=True)

    def stop(self, card: str) -> dict:
        self._stopped.add(card)
        if card == "base_drive":
            return self._stop_drive()
        if card == "arm_control":
            return self._stop_arm()
        if card == "head":
            with self._head_lock:
                self._head_generation += 1                   # a home in progress stops where it is
            return {"state": "idle", "note": "a head home in progress stops where it is; a look "
                                              "already sent is the robot's to finish"}
        return {"state": "idle", "note": "single position command: nothing is in flight in this "
                                          "driver, and no further command will be sent"}

    # ── base_drive ───────────────────────────────────────────────────────────

    def _drive_candidates(self) -> list:
        cap = self._plugins.get("base_drive") or {}
        configured = cap.get("cmd_topics") or ([cap["topic"]] if cap.get("topic") else ["/cmd_vel"])
        return [str(topic) for topic in configured]

    def _stop_drive(self) -> dict:
        """Supersede any drive and send zero velocity, as one step under `_drive_lock`.

        The worker publishes only while holding the same lock, after re-checking the generation
        (`_publish_if_current`). So once this has the lock, no motion twist can be in flight, and
        once it returns, the worker sees the new generation before it could publish again: the
        zeros below are the last word. (Previously the check and the publish were separate, and a
        motion twist could follow these zeros.)"""
        cap = self._plugins.get("base_drive") or {}
        with self._drive_lock:
            self._drive_generation += 1
            for _ in range(max(1, int(cap.get("stop_repetitions", 5)))):
                self._io.publish_twist(0.0, 0.0, 0.0)
        return {"state": "idle"}

    def drive(self, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "stop":
            return self.stop("base_drive")
        if action == "start":
            return self.start("base_drive")
        if action == "info":
            return self.info("base_drive")
        if action != "move":
            raise ValueError(f"unsupported base_drive action: {action}")
        cap = self._plugins["base_drive"]
        vx, vy, wz, duration = validate_drive(args, cap)
        self._gate()
        self._require_active(DRIVE_CONTROLLER)
        candidates = self._drive_candidates()
        topic, counts = self._io.select_twist_topic(candidates)
        if topic is None:
            raise RuntimeError("no robot node subscribes to any of " + ", ".join(candidates)
                               + f" (subscribers: {counts}); the base would not move")
        with self._drive_lock:
            self._drive_generation += 1
            generation = self._drive_generation
        action_id = f"panther_drive_{uuid4().hex[:8]}"
        self._spawn(lambda: self._drive_worker(generation, vx, vy, wz, duration, action_id, cap),
                    "panther-drive")
        return {"state": "moving", "action_id": action_id, "linear_x": vx, "linear_y": vy,
                "angular_z": wz, "duration_s": duration, "auto_stop": True,
                "topic": topic, "subscribers": counts}

    def _superseded(self, generation: int) -> bool:
        with self._drive_lock:
            return generation != self._drive_generation

    def _publish_if_current(self, generation: int, vx: float, vy: float, wz: float) -> bool:
        """Publish only if this drive is still the current one, deciding and publishing under one
        lock so a stop cannot fall between the two. False means it was superseded and nothing was sent."""
        with self._drive_lock:
            if generation != self._drive_generation:
                return False
            self._io.publish_twist(vx, vy, wz)
            return True

    def _drive_worker(self, generation, vx, vy, wz, duration, action_id, cap) -> None:
        period = 1.0 / float(cap["publish_rate_hz"])
        repetitions = int(cap["stop_repetitions"])
        status, result = "completed", {}
        try:
            deadline = self._mono() + duration
            while self._mono() < deadline:
                if self._superseded(generation) or not self._publish_if_current(generation, vx, vy, wz):
                    status = "cancelled"
                    break
                self._sleep(period)
            if status == "completed":
                for _ in range(repetitions):
                    if not self._publish_if_current(generation, 0.0, 0.0, 0.0):
                        status = "cancelled"
                        break
                    self._sleep(period)
        except Exception as exc:  # noqa: BLE001
            status, result = "error", {"message": f"{type(exc).__name__}: {exc}"}
            try:
                self._io.publish_twist(0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001
                pass
        self._notify(action_id, status, result, "base_drive")

    # ── arm_control ──────────────────────────────────────────────────────────

    def arm(self, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "stop":
            return self.stop("arm_control")
        if action == "start":
            return self.start("arm_control")
        if action == "info":
            return self.info("arm_control")
        if action not in ("move", "home"):
            raise ValueError(f"unsupported arm_control action: {action}")
        with self._arm_lock:
            started_at = self._arm_stop_generation
        cap = self._plugins["arm_control"]
        self._gate()
        current = self._state.joint_positions()
        if current is None:
            raise RuntimeError("fresh /joint_states is required")
        limits, _ = self._state.joint_limits()
        if action == "move":
            targets, duration = self._arm_move_targets(args, cap, current, limits)
        else:
            targets, duration = self._arm_home_targets(args, cap, current, limits)
        states = self._io.list_controllers()
        requested = [ARM_P2P_CONTROLLERS[arm_side] for arm_side in targets]
        for name in requested:
            if name not in states:
                raise RuntimeError(f"controller {name} does not exist on this robot")
        plan = self._switch_plan(states, requested)
        with self._arm_lock:
            if self._arm_stop_generation != started_at:
                raise RuntimeError("arm_control was stopped while this move was being checked; "
                                   "nothing was sent, send it again if it is still wanted")
            if self._arm_running:
                raise RuntimeError("arm_control is already moving; send stop first")
            self._arm_running = True
            self._arm_began = False
            self._arm_cancel.clear()
            self._arm_done.clear()
        action_id = f"panther_arm_{uuid4().hex[:8]}"
        self._spawn(lambda: self._arm_worker(action_id, targets, duration, plan), "panther-arm")
        side = "both" if len(targets) == 2 else next(iter(targets))
        return {"state": "moving", "action_id": action_id, "action": action, "side": side,
                "duration_s": duration,
                "targets": {arm_side: dict(zip(names, positions)) for arm_side, (names, positions) in targets.items()},
                "controllers_activated": list(plan[0]), "controllers_deactivated": list(plan[1])}

    def _arm_move_targets(self, args: dict, cap: dict, current: dict, limits: dict) -> tuple:
        """Targets for the joints that were filled in. An empty box keeps that joint where it is, so
        one joint can be moved alone; an arm with no box filled is not moved at all."""
        given = {}
        for field, joint in ARM_FIELDS.items():
            value = optional_number(args.get(field), field)
            if value is not None:
                given[joint] = value
        if not given:
            raise ValueError("fill in at least one joint (la0~la7 for the left arm, ra0~ra7 for the right)")
        targets, furthest = {}, 0.0
        for arm_side, names in (("left", LEFT_ARM_JOINTS), ("right", RIGHT_ARM_JOINTS)):
            if not any(name in given for name in names):
                continue
            unseen = [name for name in names if name not in current]
            if unseen:
                raise RuntimeError("fresh joint state is missing: " + ", ".join(unseen))
            positions = [given.get(name, float(current[name])) for name in names]
            targets[arm_side] = (names, validate_joint_target(names, positions, limits, current))
            furthest = max([furthest] + [abs(p - float(current[n])) for n, p in zip(names, positions)])
        # The operator should not have to guess a duration: it follows from how far the furthest
        # joint has to go and the configured traversal speed, then is bounded.
        duration = furthest / float(cap["speed_rad_s"])
        duration = min(float(cap["max_duration_s"]),
                       max(float(cap.get("min_duration_s", 1.0)), duration))
        return targets, round(duration, 3)

    def _arm_home_targets(self, args: dict, cap: dict, current: dict, limits: dict) -> tuple:
        """Every joint of the chosen arms to its home position (0 rad unless config.yaml's
        `home_pose` says otherwise). Not held to the per-command step: the duration is stretched
        instead so that no joint goes faster than `home_speed_rad_s` on average."""
        side = args.get("side") or "both"
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right or both")
        pose = cap.get("home_pose") or {}
        speed = float(cap["home_speed_rad_s"])
        targets, furthest = {}, 0.0
        for arm_side, names in (("left", LEFT_ARM_JOINTS), ("right", RIGHT_ARM_JOINTS)):
            if side not in (arm_side, "both"):
                continue
            positions = [finite_number(pose.get(name, 0.0), f"home_pose.{name}") for name in names]
            targets[arm_side] = (names, validate_joint_target(names, positions, limits, current))
            furthest = max([furthest] + [abs(p - float(current[n])) for n, p in zip(names, positions)])
        duration = max(float(cap.get("home_min_duration_s", 2.0)), furthest / speed)
        if duration > float(cap["max_duration_s"]):
            raise ValueError(f"home would take {duration:.1f} s at {speed} rad/s, over the "
                             f"{cap['max_duration_s']} s limit")
        return targets, round(duration, 3)

    @staticmethod
    def _switch_plan(states: dict, requested: list) -> tuple:
        """`(activate, deactivate)` for the move, from what is active right now.

        Only the trajectory controllers of the arms being moved are touched, and only if they
        are not already active. The forward-position controller owns the joints of both arms, so
        it has to be released for the move and is put back afterwards if it was active. Anything
        that was already active and is not in the way is left exactly as it is.
        """
        activate = tuple(name for name in requested if states.get(name) != "active")
        deactivate = (ARM_FORWARD_CONTROLLER,) if states.get(ARM_FORWARD_CONTROLLER) == "active" else ()
        return activate, deactivate

    def _switch_arm(self, plan: tuple, restore: bool) -> None:
        """Apply `plan`, or its exact inverse. Nothing is sent if there is nothing to change."""
        activate, deactivate = plan
        first, second = (deactivate, activate) if restore else (activate, deactivate)
        if first or second:
            self._io.switch_controllers(activate=list(first), deactivate=list(second))

    def _arm_worker(self, action_id: str, targets: dict, duration: float, plan: tuple) -> None:
        status, result, handles, switched = "completed", {}, {}, False
        with self._arm_lock:
            self._arm_began = True
        try:
            # A stop can land at any moment after the move was accepted, including before this
            # thread has run at all (a busy machine, a slow start). It only sets a flag, so the
            # flag is looked at before every step that does something to the robot: switching
            # controllers, and each trajectory goal. Checking it only in the polling loop below
            # let a stop that came right after a move be followed by the whole move.
            if self._arm_cancel.is_set():
                status = "cancelled"
            else:
                self._switch_arm(plan, restore=False)
                switched = True
                for arm_side, (names, positions) in targets.items():
                    if self._arm_cancel.is_set():
                        status = "cancelled"
                        break
                    handles[arm_side] = self._io.send_arm_goal(arm_side, list(names), positions, duration)
            deadline = self._mono() + duration + 5.0
            pending = dict(handles) if status == "completed" else {}
            while pending:
                if self._arm_cancel.is_set():
                    status = "cancelled"
                    break
                if self._mono() > deadline:
                    raise TimeoutError("trajectory did not finish in time")
                for arm_side, handle in list(pending.items()):
                    outcome = self._io.poll_arm_result(handle, 0.25)
                    if outcome is None:
                        continue
                    del pending[arm_side]
                    result[arm_side] = outcome
                    if outcome.get("error_code", 0) != 0:
                        raise RuntimeError(f"{arm_side} trajectory failed: {outcome.get('error_string')}")
        except Exception as exc:  # noqa: BLE001
            status = "error"
            result["message"] = f"{type(exc).__name__}: {safe_text(exc, 160)}"
        finally:
            if status != "completed":
                for handle in handles.values():
                    try:
                        self._io.cancel_arm_goal(handle)
                    except Exception:  # noqa: BLE001
                        pass
            if switched:
                try:
                    self._switch_arm(plan, restore=True)
                    result["controllers_restored"] = True
                except Exception as exc:  # noqa: BLE001
                    result["controllers_restored"] = False
                    status = "error"
                    result["restore_error"] = f"{type(exc).__name__}: {safe_text(exc, 160)}"
                    print("[panther] WARNING: could not restore the arm controllers after a "
                          f"move: {safe_text(exc, 160)}. Check `ros2 control list_controllers`.", flush=True)
            with self._arm_lock:
                self._arm_running = False
            self._arm_done.set()
        self._notify(action_id, status, result, "arm_control")

    def _stop_arm(self) -> dict:
        # Under the same lock `arm()` takes to mark a move as running, so a stop and a start are
        # never interleaved half way: either the move is already marked (and the flag set here is
        # the one its worker will see) or nothing is running and there is nothing to cancel.
        # Setting the flag with nothing running would cancel the *next* move, so it is not set.
        with self._arm_lock:
            self._arm_stop_generation += 1                   # cancels a move that is still being checked
            if not self._arm_running:
                return {"state": "idle"}
            self._arm_cancel.set()
            began = self._arm_began
        if not began:
            # The worker has not run. Nothing has been switched or sent, and when it does run it
            # sees the flag first and does nothing, so there is nothing to wait for. (Waiting
            # here would just sit out the timeout and then report "stopping" about a robot that
            # is not moving.)
            return {"state": "idle", "stopped_by": "stop", "note": "cancelled before it began"}
        finished = self._arm_done.wait(timeout=10.0)
        return {"state": "idle" if finished else "stopping", "stopped_by": "stop"}

    # ── gripper, lifting, head: one position command each ────────────────────

    def gripper(self, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "stop":
            return self.stop("gripper")
        if action == "start":
            return self.start("gripper")
        if action == "info":
            return self.info("gripper")
        if action not in ("set", "open", "close"):
            raise ValueError(f"unsupported gripper action: {action}")
        side = args.get("side") or "both"
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right or both")
        if action == "set":
            position = optional_number(args.get("position"), "position")
            if position is None:
                raise ValueError("position is required (0 = open, 1 = closed)")
        else:
            position = 0.0 if action == "open" else 1.0       # 0 = open, 1 = closed, confirmed by the operator
        if not 0.0 <= position <= 1.0:
            raise ValueError("position must be within 0 ~ 1 (0 = open, 1 = closed)")
        self._gate()
        sides = ["left", "right"] if side == "both" else [side]
        for each in sides:                                   # both checked before either is sent
            self._require_active(GRIPPER_CONTROLLER.format(side=each))
        for each in sides:
            self._io.publish_gripper(each, GRIPPER_PARALLEL, position)
        return {"state": "commanded", "side": side, "position": position}

    def lifting(self, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "stop":
            return self.stop("lifting")
        if action == "start":
            return self.start("lifting")
        if action == "info":
            return self.info("lifting")
        if action != "set_position":
            raise ValueError(f"unsupported lifting action: {action}")
        cap = self._plugins["lifting"]
        position = optional_number(args.get("position"), "position")
        if position is None:
            raise ValueError("position is required")
        if not 0.0 <= position <= float(cap["max_position"]):
            raise ValueError(f"position must be within 0 ~ {float(cap['max_position']):g}")
        self._gate()
        current = self._state.lift_position()
        if current is None:
            raise RuntimeError("a fresh lift reading is required before commanding it")
        self._require_active(LIFT_CONTROLLER)
        self._io.publish_lift(position)
        return {"state": "commanded", "position": position, "previous_reading": current}

    def head(self, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "stop":
            return self.stop("head")
        if action == "start":
            return self.start("head")
        if action == "info":
            return self.info("head")
        if action not in ("look", "home"):
            raise ValueError(f"unsupported head action: {action}")
        cap = self._plugins.get("head") or {}
        self._gate()
        limits, _ = self._state.joint_limits()
        current = self._state.joint_positions()
        if current is None:
            raise RuntimeError("fresh /joint_states is required")
        unseen = [name for name in HEAD_JOINTS if name not in current]
        if unseen:
            raise RuntimeError("fresh joint state is missing: " + ", ".join(unseen))
        if action == "look":
            given = {joint: optional_number(args.get(field), field) for field, joint in HEAD_FIELDS.items()}
            if all(value is None for value in given.values()):
                raise ValueError("fill in head0_rad, head1_rad or both")
            target = [float(current[joint]) if given[joint] is None else given[joint] for joint in HEAD_JOINTS]
            validated = validate_joint_target(HEAD_JOINTS, target, limits, current)
            self._require_active(HEAD_CONTROLLER)
            with self._head_lock:
                self._head_generation += 1                   # a look supersedes a home in progress
            self._io.publish_head(validated)
            return {"state": "commanded", "head0_rad": validated[0], "head1_rad": validated[1]}
        pose = cap.get("home_pose") or {}
        target = [finite_number(pose.get(joint, 0.0), f"home_pose.{joint}") for joint in HEAD_JOINTS]
        validated = validate_joint_target(HEAD_JOINTS, target, limits, current)
        self._require_active(HEAD_CONTROLLER)
        start = [float(current[joint]) for joint in HEAD_JOINTS]
        with self._head_lock:
            self._head_generation += 1
            generation = self._head_generation
        action_id = f"panther_head_{uuid4().hex[:8]}"
        speed = float(cap.get("home_speed_rad_s", 0.35))
        period = 1.0 / float(cap.get("home_publish_rate_hz", 20.0))
        furthest = max(abs(t - s) for t, s in zip(validated, start))
        steps = max(1, math.ceil(furthest / (speed * period)))
        self._spawn(lambda: self._head_home_worker(generation, start, validated, steps, period, action_id),
                    "panther-head")
        return {"state": "moving", "action_id": action_id, "head0_rad": validated[0],
                "head1_rad": validated[1], "duration_s": round(steps * period, 3)}

    def _head_home_worker(self, generation, start, target, steps, period, action_id) -> None:
        """Step the forward-position controller from `start` to `target`: it moves at its own top
        speed to whatever it is sent, so the speed limit is applied by sending small steps."""
        status, result = "completed", {}
        try:
            for index in range(1, steps + 1):
                with self._head_lock:
                    if generation != self._head_generation:
                        status = "cancelled"
                        break
                fraction = index / steps
                self._io.publish_head([s + (t - s) * fraction for s, t in zip(start, target)])
                self._sleep(period)
        except Exception as exc:  # noqa: BLE001
            status, result = "error", {"message": f"{type(exc).__name__}: {safe_text(exc, 160)}"}
        self._notify(action_id, status, result, "head")
