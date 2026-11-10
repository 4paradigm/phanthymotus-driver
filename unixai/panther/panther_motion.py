"""Panther motion cards: the five actuators, behind one gate, with a stop for each.

Nothing here imports ROS. Every ROS call goes through `io` (`RosMotionIO` in device.py in
production, a recorder in the tests) and everything the gate needs to know about the robot
comes from `state` (the `Hub`). That keeps the rules that decide whether the robot may move
-- the commissioning switch, confirmation, fresh state, limits, caps, stop, completion --
checkable without a robot, which is the only place a mistake here can be found safely.

Ported from the Panther driver proposed in PR #520 and extended where its review found gaps:

* every actuator answers `stop`;
* `arm_control` and `base_drive` run in a worker and return an `action_id` at once, then POST
  completion to Agent Core (README_dev "ACP / x-completion"), so the call does not block the
  MCP request for the length of the motion.

What `stop` can and cannot do. `base_drive` and `arm_control` have something in flight and stop
it: zero velocity repeated, or the trajectory goal cancelled and the controller restored.
`gripper`, `lifting` and `head` are one position command each; once published the robot's
controller owns the motion and this driver has no halt to call (the vendor halt services are
deliberately not used). For those, `stop` guarantees only that this driver sends nothing more.
"""
from __future__ import annotations

import json
import os
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from contracts import (
    HEAD_JOINTS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, finite_number, finite_vector,
    require_confirmation, validate_drive, validate_joint_target,
)

DRIVE_CONTROLLER = "movebase3_drive_controller"
ARM_FORWARD_CONTROLLER = "dual_arm_forward_position_controller"
ARM_P2P_CONTROLLERS = {"left": "left_arm_joint_trajectory_position_controller",
                       "right": "right_arm_joint_trajectory_position_controller"}
HEAD_CONTROLLER = "head_forward_position_controller"
LIFT_CONTROLLER = "lifting_controller"
GRIPPER_CONTROLLER = "{side}_unix_gripper3_controller"
GRIPPER_MODES = ("2-finger", "parallel", "centric")

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

def _confirm() -> dict:
    return {"type": "boolean", "const": True,
            "description": "必须为 true：确认机器人可以运动，周围无人且无障碍"}


def motion_tools(config: dict) -> list:
    """Tool definitions for the enabled motion cards (type `actuator`)."""
    plugins = (config or {}).get("plugins", {})
    tools = []

    def on(name):
        return bool((plugins.get(name) or {}).get("enabled", False))

    def schema(actions, properties, params, resource, completion=None):
        body = {"type": "object",
                "properties": {"action": {"type": "string", "enum": list(actions)}, **properties},
                "required": ["action"], "additionalProperties": False,
                "x-action-params": {a: {"params": params.get(a, [])} for a in actions},
                "x-resource": resource}
        if completion:
            body["x-completion"] = completion
        return body

    if on("base_drive"):
        cap = plugins["base_drive"]
        lin, ang = cap["max_linear_mps"], cap["max_angular_radps"]
        tools.append({
            "name": "base_drive", "type": "actuator", "multiInstance": False,
            "description": "Panther 全向底盘定时运动（/cmd_vel_teleop）。单次时长有上限，到时自动归零；"
                           "stop 立即停车。默认被 safety.commissioning_motion_enabled 禁止",
            "inputSchema": schema(
                ["start", "stop", "info", "move"],
                {"linear_x": {"type": "number", "minimum": -lin, "maximum": lin},
                 "linear_y": {"type": "number", "minimum": -lin, "maximum": lin},
                 "angular_z": {"type": "number", "minimum": -ang, "maximum": ang},
                 "duration_s": {"type": "number", "minimum": 0.1, "maximum": cap["max_duration_s"]},
                 "confirm_motion": _confirm()},
                {"move": ["linear_x", "linear_y", "angular_z", "duration_s", "confirm_motion"]},
                "base", {"actions": ["move"], "timeout": int(cap["max_duration_s"]) + 10})})
    if on("arm_control"):
        cap = plugins["arm_control"]
        joints = {"type": "array", "items": {"type": "number"}, "minItems": 8, "maxItems": 8}
        tools.append({
            "name": "arm_control", "type": "actuator", "multiInstance": False,
            "description": "Panther 双臂点到点运动（FollowJointTrajectory）。目标按 URDF 限位和单次步长校验；"
                           "运动期间临时切换控制器，结束后恢复；stop 取消并恢复",
            "inputSchema": schema(
                ["start", "stop", "info", "move"],
                {"side": {"type": "string", "enum": ["left", "right", "both"]},
                 "positions": joints, "left_positions": joints, "right_positions": joints,
                 "duration_s": {"type": "number", "minimum": 1.0, "maximum": cap["max_duration_s"]},
                 "confirm_motion": _confirm()},
                {"move": ["side", "positions", "left_positions", "right_positions", "duration_s",
                          "confirm_motion"]},
                ["arm_l", "arm_r"], {"actions": ["move"], "timeout": int(cap["max_duration_s"]) + 15})})
    if on("gripper"):
        tools.append({
            "name": "gripper", "type": "actuator", "multiInstance": False,
            "description": "Panther 左右夹爪位置指令（0~1，开合含义以厂商文档为准，未经实测）。"
                           "单次指令，下发后由机器人控制器执行；stop 仅保证本驱动不再下发",
            "inputSchema": schema(
                ["start", "stop", "info", "set"],
                {"side": {"type": "string", "enum": ["left", "right"]},
                 "mode": {"type": "string", "enum": list(GRIPPER_MODES), "default": "parallel"},
                 "position": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                 "confirm_motion": _confirm()},
                {"set": ["side", "mode", "position", "confirm_motion"]}, ["hand_l", "hand_r"])})
    if on("lifting"):
        cap = plugins["lifting"]
        tools.append({
            "name": "lifting", "type": "actuator", "multiInstance": False,
            "description": "Panther 升降位置指令（单位未经厂商确认，读数约 0.61）。目标限制在配置上限内，"
                           "且与当前读数相差不超过单次步长；stop 仅保证本驱动不再下发",
            "inputSchema": schema(
                ["start", "stop", "info", "set_position"],
                {"position": {"type": "number", "minimum": 0.0, "maximum": cap["max_position"]},
                 "confirm_motion": _confirm()},
                {"set_position": ["position", "confirm_motion"]}, "waist")})
    if on("head"):
        tools.append({
            "name": "head", "type": "actuator", "multiInstance": False,
            "description": "Panther 头部两个关节位置指令。按 URDF：joint_head0 绕 y（俯仰），joint_head1 绕 z（偏航）；"
                           "目标按 URDF 限位和单次步长校验；stop 仅保证本驱动不再下发",
            "inputSchema": schema(
                ["start", "stop", "info", "look"],
                {"head0_rad": {"type": "number"}, "head1_rad": {"type": "number"},
                 "confirm_motion": _confirm()},
                {"look": ["head0_rad", "head1_rad", "confirm_motion"]}, "head")})
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
        self._stopped: set = set()

    # ── gate ─────────────────────────────────────────────────────────────────

    def motion_enabled(self) -> bool:
        return (self._cfg.get("safety") or {}).get("commissioning_motion_enabled") is True

    def _gate(self) -> None:
        if not self.motion_enabled():
            raise RuntimeError("motion is disabled: set safety.commissioning_motion_enabled to true "
                               "only after the commissioning limits have been checked on this robot")
        ready, reasons = self._state.motion_ready()
        if not ready:
            raise RuntimeError("motion refused: " + "; ".join(reasons))

    def _require_active(self, name: str) -> dict:
        states = self._io.list_controllers()
        if states.get(name) != "active":
            raise RuntimeError(f"controller {name} is {states.get(name)!r}, expected 'active'")
        return states

    # ── dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, card: str, action: str, args: dict) -> dict:
        handler = {"base_drive": self.drive, "arm_control": self.arm, "gripper": self.gripper,
                   "lifting": self.lifting, "head": self.head}[card]
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
        command would be accepted -- that is `ready_for_motion` / `commissioning_motion_enabled`
        in the same result -- so a card that is deliberately gated by default still starts."""
        self._stopped.discard(card)
        return self.info(card)

    def info(self, card: str) -> dict:
        """Never raises: `start` must succeed on a robot that is not ready (agent-core rolls the
        whole project back on one failed start), so what is wrong is reported here.

        `state` is the card's lifecycle (`ready` until it is stopped, `idle` after), kept apart
        from whether motion would be accepted right now."""
        ready, reasons = self._state.motion_ready()
        result = {"state": "idle" if card in self._stopped else "ready",
                  "commissioning_motion_enabled": self.motion_enabled(),
                  "ready_for_motion": ready, "not_ready_reasons": reasons,
                  "accepts_motion": self.motion_enabled() and ready}
        needed = {"base_drive": [DRIVE_CONTROLLER],
                  "arm_control": [ARM_FORWARD_CONTROLLER, *ARM_P2P_CONTROLLERS.values()],
                  "gripper": [GRIPPER_CONTROLLER.format(side=s) for s in ("left", "right")],
                  "lifting": [LIFT_CONTROLLER], "head": [HEAD_CONTROLLER]}[card]
        try:
            states = self._io.list_controllers()
            result["controllers"] = {name: states.get(name, "missing") for name in needed}
        except Exception as exc:  # noqa: BLE001
            result["controllers_error"] = f"{type(exc).__name__}: {exc}"
        if not self.motion_enabled():
            result["note"] = "运动被 safety.commissioning_motion_enabled 禁止；传感器卡不受影响"
        if card == "arm_control":
            result["busy"] = self._arm_running
        return result

    def stop_all(self) -> None:
        for card in ("base_drive", "arm_control"):
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
        return {"state": "idle", "note": "single position command: nothing is in flight in this "
                                          "driver, and no further command will be sent"}

    # ── base_drive ───────────────────────────────────────────────────────────

    def _zero(self, repetitions: int, period: float) -> None:
        for _ in range(max(1, repetitions)):
            self._io.publish_twist(0.0, 0.0, 0.0)
            self._sleep(period)

    def _stop_drive(self) -> dict:
        cap = self._plugins.get("base_drive") or {}
        with self._drive_lock:
            self._drive_generation += 1
        self._zero(int(cap.get("stop_repetitions", 5)), 0.0)
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
        with self._drive_lock:
            self._drive_generation += 1
            generation = self._drive_generation
        action_id = f"panther_drive_{uuid4().hex[:8]}"
        self._spawn(lambda: self._drive_worker(generation, vx, vy, wz, duration, action_id, cap),
                    "panther-drive")
        return {"state": "moving", "action_id": action_id, "linear_x": vx, "linear_y": vy,
                "angular_z": wz, "duration_s": duration, "auto_stop": True}

    def _superseded(self, generation: int) -> bool:
        with self._drive_lock:
            return generation != self._drive_generation

    def _drive_worker(self, generation, vx, vy, wz, duration, action_id, cap) -> None:
        period = 1.0 / float(cap["publish_rate_hz"])
        repetitions = int(cap["stop_repetitions"])
        status, result = "completed", {}
        try:
            deadline = self._mono() + duration
            while self._mono() < deadline:
                if self._superseded(generation):
                    status = "cancelled"
                    break
                self._io.publish_twist(vx, vy, wz)
                self._sleep(period)
            if status == "completed":
                for _ in range(repetitions):
                    if self._superseded(generation):
                        status = "cancelled"
                        break
                    self._io.publish_twist(0.0, 0.0, 0.0)
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
        if action != "move":
            raise ValueError(f"unsupported arm_control action: {action}")
        require_confirmation(args)
        cap = self._plugins["arm_control"]
        duration = finite_number(args.get("duration_s", 6.0), "duration_s")
        if not 1.0 <= duration <= float(cap["max_duration_s"]):
            raise ValueError(f"duration_s must be in [1.0, {cap['max_duration_s']}]")
        side = args.get("side", "left")
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right or both")
        self._gate()
        current = self._state.joint_positions()
        if current is None:
            raise RuntimeError("fresh /joint_states is required")
        limits, _ = self._state.joint_limits()
        step = float(cap["max_target_delta_rad"])
        targets = {}
        for arm_side, names, own_key in (("left", LEFT_ARM_JOINTS, "left_positions"),
                                         ("right", RIGHT_ARM_JOINTS, "right_positions")):
            if side not in (arm_side, "both"):
                continue
            positions = args.get("positions" if side == arm_side else own_key)
            targets[arm_side] = (names, validate_joint_target(names, positions, limits, current, step))
        states = self._io.list_controllers()
        requested = [ARM_P2P_CONTROLLERS[arm_side] for arm_side in targets]
        for name in requested:
            if name not in states:
                raise RuntimeError(f"controller {name} does not exist on this robot")
        plan = self._switch_plan(states, requested)
        with self._arm_lock:
            if self._arm_running:
                raise RuntimeError("arm_control is already moving; send stop first")
            self._arm_running = True
            self._arm_cancel.clear()
            self._arm_done.clear()
        action_id = f"panther_arm_{uuid4().hex[:8]}"
        self._spawn(lambda: self._arm_worker(action_id, targets, duration, plan), "panther-arm")
        return {"state": "moving", "action_id": action_id, "side": side, "duration_s": duration,
                "controllers_activated": list(plan[0]), "controllers_deactivated": list(plan[1])}

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
        try:
            self._switch_arm(plan, restore=False)
            switched = True
            for arm_side, (names, positions) in targets.items():
                handles[arm_side] = self._io.send_arm_goal(arm_side, list(names), positions, duration)
            deadline = self._mono() + duration + 5.0
            pending = dict(handles)
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
            result["message"] = f"{type(exc).__name__}: {exc}"
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
                    result["restore_error"] = f"{type(exc).__name__}: {exc}"
                    print("[panther] WARNING: could not restore the arm controllers after a "
                          f"move: {exc}. Check `ros2 control list_controllers`.", flush=True)
            with self._arm_lock:
                self._arm_running = False
            self._arm_done.set()
        self._notify(action_id, status, result, "arm_control")

    def _stop_arm(self) -> dict:
        if not self._arm_running:
            return {"state": "idle"}
        self._arm_cancel.set()
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
        if action != "set":
            raise ValueError(f"unsupported gripper action: {action}")
        require_confirmation(args)
        side = args.get("side")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        position = finite_number(args.get("position"), "position")
        if not 0.0 <= position <= 1.0:
            raise ValueError("position must be in [0, 1]")
        mode = args.get("mode", "parallel")
        if mode not in GRIPPER_MODES:
            raise ValueError("mode must be one of " + ", ".join(GRIPPER_MODES))
        self._gate()
        self._require_active(GRIPPER_CONTROLLER.format(side=side))
        self._io.publish_gripper(side, mode, position)
        return {"state": "commanded", "side": side, "mode": mode, "position": position}

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
        require_confirmation(args)
        cap = self._plugins["lifting"]
        position = finite_number(args.get("position"), "position")
        if not 0.0 <= position <= float(cap["max_position"]):
            raise ValueError(f"position must be in [0, {cap['max_position']}]")
        self._gate()
        current = self._state.lift_position()
        if current is None:
            raise RuntimeError("a fresh lift reading is required before commanding it")
        if abs(position - current) > float(cap["max_target_delta"]):
            raise ValueError(f"position is {abs(position - current):.3f} from the current reading "
                             f"{current:.3f}, over the commissioning step limit of "
                             f"{cap['max_target_delta']}")
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
        if action != "look":
            raise ValueError(f"unsupported head action: {action}")
        require_confirmation(args)
        target = finite_vector([args.get("head0_rad"), args.get("head1_rad")], "head targets", 2)
        self._gate()
        limits, _ = self._state.joint_limits()
        current = self._state.joint_positions()
        if current is None:
            raise RuntimeError("fresh /joint_states is required")
        step = float((self._plugins.get("head") or {}).get("max_target_delta_rad", 0.35))
        validated = validate_joint_target(HEAD_JOINTS, target, limits, current, step)
        self._require_active(HEAD_CONTROLLER)
        self._io.publish_head(validated)
        return {"state": "commanded", "head0_rad": validated[0], "head1_rad": validated[1]}
