"""`upper_body_mimic` — drive Bumi's arms from the `upper_body_track` angles.

This is the writing half of upper-body mirroring. The read-only card publishes
six retargeted arm angles; this one turns them into motor commands.

It is not an "arm card", and that distinction matters more than anything else
in this file. `LowController.set_joint` takes exactly 21 commands — it rejects
anything else — so every tick this card sends all 21 motors: thirteen leg/waist
joints and the two unobservable shoulder-yaw joints hold their measured
position, six arm joints follow the operator. Sending 21 commands means taking
the motors away from whatever was driving them, which is why the plugin is
disabled by default, requires an explicit DDS sign-off, and is documented as a
whole-body takeover rather than an arm feature. The card does not implement a
balance controller: it holds the legs where they already are. Large arm swings
still shift the centre of mass, so the first runs belong on a supported robot.

Safety properties, each of which has a test:

- workmode must be 2 (`walking`) at start and on every tick; leaving it stops
  commanding and latches the run as an operator intervention
- `LowController`/`HighController` coexistence must be verified on the real
  robot and acknowledged with `dds_contention_verified: true` before `start`
- 2 rad/s velocity and 6 rad/s² acceleration bounds (PR #485 preview), applied
  to the commanded pose; mechanical stops come from the shipped URDF
- the first command after `start` is the pose the arm is already in
- lost or low-confidence tracking → return to the safe pose, never hold the
  last human command
- missing controller, unreadable workmode and SDK failure all refuse to command
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import upper_body_mimic_spec as spec

# The camera path uses a best-effort, depth-1 subscription so a stalled consumer
# cannot make the publisher queue frames. The angles topic is published by the
# tracking card with the same profile.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

#: `device.py`'s `_WORKMODE_NAMES`: only walking mode is acceptable. Any exit
#: from 2 — including protection (26) and disabled (30) — is treated as a
#: latched operator/robot intervention: the run stops and must be re-armed.
_WALKING_WORKMODE = 2
_PROTECTION_WORKMODE = 26
_DISABLED_WORKMODE = 30

_SDK_PATH = "/work/noetix_sdk_bumi/build"


def _load_low_level():
    """Import the low-level control bindings.

    Deferred to call time for two reasons: the module needs `PYTHONPATH` set up
    by `main.py`, and importing it during module load would make every test
    require the ARM64 `.so`. Tests replace this function rather than the SDK.
    """
    if _SDK_PATH not in sys.path:
        sys.path.insert(0, _SDK_PATH)
    from lowcontrol_py import LowController, MotorCmd  # noqa: PLC0415

    return LowController, MotorCmd


def _probe_low_level(timeout_s: float = 10.0) -> tuple[bool, str]:
    """Check that `LowController.init()` can complete before calling it.

    `main.py` does the same dance for `HighController`: `init()` blocks in C++
    with the GIL held when the robot is unreachable, which would hang this
    plugin's `start` action and, through it, the MCP request. A subprocess with
    a timeout turns "the robot is not there" into a message instead of a hang.
    """
    code = (
        "import sys;"
        f"sys.path.insert(0, {_SDK_PATH!r});"
        "from lowcontrol_py import LowController;"
        "LowController.instance().init();"
        "print('OK')"
    )
    try:
        result = subprocess.run(
            [sys.executable, "-c", code],
            capture_output=True, text=True, timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return False, "LowController.init() timed out (robot unreachable?)"
    except Exception as exc:  # noqa: BLE001 - surfaced as the start error
        return False, f"LowController probe failed: {exc}"
    if "OK" in result.stdout:
        return True, ""
    detail = (result.stderr or "").strip().splitlines()
    return False, "LowController probe failed: " + (detail[-1] if detail else "unknown")


class UpperBodyMimicPlugin:
    PREFIX = "upper_body_mimic"

    def __init__(self, plugin_config: dict, namespace: str,
                 executor, high_ctrl=None, loco_plugin=None, loco_servo=None):
        cfg = plugin_config or {}
        self._namespace = namespace
        self._default_angles_topic = f"/{namespace}/upper_body_track/angles"
        self._angles_topic = self._default_angles_topic
        self._high_ctrl = high_ctrl
        # set_joint takes the whole robot, not just the arms. A running chassis
        # writer (`loco` motion or `loco_servo` stream) must not coexist with
        # this loop. These are the same two references the servo card uses for
        # its own arbitration; the bundle wires them in registration order.
        self._loco = loco_plugin
        self._servo = loco_servo
        # Both cards are constructed before this one. Attach from this side the
        # same way LocoServoPlugin attaches itself to `loco`: each now knows it
        # must yield when the other owns the robot.
        if self._loco is not None and hasattr(self._loco, "attach_mimic"):
            self._loco.attach_mimic(self)
        if self._servo is not None and hasattr(self._servo, "attach_mimic"):
            self._servo.attach_mimic(self)

        self._control_hz = float(cfg.get("control_hz", 50.0))
        # PR #485 preview bounds, in rad/s and rad/s²; the spec converts them.
        self._max_velocity_rad_s = float(cfg.get("max_velocity_rad_s",
                                                  spec.MAX_VELOCITY_RAD_S))
        self._max_acceleration_rad_s2 = float(cfg.get(
            "max_acceleration_rad_s2", spec.MAX_ACCELERATION_RAD_S2))
        self._max_velocity_deg_s = math.degrees(self._max_velocity_rad_s)
        self._max_acceleration_deg_s2 = math.degrees(self._max_acceleration_rad_s2)
        #: The operator must confirm on the real robot that LowController and
        #: HighController do not fight over DDS before any motion is allowed.
        self._dds_verified = bool(cfg.get("dds_contention_verified", False))
        self._lost_timeout_s = float(cfg.get("lost_timeout_s", 0.5))
        self._margin_deg = float(cfg.get("limit_margin_deg", spec.LIMIT_MARGIN_DEG))
        self._keep_kp = float(cfg.get("keep_kp", 10.0))
        self._keep_kd = float(cfg.get("keep_kd", 0.5))
        self._arm_kp = float(cfg.get("arm_kp", 5.0))
        self._arm_kd = float(cfg.get("arm_kd", 0.5))
        #: False is a debugging aid, not a shipping mode: it runs the whole
        #: pipeline including the rate limiter and the command builder, and then
        #: skips `set_joint`. Useful to check the mapping and the limits on a
        #: robot without letting it move.
        self._send_commands = bool(cfg.get("send_commands", True))

        self._node = Node("bumi_upper_body_mimic")
        executor.add_node(self._node)

        self._angles_sub = None
        self._worker = None
        self._run_token = None
        self._stop_event = threading.Event()

        self._lock = threading.Lock()
        self._latest_targets = dict(spec.ZERO_TARGETS_DEG)
        self._latest_tracked = False
        self._latest_at = 0.0

        self._commanded_deg = None
        self._velocity_deg_s = {key: 0.0 for key in spec.ZERO_TARGETS_DEG}
        #: Whether this run has ever seen a tracked operator. Until it has,
        #: "lost" means "not started yet", not "the operator walked away":
        #: a card armed before the camera is wired must hold the measured pose
        #: rather than immediately drive the arms down.
        self._ever_tracked = False
        self._low_ctrl = None
        self._motor_cmd_cls = None

        self._status = "idle"
        self._ticks = 0
        self._lost = True
        self._last_error = None
        self._last_workmode = None

        self._validate()

    # ── configuration ────────────────────────────────────────────────────────

    def _validate(self) -> None:
        """Range checks belong here, not as silent clamps on the way out.

        A 200 Hz control loop is not "responsive", it is a budget the process
        cannot meet; a wide velocity/acceleration bound is not "fast", it is
        the number at which the limiter stops protecting anything. Both would
        present as plausible numbers in `info`, which is why they raise.
        """
        checks = (
            ("control_hz", self._control_hz, 10.0, 200.0),
            ("max_velocity_rad_s", self._max_velocity_rad_s, 0.1, 5.0),
            ("max_acceleration_rad_s2", self._max_acceleration_rad_s2, 0.1, 30.0),
            ("lost_timeout_s", self._lost_timeout_s, 0.1, 5.0),
            ("limit_margin_deg", self._margin_deg, 0.0, 30.0),
        )
        for name, value, low, high in checks:
            if not low <= value <= high:
                raise ValueError(f"{name} must be in [{low}, {high}], got {value}")
        for name, value in (("keep_kp", self._keep_kp), ("keep_kd", self._keep_kd),
                            ("arm_kp", self._arm_kp), ("arm_kd", self._arm_kd)):
            if value < 0:
                raise ValueError(f"{name} must be non-negative, got {value}")
        if self._keep_kp <= 0:
            raise ValueError("keep_kp must be positive: a zero hold gain drops the legs")

    # ── tool description ─────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "upper_body_mimic",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "把人的上半身动作镜像到 Bumi 的手臂上。订阅上半身跟踪卡输出的六个手臂角度，"
                "每帧下发 21 条电机指令：13 个腿/腰关节和 2 个单目不可观测的肩关节保持在实测位置，"
                "6 个手臂关节跟随人。注意：LowController 只接受 21 条指令，所以这张卡在运行时"
                "接管了全部电机（包括腿），它不做平衡控制；手臂大幅摆动仍会改变重心，"
                "首次运行请确保机器人有支撑。只在 workmode=2（walking）下运行，离开该模式立即停止；"
                "跟踪丢失或置信度不足时按 2 rad/s、6 rad/s² 回到安全姿态，不保留上一帧人的命令。"
                "限位取自机器人 URDF。默认关闭，且必须先在真机确认 LowController 与 HighController"
                "不抢 DDS 控制权并把 dds_contention_verified 设为 true，才能 start。"
                "这是一张危险动作卡：start 会真实接管全部电机，必须显式传入 confirm=true；"
                "缺少或非布尔 true 一律拒绝，Agent Core 的危险动作确认门禁也依赖 x-is-dangerous 标记。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "卡片生命周期或实时状态查询",
                    },
                    "angles_topic": {
                        "type": "string",
                        "description": (
                            "上半身跟踪卡输出六轴角度的话题，默认即本 bundle 内的 "
                            f"{self._default_angles_topic}；仅在手动接到别的源时需要指定"
                        ),
                    },
                    "confirm": {
                        "type": "boolean",
                        "description": (
                            "start 必填且必须为字面 true，确认授权真实驱动机器人；"
                            "stop/info 不需要"
                        ),
                    },
                },
                "required": ["action"],
                "x-is-dangerous": True,
                # An interrupt must stop this card too: a running 21-motor loop
                # is exactly what on_interrupt_motion has to silence.
                "x-hooks": {"on_interrupt_motion": {"action": "stop"},
                            "on_interrupt_all": {"action": "stop"}},
                "x-action-params": {
                    "start": {
                        "params": ["angles_topic", "confirm"],
                        "description": "初始化低层控制器并开始按限速下发手臂指令；机器人会被真实驱动，需 confirm=true",
                    },
                    "stop": {
                        "params": [],
                        "description": "停止下发并销毁订阅，手臂停在当前位置",
                    },
                    "info": {
                        "params": [],
                        "description": "当前目标角度、实测角度、是否失联与 workmode",
                    },
                },
                # set_joint takes exactly 21 commands, so this card commands the
                # whole robot: the six observable arm joints follow the operator
                # and the other fifteen — both legs and the waist included —
                # hold their measured pose. Declaring only the arm channels
                # would let a correctly-declared leg/waist action run alongside
                # this all-motor takeover. Use the canonical names from
                # README_dev.md's resource table.
                "x-resource": ["base", "arm_l", "arm_r", "leg", "waist"],
            },
            "topic_in": [{
                "topic": self._default_angles_topic,
                "format": "data/json",
                "message_type": "std_msgs/msg/String",
            }],
            # An actuator that writes motors has nothing to render: the tracking
            # card already publishes both the keypoints and the angles.
            "topic_out": [],
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle hook. Deliberately does not command the motors.

        Starting the whole bundle must never move a robot; the operator or the
        canvas calls `dispatch("start")` to arm this card.
        """
        return {"state": "ready"}

    def stop(self):
        stopped = self._teardown()
        return {"state": "idle" if stopped else "stopping"}

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self._start(args or {})
        if action == "stop":
            return self._stop()
        if action == "info":
            return self._info()
        return None

    # ── arbitration with the call-shaped and stream-shaped chassis cards ────

    def is_driving(self) -> bool:
        """Whether the 21-motor loop is really writing (for other cards)."""
        worker = self._worker
        return bool(worker is not None and worker.is_alive()
                    and not self._stop_event.is_set())

    def pause_for_explicit_command(self, reason: str = "") -> bool:
        """Give up the motors to an explicit loco/posture action.

        The same protocol `loco_servo` implements: a person's or the LLM's
        action outranks this loop. Return True only if a live loop was stopped,
        so callers can report the preemption. Unlike a servo pause there is no
        resume — the operator must confirm `start` again before this dangerous
        card may write all 21 motors.
        """
        if not self.is_driving():
            return False
        self._last_error = f"preempted by explicit command: {reason}" if reason \
            else "preempted by explicit command"
        # _teardown reports whether the worker really exited. Do not claim idle
        # while a mid-tick send may still complete after a join timeout.
        self._teardown()
        return True

    def _chassis_conflict(self) -> str:
        """Why this card must not own the motors right now, or "".

        `x-resource` is scheduling metadata, not a lock: a `loco` motion or a
        `loco_servo` stream can already be publishing through `HighController`
        when this card arms. Both write the same robot; only one may own it.
        """
        if self._loco is not None and getattr(self._loco, "is_moving", None):
            if self._loco.is_moving():
                return ("loco 卡片正在驱动底盘（move 未 stop_move）——"
                        "请先调用 loco 的 stop_move 再启动 upper_body_mimic。")
        if self._servo is not None and getattr(self._servo, "is_running", None):
            if self._servo.is_running():
                return ("loco_servo 正在发布底盘速度流 —— 请先 stop 该卡片"
                        "再启动 upper_body_mimic。")
        return ""

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict) -> dict:
        # This is a dangerous action: starting takes over all 21 motors, so a
        # bare `start` from a user or an agent must never arm the loop. Mirror
        # the driver contract's `confirm is true` rule used by other dangerous
        # cards; `1`, `"true"` and truthy strings are not an explicit boolean.
        if args.get("confirm") is not True:
            return {"state": "error",
                    "error": ("start requires confirm=true: this card takes "
                              "over all 21 motors and really drives the robot")}

        if self._high_ctrl is None:
            return {"state": "error",
                    "error": "no robot controller: the SDK did not initialise"}

        if not spec.URDF_LIMITS_LOADED:
            return {"state": "error",
                    "error": f"URDF limits unavailable: {spec.URDF_LIMITS_ERROR}"}
        if not self._dds_verified:
            return {"state": "error",
                    "error": ("DDS contention not verified: confirm on the real "
                              "robot that LowController and HighController do "
                              "not fight, then set dds_contention_verified: true")}

        # A chassis writer may already be publishing. x-resource only orders
        # Agent Core's own actions; it cannot stop a topic-driven loop.
        conflict = self._chassis_conflict()
        if conflict:
            return {"state": "error", "error": conflict}

        mode = self._read_workmode()
        if mode is None:
            return {"state": "error", "error": "could not read workmode"}
        if mode != _WALKING_WORKMODE:
            name = {_PROTECTION_WORKMODE: "protection",
                    _DISABLED_WORKMODE: "disabled"}.get(mode, "not walking")
            return {"state": "error", "workmode": mode,
                    "error": f"robot is {name}; only workmode=2 may be driven"}

        error = self._ensure_low_level()
        if error:
            return {"state": "error", "error": error}

        try:
            measured = self._measured_rad()
        except Exception as exc:  # noqa: BLE001
            return {"state": "error", "error": f"could not read joint state: {exc}"}

        topic = str(args.get("angles_topic") or self._default_angles_topic).strip()
        if not topic:
            return {"state": "error", "error": "angles_topic must not be empty"}

        self._teardown()

        # Start from the pose the arm is already in. Without this the first
        # command would be the rate limiter's first step away from zero, which
        # is a jump the operator never asked for.
        self._commanded_deg = spec.measured_arm_deg(measured)
        self._velocity_deg_s = {key: 0.0 for key in spec.ZERO_TARGETS_DEG}
        self._angles_topic = topic
        self._ticks = 0
        self._last_error = None
        self._lost = True
        self._ever_tracked = False
        self._last_workmode = mode
        with self._lock:
            self._latest_targets = dict(spec.ZERO_TARGETS_DEG)
            self._latest_tracked = False
            self._latest_at = 0.0

        self._angles_sub = self._node.create_subscription(
            String, topic, self._on_angles, _LATEST_QOS)

        self._stop_event = threading.Event()
        token = object()
        self._run_token = token
        self._worker = threading.Thread(
            target=self._control_loop, args=(token,),
            daemon=True, name="upper_body_mimic")
        self._worker.start()

        self._status = "running"
        return {
            "state": "ready",
            "angles_topic": topic,
            "workmode": mode,
            "control_hz": self._control_hz,
            "max_velocity_rad_s": self._max_velocity_rad_s,
            "max_acceleration_rad_s2": self._max_acceleration_rad_s2,
            "send_commands": self._send_commands,
            "commanded_deg": {k: round(v, 1) for k, v in self._commanded_deg.items()},
        }

    def _stop(self) -> dict:
        if self._teardown():
            return {"state": "idle"}
        return {"state": "stopping",
                "error": ("control loop is still unwinding inside an SDK call; "
                          "it is fenced and will not command again")}

    def _info(self) -> dict:
        mode = self._read_workmode()
        commanded = None
        if self._commanded_deg is not None:
            commanded = {k: round(float(v), 1) for k, v in self._commanded_deg.items()}
        measured = None
        try:
            measured = {k: round(v, 1) for k, v in
                        spec.measured_arm_deg(self._measured_rad()).items()}
        except Exception:  # noqa: BLE001 - info must never raise
            measured = None
        with self._lock:
            tracked = self._latest_tracked
            last_at = self._latest_at
            ever_tracked = self._ever_tracked
        age_ms = None if not last_at else round((time.monotonic() - last_at) * 1000.0, 1)
        return {
            "state": self._status,
            "topic_in": [{
                "topic": self._angles_topic,
                "format": "data/json",
                "message_type": "std_msgs/msg/String",
            }],
            "commanded_deg": commanded,
            "measured_deg": measured,
            "tracked": tracked,
            "ever_tracked": ever_tracked,
            "lost": self._lost,
            "angles_age_ms": age_ms,
            "workmode": mode,
            "walking_mode_ok": mode == _WALKING_WORKMODE,
            "dds_contention_verified": self._dds_verified,
            "chassis_conflict": self._chassis_conflict(),
            "driving": self.is_driving(),
            "urdf_limits_loaded": spec.URDF_LIMITS_LOADED,
            "ticks": self._ticks,
            "control_hz": self._control_hz,
            "max_velocity_rad_s": self._max_velocity_rad_s,
            "max_acceleration_rad_s2": self._max_acceleration_rad_s2,
            "gains": {
                "keep_kp": self._keep_kp, "keep_kd": self._keep_kd,
                "arm_kp": self._arm_kp, "arm_kd": self._arm_kd,
            },
            "send_commands": self._send_commands,
            "last_error": self._last_error,
        }

    # ── SDK plumbing ─────────────────────────────────────────────────────────

    def _read_workmode(self):
        if self._high_ctrl is None:
            return None
        try:
            return int(self._high_ctrl.get_mode())
        except Exception:  # noqa: BLE001
            return None

    def _measured_rad(self) -> list:
        if self._high_ctrl is None:
            raise RuntimeError("no robot controller")
        states = self._high_ctrl.get_joint_state()
        values = [float(state.pos) for state in states]
        if len(values) != spec.JOINT_COUNT:
            raise RuntimeError(
                f"controller returned {len(values)} joints, expected {spec.JOINT_COUNT}")
        return values

    def _ensure_low_level(self):
        """Make sure `LowController` is initialised, or say why it cannot be."""
        if self._low_ctrl is not None:
            return None
        try:
            LowController, MotorCmd = _load_low_level()
        except Exception as exc:  # noqa: BLE001
            return f"low-level SDK unavailable: {exc}"
        ok, why = _probe_low_level()
        if not ok:
            return why
        try:
            ctrl = LowController.instance()
            ctrl.init()
        except Exception as exc:  # noqa: BLE001
            return f"LowController.init() failed: {exc}"
        self._low_ctrl = ctrl
        self._motor_cmd_cls = MotorCmd
        return None

    def _send(self, commands: list) -> None:
        if not self._send_commands:
            return
        MotorCmd = self._motor_cmd_cls
        payload = []
        for command in commands:
            cmd = MotorCmd()
            cmd.pos = command["pos"]
            cmd.vel = command["vel"]
            cmd.tau = command["tau"]
            cmd.kp = command["kp"]
            cmd.kd = command["kd"]
            cmd.motor_id = command["motor_id"]
            payload.append(cmd)
        self._low_ctrl.set_joint(payload)

    # ── control loop ─────────────────────────────────────────────────────────

    def _on_angles(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:  # noqa: BLE001 - a malformed frame must not kill the loop
            return
        if not isinstance(payload, dict):
            return
        targets = spec.targets_from_payload(payload)
        tracked = bool(payload.get("tracked"))
        with self._lock:
            self._latest_targets = targets
            self._latest_tracked = tracked
            self._latest_at = time.monotonic()
            if tracked:
                self._ever_tracked = True

    def _control_loop(self, token) -> None:
        period = 1.0 / self._control_hz
        next_at = time.monotonic()
        log = self._node.get_logger()

        while not self._stop_event.is_set() and token is self._run_token:
            # Workmode is re-read every tick, not just at start. Leaving 2 — a
            # fall, a motor fault, an operator pressing estop — is a latched
            # intervention: stop writing rather than override the robot.
            mode = self._read_workmode()
            if mode != _WALKING_WORKMODE:
                self._last_error = (
                    f"workmode left 2 (now {mode}); stopped commanding"
                    if mode is not None else "workmode unreadable; stopped commanding")
                self._status = "error"
                self._stop_event.set()
                log.error(f"upper_body_mimic stopped: {self._last_error}")
                break

            # A chassis card can be started after this one. Re-check ownership
            # every tick, not only at start, so a newly running loco stream
            # cannot end up writing HighController while we write LowController.
            conflict = self._chassis_conflict()
            if conflict:
                self._last_error = conflict
                self._status = "error"
                self._stop_event.set()
                log.error(f"upper_body_mimic stopped: {conflict}")
                break

            now = time.monotonic()
            with self._lock:
                targets = dict(self._latest_targets)
                tracked = self._latest_tracked
                last_at = self._latest_at
                ever_tracked = self._ever_tracked

            # Low confidence (`tracked=false`) and a stale stream are the same
            # state: return to the safe pose. Never keep the last human
            # command — an unobserved arm must not stay frozen mid-air.
            #
            # Before the first tracked frame there is nothing to return from:
            # the card may have been armed before the camera was wired, and
            # lowering the arms then is movement nobody asked for. Hold the
            # pose the card started in until mirroring has actually begun.
            lost = (not tracked) or (last_at == 0.0) or \
                   (now - last_at > self._lost_timeout_s)
            if lost and ever_tracked:
                desired = spec.clamp_targets(spec.safe_pose_deg(), self._margin_deg)
            elif lost:
                desired = dict(self._commanded_deg)
            else:
                desired = spec.clamp_targets(targets, self._margin_deg)

            self._commanded_deg, self._velocity_deg_s = spec.accel_limited_targets(
                self._commanded_deg, self._velocity_deg_s, desired, period,
                self._max_velocity_deg_s, self._max_acceleration_deg_s2)

            try:
                measured = self._measured_rad()
                commands = spec.build_joint_commands(
                    measured, self._commanded_deg,
                    keep_kp=self._keep_kp, keep_kd=self._keep_kd,
                    arm_kp=self._arm_kp, arm_kd=self._arm_kd,
                )
                # Fence immediately before the only call that moves the robot.
                # The loop condition alone is not enough: a tick can be blocked
                # in `get_joint_state()` while `stop` invalidates the token and
                # returns, and would then send one more command from the stopped
                # run. Re-checking here makes a stopped run unable to write.
                if self._stop_event.is_set() or token is not self._run_token:
                    break
                self._send(commands)
            except Exception as exc:  # noqa: BLE001 - stop rather than keep writing
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._status = "error"
                self._stop_event.set()
                log.error(f"upper_body_mimic stopped: {self._last_error}")
                break

            self._lost = lost
            self._ticks += 1

            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                # Fell behind: resynchronise instead of accumulating a backlog.
                next_at = time.monotonic()

    def _teardown(self) -> bool:
        """Stop the control loop; True only once the worker has really exited.

        Clearing the token before the join makes a mid-tick loop skip its send.
        If the worker is stuck inside an SDK call the join can time out, and
        the caller must not be told "idle" while that call may still return and
        write: that is why the result is reported instead of assumed.
        """
        # Clear the run token first: a loop that is mid-tick will see it change
        # and exit at the top of its next iteration instead of sending one more
        # command from a run the operator already stopped.
        self._run_token = None
        self._stop_event.set()
        worker = self._worker
        self._worker = None
        stopped = True
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
            stopped = not worker.is_alive()
        if self._angles_sub is not None:
            try:
                self._node.destroy_subscription(self._angles_sub)
            except Exception:  # noqa: BLE001
                pass
            self._angles_sub = None
        self._status = "idle" if stopped else "stopping"
        return stopped
