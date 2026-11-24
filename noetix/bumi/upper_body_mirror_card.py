"""`upper_body_mirror` — drive Bumi's arms from the `upper_body_track` angles.

This is the writing half of upper-body mirroring. The read-only card publishes
six retargeted arm angles; this one turns them into motor commands.

It is not an "arm card", and that distinction matters more than anything else
in this file. `LowController.set_joint` takes exactly 21 commands — it rejects
anything else — so every tick this card sends all 21 motors: fifteen hold the
position they are measured at, six follow the operator. Sending 21 commands
means taking the motors away from whatever was driving them, which is why the
plugin is disabled by default, refuses to start in protection mode, and is
documented as a whole-body takeover rather than an arm feature. The card does
not implement a balance controller: it holds the legs where they already are
and relies on the fact that a standing robot whose legs do not move stays
standing. Large arm swings still shift the centre of mass, so the first runs
belong on a supported robot.

Safety properties, each of which has a test:

- rate limited to `rate_limit_deg_s`, applied to the *commanded* pose
- clamped inside the mechanical stops with a margin
- the first command after `start` is the pose the arm is already in
- operator lost from frame → drift back to zero at the same rate limit
- protection mode, missing controller, and SDK failure all refuse to command
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import upper_body_mirror_spec as spec

# The camera path uses a best-effort, depth-1 subscription so a stalled consumer
# cannot make the publisher queue frames. The angles topic is published by the
# tracking card with the same profile.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

#: `device.py`'s `_WORKMODE_NAMES[26]`. In protection mode the robot's own
#: controller has decided something is wrong; taking the motors anyway would
#: undo the only safety decision the robot made on its own.
_PROTECTION_WORKMODE = 26

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


class UpperBodyMirrorPlugin:
    PREFIX = "upper_body_mirror"

    def __init__(self, plugin_config: dict, namespace: str,
                 executor, high_ctrl=None):
        cfg = plugin_config or {}
        self._namespace = namespace
        self._default_angles_topic = f"/{namespace}/upper_body_track/angles"
        self._angles_topic = self._default_angles_topic
        self._high_ctrl = high_ctrl

        self._control_hz = float(cfg.get("control_hz", 50.0))
        self._rate_limit_deg_s = float(cfg.get("rate_limit_deg_s", 30.0))
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

        self._node = Node("bumi_upper_body_mirror")
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
        self._low_ctrl = None
        self._motor_cmd_cls = None

        #: Whether this run has ever seen a tracked operator. The drift-back
        #: behaviour only makes sense once mirroring has started: a card that
        #: was just started with nobody in frame must leave the arms where they
        #: are, not begin lowering them before it ever knew where "down" was.
        self._ever_tracked = False

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
        cannot meet; a 300 deg/s limit is not "fast", it is the number at which
        the limiter stops protecting anything. Both would present as plausible
        numbers in `info`, which is why they raise instead.
        """
        checks = (
            ("control_hz", self._control_hz, 10.0, 200.0),
            ("rate_limit_deg_s", self._rate_limit_deg_s, 1.0, 180.0),
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
            "name": "upper_body_mirror",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "把人的上半身动作镜像到 Bumi 的手臂上。订阅上半身跟踪卡输出的六个手臂角度，"
                "每帧下发 21 条电机指令：15 个非手臂关节保持在实测位置，6 个手臂关节跟随人。"
                "注意：LowController 只接受 21 条指令，所以这张卡在运行时接管了全部电机（包括腿），"
                "它不做平衡控制，只是让你站着的腿保持不动；手臂大幅摆动仍会改变重心，"
                "首次运行请确保机器人有支撑。人在画面外超过 lost_timeout 后手臂会按限速缓慢回到零位。"
                "默认关闭，需在 config.yaml 打开并在画布上显式 start 才会真正下发指令。"
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
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["angles_topic"],
                        "description": "初始化低层控制器并开始按限速下发手臂指令；机器人会被真实驱动",
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
                "x-resource": ["base", "arm_l", "arm_r"],
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
        self._teardown()
        return {"state": "idle"}

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self._start(args or {})
        if action == "stop":
            return self._stop()
        if action == "info":
            return self._info()
        return None

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict) -> dict:
        if self._high_ctrl is None:
            return {"state": "error",
                    "error": "no robot controller: the SDK did not initialise"}

        mode = self._read_workmode()
        if mode is None:
            return {"state": "error", "error": "could not read workmode"}
        if mode == _PROTECTION_WORKMODE:
            return {"state": "error", "workmode": mode,
                    "error": "robot is in protection mode; refusing to take the motors"}

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
            daemon=True, name="upper_body_mirror")
        self._worker.start()

        self._status = "running"
        return {
            "state": "ready",
            "angles_topic": topic,
            "workmode": mode,
            "control_hz": self._control_hz,
            "rate_limit_deg_s": self._rate_limit_deg_s,
            "send_commands": self._send_commands,
            "commanded_deg": {k: round(v, 1) for k, v in self._commanded_deg.items()},
        }

    def _stop(self) -> dict:
        self._teardown()
        return {"state": "idle"}

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
            "lost": self._lost,
            "angles_age_ms": age_ms,
            "workmode": mode,
            "protection": mode == _PROTECTION_WORKMODE,
            "ticks": self._ticks,
            "control_hz": self._control_hz,
            "rate_limit_deg_s": self._rate_limit_deg_s,
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
        max_step = spec.max_step_for(self._rate_limit_deg_s, self._control_hz)
        next_at = time.monotonic()
        log = self._node.get_logger()

        while not self._stop_event.is_set() and token is self._run_token:
            now = time.monotonic()
            with self._lock:
                targets = dict(self._latest_targets)
                tracked = self._latest_tracked
                last_at = self._latest_at
                ever_tracked = self._ever_tracked

            # Losing the operator is not an error, it is an expected state: the
            # person walked away. Drift back rather than freezing an arm in
            # mid-air where nobody can predict what it will do next.
            #
            # Before the first tracked frame there is nothing to drift back
            # from: the card has just started, the operator may still be walking
            # into frame, and lowering the arms now would be a movement nobody
            # asked for. Hold until mirroring has actually begun.
            lost = (not tracked) or (last_at == 0.0) or \
                   (now - last_at > self._lost_timeout_s)
            if lost and ever_tracked:
                desired = dict(spec.ZERO_TARGETS_DEG)
            elif lost:
                desired = dict(self._commanded_deg)
            else:
                desired = targets

            self._commanded_deg = spec.rate_limited_targets(
                self._commanded_deg, desired, max_step)

            try:
                measured = self._measured_rad()
                commands = spec.build_joint_commands(
                    measured, self._commanded_deg,
                    keep_kp=self._keep_kp, keep_kd=self._keep_kd,
                    arm_kp=self._arm_kp, arm_kd=self._arm_kd,
                    margin=self._margin_deg,
                )
                self._send(commands)
            except Exception as exc:  # noqa: BLE001 - stop rather than keep writing
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._status = "error"
                self._stop_event.set()
                log.error(f"upper_body_mirror stopped: {self._last_error}")
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

    def _teardown(self) -> None:
        # Clear the run token first: a loop that is mid-tick will see it change
        # and exit at the top of its next iteration instead of sending one more
        # command from a run the operator already stopped.
        self._run_token = None
        self._stop_event.set()
        worker = self._worker
        self._worker = None
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        if self._angles_sub is not None:
            try:
                self._node.destroy_subscription(self._angles_sub)
            except Exception:  # noqa: BLE001
                pass
            self._angles_sub = None
        self._status = "idle"
