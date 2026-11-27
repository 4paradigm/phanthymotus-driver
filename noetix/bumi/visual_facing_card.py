"""VisualFacingPlugin — the Bumi `visual_facing` actuator card.

Left link: a vision card's ``data/json`` target pose (ArUco ``tvec``, a stamped
pose, or a tf-shaped translation).  The card turns Bumi **in place** until the
target is centred in the picture; it never translates.

Safety is fail-closed.  Nothing moves until an operator calls ``enable`` with
``confirm=true``, and that authorisation expires after a short window.  Every
control tick re-checks the target's age, workmode (26 = protection, 2 =
walking), the estop topic and chassis ownership; any failure publishes a zero
immediately.  Hysteresis between ``center_tolerance_deg`` and
``exit_tolerance_deg`` stops the robot from hunting around the centre.

The card deliberately does **not** look up a camera mount transform.  The
error is the optical-frame bearing ``atan2(-x, z)``, so an A5 camera-mount
correction changes nothing about the closed loop.  See visual_facing_spec.py.
"""

from __future__ import annotations

import json
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import visual_facing_spec as spec

_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
_MIN_CMD_INTERVAL_S = 0.002       # vendor requirement: publish_cmd >= 2 ms apart
_STATUS_INTERVAL_S = 0.2          # 5 Hz status topic; motion loop runs faster


def _default_cmd():
    """ControlCmd.DEFAULT, imported lazily so the module loads without the SDK."""
    from highcontrol_py import ControlCmd
    return ControlCmd.DEFAULT


class VisualFacingPlugin:
    PREFIX = "visual_facing"

    def __init__(self, plugin_config, namespace, executor, high_ctrl,
                 loco_plugin=None, loco_servo_plugin=None):
        cfg = spec.validate_config(plugin_config)
        self._cfg = cfg
        self._namespace = namespace
        self._executor = executor
        self._ctrl = high_ctrl
        self._loco = loco_plugin
        self._servo = loco_servo_plugin

        self._status_topic = f"/{namespace}/visual_facing/status"
        self._estop_topic = cfg["estop_topic"] or f"/{namespace}/state/estop"
        self._node = Node("bumi_visual_facing")
        executor.add_node(self._node)
        self._status_pub = self._node.create_publisher(String, self._status_topic, 10)

        self._estop_state = "unknown"
        self._estop_at = 0.0
        self._estop_sub = self._node.create_subscription(
            String, self._estop_topic, self._on_estop, _LATEST_QOS)

        self._target_sub = None
        self._running = False
        self._worker = None
        self._generation = 0
        self._stop_event = threading.Event()
        self._run_token = None
        self._cmd_lock = threading.Lock()
        self._last_cmd_at = 0.0
        self._frame_lock = threading.Lock()
        self._latest_raw = None
        self._seq = 0
        self._converted_seq = -1
        self._last_target = None
        self._target_error = "no_fresh_target"

        self._deadline_ms = None
        self._auth_reason = "never_armed"
        self._turning = False
        self._turn_started = 0.0
        self._settle_count = 0
        self._duty_acc = 0.0
        self._last_z = 0.0
        self._last_status_publish = 0.0
        self._last_status = {}

        self._input_topic = ""
        self._received = 0
        self._turns = 0
        self._stops = 0
        self._last_error = ""
        self._workmode = -1
        self._state = "idle"
        self._stop = "not_started"

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "visual_facing",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "让 Bumi 看向视觉目标：消费一路视觉定位 data/json（ArUco tvec、"
                "带时间戳的位姿或 tf 变换），只发 publish_cmd(0,0,z) 小步原地转身，"
                "直到目标进入画面中心容差内；不平移、不发 /tf、不需要相机安装外参。"
                "误差取光学系 atan2(-x, z)，滞回避免来回抖动。"
                "start 只订阅，不会让机器人动；必须再调 enable(confirm=true) "
                "获得短时授权才会转身，enable 可续期，disable/stop 立即停。"
                "目标丢失或过期、授权过期、workmode!=2、保护模式(26)、estop 锁存、"
                "底盘被其它卡占用时立即归零停止。首次真机须核对 yaw_sign。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "enable", "disable", "info"],
                               "description": "卡片生命周期、显式授权或状态查询"},
                    "input_topic": {"type": "string",
                                    "description": "视觉定位卡输出的 data/json 话题"},
                    "confirm": {"type": "boolean",
                                "description": "enable 必填 true，确认机器人周边无人、可原地转身"},
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"],
                              "description": "订阅视觉位置话题；此时机器人仍静止"},
                    "stop": {"params": [], "description": "停止订阅并立即归零，同时撤销授权"},
                    "enable": {"params": ["confirm"],
                               "description": "显式授权短时转身窗口；重复调用可续期"},
                    "disable": {"params": [], "description": "立即撤销授权并归零；不解除急停"},
                    "info": {"params": [], "description": "返回当前偏航误差、授权剩余、停止原因与诊断计数"},
                },
                # Interrupt hooks must not leave a live velocity on the chassis.
                "x-hooks": {"on_interrupt_motion": {"action": "stop"},
                            "on_interrupt_all": {"action": "stop"}},
                "x-is-dangerous": True,
                "x-resource": ["base"],
            },
            "topic_in": [{"format": "data/json",
                          "desc": "视觉目标位姿：tvec/position/translation，支持单目标或 transforms/targets 列表"}],
            "topic_out": [{"topic": self._status_topic, "format": "data/json"}],
        }

    # ── Bundle lifecycle ─────────────────────────────────────────────────────

    def start(self):
        """Inert. A container restart must never arm a turning robot."""

    def stop(self):
        self._teardown()

    def dispatch(self, action, args):
        args.pop("_tool_name", None)
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "enable":
            return self._enable(args)
        if action == "disable":
            return self._disable()
        if action == "info":
            return self._info()
        return None

    # ── Actions ──────────────────────────────────────────────────────────────

    def _start(self, args):
        topic = str(args.get("input_topic") or "").strip()
        if not topic:
            return {"state": "error", "error": "缺少 input_topic，请把视觉定位卡连到本卡"}
        if self._ctrl is None:
            return {"state": "error", "error": "HighController 不可用，无法驱动底盘"}
        busy = self._chassis_busy()
        if busy:
            return {"state": "error", "error": busy}

        self._teardown(publish_zero=False)
        self._input_topic = topic
        self._stop_event = threading.Event()
        self._last_error = ""
        self._received = 0
        self._last_target = None
        self._target_error = "no_fresh_target"
        self._converted_seq = -1

        def _on_target(msg):
            with self._frame_lock:
                self._latest_raw = msg.data
                self._seq += 1

        self._target_sub = self._node.create_subscription(
            String, topic, _on_target, _LATEST_QOS)
        self._generation += 1
        token = object()
        self._run_token = token
        self._worker = threading.Thread(
            target=self._worker_loop, args=(self._generation, token),
            daemon=True, name="bumi_visual_facing")
        self._worker.start()
        self._running = True
        self._state = "ready"
        self._stop = "waiting_for_enable"
        return {"state": "running", "input_topic": topic,
                "topic_in": [{"topic": topic, "format": "data/json"}],
                "note": "start 不会移动机器人；请再调用 enable(confirm=true)"}

    def _stop(self):
        self._teardown()
        self._state = "idle"
        self._stop = "stopped"
        return {"state": "idle"}

    def _enable(self, args):
        try:
            deadline = spec.arm_authorization(
                time.time() * 1000.0, self._cfg["auth_window_s"],
                args.get("confirm"))
        except ValueError as exc:
            return {"state": "error", "error": str(exc)}
        self._deadline_ms = deadline
        self._auth_reason = ""
        return {"state": "armed",
                "authorization_remaining_ms": int(
                    spec.authorization_remaining_ms(time.time() * 1000.0, deadline))}

    def _disable(self):
        self._deadline_ms = None
        self._auth_reason = "authorization_disabled"
        self._halt_motion("authorization_disabled")
        return {"state": "disarmed"}

    def _info(self):
        topic_in = [{"format": "data/json"}]
        if self._input_topic:
            topic_in[0]["topic"] = self._input_topic
        status = dict(self._last_status) or spec.status_payload(
            state=self._state, now_ms=time.time() * 1000.0,
            estop=self._estop_state, stop=self._stop)
        status.update({
            "state": self._state if self._running else "idle",
            "stop_reason": self._stop,
            "topic_in": topic_in,
            "topic_out": [{"topic": self._status_topic, "format": "data/json"}],
            "target_id": self._cfg["target_id"],
            "yaw_sign": self._cfg["yaw_sign"],
            "center_tolerance_deg": self._cfg["center_tolerance_deg"],
            "exit_tolerance_deg": self._cfg["exit_tolerance_deg"],
            "auth_window_s": self._cfg["auth_window_s"],
            "estop_topic": self._estop_topic,
            "require_estop": self._cfg["require_estop"],
            "running": self._running,
            "received": self._received,
            "turns": self._turns,
            "stops": self._stops,
            "last_error": self._last_error,
        })
        return status

    # ── Estop / chassis ownership ────────────────────────────────────────────

    def _on_estop(self, msg):
        try:
            obj = json.loads(msg.data)
            latched = bool(obj.get("latched", False))
            self._estop_state = "latched" if latched else "clear"
        except (ValueError, TypeError, AttributeError):
            # An unreadable estop frame is not evidence of a clear estop.
            self._estop_state = "unknown"
        self._estop_at = time.time()

    def _chassis_busy(self):
        if self._loco is not None and getattr(self._loco, "is_moving", None):
            try:
                if self._loco.is_moving():
                    return "loco 正在驱动底盘，请先 stop_move"
            except Exception:  # noqa: BLE001 - a broken peer must fail closed
                return "无法确认 loco 底盘状态"
        if self._servo is not None:
            try:
                if getattr(self._servo, "is_streaming", None) and self._servo.is_streaming():
                    return "loco_servo 正在输出速度，请先 pause"
            except Exception:  # noqa: BLE001
                return "无法确认 loco_servo 底盘状态"
        return ""

    # ── Publishing ───────────────────────────────────────────────────────────

    def _publish_z(self, z):
        """Rate-limited publish_cmd(0, 0, z); x/y are always exactly zero."""
        with self._cmd_lock:
            now = time.monotonic()
            wait = _MIN_CMD_INTERVAL_S - (now - self._last_cmd_at)
            if wait > 0.0:
                time.sleep(wait)
            try:
                self._ctrl.publish_cmd(0.0, 0.0, float(z), _default_cmd(), 0)
            except Exception as exc:  # noqa: BLE001
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._last_z = 0.0
                return False
            self._last_cmd_at = time.monotonic()
            self._last_z = float(z)
            return True

    def _halt_motion(self, reason):
        was_moving = self._last_z != 0.0
        if was_moving:
            self._publish_z(0.0)
        changed = self._stop != reason
        self._turning = False
        self._settle_count = 0
        self._duty_acc = 0.0
        self._stop = reason
        # Count transitions, not every silent tick with no target.
        if reason and (was_moving or changed):
            self._stops += 1

    def _teardown(self, publish_zero=True):
        self._run_token = None
        self._stop_event.set()
        worker = self._worker
        self._worker = None
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        for name in ("_target_sub",):
            sub = getattr(self, name, None)
            if sub is not None:
                try:
                    self._node.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, name, None)
        with self._frame_lock:
            self._latest_raw = None
        self._last_target = None
        self._running = False
        self._deadline_ms = None
        self._auth_reason = "never_armed"
        if publish_zero:
            self._halt_motion("stopped")

    # ── Control loop ─────────────────────────────────────────────────────────

    def _read_target(self, now_ms):
        """Return (error, point, id, age_ms); only decode a message once."""
        with self._frame_lock:
            raw = self._latest_raw
            seq = self._seq
        if raw is not None and seq != self._converted_seq:
            self._converted_seq = seq
            self._received += 1
            try:
                selected = spec.select_target(json.loads(raw), now_ms, self._cfg)
            except (ValueError, TypeError) as exc:
                self._last_error = f"{type(exc).__name__}: {exc}"
                selected = {"error": "malformed_target"}
            if selected.get("error"):
                self._last_target = None
                return selected["error"], None, "", -1.0
            self._last_target = selected
            return "", selected["point"], selected["target_id"], selected["age_ms"]

        target = self._last_target
        if target is None:
            return "no_fresh_target", None, "", -1.0
        age = now_ms - target["stamp_ms"]
        if age > self._cfg["target_ttl_ms"] or age < -self._cfg["max_future_ms"]:
            self._last_target = None
            return "target_expired", None, "", age
        return "", target["point"], target["target_id"], max(0.0, age)

    def _tick(self, run_token):
        # A superseded worker must not publish a command or a status sample,
        # even if its bounded join lost a race with stop->start.
        if run_token is not self._run_token:
            return
        now = time.time()
        now_ms = now * 1000.0
        target_error, point, target_id, age_ms = self._read_target(now_ms)

        yaw_err = 0.0
        if target_error and point is not None:  # defensive: spec always pairs these
            target_error, point = "malformed_target", None
        if point is not None:
            try:
                yaw_err = spec.yaw_error_deg(point, self._cfg)
            except ValueError as exc:
                self._last_error = str(exc)
                target_error, point = "target_behind", None

        mode = -1
        try:
            mode = int(self._ctrl.get_mode())
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"get_mode failed: {exc}"
        self._workmode = mode
        estop = self._estop_state
        if estop != "unknown" and (now - self._estop_at) > self._cfg["estop_ttl_ms"] / 1000.0:
            estop = "unknown"

        remaining = spec.authorization_remaining_ms(now_ms, self._deadline_ms)
        if self._deadline_ms is None:
            auth_reason = self._auth_reason
        elif remaining <= 0.0:
            auth_reason = "authorization_expired"
            self._deadline_ms = None
            self._auth_reason = auth_reason
        else:
            auth_reason = ""

        turn_elapsed = (now - self._turn_started) if self._turning else 0.0
        reason = spec.stop_reason(
            mode, estop, auth_reason, target_error,
            chassis_busy=bool(self._chassis_busy()),
            require_estop=self._cfg["require_estop"],
            turn_elapsed_s=turn_elapsed, cfg=self._cfg)

        if reason:
            self._halt_motion(reason)
            if reason == "turn_timeout":
                # A timed-out turn needs a fresh enable; otherwise the next
                # tick would immediately start another bounded turn.
                self._deadline_ms = None
                self._auth_reason = "turn_timeout"
                remaining = 0.0
            self._state = "stopped" if self._running else "idle"
            z_cmd = 0.0
            control_mode = "hold"
        else:
            ctl = spec.control_for_error(yaw_err, self._turning, self._cfg)
            control_mode = ctl["mode"]
            if ctl["mode"] == "settle":
                z_cmd = 0.0
                self._settle_count += 1
                if self._settle_count >= self._cfg["settle_ticks"]:
                    self._halt_motion("")
                    self._stop = "centered"
                    self._state = "centered"
                else:
                    self._state = "centering"
            elif ctl["mode"] == "hold":
                z_cmd = 0.0
                self._halt_motion("")
                self._stop = "centered"
                self._state = "centered"
            else:
                z_cmd, self._duty_acc = spec.pulse_velocity(
                    ctl["z"], ctl["duty"], self._duty_acc)
                if not self._turning:
                    self._turning = True
                    self._turn_started = now
                    self._turns += 1
                    self._settle_count = 0
                self._stop = ""
                self._state = "turning"
                if not self._publish_z(z_cmd):
                    self._halt_motion("command_failed")
                    z_cmd = 0.0
                    self._state = "stopped"

        # A zero edge must be published exactly once when motion stops, or the
        # chassis keeps the last nonzero velocity.
        if z_cmd == 0.0 and self._last_z != 0.0:
            if not self._publish_z(0.0):
                self._last_z = 0.0

        status = spec.status_payload(
            state=self._state, now_ms=now_ms,
            target_id=target_id, target_age_ms=age_ms,
            yaw_error_deg_value=yaw_err, z_cmd=self._last_z,
            mode=control_mode, authorized=bool(remaining > 0.0),
            auth_remaining_ms=remaining, workmode=mode,
            estop=estop, stop=self._stop, received=self._received,
            turns=self._turns, last_error=self._last_error)
        changed = (self._last_status.get("state") != status["state"]
                   or self._last_status.get("stop_reason") != status["stop_reason"])
        self._last_status = status
        # State changes (especially a safety stop) are published at once; the
        # periodic refresh only carries slowly changing measurements.
        if run_token is self._run_token and (
                changed or (now - self._last_status_publish) >= _STATUS_INTERVAL_S):
            self._last_status_publish = now
            msg = String()
            msg.data = json.dumps(status, ensure_ascii=False)
            self._status_pub.publish(msg)

    def _worker_loop(self, generation, run_token):
        period = 1.0 / self._cfg["control_hz"]
        while (not self._stop_event.is_set()
               and generation == self._generation
               and run_token is self._run_token):
            try:
                self._tick(run_token)
            except Exception as exc:  # noqa: BLE001 - keep the worker alive
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._halt_motion("internal_error")
            self._stop_event.wait(period)
