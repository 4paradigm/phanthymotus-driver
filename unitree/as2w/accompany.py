"""As2W ISS 3.0 accompany-mode supervisor card.

Unitree documents accompany mode (ISS 3.0) as a firmware feature driven by the
physical accompany remote, not by a service call. The ``As2-W`` manual, page 20,
lists the whole control surface:

* enter: wear the remote on the right hip, joystick face to the right, antenna
  vertical, keep the dog within 2.4 m, then **short-press the M key**;
* exit: press **P**, deflect any joystick, tilt the remote more than 40 degrees,
  press any key on the second (two-hand) remote, or hold the power key 2 s.

No such call exists in the vendor interface. ``unitree_sdk2py/as2`` registers 21
sport API ids (1001-1041) and none of them enters or leaves accompany mode; the
only ``*FOLLOW*`` symbol anywhere in the SDK belongs to the B2 model
(``ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW``, a path-following action) and the AS2
client does not register it. The manual's FAQ 25 gives the remaining control
surface, the Unitree Go App toggle, and FAQ 4 adds that the accompany remote and
the App cannot drive the robot at the same time.

So this card never pretends to switch the mode. It does what the interfaces that
do exist can support:

* ``preflight`` — check each precondition the manual requires before an operator
  presses M, marking the ones that only a human can confirm;
* ``status`` — report the live remote state and the last operator input, which is
  the evidence that matters because a joystick deflection *is* an exit trigger;
* ``guide`` — publish the enter/exit procedure and the safety envelope so the
  canvas can show it before anyone touches the remote.

One hardware fact is worth stating plainly because it bounds every inference
here: the 24-byte wireless-remote frame carries LT/RT/start/LB/RB, the four D-pad
directions and A/B/X/Y only (see ``_parse_wireless_remote`` in ``device.py``). The
M and P keys are not in that bitmask, so this card can never observe the enter
key or the P exit key. Joystick deflection and the shared buttons are the only
operator inputs it can see, and those are the ones it reports.
"""

from __future__ import annotations

import threading
import time

from std_msgs.msg import String

try:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    _LOW_LAT_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
except ImportError:
    # Unit tests load the card contract without a ROS installation.
    _LOW_LAT_QOS = None


# The remote is considered fresh for the same window device.py uses.
_REMOTE_FRESH_S = 0.5

# An operator input only counts as exit evidence while it is recent and was
# seen *after* the mode was confirmed as accompanying. Without both bounds a
# joystick nudge made before entering accompany mode stays stored forever and
# becomes a false remote-takeover alert as soon as fsm_name later matches.
_INPUT_EVIDENCE_TTL_S = 10.0

# Keep the default deadzone aligned with _REMOTE_DEADZONE in device.py, where
# float32(0.1) is slightly above Python's 0.1 and needs the same tolerance.
_DEFAULT_DEADZONE = 0.1
_DEADZONE_EPS = 1e-6

# Manual page 20: "禁止快速奔跑，需保持在 2m/s 以内匀速前行".
MAX_RUNNING_SPEED_MPS = 2.0
# Manual page 20: "遥控器与机器狗头顶的距离小于 2.4 米".
MAX_ACCOMPANY_DISTANCE_M = 2.4
# Manual page 21: best height band above the dog's head, and the best straight
# line distance between remote and dog head.
BEST_HEIGHT_M = (0.2, 0.5)
BEST_DISTANCE_M = 0.8
# Manual page 20: tilt beyond 40 degrees compared with the start orientation.
MAX_TILT_DEG = 40.0

# Manual page 14: "紫色 常亮 伴随模式开启".
INDICATOR_PURPLE = {
    "color": "purple",
    "pattern": "solid",
    "meaning": "伴随模式开启",
}

# Postures from which accompany mode cannot start. Mirrors the down states that
# device.py already reasons about, plus the passive/damping pair.
_NOT_STANDING = {
    "PASSIVE", "DAMPING", "AI_DAMPING", "STAND_DOWN", "AI_STAND_DOWN",
    "LYING", "FALL", "FALLEN", "AI_FALL", "AI_FALLEN", "SQUAT",
}

EXIT_TRIGGERS = (
    ("p_key", "按下伴随遥控器 P 键"),
    ("joystick", "拨动任一摇杆"),
    ("tilt", "水平放置遥控器，或对比启动状态倾斜角度大于 40°"),
    ("second_remote", "按下双手遥控器任意按键或拨动其摇杆"),
    ("power_off", "长按 2 秒伴随遥控器电源键，关闭遥控器"),
)

ENTER_PROCEDURE = (
    "1. 确认伴随遥控器已开机：短按电源键，指示灯稳定亮起，机器狗发出语音提示。",
    "2. 将遥控器外扣于人体右侧腰带，站在机器狗正右侧，躯干与机器狗朝向一致。",
    "3. 摇杆所在面朝向右侧、正对机器狗，天线保持竖直向上；严禁抓握天线部位。",
    "4. 确认机器狗位于遥控器右侧，且遥控器与狗头距离小于 2.4 米。",
    "5. 短按 M 功能键，机器狗进入伴随模式并发出语音提示；头部指示灯转为紫色常亮。",
)

CAUTIONS = (
    "禁止在室内狭窄空间、金属反射面密集的场地使用。",
    "确保伴随模式开始及退出前，按键方向朝右侧、天线方向竖直向上。",
    "伴随模式运行时机器狗与遥控器间距不得超过 2.4 米，过远易引起信号中断、设备失控。",
    "伴随模式运行时禁止快速奔跑，需保持在 2 m/s 以内匀速前行。",
    "伴随遥控器与 App 无法同时控制机器人；使用 App 前必须先退出伴随模式。",
    "请勿在台阶或陡坡等特殊路段使用，容易侧翻、摔倒并导致机器狗损坏。",
    "确保遥控器跟随佩戴者躯干移动，禁止握持随意挥舞。",
)


def _unavailable_remote():
    return {"available": False, "fresh": False, "buttons": {}, "axes": {},
            "active": False}


def _peak_axis(axes):
    values = []
    for value in (axes or {}).values():
        if isinstance(value, (int, float)):
            values.append(abs(float(value)))
    return max(values) if values else 0.0


def _is_post_entry_evidence(last_input, entered_ms, now_ms, ttl_s=_INPUT_EVIDENCE_TTL_S):
    """True only for a recent operator input seen after the mode was confirmed.

    Both bounds matter. Without the ``entered_ms`` bound a stick nudge from
    before the operator pressed M is reported forever as a takeover; without
    the TTL an old input during a long accompany run still looks like a fresh
    exit trigger. Either way the alert becomes noise and gets ignored.
    """
    if not last_input or entered_ms is None:
        return False
    at_ms = last_input.get("at_ms")
    if not isinstance(at_ms, (int, float)):
        return False
    return at_ms >= entered_ms and (now_ms - at_ms) <= ttl_s * 1000


class AccompanyPlugin:
    """Read-only supervisor for the firmware-owned ISS 3.0 accompany mode."""

    PREFIX = "accompany"

    def __init__(self, config, namespace, executor, proxy=None, state_plugin=None):
        self._namespace = namespace
        self._topic = f"/{namespace}/state/accompany"
        self._executor = executor
        self._proxy = proxy
        self._state_plugin = state_plugin
        self._config = config or {}
        self._hz = max(1.0, float(self._config.get("publish_hz", 5.0)))
        self._deadzone = float(self._config.get("joystick_deadzone", _DEFAULT_DEADZONE))
        self._fsm_names = {
            str(name).strip().upper()
            for name in (self._config.get("fsm_names") or []) if str(name).strip()
        }
        self._poll_state = bool(self._config.get("poll_state", True))
        self._fsm_period = max(1.0, float(self._config.get("state_poll_period_s", 3.0)))

        self._lock = threading.RLock()
        self._fsm_name = None
        self._fsm_at_ms = None
        self._last_remote_ms = None
        self._last_operator_input = None
        self._accompany_entered_ms = None
        self._quiet_since_ms = None
        self._pub = None
        self._thread = None
        self._closing = threading.Event()
        self._last_error = ""

    # ------------------------------------------------------------------ node

    def _publisher(self):
        if self._pub is not None:
            return self._pub
        if _LOW_LAT_QOS is None:
            return None
        try:
            from rclpy.node import Node
            node = Node("as2w_accompany")
            self._pub = node.create_publisher(String, self._topic, _LOW_LAT_QOS)
        except Exception as exc:
            self._last_error = f"publisher unavailable: {exc}"
            self._pub = None
        return self._pub

    def _publish(self, payload):
        publisher = self._publisher()
        if publisher is None:
            return
        try:
            message = String()
            message.data = _dumps(payload)
            publisher.publish(message)
        except Exception as exc:
            self._last_error = f"publish failed: {exc}"

    # --------------------------------------------------------------- sources

    def _remote_snapshot(self):
        state = self._state_plugin
        if state is None:
            return _unavailable_remote()
        try:
            snapshot = state.remote_snapshot()
        except Exception:
            return _unavailable_remote()
        if not isinstance(snapshot, dict):
            return _unavailable_remote()
        return snapshot

    def _poll_fsm(self):
        proxy = self._proxy
        if proxy is None:
            return
        try:
            # Prefer the isolated, short-deadline channel. A plain proxy that
            # only exposes GetState (test doubles, older hosts) still works,
            # but a real RpcProxy never routes an observer through the motion
            # channel that StopMove depends on.
            observer = getattr(proxy, "observe_state", None)
            result = observer() if callable(observer) else proxy.GetState()
        except Exception as exc:
            self._last_error = f"GetState failed: {exc}"
            return
        if not isinstance(result, tuple) or len(result) != 2:
            return
        code, state = result
        if code != 0 or not isinstance(state, dict):
            return
        name = str(state.get("fsm_name", "")).strip().upper()
        now_ms = int(time.time() * 1000)
        with self._lock:
            in_accompany = bool(name and name in self._fsm_names)
            if in_accompany and self._accompany_entered_ms is None:
                # Anchor the window at the first confirmed transition into
                # the mode. Inputs seen before this instant are pre-entry and
                # must never be reported as exit evidence.
                self._accompany_entered_ms = now_ms
            elif not in_accompany and name:
                self._accompany_entered_ms = None
            self._fsm_name = name or None
            self._fsm_at_ms = now_ms

    # --------------------------------------------------------------- worker

    def _loop(self):
        interval = 1.0 / self._hz
        next_poll = 0.0
        while not self._closing.is_set():
            try:
                self._observe(self._remote_snapshot())
                if self._poll_state:
                    now = time.monotonic()
                    if now >= next_poll:
                        next_poll = now + self._fsm_period
                        self._poll_fsm()
                self._publish(self._evaluate())
            except Exception as exc:
                self._last_error = f"supervisor loop: {exc}"
            self._closing.wait(interval)

    def _observe(self, snapshot):
        available = bool(snapshot.get("available"))
        fresh = bool(snapshot.get("fresh"))
        if not (available and fresh):
            return
        axes = snapshot.get("axes") or {}
        peak = _peak_axis(axes)
        active = bool(snapshot.get("active")) or peak > self._deadzone + _DEADZONE_EPS
        now_ms = int(time.time() * 1000)
        with self._lock:
            self._last_remote_ms = now_ms
            if not active:
                if self._quiet_since_ms is None:
                    self._quiet_since_ms = now_ms
                return
            quiet_for_ms = (now_ms - self._quiet_since_ms
                            if self._quiet_since_ms else 0)
            self._quiet_since_ms = None
            self._last_operator_input = {
                "at_ms": now_ms,
                "peak_axis": round(peak, 4),
                "quiet_for_ms": max(0, quiet_for_ms),
                "buttons": sorted(
                    name for name, pressed in (snapshot.get("buttons") or {}).items()
                    if pressed),
            }

    # -------------------------------------------------------------- contract

    def topic_out(self):
        """The sensor stream this card publishes.

        Agent Core learns sensor streams from ``tools/list``, so this must be
        declared on the tool definition and not only returned by ``info``;
        otherwise the dashboard never attaches the ``data/json`` renderer to
        the stream the worker keeps publishing.
        """
        return [{"topic": self._topic, "format": "data/json"}]

    def get_tool(self):
        description = (
            "As2W ISS 3.0 accompany mode (伴随模式) supervisor — read-only. "
            "The vendor exposes no enter/exit call: the AS2 sport service registers "
            "21 API ids and none of them touches accompany mode, so the mode can "
            "only be started by a short press on the M key of the physical "
            "accompany remote and can only be stopped by the P key, a joystick "
            "deflection, tilting the remote past 40 degrees, pressing any key on "
            "the two-hand remote, or holding the power key for 2 seconds. "
            "Preconditions before pressing M: the accompany remote MUST be powered "
            "on and connected (short press its power key until the indicator is "
            "steady and the dog speaks), it must be worn on the RIGHT hip with the "
            "joystick face pointing right and the antenna vertical, the dog must be "
            "within 2.4 m of the remote (best 0.2-0.5 m above the dog's head, about "
            "0.8 m in a straight line). Leaving the mode is easy to trigger by "
            "accident: any joystick deflection exits it, so a stray stick input "
            "during following silently drops the robot back to joystick mode. "
            "The accompany remote and the Unitree Go App cannot both control the "
            "robot; exit accompany mode before using the App, and note that App/SDK "
            "control is unavailable while it runs. Safety envelope: stay at or "
            "below 2 m/s, never use it indoors in narrow spaces or near dense "
            "reflective metal, and never on steps or steep slopes. Confirmation is "
            "visual: the head indicator turns solid purple (状态指示: 紫色常亮 = "
            "伴随模式开启). This card cannot read that lamp and cannot read the M or "
            "P keys — the 24-byte remote frame only carries LT/RT/start/LB/RB, the "
            "D-pad and A/B/X/Y. Use preflight before the operator acts, status to "
            "watch for exit evidence, and guide to show the procedure."
        )
        return {"name": "accompany", "type": "sensor", "multiInstance": False,
                "topic_out": self.topic_out(),
                "description": description,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string",
                               "enum": ["info", "preflight", "status", "guide"],
                               "description": "info: capability and topic; preflight: "
                                              "check preconditions; status: live mode "
                                              "and exit evidence; guide: procedure."}},
                    "required": ["action"]},
                "x-action-params": {
                    "info": {"params": [], "description": "Capability boundary and output topic."},
                    "preflight": {"params": [], "description": "Check preconditions before pressing M."},
                    "status": {"params": [], "description": "Current mode inference and exit evidence."},
                    "guide": {"params": [], "description": "Enter/exit procedure and safety envelope."}}}

    def start(self):
        if self._thread is not None:
            return
        self._closing.clear()
        self._publisher()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="as2w_accompany")
        self._thread.start()

    def stop(self):
        self._closing.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)

    # ---------------------------------------------------------------- states

    def _evaluate(self):
        snapshot = self._remote_snapshot()
        available = bool(snapshot.get("available"))
        fresh = bool(snapshot.get("fresh"))
        axes = snapshot.get("axes") or {}
        buttons = snapshot.get("buttons") or {}
        peak = _peak_axis(axes)
        active = bool(snapshot.get("active")) or peak > self._deadzone + _DEADZONE_EPS
        now_ms = int(time.time() * 1000)
        with self._lock:
            fsm_name = self._fsm_name
            fsm_at_ms = self._fsm_at_ms
            last_input = dict(self._last_operator_input or {}) or None
            entered_ms = self._accompany_entered_ms
            remote_ms = self._last_remote_ms

        result = {
            "mode": "unknown",
            "confidence": "none",
            "mode_source": "none",
            "fsm_name": fsm_name,
            "remote": {
                "available": available,
                "fresh": fresh,
                "active": active,
                "peak_axis": round(peak, 4),
                "deadzone": self._deadzone,
                "buttons": {name: bool(pressed)
                            for name, pressed in buttons.items() if pressed},
                "axes": axes,
            },
            "last_operator_input": last_input,
            "exit_evidence": None,
            "observed_at_ms": remote_ms,
            "indicator": dict(INDICATOR_PURPLE),
            "control_owner": "accompany_remote" if available else "unknown",
            "note": "",
        }

        if not available:
            result["note"] = ("伴随遥控器未开机或不在范围内（wireless_remote 帧头不匹配）。"
                              "此时 M 键无法被处理，伴随模式既无法进入也无法观测。"
                              "请短按伴随遥控器电源键，确认指示灯稳定亮起。")
            return result
        if not fresh:
            result["note"] = ("遥控器数据超过 0.5 s 未更新，最后一次有效帧已过期。"
                              "可能是距离过远、信号被遮挡或遥控器已关机；"
                              "伴随模式要求间距不超过 2.4 米。")
            return result

        # A cached name is only evidence while polls keep succeeding. If the
        # DDS link drops, the robot may exit via a key this card cannot read
        # (P key / joystick), so a formerly confirmed name must expire instead
        # of reporting "accompanying" forever. Same three-period bound as
        # _preflight().
        fsm_age_ms = None if fsm_at_ms is None else max(0, now_ms - fsm_at_ms)
        if fsm_age_ms is not None and fsm_age_ms > self._fsm_period * 3000:
            result["confidence"] = "stale"
            result["mode_source"] = "fsm_stale"
            result["note"] = (
                f"最后一次成功 GetState 已过期（{fsm_age_ms / 1000:.0f} s 前，"
                f"fsm_name={fsm_name or '(空)'}），不能据此确认当前模式；"
                "请检查 DDS 链路或等待下一次轮询成功。")
            return result

        if fsm_name and self._fsm_names and fsm_name in self._fsm_names:
            result["mode"] = "accompanying"
            result["confidence"] = "confirmed"
            result["mode_source"] = "fsm"
            result["note"] = f"GetState 返回 fsm_name={fsm_name}，命中已登记的伴随态。"
            if _is_post_entry_evidence(last_input, entered_ms, now_ms):
                result["exit_evidence"] = {
                    "reason": "operator_input_while_accompanying",
                    "at_ms": last_input.get("at_ms"),
                    "peak_axis": last_input.get("peak_axis"),
                    "buttons": last_input.get("buttons") or [],
                    "detail": ("伴随模式运行期间检测到操作员输入。拨动摇杆或按键本身就是"
                               "文档列明的退出条件，请确认头部指示灯是否仍为紫色常亮。"),
                }
            return result

        if fsm_name and self._fsm_names:
            result["mode"] = "not_accompanying"
            result["confidence"] = "confirmed"
            result["mode_source"] = "fsm"
            result["note"] = f"fsm_name={fsm_name} 不在已登记的伴随态名单内。"
            return result

        if not self._fsm_names:
            result["note"] = (
                "未登记伴随态 fsm_name，无法确认模式。请在真机进入伴随模式时调用 GetState，"
                "把返回的 fsm_name 填入 config.yaml 的 accompany.fsm_names，"
                "之后本卡即可给出 confirmed 判定。当前只能给出操作员输入证据；"
                "请以头部指示灯紫色常亮为准。")
        else:
            result["note"] = "尚未取到 GetState 状态，无法判定模式。"
        return result

    def _preflight(self):
        snapshot = self._remote_snapshot()
        available = bool(snapshot.get("available"))
        fresh = bool(snapshot.get("fresh"))
        axes = snapshot.get("axes") or {}
        peak = _peak_axis(axes)
        active = bool(snapshot.get("active")) or peak > self._deadzone + _DEADZONE_EPS
        with self._lock:
            fsm_name = self._fsm_name
            fsm_at_ms = self._fsm_at_ms
            remote_ms = self._last_remote_ms
        stale_fsm = (fsm_at_ms is None or
                     int(time.time() * 1000) - fsm_at_ms > self._fsm_period * 3000)

        checks = [
            {"id": "remote_powered",
             "label": "伴随遥控器已开机并与机器狗连接",
             "ok": available and fresh,
             "evidence": ("wireless_remote 帧有效" if available and fresh
                          else "未收到有效遥控器帧"),
             "manual": "短按伴随遥控器电源键，指示灯稳定亮起后机器狗发出语音提示。"},
            {"id": "remote_idle",
             "label": "遥控器当前未被操作（避免刚进入就触发退出）",
             "ok": (not active) if available and fresh else None,
             "evidence": f"峰值摇杆 {peak:.3f} / 死区 {self._deadzone}",
             "manual": "拨动摇杆会退出伴随模式，进入前请松开摇杆。"},
            {"id": "standing",
             "label": "机器狗已开机并自动站起",
             "ok": (None if (not fsm_name or stale_fsm)
                    else fsm_name not in _NOT_STANDING),
             "evidence": (f"fsm_name={fsm_name}" if fsm_name and not stale_fsm
                          else "未取到状态"),
             "manual": "开机约 1 分钟完成自检后自动起立。"},
            {"id": "wear_position",
             "label": "遥控器戴在右侧腰带上，摇杆面朝右，天线竖直向上",
             "ok": None,
             "evidence": "本卡无姿态源（遥控器帧不含 IMU）",
             "manual": "外扣于人体右侧腰带，站在机器狗正右侧，严禁抓握天线部位。"},
            {"id": "distance",
             "label": "与狗头距离小于 2.4 米（最佳 0.2-0.5 米，直线约 0.8 米）",
             "ok": None,
             "evidence": "本卡无测距源，无法测量遥控器与机器狗的间距",
             "manual": "超出 2.4 米时机器狗会语音提示距离太远，走近后重试。"},
            {"id": "environment",
             "label": "场地开阔，非狭窄室内、非金属反射面密集、无台阶陡坡",
             "ok": None,
             "evidence": "本卡无环境感知结论",
             "manual": "禁止在室内狭窄空间与金属反射面密集场地使用；勿用于台阶或陡坡。"},
            {"id": "control_exclusive",
             "label": "无 App/SDK 正在争夺底盘控制",
             "ok": None,
             "evidence": "当前连接的是伴随遥控器；App 与伴随遥控器不能同时控制",
             "manual": "如刚用过 App，请先退出伴随模式再操作。",
            },
        ]
        blocking = [item["id"] for item in checks if item["ok"] is False]
        pending = [item["id"] for item in checks if item["ok"] is None]
        return {
            "ready": not blocking,
            "blocking": blocking,
            "needs_human_confirmation": pending,
            "checks": checks,
            "observed_at_ms": remote_ms,
            "next_step": ("前置条件已满足。按 ENTER_PROCEDURE 第 4-5 步短按 M 键，"
                          "然后确认头部指示灯转为紫色常亮。" if not blocking
                          else "存在未满足的硬性条件，请先处理 blocking 列表。"),
            "note": ("needs_human_confirmation 中的条件由人工核对，本卡没有任何传感器"
                     "能测量佩戴姿态、间距或场地情况。"
                     "另：本卡读不到 M 键与 P 键，遥控器帧只有 LT/RT/start/LB/RB、"
                     "方向键与 A/B/X/Y。"),
        }

    def _guide(self):
        return {
            "enter": {
                "procedure": list(ENTER_PROCEDURE),
                "key": "M 功能键（短按）",
                "key_observable": False,
                "key_note": "M 键不在遥控器 24 字节帧的位图内，本卡无法观测该按键。",
            },
            "exit": {
                "triggers": [{"id": key, "how": how,
                              "observable": key in ("joystick", "second_remote")}
                             for key, how in EXIT_TRIGGERS],
                "note": ("拨动摇杆即退出，是最容易误触的一条。"
                         "P 键与电源键长按本卡无法观测；"
                         "倾斜可以通过佩戴姿态间接避免，但本卡无姿态源。"),
                "exit_to_app": ("伴随遥控器与 App 无法同时控制机器人；"
                                "要使用 App 操控，请长按 2 秒伴随遥控器电源键退出伴随模式。"),
            },
            "cautions": list(CAUTIONS),
            "safety": {
                "max_speed_mps": MAX_RUNNING_SPEED_MPS,
                "max_distance_m": MAX_ACCOMPANY_DISTANCE_M,
                "best_height_m": list(BEST_HEIGHT_M),
                "best_distance_m": BEST_DISTANCE_M,
                "max_tilt_deg": MAX_TILT_DEG,
            },
            "confirmation": {
                "indicator": dict(INDICATOR_PURPLE),
                "note": "头部指示灯为物理灯，本卡无法读取，需人眼确认。",
            },
            "source": ("Unitree As2-W 使用手册 V1.1：第 14 页（状态指示）、"
                       "第 19-21 页（操控您的 As2-W / 伴随模式）、"
                       "第 24 页 FAQ 4（伴随遥控器与 App 互斥）、"
                       "第 27 页 FAQ 25（ISS3.0 跟随异常与模组自检）。"),
        }

    def dispatch(self, action, args):
        if action in ("start", "info"):
            if action == "start":
                self.start()
            return {"state": "running" if self._thread is not None else "idle",
                    "topic_out": self.topic_out(),
                    "capability": {
                        "can_enter": False,
                        "can_exit": False,
                        "reason": ("AS2 sport 服务没有伴随模式接口，"
                                   "进入与退出都只能由伴随遥控器或 Unitree Go App 完成。"),
                        "provides": ["preflight", "status", "guide"],
                    }}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "preflight":
            return self._preflight()
        if action == "status":
            return self._evaluate()
        if action == "guide":
            return self._guide()
        return None


def _dumps(payload):
    import json
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
