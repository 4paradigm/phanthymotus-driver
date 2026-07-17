"""
battery_guard.py — Go1 低电量报警卡（A: 面部 LED 红灯闪烁 + 高电量常亮绿灯 + B: 报警事件/topic 上报，后台监测）。

自包含：一张卡 = 一个文件。start() 起后台守护线程，以 MONITOR_HZ 持续读共享 client 的
snapshot()["battery"]（soc_percent + status），去抖后判定电量级别，布防后自动报警。

灯光规则：
1. SOC > 50% 常亮绿色
2. 20% ＜ SOC ≤ 50% 灯熄灭无显示
3. 10% ＜ SOC ≤ 20% 红灯慢闪
4. SOC ≤ 10% 红灯快闪
5. 充电状态、无有效电量数据、数据未刷新：强制灭灯

为什么不是"蜂鸣器"：unitree_legged_sdk 高层 HighCmd 里**没有蜂鸣器控制字段**，狗低电时那声
"滴滴滴"是固件自带、我们触发不了；SDK 能碰的唯一物理输出是 HighCmd.led[4]（面部 4 颗 RGB 灯）。
因此本卡用**双通道报警**代替蜂鸣器：
  · A 物理（狗本体）：低电量时后台线程让面部 LED 红灯按节奏闪烁 = 灯光版"滴滴滴"。
    经 client.set_led()（与运动正交，不改 mode/velocity，不打断行走）。
  · B 软件（平台/面板）：把报警状态发到 ROS2 topic /{ns}/alarm/battery，并在级别变化时打事件，
    由面板/宿主那边播放提示音，声音在电脑端出。

三态状态机 normal → low → critical（升级触发/刷新报警，降级或充电时清报警、灭灯）：
  · LOW 低电量（默认 SOC≤20%）：红灯慢闪（每 BLINK_LOW_TICKS 拍翻转）。
  · CRIT 极低电量（默认 SOC≤10%）：红灯快闪（每 BLINK_CRIT_TICKS 拍翻转），更急促。
带迟滞：回升需高于阈值 + CLEAR_MARGIN 才降级，避免在阈值附近抖动反复报警。
充电中（status=charge/charger）视为安全，抑制报警并灭灯。

⚠ 只在 fresh=true 时判定/报警；fresh=false（STUB/未独占 8090/丢真值）时清计数、灭灯、如实上报无反馈。
⚠ 本卡只点灯 + 发事件，不下发任何运动命令，不会让狗移动，因此 arm 默认免 confirm、且默认开机自动布防
   （随时可 disarm）。set_led 是否真点亮需上狗 A/B 验证：先 action=test 看灯是否变红。
"""

from __future__ import annotations

import json
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

# ── 卡片元数据 ──────────────────────────────────────────────────────────────
CARD = "test_battery_guard_zjh"
TYPE = "control"                     # 会下发 LED 命令（非只读）
NODE = "go1_test_battery_guard_zjh"
TOPIC = "/{ns}/alarm/battery"        # 报警状态 topic（供面板/宿主播提示音）
FMT = "data/json"
DESC = ("Go1 电量指示灯：SOC＞50%常亮绿灯；低电量(≤20%)→面部LED红灯慢闪；极低(≤10%)→红灯快闪；"
        "并把报警状态发到 /{ns}/alarm/battery 供平台出声（SDK无蜂鸣器，以灯+事件代之）。"
        "actions=arm(布防)/disarm/check/info/test(试灯2秒)。")

# ── 电量阈值（SOC%）────────────────────────────────────────────────────────
LOW_SOC = 20                         # ≤ 此值进入低电量（慢闪）
CRIT_SOC = 10                        # ≤ 此值进入极低电量（快闪）
GREEN_SOC_THRESHOLD = 50             # 大于该值常亮绿灯
CLEAR_MARGIN = 3                     # 迟滞：回升需高于对应阈值 + 此余量才降级
DEBOUNCE_N = 3                       # 连续 N 拍越阈才确认（防单帧噪声）
CLEAR_N = 3                          # 连续 N 拍回到安全区才降级

# ── 报警节奏 / 灯 ───────────────────────────────────────────────────────────
MONITOR_HZ = 5.0                     # 监测频率（电量变化慢，兼顾闪灯节奏）
BLINK_LOW_TICKS = 3                  # 低电量：每 3 拍(≈0.6s)翻转一次红灯（慢闪）
BLINK_CRIT_TICKS = 1                 # 极低：每 1 拍(≈0.2s)翻转（快闪，急促）
LED_RED = (255, 0, 0)
LED_GREEN = (0, 255, 0)
LED_OFF = (0, 0, 0)
TEST_SECONDS = 2.0                   # action=test 试灯时长

LEVELS = {0: "normal", 1: "low", 2: "critical"}
CHARGING_STATUS = {"charge", "charger", "precharge"}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._ns = namespace
        cfg = plugin_config or {}
        self._low_soc = int(cfg.get("low_soc_percent", LOW_SOC))
        self._crit_soc = int(cfg.get("critical_soc_percent", CRIT_SOC))
        self._green_soc = int(cfg.get("green_soc_percent", GREEN_SOC_THRESHOLD))
        self._debounce = int(cfg.get("debounce_samples", DEBOUNCE_N))
        self._clear_n = int(cfg.get("clear_samples", CLEAR_N))
        # LED 只点灯不动狗 → 默认免 confirm、默认开机布防（当作常驻安全监测）
        self._require_confirm = bool(cfg.get("require_confirm", False))

        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None

        # 监测状态（受 _lock 保护）
        self._lock = threading.Lock()
        self._armed = bool(cfg.get("armed_on_start", True))
        self._level = 0                # 0 normal / 1 low / 2 critical
        self._low_count = 0            # 连续越低电量阈计数
        self._crit_count = 0           # 连续越极低阈计数
        self._clear_count = 0          # 连续回安全区计数
        self._blink_tick = 0           # 闪灯相位计数
        self._led_on = False           # 当前灯相位（红/灭）
        self._led_active = False       # 我方是否正在驱动 LED（用于收尾灭灯）
        self._in_blink = False         # 是否处于告警闪烁模式（区分于绿灯常亮，用于切到告警时立即亮红灯）
        self._last = {"soc": None, "status": None, "charging": False, "fresh": False}
        self._last_event = None
        self._events = 0
        self._testing = False          # action=test 进行中，暂停监测驱动灯

        self._running = False
        self._thread = None

        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.get_logger().info(f"battery_guard alarm → {self._topic}")
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False,
            "description": DESC + (f" — → {self._topic}" if self._node else ""),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "arm", "disarm", "check", "info", "test"],
                               "description": "start/stop 生命周期；arm 布防/disarm 撤防/check 查当前电量与级别/info 查配置与统计/test 红灯试亮2秒(验证灯可用)"},
                    "confirm": {"type": "boolean", "description": "require_confirm=true 时 arm 需传 true（默认免确认，仅点灯不动狗）"},
                },
                "required": ["action"],
            },
            "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else []),
        }

    # ── LED 下发（防御式）────────────────────────────────────────────────────
    #   共享 client 未必有 set_led/clear_led（不同 bundle 版本 client 架构不同）→ 用 hasattr 探测：
    #   有就走 A 通道点灯，没有则 A 通道降级、只留 B 通道(topic 事件)，不报错、不影响告警主逻辑。
    def _led_set(self, r, g, b) -> bool:
        fn = getattr(self._client, "set_led", None)
        if callable(fn):
            try:
                return bool(fn(r, g, b))
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] set_led 调用异常，A 通道降级: {e}", flush=True)
        return False

    def _led_clear(self) -> bool:
        fn = getattr(self._client, "clear_led", None)
        if callable(fn):
            try:
                return bool(fn())
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] clear_led 调用异常: {e}", flush=True)
        return False

    # ── 生命周期 ────────────────────────────────────────────────────────────
    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._monitor, daemon=True, name=NODE)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self._led_clear()

    # ── 后台监测回路 ─────────────────────────────────────────────────────────
    def _monitor(self):
        period = 1.0 / MONITOR_HZ
        while self._running:
            try:
                self._tick()
                if self._pub is not None:
                    self._publish()
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] monitor error: {e}", flush=True)
            time.sleep(period)

    @staticmethod
    def _read(s):
        """从 snapshot 取 (fresh, soc, status, charging)。soc 为 None 表示无 BMS 数据。
        snapshot 的 battery 只有 status_code(数字) + status_name(字符串)，无 "status" 字段；
        充电判定按 status_name 匹配 CHARGING_STATUS（值来自 BMS_STATUS_NAMES，已小写）。"""
        fresh = bool(s.get("fresh"))
        bat = s.get("battery") or {}
        soc = bat.get("soc_percent")
        status_name = bat.get("status_name")
        status_code = bat.get("status_code")
        # 上报用 status_name（更可读），fallback 到 status_code
        status = status_name if status_name is not None else status_code
        charging = bool(status_name and str(status_name).lower() in CHARGING_STATUS)
        return fresh, (int(soc) if soc is not None else None), status, charging

    def _tick(self):
        fresh, soc, status, charging = self._read(self._client.snapshot())

        fire = None
        led_cmd = None
        with self._lock:
            self._last = {"soc": soc, "status": status, "charging": charging, "fresh": fresh}

            # test 手动试灯期间，完全跳过自动灯控
            if self._testing:
                return

            # ========== 最高优先级：充电 / 无有效数据 直接关灯 ==========
            if not fresh or soc is None or charging:
                self._low_count = self._crit_count = self._clear_count = 0
                self._in_blink = False
                if self._level != 0:
                    self._level = 0
                if self._led_active:
                    self._led_active = False
                    self._led_on = False
                    self._blink_tick = 0
                    led_cmd = "clear"
            else:
                # ========== 第二优先级：电量大于绿阈 强制常亮绿灯 ==========
                if soc > self._green_soc:
                    self._in_blink = False
                    self._led_active = True
                    led_cmd = LED_GREEN
                else:
                    # ========== 第三优先级：低电量告警闪烁逻辑 ==========
                    crit_over = soc <= self._crit_soc
                    low_over = soc <= self._low_soc
                    safe = soc > (self._low_soc + CLEAR_MARGIN)

                    self._crit_count = self._crit_count + 1 if crit_over else 0
                    self._low_count = self._low_count + 1 if low_over else 0
                    self._clear_count = self._clear_count + 1 if safe else 0

                    target = self._level
                    if self._crit_count >= self._debounce:
                        target = 2
                    elif self._low_count >= self._debounce:
                        target = 1
                    elif self._clear_count >= self._clear_n:
                        target = 0

                    if target != self._level and self._armed:
                        fire = (LEVELS[target], soc, status)
                    self._level = target

                    # 进入告警：红灯闪烁
                    if self._armed and self._level > 0:
                        step = BLINK_CRIT_TICKS if self._level == 2 else BLINK_LOW_TICKS
                        if not self._in_blink:
                            # 刚进入告警闪烁模式：立即亮红灯（不轮等翻转，
                            # 解决从绿灯常亮切到告警时第一拍不亮的问题）
                            self._in_blink = True
                            self._led_on = True
                            self._blink_tick = 0
                        else:
                            self._blink_tick += 1
                            if self._blink_tick >= step:
                                self._blink_tick = 0
                                self._led_on = not self._led_on
                        self._led_active = True
                        led_cmd = LED_RED if self._led_on else LED_OFF
                    else:
                        # 安全区（低阈~绿阈区间）：熄灭灯光
                        self._in_blink = False
                        if self._led_active:
                            self._led_active = False
                            self._led_on = False
                            self._blink_tick = 0
                            led_cmd = "clear"

        # 锁外下发LED指令
        if led_cmd == "clear":
            self._led_clear()
        elif led_cmd is not None:
            self._led_set(*led_cmd)

        if fire:
            self._emit(fire)

    def _emit(self, fire):
        level, soc, status = fire
        ev = {"level": level, "soc_percent": soc, "status": status,
              "ts_ms": int(time.time() * 1000)}
        with self._lock:
            if level != "normal":
                self._events += 1
            self._last_event = ev   # normal 也更新，让 info 能看到恢复时间
        tag = {"normal": "✓ 电量恢复", "low": "⚠ 低电量", "critical": "‼ 极低电量"}.get(level, level)
        print(f"[{CARD}] {tag} SOC={soc}% → {ev}", flush=True)

    def _publish(self):
        try:
            with self._lock:
                payload = {
                    "timestamp_ms": int(time.time() * 1000),
                    "fresh": bool(self._last.get("fresh", False)),
                    "armed": self._armed,
                    "level": LEVELS[self._level],
                    "alarming": bool(self._armed and self._level > 0),
                    "led_on": bool(self._led_on and self._led_active),
                    "soc_percent": self._last.get("soc"),
                    "status": self._last.get("status"),
                    "charging": bool(self._last.get("charging")),
                    "thresholds": {"low": self._low_soc, "critical": self._crit_soc, "green": self._green_soc},
                }
            m = String()
            m.data = json.dumps(payload)
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            if self._node:
                self._node.get_logger().error(f"publish {self._topic} error: {e}")

    # ── MCP 调用 ─────────────────────────────────────────────────────────────
    def dispatch(self, action, args):
        args = args or {}
        now = int(time.time() * 1000)

        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return self._status(now, "info")
        if action == "check":
            return self._check(now)
        if action == "test":
            return self._test(now)

        if action == "arm":
            if self._require_confirm and not bool(args.get("confirm", False)):
                return {"ok": False, "code": "PRECONDITION_FAILED",
                        "message": "require_confirm=true，arm 需传 confirm:true",
                        "card": CARD, "action": "arm", "timestamp_ms": now}
            with self._lock:
                self._armed = True
                self._level = 0
                self._low_count = self._crit_count = self._clear_count = 0
                self._in_blink = False
            return self._status(now, "arm")

        if action == "disarm":
            with self._lock:
                self._armed = False
                self._led_active = False
                self._led_on = False
                self._in_blink = False
            self._led_clear()
            return self._status(now, "disarm")

        return None   # 未知 action → 契约：MCP 报 Unknown

    def _test(self, now):
        """试灯：亮红灯 TEST_SECONDS 秒后灭，验证 set_led 在本机是否真点亮（A/B 用）。
        用独立线程，避免阻塞 MCP 调用；期间监测让路不抢灯。"""
        def _run():
            with self._lock:
                self._testing = True
            ok = self._led_set(*LED_RED)
            time.sleep(TEST_SECONDS)
            self._led_clear()
            with self._lock:
                self._testing = False
            print(f"[{CARD}] test 试灯完成（set_led 下发={'成功' if ok else '失败/降级'}）", flush=True)
        applied = self._client.available
        threading.Thread(target=_run, daemon=True, name=NODE + "_test").start()
        return {"ok": True, "card": CARD, "action": "test",
                "message": f"红灯亮 {TEST_SECONDS}s 后自动灭，请观察狗面部灯是否变红",
                "sdk_available": bool(applied), "timestamp_ms": now}

    def _status(self, now, action):
        with self._lock:
            return {"ok": True, "card": CARD, "action": action,
                    "control_level": "HIGHLEVEL",
                    "armed": self._armed, "level": LEVELS[self._level],
                    "alarming": bool(self._armed and self._level > 0),
                    "require_confirm": self._require_confirm,
                    "thresholds": {"low_soc_percent": self._low_soc,
                                   "critical_soc_percent": self._crit_soc,
                                   "green_threshold": self._green_soc,
                                   "clear_margin": CLEAR_MARGIN,
                                   "debounce_samples": self._debounce,
                                   "clear_samples": self._clear_n},
                    "battery": {"soc_percent": self._last.get("soc"),
                                "status": self._last.get("status"),
                                "charging": bool(self._last.get("charging"))},
                    "fresh": bool(self._last.get("fresh", False)),
                    "events": self._events, "last_event": self._last_event,
                    "topic_out": ([self._topic] if self._node else []),
                    "note": "SOC＞50常亮绿灯；20%以下红灯闪烁报警；充电/无数据强制灭灯；SDK无蜂鸣器，灯效+ROS话题双通道提示；"
                            "灯是否真亮先用 action=test 验证",
                    "timestamp_ms": now}

    def _check(self, now):
        fresh, soc, status, charging = self._read(self._client.snapshot())
        low_over = fresh and soc is not None and soc <= self._low_soc
        crit_over = fresh and soc is not None and soc <= self._crit_soc
        with self._lock:
            level, armed = LEVELS[self._level], self._armed
        return {"ok": True, "card": CARD, "action": "check",
                "fresh": fresh, "level": level, "armed": armed,
                "soc_percent": soc, "status": status, "charging": bool(charging),
                "low_over": bool(low_over), "crit_over": bool(crit_over),
                "note": ("无新鲜反馈/无BMS数据，读数不可信" if (not fresh or soc is None)
                         else ("充电中，报警已抑制" if charging
                               else "low_over/crit_over=瞬时越阈；level=经去抖判定")),
                "timestamp_ms": now}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)