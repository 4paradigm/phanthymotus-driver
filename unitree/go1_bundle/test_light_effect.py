"""
test_light_effect.py — Go1 面部灯带动效卡(actuator,经 MQTT)。【test 前缀 = 未验收,验收后改名 light_effect】

与 face_light(静态色)互补:face_light 发一个固定色,本卡起后台线程**按时间连续发一串颜色**
到同一 MQTT 主题 `face_light/color`(bytes([r,g,b])),做出动效。控同一条灯带 → 两卡同时用会互相覆盖。

模式(每个可带 duration 秒,跑完自动关灯;duration=0/不填 → 常驻循环到下条指令或 off):
  · solid          定时常亮(亮 N 秒后自动灭,默认3s;要"持续常亮不灭"用 face_light 卡)
  · blink          某色亮/灭交替(period_ms 控频率)         —— 实用:告警
  · breathe        某色亮度平滑一涨一落循环(period_ms)      —— 实用:处理中/思考
  · fade           从色 A 平滑过渡到色 B(需 duration)
  · brightness_up  某色亮度 0→满(需 duration)
  · brightness_down某色亮度 满→0(需 duration,结束即灭)
  · preset         少量语义预设:thinking/listening/alert/warning/done
  · off            停止动效并关灯

颜色:给 r/g/b(0-255)或 color 预设名(red/green/…)。独立走 MQTT,只控灯不动腿,安全。
容器以 --network host 跑在树莓派上时 localhost:1883 即狗的 broker。
"""

from __future__ import annotations

import math
import threading
import time

try:
    import paho.mqtt.client as mqtt
    _HAS_MQTT = True
except Exception:
    _HAS_MQTT = False

CARD = "test_light_effect"
TYPE = "actuator"
DESC = ("Go1 face LED strip DYNAMIC/TIMED effects via MQTT — everything has a time dimension: "
        "blink/breathe/fade/brightness_up/brightness_down, timed solid (hold N secs then auto-off), "
        "plus a few semantic presets. 【test = 未验收】 For a persistent static color (hold indefinitely), "
        "use face_light instead.")

_PRESETS = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
            "yellow": (255, 255, 0), "cyan": (0, 255, 255), "magenta": (255, 0, 255),
            "white": (255, 255, 255), "off": (0, 0, 0)}

# 语义预设(少量实用):name → 动效 spec。含义固化在驱动,LLM 只给标签,返回带 meaning。
_SEMANTIC = {
    "thinking":  {"mode": "breathe", "rgb": (0, 0, 255),   "period_ms": 2000, "meaning": "处理中/思考"},
    "listening": {"mode": "breathe", "rgb": (0, 255, 255), "period_ms": 1800, "meaning": "聆听中"},
    "alert":     {"mode": "blink",   "rgb": (255, 0, 0),   "period_ms": 400,  "meaning": "告警"},
    "warning":   {"mode": "blink",   "rgb": (255, 255, 0), "period_ms": 600,  "meaning": "警告"},
    "done":      {"mode": "solid",   "rgb": (0, 255, 0),   "duration": 2.0,   "meaning": "完成"},
}

_MODES = ["solid", "blink", "breathe", "fade", "brightness_up", "brightness_down"]
_TICK = 0.04          # ~25Hz 刷新
_MAX_DURATION = 600.0  # 单次灯效最长 10 分钟(防跑飞)


def _env(action, ok, **extra):
    d = {"ok": ok, "action": action, "card": CARD,
         "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000)}
    d.update(extra)
    return d


def _clamp8(v):
    return max(0, min(255, int(v)))


def _scale(rgb, f):
    f = max(0.0, min(1.0, f))
    return (_clamp8(rgb[0] * f), _clamp8(rgb[1] * f), _clamp8(rgb[2] * f))


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._host = c.get("mqtt_host", "localhost")
        self._port = int(c.get("mqtt_port", 1883))
        self._client = None
        self._gen = 0
        self._cur = None          # 当前灯效描述(供 info)
        self._lock = threading.Lock()

    def start(self):
        if not _HAS_MQTT:
            print("[test_light_effect] paho-mqtt 未安装,灯带不可用(模拟)", flush=True)
            return
        try:
            self._client = mqtt.Client()
            self._client.connect(self._host, self._port, 60)
            self._client.loop_start()
            print(f"[test_light_effect] MQTT 已连接 → {self._host}:{self._port}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[test_light_effect] MQTT 连接失败: {e}", flush=True)
            self._client = None

    def stop(self):
        self._stop_effect()
        try:
            if self._client:
                self._pub(0, 0, 0)
                time.sleep(0.1)
                self._client.loop_stop()
                self._client.disconnect()
        except Exception:
            pass

    def _pub(self, r, g, b):
        if self._client is None:
            return False
        self._client.publish("face_light/color", bytes([r & 0xFF, g & 0xFF, b & 0xFF]))
        return True

    def _stop_effect(self):
        with self._lock:
            self._gen += 1        # 让在跑的动效线程退出
            self._cur = None

    def _resolve_color(self, args, r_key="r", g_key="g", b_key="b", color_key="color", default=(255, 255, 255)):
        name = args.get(color_key)
        if name is not None:
            rgb = _PRESETS.get(str(name).lower())
            if rgb is None:
                return None
            return rgb
        if any(k in args for k in (r_key, g_key, b_key)):
            return (_clamp8(args.get(r_key, 0)), _clamp8(args.get(g_key, 0)), _clamp8(args.get(b_key, 0)))
        return default

    def _start_effect(self, spec):
        """spec: {mode, rgb, [to_rgb], duration, period_ms}。启动后台线程跑动效。"""
        with self._lock:
            self._gen += 1
            gen = self._gen
            self._cur = spec
        threading.Thread(target=self._run, args=(gen, spec), daemon=True).start()

    def _run(self, gen, spec):
        mode = spec["mode"]
        rgb = spec.get("rgb", (255, 255, 255))
        to_rgb = spec.get("to_rgb", (0, 0, 0))
        duration = spec.get("duration", 0.0)
        period = max(0.05, spec.get("period_ms", 1000) / 1000.0)
        # fade/brightness 需要时间基准;没给 duration 就默认 3 秒
        if mode in ("fade", "brightness_up", "brightness_down") and duration <= 0:
            duration = 3.0
        t0 = time.monotonic()
        while gen == self._gen:
            elapsed = time.monotonic() - t0
            if duration > 0 and elapsed >= duration:
                break
            if mode == "solid":
                col = rgb
            elif mode == "blink":
                col = rgb if (elapsed % period) < period / 2 else (0, 0, 0)
            elif mode == "breathe":
                f = (1 - math.cos(2 * math.pi * (elapsed % period) / period)) / 2  # 0→1→0 平滑
                col = _scale(rgb, f)
            elif mode == "fade":
                f = min(1.0, elapsed / duration) if duration > 0 else 1.0
                col = tuple(_clamp8(rgb[i] + (to_rgb[i] - rgb[i]) * f) for i in range(3))
            elif mode == "brightness_up":
                col = _scale(rgb, min(1.0, elapsed / duration))
            elif mode == "brightness_down":
                col = _scale(rgb, max(0.0, 1 - elapsed / duration))
            else:
                col = rgb
            self._pub(*col)
            time.sleep(_TICK)
        # 结束:被新灯效/off 抢占则不动(交给新的);自己 duration 到点则关灯
        if gen == self._gen:
            self._pub(0, 0, 0)
            with self._lock:
                if self._gen == gen:
                    self._cur = None

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": _MODES + ["preset", "off"],
                               "description": "灯效模式 / 语义预设 preset / 关灯 off"},
                    "r": {"type": "integer", "description": "Red 0-255"},
                    "g": {"type": "integer", "description": "Green 0-255"},
                    "b": {"type": "integer", "description": "Blue 0-255"},
                    "color": {"type": "string", "description": "预设色名 red/green/blue/yellow/cyan/magenta/white(替代 r/g/b)"},
                    "to_color": {"type": "string", "description": "fade 目标预设色名"},
                    "to_r": {"type": "integer", "description": "fade 目标 Red 0-255"},
                    "to_g": {"type": "integer", "description": "fade 目标 Green 0-255"},
                    "to_b": {"type": "integer", "description": "fade 目标 Blue 0-255"},
                    "duration": {"type": "number", "description": "灯效持续秒数;0/不填=常驻循环到下条指令"},
                    "period_ms": {"type": "integer", "description": "blink/breathe 单周期毫秒(默认 blink 500 / breathe 2000)"},
                    "name": {"type": "string", "description": "preset 语义名:thinking/listening/alert/warning/done"},
                },
                "required": ["action"],
                "x-action-params": {
                    "solid": {"params": ["r", "g", "b", "color", "duration"], "description": "定时常亮(亮 duration 秒后自动灭,默认3s;持续常亮不灭请用 face_light 卡)"},
                    "blink": {"params": ["r", "g", "b", "color", "duration", "period_ms"], "description": "闪烁"},
                    "breathe": {"params": ["r", "g", "b", "color", "duration", "period_ms"], "description": "呼吸"},
                    "fade": {"params": ["r", "g", "b", "color", "to_r", "to_g", "to_b", "to_color", "duration"], "description": "A→B 渐变"},
                    "brightness_up": {"params": ["r", "g", "b", "color", "duration"], "description": "渐亮 0→满"},
                    "brightness_down": {"params": ["r", "g", "b", "color", "duration"], "description": "渐暗 满→0"},
                    "preset": {"params": ["name", "duration"], "description": "语义预设"},
                    "off": {"params": [], "description": "停止动效并关灯"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._stop_effect()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._cur else "ready",
                    "current_effect": self._cur, "modes": _MODES,
                    "semantic_presets": {k: v["meaning"] for k, v in _SEMANTIC.items()}}

        if self._client is None:
            return _env(action, False, code="NOT_AVAILABLE",
                        message="MQTT not connected (paho missing or broker unreachable)")

        if action == "off":
            self._stop_effect()
            self._pub(0, 0, 0)
            return _env("off", True, applied={"rgb": [0, 0, 0]})

        dur = float(args.get("duration", 0) or 0)
        dur = max(0.0, min(_MAX_DURATION, dur))
        # solid=定时常亮(亮 N 秒后自动灭);默认 3 秒。要"持续常亮不灭"请用 face_light(职责区分)。
        if action == "solid" and dur <= 0:
            dur = 3.0

        if action == "preset":
            nm = str(args.get("name", "")).lower()
            sem = _SEMANTIC.get(nm)
            if sem is None:
                return _env("preset", False, code="INVALID_ARG",
                            message=f"unknown preset {nm}; available: {', '.join(_SEMANTIC)}")
            spec = {"mode": sem["mode"], "rgb": sem["rgb"],
                    "duration": dur if dur > 0 else sem.get("duration", 0.0),
                    "period_ms": sem.get("period_ms", 1000)}
            self._start_effect(spec)
            return _env("preset", True, applied={"name": nm, "meaning": sem["meaning"],
                                                 "mode": sem["mode"], "rgb": list(sem["rgb"]),
                                                 "duration": spec["duration"]})

        if action in _MODES:
            rgb = self._resolve_color(args)
            if rgb is None:
                return _env(action, False, code="INVALID_ARG",
                            message=f"unknown color name; presets: {', '.join(_PRESETS)}")
            spec = {"mode": action, "rgb": rgb, "duration": dur}
            if action == "blink":
                spec["period_ms"] = int(args.get("period_ms", 500))
            elif action == "breathe":
                spec["period_ms"] = int(args.get("period_ms", 2000))
            elif action == "fade":
                to_rgb = self._resolve_color(args, "to_r", "to_g", "to_b", "to_color", default=(0, 0, 0))
                if to_rgb is None:
                    return _env(action, False, code="INVALID_ARG", message="unknown to_color name")
                spec["to_rgb"] = to_rgb
            self._start_effect(spec)
            applied = {"mode": action, "rgb": list(rgb), "duration": dur}
            if "to_rgb" in spec:
                applied["to_rgb"] = list(spec["to_rgb"])
            if "period_ms" in spec:
                applied["period_ms"] = spec["period_ms"]
            return _env(action, True, applied=applied)

        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
