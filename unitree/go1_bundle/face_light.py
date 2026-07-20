"""
face_light.py — Go1 面部灯带颜色卡(actuator,经 MQTT)。

自包含:一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并调用 make_plugin()。
接口来自狗官方 programming.py:client.publish("face_light/color", bytes([r,g,b]))。
独立走 MQTT(狗自带 broker,默认 localhost:1883),不依赖运控 SDK,只控灯、不动腿,安全。
容器以 --network host 跑在树莓派上时,localhost:1883 即狗的 broker。
"""

from __future__ import annotations

import time

try:
    import paho.mqtt.client as mqtt
    _HAS_MQTT = True
except Exception:
    _HAS_MQTT = False

CARD = "face_light"
TYPE = "actuator"
DESC = ("Go1 face LED strip STATIC color via MQTT — set a persistent RGB/preset color and hold it "
        "(steady-state indication; stays until changed or off). For blink/breathe/fade/timed effects, "
        "use test_light_effect instead.")

_PRESETS = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
            "yellow": (255, 255, 0), "cyan": (0, 255, 255), "magenta": (255, 0, 255),
            "white": (255, 255, 255), "off": (0, 0, 0)}


def _env(action, ok, **extra):
    d = {"ok": ok, "action": action, "card": CARD,
         "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000)}
    d.update(extra)
    return d


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._host = c.get("mqtt_host", "localhost")
        self._port = int(c.get("mqtt_port", 1883))
        self._client = None

    def start(self):
        if not _HAS_MQTT:
            print("[face_light] paho-mqtt 未安装,灯带不可用(模拟)", flush=True)
            return
        try:
            self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self._client.connect(self._host, self._port, 60)
            self._client.loop_start()
            print(f"[face_light] MQTT 已连接 → {self._host}:{self._port}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[face_light] MQTT 连接失败: {e}", flush=True)
            self._client = None

    def stop(self):
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

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["set_color", "preset", "off"],
                               "description": "Light action"},
                    "r": {"type": "integer", "description": "Red 0-255"},
                    "g": {"type": "integer", "description": "Green 0-255"},
                    "b": {"type": "integer", "description": "Blue 0-255"},
                    "name": {"type": "string",
                             "description": "Preset: red/green/blue/yellow/cyan/magenta/white/off"},
                },
                "required": ["action"],
                "x-action-params": {

                    "set_color": {"params": ["r", "g", "b"], "description": "Set RGB (0-255 each)"},
                    "preset": {"params": ["name"], "description": "Preset color by name"},
                    "off": {"params": [], "description": "Turn light off"},
                },
            },
            "topic_out": [],
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}

        if self._client is None:
            return _env(action, False, code="NOT_AVAILABLE",
                        message="MQTT not connected (paho missing or broker unreachable)")

        if action == "off":
            self._pub(0, 0, 0)
            return _env("off", True, applied={"rgb": [0, 0, 0]})
        if action == "preset":
            nm = str(args.get("name", "off")).lower()
            rgb = _PRESETS.get(nm)
            if rgb is None:
                return _env("preset", False, code="INVALID_ARG",
                            message=f"unknown preset {nm}; available: {', '.join(_PRESETS)}")
            self._pub(*rgb)
            return _env("preset", True, applied={"name": nm, "rgb": list(rgb)})
        if action == "set_color":
            r = max(0, min(255, int(args.get("r", 0))))
            g = max(0, min(255, int(args.get("g", 0))))
            b = max(0, min(255, int(args.get("b", 0))))
            self._pub(r, g, b)
            return _env("set_color", True, applied={"r": r, "g": g, "b": b})
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
