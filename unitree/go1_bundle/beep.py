"""
beep.py — Go1 头部扬声器 beep 动作卡（自包含，一卡一文件）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == MCP 工具名 == "beep"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。

架构：
  ┌─ Pi 驱动容器 ──────────────┐   HTTP/JSON    ┌─ Head Nano (192.168.123.13) ─┐
  │ beep.py: Plugin            │ ── POST ─────▶ │ beep_adapter.py  :18082       │
  │  · 校验 action/参数        │  /v1/beep/     │  · 生成正弦波 → aplay 出声    │
  │  · 转发给 Nano beep_adapter│    actions     │  · set/get 音量（amixer）     │
  └────────────────────────────┘                └───────────────────────────────┘

beep 不经 Go1 的 UDP HighCmd/HighState 链路，故与共享的只读 SDK client 完全解耦
（make_plugin 收到 client 但忽略它）。Nano 侧 beep_adapter.py 由 deploy/nano_bootstrap.sh
在容器首启时自动部署，无需手动布 Nano。

ROS2 可选：装了 rclpy 且有 executor 时，adapter 不可达会发一条 device_alarms 告警 topic；
否则 beep 的 HTTP 调用照常工作，只是不发告警（与 battery.py 的“ROS2 可选”一致）。
"""

from __future__ import annotations

import json
import time
import urllib.request

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=200,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "beep"          # 卡名 = 模块名 = 文件名 = config key = MCP 工具名


def _now_ms() -> int:
    return int(time.time() * 1000)


def _failure(action, request_id, code, message, retryable=False, details=None) -> dict:
    return {"ok": False, "card": CARD, "action": action, "request_id": request_id,
            "code": code, "message": message, "details": details or {},
            "retryable": retryable, "timestamp_ms": _now_ms()}


class _BeepAdapterClient:
    """访问 Nano 上 beep_adapter 的最小 JSON-over-HTTP 客户端（只打固定端点）。"""

    def __init__(self, config: dict):
        self.base_url = (config.get("adapter_url")
                         or "http://%s:%s/v1" % (config.get("adapter_host", "192.168.123.13"),
                                                 config.get("adapter_port", 18082)))
        self.base_url = self.base_url.rstrip("/")
        self.timeout = float(config.get("rpc_timeout_sec", 2.0))

    def request(self, path: str, payload: dict) -> dict:
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(self.base_url + path, data=data,
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (OSError, ValueError) as exc:
            # OSError 覆盖 URLError / socket.timeout / 连接被拒 / 无路由（跨 py3.9~3.12 都稳），
            # ValueError 覆盖响应非 JSON。上层据此返回 COMMUNICATION_ERROR。
            raise ConnectionError(str(exc)) from exc


class Plugin:
    """beep 动作卡：校验参数后把动作转发给 Nano beep_adapter（/beep/actions）。"""

    def __init__(self, plugin_config, namespace, executor):
        self._config = plugin_config or {}
        self._client = _BeepAdapterClient(self._config)
        # ROS2 告警发布（可选）：无 rclpy / 无 executor 时全程降级为 no-op。
        self._alarm_state = None
        self._alarm_pub = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._alarm_node = Node("go1_%s_alarms" % CARD)
                self._alarm_pub = self._alarm_node.create_publisher(
                    String, "/%s/state/device_alarms" % namespace, _QOS)
                executor.add_node(self._alarm_node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 告警不可用（不影响 beep）: {e}", flush=True)
                self._alarm_pub = None

    # ── 告警（best-effort；无 publisher 时静默）─────────────────────────────
    def _alarm(self, code, message, retryable):
        if self._alarm_pub is None or self._alarm_state == code:
            return
        self._alarm_state = code
        now = _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD, code), "active": True, "severity": "error",
            "card": CARD, "code": code, "message": message, "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": None, "retryable": retryable, "details": {}})))

    def _clear_alarm(self):
        if self._alarm_pub is None or not self._alarm_state:
            return
        code, self._alarm_state, now = self._alarm_state, None, _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD, code), "active": False, "severity": "error",
            "card": CARD, "code": code, "message": "condition recovered", "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": now, "retryable": False, "details": {}})))

    def _call(self, action, args) -> dict:
        request_id = args.get("request_id")
        # 只透传真实卡参数：剔除 main.py dispatch 注入的框架内部键（_tool_name 等）。
        payload = {k: v for k, v in args.items() if not k.startswith("_")}
        payload["action"], payload["request_id"] = action, request_id
        try:
            result = self._client.request("/%s/actions" % CARD, payload)
        except ConnectionError:
            self._alarm("COMMUNICATION_ERROR", "beep adapter is unreachable", True)
            return _failure(action, request_id, "COMMUNICATION_ERROR",
                            "beep adapter is unreachable", True)
        if result.get("ok"):
            self._clear_alarm()
        else:
            self._alarm(result.get("code", "INTERNAL_ERROR"),
                        result.get("message", "beep adapter request failed"),
                        result.get("retryable", False))
        return result

    # ── 插件契约 ───────────────────────────────────────────────────────────
    def get_tool(self):
        return {"name": CARD, "type": "actuator", "multiInstance": False,
          "description": "Go1 head speaker beep — play a beep tone and control volume (action card, no topics)",
          "inputSchema": {"type": "object",
            "properties": {
              "action": {"type": "string", "enum": ["beep", "set_volume", "get_volume"],
                         "description": "Beep action to perform"},
              "request_id": {"type": "string"},
              "duration_sec": {"type": "number", "minimum": 0.1, "maximum": 10,
                               "description": "Beep length in seconds (0.1–10)"},
              "frequency_hz": {"type": "number", "minimum": 100, "maximum": 8000,
                               "description": "Beep tone frequency in Hz (100–8000, default 1000)"},
              "volume_percent": {"type": "integer", "minimum": 0, "maximum": 100,
                                 "description": "Volume 0–100% (set_volume)"}},
            "required": ["action"],
            "x-action-params": {
              "beep":       {"params": ["duration_sec", "frequency_hz"],
                             "description": "Play a beep tone of the given length (seconds) and frequency"},
              "set_volume": {"params": ["volume_percent"], "description": "Set speaker volume 0–100%"},
              "get_volume": {"params": [], "description": "Read current speaker volume"}}}}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        rid = args.get("request_id")
        if action == "beep":
            dur = args.get("duration_sec", 0.3)
            freq = args.get("frequency_hz", 1000)
            if not isinstance(dur, (int, float)) or isinstance(dur, bool) or not 0 < dur <= 10:
                return _failure(action, rid, "INVALID_ARGUMENT", "duration_sec must be a number in (0, 10]")
            if not isinstance(freq, (int, float)) or isinstance(freq, bool) or not 100 <= freq <= 8000:
                return _failure(action, rid, "INVALID_ARGUMENT", "frequency_hz must be a number in [100, 8000]")
            return self._call(action, args)
        if action == "set_volume" and (type(args.get("volume_percent")) is not int
                                        or not 0 <= args["volume_percent"] <= 100):
            return _failure(action, rid, "INVALID_ARGUMENT", "volume_percent must be an integer from 0 to 100")
        if action not in ("set_volume", "get_volume"):
            return _failure(action, rid, "INVALID_ARGUMENT", "unsupported beep action")
        return self._call(action, args)


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。beep 不用共享 SDK client（HighState），故忽略 client。"""
    return Plugin(plugin_config, namespace, executor)
