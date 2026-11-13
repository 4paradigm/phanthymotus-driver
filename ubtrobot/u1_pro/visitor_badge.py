"""U1 Pro 访客贴打印卡片（hp_printer）。

卡片本身不碰打印机，也不直接登录范式+：它只向前台电脑上的「桥接服务」发 HTTP
请求，由桥接服务去范式+ 查访客、调用前台电脑上的 USB 打印机打印访客贴。

    机器人 skill → hp_printer 卡片 ──HTTP──> 前台电脑桥接服务 → 范式+ / USB 打印机

桥接服务要实现的接口见 README.md「桥接服务接口约定」。
"""

from __future__ import annotations

import json
import re
import threading
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

from common.vendor_runtime import action_schema, tool

TOOL_NAME = "hp_printer"

_ID_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,128}$")


class BridgeError(Exception):
    """桥接服务调用失败。message 是可以直接转述给人的中文说明。"""

    def __init__(self, message: str, code: str = "bridge_error"):
        super().__init__(message)
        self.code = code


class BridgeClient:
    """前台电脑桥接服务的最小 HTTP 客户端（只用标准库）。"""

    def __init__(self, base_url: str, token: str = "", timeout_s: float = 8.0):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout_s = timeout_s

    def request(self, method: str, path: str, *, query: dict | None = None, body: dict | None = None) -> dict:
        url = self.base_url + path
        if query:
            url += "?" + urllib.parse.urlencode({k: v for k, v in query.items() if v not in (None, "")})
        data = json.dumps(body, ensure_ascii=False).encode() if body is not None else None
        headers = {"Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                raw = resp.read()
        except urllib.error.HTTPError as exc:
            payload = _safe_json(exc.read())
            if exc.code in (401, 403):
                raise BridgeError("桥接服务拒绝了请求：令牌不对，请检查卡片配置里的访问令牌", "unauthorized")
            if exc.code == 404 and not payload:
                raise BridgeError("桥接服务没有这个接口，请确认前台电脑上运行的是访客贴桥接服务", "not_found")
            message = str(payload.get("error") or f"桥接服务返回 HTTP {exc.code}")
            raise BridgeError(message, str(payload.get("code") or f"http_{exc.code}"))
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            raise BridgeError(
                f"连不上前台电脑的桥接服务（{self.base_url}）：{reason}。请确认前台电脑开着、桥接服务在运行、网络通",
                "unreachable",
            )
        payload = _safe_json(raw)
        if not isinstance(payload, dict):
            raise BridgeError("桥接服务返回的不是 JSON 对象", "bad_response")
        return payload


def _safe_json(raw: bytes) -> Any:
    try:
        return json.loads(raw.decode("utf-8") or "{}")
    except (UnicodeDecodeError, json.JSONDecodeError):
        return {}


def _visit(item: dict) -> dict:
    """只保留桥接协议允许交给 skill 的字段。"""
    return {
        "ticket": str(item.get("ticket", "")),
        "tail": str(item.get("tail", ""))[-4:],
        "host": str(item.get("host", "")),
        "time": str(item.get("time", "")),
        "reason": str(item.get("reason", "")),
        "status": str(item.get("status", "")),
    }


class HPPrinterPlugin:
    PREFIX = TOOL_NAME

    def __init__(self, plugin_config: dict | None = None):
        plugin_config = plugin_config or {}
        self._lock = threading.Lock()
        self._cfg = {
            "bridge_url": str(plugin_config.get("bridge_url", "") or ""),
            "token": str(plugin_config.get("token", "") or ""),
            "timeout_s": float(plugin_config.get("timeout_s", 8)),
        }

    # ── 卡片定义 ──────────────────────────────────────────────────────────
    def get_tool(self) -> dict:
        actions = {
            "start": ([], "启动卡片"),
            "stop": ([], "停止卡片"),
            "info": ([], "查看卡片配置是否完整"),
            "config": (["bridge_url", "token", "timeout_s"], "保存桥接服务配置（平台在 start 前自动调用）"),
            "find": (["name", "host"], "按访客姓名查询今天的预约；同名时可用被访人 host 缩小范围，返回短期 ticket"),
            "print": (["ticket", "request_id"], "使用 find 返回的短期 ticket 打印访客贴；request_id 用于防止重复打印"),
        }
        properties = {
            "name": {"type": "string", "description": "访客姓名"},
            "host": {"type": "string", "description": "被访人姓名；同名访客时用于缩小范围"},
            "ticket": {"type": "string", "description": "find 返回的短期票据，不能自己编，也不能改写"},
            "request_id": {
                "type": "string",
                "description": "本次打印的唯一编号；同一次请求重试时必须复用",
            },
            "bridge_url": {"type": "string"},
            "token": {"type": "string"},
            "timeout_s": {"type": "number"},
        }
        definition = tool(
            TOOL_NAME,
            "actuator",
            "访客贴打印：通过前台电脑上的桥接服务查询范式+ 里的访客预约，并用前台的打印机打印访客贴。"
            "先 find 找到访客并和访客确认身份，再用返回的短期 ticket 调用 print；不要按猜测的名字打印。",
            action_schema(actions, properties),
        )
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "bridge_url": {
                    "type": "string", "title": "桥接服务地址", "scope": "shared", "x-sensitive": True,
                    "description": "前台电脑上桥接服务的地址，例如 http://192.168.1.20:8765（前台电脑要设固定 IP）",
                },
                "token": {
                    "type": "string", "title": "访问令牌", "scope": "shared", "format": "password",
                    "description": "和桥接服务里设置的令牌一致，防止局域网里别人随便打印",
                },
                "timeout_s": {
                    "type": "number", "title": "请求超时（秒）", "scope": "shared",
                    "minimum": 2, "maximum": 30, "default": 8, "advanced": True,
                },
            },
            "required": ["bridge_url", "token"],
        }
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    # ── 调度 ──────────────────────────────────────────────────────────────
    def dispatch(self, action: str, args: dict) -> dict | None:
        args = {k: v for k, v in args.items() if not k.startswith("_")}
        if action == "start":
            return {"state": "ready", "configured": self._configured()}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready", "configured": self._configured(), "bridge_url": self._cfg["bridge_url"]}
        if action == "config":
            return self._config(args)
        handlers = {"find": self._find, "print": self._print}
        handler = handlers.get(action)
        if handler is None:
            return {"status": "error", "code": "bad_action", "error": f"不支持的动作：{action}"}
        if not self._configured():
            return {"status": "error", "code": "not_configured", "error": "卡片还没配置桥接服务地址，请在卡片配置里填写"}
        try:
            return handler(args)
        except BridgeError as exc:
            print(f"[hp_printer] {action} failed: {exc.code}", flush=True)
            return {"status": "error", "code": exc.code, "error": str(exc)}

    def _configured(self) -> bool:
        return bool(self._cfg["bridge_url"] and self._cfg["token"])

    def _client(self) -> BridgeClient:
        with self._lock:
            cfg = dict(self._cfg)
        return BridgeClient(cfg["bridge_url"], cfg["token"], cfg["timeout_s"])

    def _config(self, args: dict) -> dict:
        update: dict = {}
        if "bridge_url" in args:
            url = str(args["bridge_url"] or "").strip()
            if url:
                parsed = urllib.parse.urlparse(url)
                if parsed.scheme not in ("http", "https") or not parsed.hostname:
                    return {"status": "error", "adapter_ok": False,
                            "error": "桥接服务地址要写成 http://IP:端口 的形式"}
            update["bridge_url"] = url
        if "token" in args:
            update["token"] = str(args["token"] or "")
        for key, low, high in (("timeout_s", 2, 30),):
            if key in args and args[key] not in (None, ""):
                try:
                    update[key] = min(high, max(low, float(args[key])))
                except (TypeError, ValueError):
                    return {"status": "error", "adapter_ok": False, "error": f"{key} 要填数字"}
        with self._lock:
            self._cfg.update(update)
            configured = bool(self._cfg["bridge_url"] and self._cfg["token"])
        return {"status": "ok", "adapter_ok": configured, "configured": configured}

    # ── 动作 ──────────────────────────────────────────────────────────────
    def _find(self, args: dict) -> dict:
        name = str(args.get("name", "")).strip()
        host = str(args.get("host", "")).strip()
        if not name:
            return {"status": "error", "code": "bad_args", "error": "find 需要访客姓名 name"}
        if len(name) > 32 or len(host) > 64:
            return {"status": "error", "code": "bad_args", "error": "姓名或被访人信息太长"}
        body = {"name": name}
        if host:
            body["host"] = host
        payload = self._client().request("POST", "/find_visit", body=body)
        visits = [_visit(v) for v in payload.get("visits") or [] if isinstance(v, dict)]
        visits = [v for v in visits if v["ticket"]]
        if not visits:
            return {"status": "not_found", "name": name, "visits": [],
                    "hint": "范式+ 里今天没有这个访客的预约，不能打印访客贴，请联系被访人或前台同事"}
        return {"status": "found" if len(visits) == 1 else "multiple", "name": name, "visits": visits}

    def _print(self, args: dict) -> dict:
        ticket = str(args.get("ticket", "")).strip()
        if not ticket or len(ticket) > 512:
            return {"status": "error", "code": "bad_args", "error": "print 需要 find 返回的短期 ticket"}
        request_id = str(args.get("request_id", "")).strip()
        if not _ID_RE.match(request_id):
            return {"status": "error", "code": "bad_args",
                    "error": "print 需要唯一的 request_id；同一次请求重试时必须复用"}
        payload = self._client().request(
            "POST", "/print_tag", body={"ticket": ticket, "request_id": request_id}
        )
        if payload.get("ok") is True:
            return {"status": "printed", "request_id": request_id,
                    "say": "访客贴打好了，在前台打印机上"}
        return {
            "status": "failed",
            "code": str(payload.get("code") or "print_failed"),
            "error": str(payload.get("error") or "打印失败，请前台同事检查打印机"),
            "request_id": request_id,
        }


