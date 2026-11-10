"""U1 Pro 访客贴打印卡片（visitor_badge）。

卡片本身不碰打印机，也不直接登录范式+：它只向前台电脑上的「桥接服务」发 HTTP
请求，由桥接服务去范式+ 查访客、调用前台电脑上的 USB 打印机打印访客贴。

    机器人 skill → visitor_badge 卡片 ──HTTP──> 前台电脑桥接服务 → 范式+ / USB 打印机

桥接服务要实现的接口见 README.md「桥接服务接口约定」。
"""

from __future__ import annotations

import json
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

from common.vendor_runtime import action_schema, tool

TOOL_NAME = "visitor_badge"

# 打印任务的终态；其余（queued / printing）视为进行中。
DONE_STATES = {"done", "failed", "canceled"}

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


def _visitor(item: dict) -> dict:
    """只保留 skill 需要的字段，避免把访客系统的整条记录（手机号、证件号）交给模型。"""
    return {
        "visitor_id": str(item.get("id", "")),
        "name": str(item.get("name", "")),
        "company": str(item.get("company", "")),
        "host": str(item.get("host", "")),
        "visit_time": str(item.get("visit_time", "")),
        "phone_tail": str(item.get("phone_tail", ""))[-4:],
        "printed": bool(item.get("printed", False)),
    }


class VisitorBadgePlugin:
    PREFIX = TOOL_NAME

    def __init__(self, plugin_config: dict | None = None):
        plugin_config = plugin_config or {}
        self._lock = threading.Lock()
        self._cfg = {
            "bridge_url": str(plugin_config.get("bridge_url", "") or ""),
            "token": str(plugin_config.get("token", "") or ""),
            "timeout_s": float(plugin_config.get("timeout_s", 8)),
            "print_wait_s": float(plugin_config.get("print_wait_s", 20)),
        }
        self._poll_interval_s = float(plugin_config.get("poll_interval_s", 1.0))

    # ── 卡片定义 ──────────────────────────────────────────────────────────
    def get_tool(self) -> dict:
        actions = {
            "start": ([], "启动卡片"),
            "stop": ([], "停止卡片"),
            "info": ([], "查看卡片配置是否完整"),
            "config": (["bridge_url", "token", "timeout_s", "print_wait_s"], "保存桥接服务配置（平台在 start 前自动调用）"),
            "check": ([], "检查前台电脑桥接服务、范式+ 登录状态和打印机是否就绪"),
            "find": (["keyword"], "在范式+ 里查今天的访客预约，keyword 填访客姓名或手机后四位；返回候选列表和 visitor_id"),
            "print": (["visitor_id", "reprint", "wait"], "按 find 返回的 visitor_id 打印访客贴；默认等打印完成再返回"),
            "job": (["job_id"], "查询一次打印任务的结果"),
        }
        properties = {
            "keyword": {"type": "string", "description": "访客姓名或手机后四位"},
            "visitor_id": {"type": "string", "description": "find 返回的 visitor_id，不能自己编"},
            "reprint": {"type": "boolean", "description": "访客今天已经打过访客贴时是否再打一张，默认 false", "default": False},
            "wait": {"type": "boolean", "description": "是否等打印完成再返回，默认 true", "default": True},
            "job_id": {"type": "string", "description": "print 返回的 job_id"},
            "bridge_url": {"type": "string"},
            "token": {"type": "string"},
            "timeout_s": {"type": "number"},
            "print_wait_s": {"type": "number"},
        }
        definition = tool(
            TOOL_NAME,
            "actuator",
            "访客贴打印：通过前台电脑上的桥接服务查询范式+ 里的访客预约，并用前台的打印机打印访客贴。"
            "先 find 找到访客并和访客确认身份，再用返回的 visitor_id 调用 print；不要按猜测的名字打印。",
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
                "print_wait_s": {
                    "type": "number", "title": "等待打印完成（秒）", "scope": "shared",
                    "minimum": 0, "maximum": 60, "default": 20, "advanced": True,
                    "description": "print 默认等打印结果；超过这个时间返回 printing，之后用 job 查询",
                },
            },
            "required": ["bridge_url"],
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
        handlers = {"check": self._check, "find": self._find, "print": self._print, "job": self._job}
        handler = handlers.get(action)
        if handler is None:
            return {"status": "error", "code": "bad_action", "error": f"不支持的动作：{action}"}
        if not self._configured():
            return {"status": "error", "code": "not_configured", "error": "卡片还没配置桥接服务地址，请在卡片配置里填写"}
        try:
            return handler(args)
        except BridgeError as exc:
            print(f"[visitor_badge] {action} failed: {exc.code}", flush=True)
            return {"status": "error", "code": exc.code, "error": str(exc)}

    def _configured(self) -> bool:
        return bool(self._cfg["bridge_url"])

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
        for key, low, high in (("timeout_s", 2, 30), ("print_wait_s", 0, 60)):
            if key in args and args[key] not in (None, ""):
                try:
                    update[key] = min(high, max(low, float(args[key])))
                except (TypeError, ValueError):
                    return {"status": "error", "adapter_ok": False, "error": f"{key} 要填数字"}
        with self._lock:
            self._cfg.update(update)
            configured = bool(self._cfg["bridge_url"])
        return {"status": "ok", "adapter_ok": configured, "configured": configured}

    # ── 动作 ──────────────────────────────────────────────────────────────
    def _check(self, _args: dict) -> dict:
        health = self._client().request("GET", "/health")
        printer = health.get("printer") or {}
        visitor_system = health.get("visitor_system") or {}
        problems = []
        if not printer.get("ready", False):
            problems.append(f"打印机未就绪：{printer.get('message') or '请检查前台打印机电源、纸和连线'}")
        if not visitor_system.get("ready", False):
            problems.append(f"范式+ 未就绪：{visitor_system.get('message') or '请在前台电脑上重新登录范式+'}")
        return {
            "status": "ok" if not problems else "not_ready",
            "printer": {"ready": bool(printer.get("ready")), "name": str(printer.get("name", ""))},
            "visitor_system": {"ready": bool(visitor_system.get("ready"))},
            "problems": problems,
        }

    def _find(self, args: dict) -> dict:
        keyword = str(args.get("keyword", "")).strip()
        if not keyword:
            return {"status": "error", "code": "bad_args", "error": "find 需要 keyword（访客姓名或手机后四位）"}
        if len(keyword) > 32:
            return {"status": "error", "code": "bad_args", "error": "keyword 太长"}
        payload = self._client().request("GET", "/visitors", query={"keyword": keyword})
        visitors = [_visitor(v) for v in payload.get("visitors") or [] if isinstance(v, dict)]
        visitors = [v for v in visitors if v["visitor_id"]]
        if not visitors:
            return {"status": "not_found", "keyword": keyword, "visitors": [],
                    "hint": "范式+ 里今天没有这个访客的预约，不能打印访客贴，请联系被访人或前台同事"}
        return {"status": "found" if len(visitors) == 1 else "multiple", "keyword": keyword, "visitors": visitors}

    def _print(self, args: dict) -> dict:
        visitor_id = str(args.get("visitor_id", "")).strip()
        if not _ID_RE.match(visitor_id):
            return {"status": "error", "code": "bad_args", "error": "print 需要 find 返回的 visitor_id"}
        body = {"visitor_id": visitor_id, "reprint": bool(args.get("reprint", False))}
        payload = self._client().request("POST", "/print", body=body)
        status = str(payload.get("status", ""))
        if status == "already_printed":
            return {"status": "already_printed", "visitor_id": visitor_id,
                    "hint": "这位访客今天已经打过访客贴；确实要再打一张时加 reprint=true"}
        job_id = str(payload.get("job_id", ""))
        if not job_id:
            raise BridgeError(str(payload.get("error") or "桥接服务没有返回打印任务编号"), "bad_response")
        job = {"job_id": job_id, "status": status or "queued"}
        wait = args.get("wait", True)
        if wait in (False, "false", 0) or self._cfg["print_wait_s"] <= 0:
            return self._job_result(job, visitor_id)
        deadline = time.monotonic() + self._cfg["print_wait_s"]
        while job["status"] not in DONE_STATES and time.monotonic() < deadline:
            time.sleep(self._poll_interval_s)
            job = self._fetch_job(job_id)
        return self._job_result(job, visitor_id)

    def _job(self, args: dict) -> dict:
        job_id = str(args.get("job_id", "")).strip()
        if not _ID_RE.match(job_id):
            return {"status": "error", "code": "bad_args", "error": "job 需要 print 返回的 job_id"}
        return self._job_result(self._fetch_job(job_id))

    def _fetch_job(self, job_id: str) -> dict:
        payload = self._client().request("GET", f"/jobs/{urllib.parse.quote(job_id, safe='')}")
        return {"job_id": job_id, "status": str(payload.get("status", "")), "error": str(payload.get("error", "") or "")}

    @staticmethod
    def _job_result(job: dict, visitor_id: str | None = None) -> dict:
        state = job.get("status", "")
        result = {"job_id": job["job_id"]}
        if visitor_id:
            result["visitor_id"] = visitor_id
        if state == "done":
            result.update(status="printed", say="访客贴打好了，在前台打印机上")
        elif state in ("failed", "canceled"):
            result.update(status="failed", error=job.get("error") or "打印失败，请前台同事检查打印机")
        else:
            result.update(status="printing", hint="还在打印，稍后用 job 查询结果")
        return result

