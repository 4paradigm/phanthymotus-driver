"""visitor_badge: look up today's visitors and print their badge at the front desk.

    agent ── visitor_badge ──SSH──> front-desk Windows PC: Edge (visitor site) + printer

The robot holds no visitor data and opens no port; every call is an SSH
command on the front-desk PC (frontdesk.py), which runs the job in the logged-in
user's desktop session (windows/worker.py).
"""

from __future__ import annotations

import hashlib
import json
import os
import ssl
import threading
import time
import urllib.request
import uuid
from pathlib import Path

from common.vendor_runtime import action_schema, tool
from frontdesk import FrontDesk, FrontDeskError


WINDOWS_PACKAGE = Path(__file__).resolve().parent / "windows"
# What the front desk must be running; a mismatch makes the next job reinstall.
PACKAGE_VERSION = hashlib.sha256(b"".join(
    p.read_bytes() for p in sorted(WINDOWS_PACKAGE.iterdir()) if p.is_file())).hexdigest()[:12]
_CONFIG_KEYS = ("host", "user", "password", "port", "page_url", "printer")


def build_plugins(config: dict) -> list:
    return [VisitorBadgePlugin(config.get("plugins", {}).get("visitor_badge", {}))]


class VisitorBadgePlugin:
    PREFIX = "visitor_badge"

    def __init__(self, cfg: dict):
        self._lock = threading.Lock()
        self._cfg = {"host": "", "user": "", "password": "", "port": 22, "page_url": "", "printer": ""}
        self._cfg.update({k: cfg[k] for k in _CONFIG_KEYS if k in cfg})
        self._list_timeout_s = float(cfg.get("list_timeout_s", 45))
        self._print_timeout_s = float(cfg.get("print_timeout_s", 150))
        self._known_hosts = str(cfg.get("known_hosts", "/opt/phanthy-motus/data/visitor_badge/known_hosts"))
        self._printing = False
        self._installer: threading.Thread | None = None
        self._install_error: FrontDeskError | None = None

    def get_tool(self) -> dict:
        actions = {
            "start": ([], "启动"), "stop": ([], "停止"), "info": ([], "配置状态"), "config": (list(_CONFIG_KEYS), "保存配置"),
            "list": ([], "列出今天还没入场的访客（姓名、被访人、时间、手机尾号）"),
            "print": (["name", "host"], "给今天一位未入场访客打印访客贴；同名时用 host（被访人）区分。"
                                        "结果异步返回，失败时把 error 原样告诉用户，不要自行重试"),
            "status": ([], "（管理员）检查前台电脑连接，列出已安装的打印机"),
        }
        schema = action_schema(actions, {
            "name": {"type": "string", "description": "访客姓名"},
            "host": {"type": "string", "description": "被访人姓名，可选"},
        })
        # A first print may also install the front desk (a few minutes).
        schema["x-completion"] = {"actions": ["print"], "timeout": int(self._print_timeout_s) + 300}
        definition = tool(self.PREFIX, "actuator",
                          "前台访客贴打印：查询今天的未入场访客，并在前台打印机上打印访客贴。", schema)
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "host": {"type": "string", "title": "前台电脑 IP", "scope": "shared"},
                "user": {"type": "string", "title": "Windows 用户名（管理员）", "scope": "shared"},
                "password": {"type": "string", "title": "Windows 密码", "scope": "shared",
                             "format": "password", "x-sensitive": True},
                "page_url": {"type": "string", "title": "访客列表网页", "scope": "shared",
                             "description": "在前台 Edge 里打开访客列表后，复制地址栏的网址"},
                "printer": {"type": "string", "title": "打印机", "scope": "shared",
                            "description": "Windows 里的打印机名称；留空用默认打印机。status 会列出可选名称"},
                "port": {"type": "integer", "title": "SSH 端口", "scope": "shared", "default": 22, "advanced": True},
            },
            "required": ["host", "user", "password", "page_url"],
        }
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop", "info"):
            return {"state": "ready" if self._configured() else "unconfigured"}
        if action == "config":
            with self._lock:
                self._cfg.update({k: args[k] for k in _CONFIG_KEYS if k in args})
            return {"status": "ok", "adapter_ok": self._configured()}
        handler = {"list": self._list, "print": self._print, "status": self._status}.get(action)
        if handler is None:
            return {"status": "error", "code": "bad_action", "error": f"不支持的动作：{action}"}
        try:
            return handler(args)
        except FrontDeskError as exc:
            return {"status": "error", "code": exc.code, "error": str(exc)}

    # ── helpers ──────────────────────────────────────────────────────────────

    def _configured(self) -> bool:
        return all(self._cfg[k] for k in ("host", "user", "password", "page_url"))

    def _desk(self, cfg: dict | None = None) -> FrontDesk:
        cfg = cfg or self._snapshot()
        return FrontDesk(cfg["host"], cfg["user"], cfg["password"], int(cfg["port"] or 22), self._known_hosts)

    def _snapshot(self) -> dict:
        with self._lock:
            return dict(self._cfg)

    def _job(self, cfg: dict, deadline: float, job_timeout: float, **job) -> dict:
        """Run a job, installing or upgrading the front desk first when needed."""
        desk = self._desk(cfg)
        job = {"page_url": cfg["page_url"], "printer": cfg["printer"], **job}
        remaining = lambda: max(10.0, min(job_timeout, deadline - time.monotonic()))
        try:
            result = desk.job(job, PACKAGE_VERSION, remaining())
        except FrontDeskError as exc:
            if exc.code != "not_installed":
                raise
            self._install(cfg, deadline - time.monotonic() - 20)
            result = desk.job(job, PACKAGE_VERSION, remaining())
        if result.get("code") == "login_required":
            # Put the login window in front of the operator; it is left alone
            # if already open, so a retry does not interrupt a login.
            try:
                desk.job({"action": "login", "page_url": cfg["page_url"]}, PACKAGE_VERSION, 20)
            except FrontDeskError:
                pass
            raise FrontDeskError("login_required", "访客系统需要登录：已在前台电脑打开 Edge，请前台登录后再试")
        if not result.get("ok"):
            raise FrontDeskError(str(result.get("code") or "failed"), str(result.get("error") or "前台电脑执行失败"))
        return result

    def _install(self, cfg: dict, wait_s: float) -> None:
        """Install the Windows side once, shared by every caller that needs it."""
        with self._lock:
            if self._installer is None or not self._installer.is_alive():
                self._install_error = None
                self._installer = threading.Thread(target=self._run_install, args=(cfg,),
                                                   name="visitor_install", daemon=True)
                self._installer.start()
            installer = self._installer
        installer.join(max(0.0, wait_s))
        if installer.is_alive():
            raise FrontDeskError("preparing", "正在首次准备前台电脑（约 2–3 分钟），请稍后再试")
        if self._install_error is not None:
            raise self._install_error

    def _run_install(self, cfg: dict) -> None:
        print(f"[visitor_badge] installing front desk {PACKAGE_VERSION}", flush=True)
        try:
            self._desk(cfg).install(WINDOWS_PACKAGE, PACKAGE_VERSION)
            print("[visitor_badge] front desk installed", flush=True)
        except Exception as exc:   # noqa: BLE001 — reported to whoever waits on it
            self._install_error = exc if isinstance(exc, FrontDeskError) else \
                FrontDeskError("install_failed", f"前台电脑安装失败：{str(exc)[:200]}")
            print(f"[visitor_badge] install failed: {self._install_error.code}", flush=True)

    # ── actions ──────────────────────────────────────────────────────────────

    def _list(self, args: dict) -> dict:
        # Synchronous: must answer inside Agent Core's 60 s limit for an actuator.
        result = self._job(self._snapshot(), time.monotonic() + self._list_timeout_s, self._list_timeout_s,
                           action="list")
        return {"status": "listed", "date": result["date"], "count": len(result["visits"]),
                "visits": result["visits"]}

    def _print(self, args: dict) -> dict:
        name = str(args.get("name") or "").strip()
        host = str(args.get("host") or "").strip()
        if not name:
            return {"status": "error", "code": "bad_args", "error": "需要访客姓名 name"}
        if not self._configured():
            return {"status": "error", "code": "not_configured", "error": "请先在卡片配置里填写前台电脑和访客网页"}
        cfg = self._snapshot()   # a print keeps the settings it was accepted with
        with self._lock:
            if self._printing:
                return {"status": "error", "code": "busy", "error": "上一张访客贴还在打印，请等它完成"}
            self._printing = True
        action_id = f"visitor_print_{uuid.uuid4().hex[:8]}"

        def run():
            try:
                # Asynchronous, so it may wait out a first-time install.
                deadline = time.monotonic() + self._print_timeout_s + 300
                result = {"status": "printed", **self._job(cfg, deadline, self._print_timeout_s,
                                                           action="print", name=name, host=host)}
                result.pop("ok", None)
                status = "completed"
            except FrontDeskError as exc:
                result, status = {"status": "failed", "code": exc.code, "error": str(exc)}, "error"
            except Exception as exc:   # noqa: BLE001 — the barrier must always be released
                result, status = {"status": "failed", "code": "internal_error", "error": str(exc)[:200]}, "error"
            finally:
                with self._lock:
                    self._printing = False
            _acp_complete(action_id, status, result)

        threading.Thread(target=run, name=action_id, daemon=True).start()
        return {"status": "printing", "action_id": action_id}

    def _status(self, args: dict) -> dict:
        return {"configured": self._configured(), "printers": self._desk().printers()}


def _acp_complete(action_id: str, status: str, result: dict) -> None:
    """Report an async action's outcome to Agent Core (README_dev § ACP)."""
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/") + "/api/acp/complete"
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    body = json.dumps({"action_id": action_id, "status": status, "result": result,
                       "tool": VisitorBadgePlugin.PREFIX, "ts": time.time()}, ensure_ascii=False).encode()
    try:
        request = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(request, timeout=5, context=ctx).close()
    except Exception as exc:   # noqa: BLE001
        print(f"[ACP] callback failed for {action_id}: {exc}", flush=True)
