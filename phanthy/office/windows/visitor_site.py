"""Drive the visitor-management website in a dedicated Edge profile.

This is the only file that knows the visitor site.  It reads visits through the
site's own query API (replaying the logged-in page's auth headers) and prints a
badge by clicking the site's "打印访客贴" action, with Edge in kiosk-printing
mode so the job goes straight to the Windows default printer.

Runs on the front-desk PC, inside the interactive user's session (see worker.py).
Only the standard library is imported at module level; Playwright is imported
when a browser is actually needed.
"""

from __future__ import annotations

import json
import re
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit


QUERY_PATH = "/smart-admin-api/visitInfo/page/query"
NOT_ADMITTED, ADMITTED, LEFT = "未入场", "已入场", "已离场"
STATUSES = (NOT_ADMITTED, ADMITTED, LEFT)


class SiteError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class Visit:
    visit_id: str
    name: str
    phone_tail: str = ""
    host: str = ""
    visit_time: str = ""
    reason: str = ""
    status: str = ""
    booking_code: str = ""

    def public(self) -> dict:
        return {"name": self.name, "host": self.host, "time": self.visit_time,
                "reason": self.reason, "tail": self.phone_tail, "status": self.status}


# ── Parsing the query response ───────────────────────────────────────────────

_ALIASES = {
    "id": ("visitId", "visitorId", "visitorInfoId", "id", "code", "number"),
    "name": ("visitorName", "visitName", "name"),
    "phone": ("visitorPhone", "phone", "mobile", "contactWay", "contact"),
    "host": ("bookingPerson", "hostName", "receiverName", "interviewedName", "reserveName",
             "inviteName", "employeeName"),
    "time": ("visitStartDate", "visitTime", "arriveTime", "appointmentTime", "reserveTime", "startTime"),
    "reason": ("visitReasonName", "reasonName", "visitReason", "reason"),
    "status": ("visitorStatusName", "statusName", "visitStatusName", "visitorStatus", "status"),
}
_STATUS_CODES = {"0": NOT_ADMITTED, "1": ADMITTED, "2": LEFT}


def _pick(node: dict, keys: tuple[str, ...]) -> str:
    for key in keys:
        value = node.get(key)
        if value not in (None, "") and not isinstance(value, (dict, list)):
            return str(value).strip()
    return ""


def _status(value: str) -> str:
    return _STATUS_CODES.get(value, value)


def extract_visits(payload: Any) -> list[Visit]:
    """Every visitor row in a query response.

    A booking (which carries the host, time and booking code) holds its guests
    in `visitGuestVOList`; guest fields override what they inherit.  Only the
    last four digits of a phone number are kept.
    """
    found: dict[tuple[str, str, str], Visit] = {}

    def walk(node: Any, inherited: dict[str, str]) -> None:
        if isinstance(node, list):
            for item in node:
                walk(item, inherited)
            return
        if not isinstance(node, dict):
            return
        context = dict(inherited)
        if isinstance(node.get("visitGuestVOList"), list):
            context["booking_code"] = _pick(node, ("code",))
        for field in ("host", "time", "reason", "status", "id"):
            value = _pick(node, _ALIASES[field])
            if value:
                context[field] = _status(value) if field == "status" else value
        name = _pick(node, _ALIASES["name"])
        if name and context.get("id"):
            visit = Visit(
                visit_id=context["id"], name=name,
                phone_tail=_pick(node, _ALIASES["phone"])[-4:],
                host=context.get("host", ""), visit_time=context.get("time", ""),
                reason=context.get("reason", ""), status=context.get("status", ""),
                booking_code=context.get("booking_code", ""),
            )
            found[(visit.visit_id, visit.name, visit.phone_tail)] = visit
        for child in node.values():
            if isinstance(child, (dict, list)):
                walk(child, context)

    walk(payload, {})
    return list(found.values())


def auth_headers(headers: dict[str, str]) -> dict[str, str]:
    """The application's own auth headers, never cookies or transport headers."""
    return {str(k).lower(): str(v) for k, v in headers.items()
            if str(k).lower() != "cookie"
            and any(m in str(k).lower() for m in ("authorization", "token", "csrf", "xsrf", "tenant"))}


# Installed into every page: the site's print page calls window.print() as soon
# as it loads, before its images and fonts are in, and closes itself right after.
# Delay the print until the badge has rendered, record progress in
# window.__badgePrint, and hold the close until afterprint.
_PRINT_READY_JS = """
(() => {
  if (window.__badgePrintInstalled) return;
  window.__badgePrintInstalled = true;
  const nativePrint = window.print.bind(window), nativeClose = window.close.bind(window);
  let closeWanted = false;
  window.__badgePrint = 'idle';
  window.addEventListener('afterprint', () => {
    window.__badgePrint = 'complete';
    if (closeWanted) setTimeout(nativeClose, 2000);
  });
  window.close = () => {
    if (window.__badgePrint === 'preparing' || window.__badgePrint === 'submitted') { closeWanted = true; return; }
    nativeClose();
  };
  window.print = () => {
    if (window.__badgePrint === 'preparing') return;
    window.__badgePrint = 'preparing';
    const images = Array.from(document.images || []).map(img => img.complete ? null :
      new Promise(done => { img.addEventListener('load', done, {once: true});
                            img.addEventListener('error', done, {once: true}); }));
    const fonts = document.fonts && document.fonts.ready ? document.fonts.ready.catch(() => {}) : null;
    Promise.race([Promise.all([...images, fonts]), new Promise(done => setTimeout(done, 5000))])
      .then(() => new Promise(done => requestAnimationFrame(() => requestAnimationFrame(done))))
      .then(() => new Promise(done => setTimeout(done, 1500)))
      .then(() => { window.__badgePrint = 'submitted'; nativePrint(); });
  };
})();
"""


def _powershell(script: str, timeout: float = 10) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command",
         "$ErrorActionPreference='Stop'; [Console]::OutputEncoding=[Text.Encoding]::UTF8; " + script],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout,
        check=False, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))


def use_printer(name: str) -> None:
    """Make `name` the Windows default printer; kiosk printing always uses the default."""
    if not name:
        return
    quoted = name.replace("'", "''")
    default = "(Get-CimInstance Win32_Printer | Where-Object Default | Select-Object -First 1).Name"
    check = _powershell(f"if (-not (Get-CimInstance Win32_Printer | Where-Object Name -eq '{quoted}')) {{ exit 3 }}; {default}")
    if check.returncode == 3:
        raise SiteError("printer_not_installed", f"前台电脑没有安装打印机“{name}”")
    if check.returncode:
        raise SiteError("printer_check_failed", "无法读取 Windows 打印机：" + (check.stderr or check.stdout).strip()[:200])
    if check.stdout.strip() == name:
        return
    # PrintUI, not WScript.Network: the latter can hang without a desktop.
    subprocess.run(["rundll32.exe", "printui.dll,PrintUIEntry", "/y", "/n", name], timeout=10, check=False)
    time.sleep(0.75)
    if _powershell(default).stdout.strip() != name:
        raise SiteError("printer_switch_failed", f"无法把默认打印机切换为“{name}”")


def close_profile_edge(profile_dir: Path) -> None:
    """Close any Edge window using this profile (e.g. the one left open for login).

    Playwright cannot open a profile another Edge process holds.
    """
    path = str(profile_dir).replace("'", "''")
    _powershell(
        f"$p = Get-CimInstance Win32_Process -Filter \"Name='msedge.exe'\" | "
        f"Where-Object {{ $_.CommandLine -and $_.CommandLine.IndexOf('{path}', [StringComparison]::OrdinalIgnoreCase) -ge 0 }}; "
        "$p | ForEach-Object { (Get-Process -Id $_.ProcessId -ErrorAction SilentlyContinue).CloseMainWindow() } | Out-Null; "
        "if ($p) { Start-Sleep 3; $p | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue } }",
        timeout=20)


def login_window_open(profile_dir: Path) -> bool:
    """Is a login window (ours, not Playwright's) open on this profile?

    Counts browser main processes only (no --type=), and not Playwright's own,
    which it launches with --remote-debugging-pipe.
    """
    path = str(profile_dir).replace("'", "''")
    found = _powershell(
        f"@(Get-CimInstance Win32_Process -Filter \"Name='msedge.exe'\" | Where-Object {{ $_.CommandLine -and "
        f"$_.CommandLine.IndexOf('{path}', [StringComparison]::OrdinalIgnoreCase) -ge 0 -and "
        "$_.CommandLine -notmatch '--type=|--remote-debugging-pipe' }).Count")
    return found.stdout.strip() not in ("", "0")


def open_for_login(profile_dir: Path, url: str) -> None:
    """Open a plain Edge window on the dedicated profile for the operator to log in.

    Left alone if one is already open, so a repeated request does not close a
    login in progress.
    """
    if login_window_open(profile_dir):
        return
    subprocess.Popen(["cmd.exe", "/c", "start", "", "msedge", f"--user-data-dir={profile_dir}",
                      "--no-first-run", url or "about:blank"],
                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))


def recent_pages(profile_dir: Path, limit: int = 10) -> list[dict]:
    """Web pages recently opened in the dedicated Edge, newest first.

    These are what the card offers for 访客列表网页: the operator logs in
    there and opens the visitor list, then picks it. Read from a copy of the
    profile's History database, which Edge keeps open while it runs.
    """
    history = profile_dir / "Default" / "History"
    if not history.is_file():
        return []
    with tempfile.TemporaryDirectory() as tmp:
        copy = Path(tmp) / "History"
        shutil.copyfile(history, copy)
        with closing(sqlite3.connect(copy)) as db:
            rows = db.execute("SELECT url, title FROM urls ORDER BY last_visit_time DESC LIMIT 200").fetchall()
    pages = []
    for url, title in rows:
        if urlsplit(url).scheme not in ("http", "https") or "login" in url.lower():
            continue
        if url not in (p["url"] for p in pages):
            pages.append({"url": url, "title": title or ""})
        if len(pages) == limit:
            break
    return pages


class VisitorSite:
    def __init__(self, list_url: str, profile_dir: Path):
        parts = urlsplit(list_url)
        if parts.scheme not in ("http", "https") or not parts.netloc:
            raise SiteError("page_url_required", "请在卡片配置中填写访客列表网页地址")
        self.list_url = list_url
        self.query_url = f"{parts.scheme}://{parts.netloc}{QUERY_PATH}"
        self.profile_dir = profile_dir
        self._pw = self._context = self._page = None

    # ── Browser ──────────────────────────────────────────────────────────────

    def _browser(self):
        if self._page is not None and not self._page.is_closed():
            return self._page
        if self._context is None:
            from playwright.sync_api import sync_playwright
            close_profile_edge(self.profile_dir)
            self._pw = sync_playwright().start()
            try:
                self._context = self._pw.chromium.launch_persistent_context(
                    str(self.profile_dir), channel="msedge", headless=False, accept_downloads=False,
                    args=["--no-first-run", "--disable-session-crashed-bubble", "--kiosk-printing"])
            except Exception as exc:
                raise SiteError("edge_start_failed", f"无法启动专用 Edge：{exc}") from exc
            self._context.add_init_script(script=_PRINT_READY_JS)
        self._page = next((p for p in self._context.pages if not p.is_closed()), None) or self._context.new_page()
        return self._page

    def close(self) -> None:
        try:
            if self._context is not None:
                self._context.close()
            if self._pw is not None:
                self._pw.stop()
        except Exception:
            pass
        self._pw = self._context = self._page = None

    @staticmethod
    def _check_login(page) -> None:
        if "login" in page.url.lower():
            raise SiteError("login_required", "访客系统需要登录")

    def _load_list(self, page) -> dict[str, str]:
        """Open the list page and capture the headers of its own visit query."""
        from playwright.sync_api import TimeoutError as Timeout

        captured = []
        observe = lambda r: captured.append(r) if QUERY_PATH in r.url and r.method == "POST" else None
        page.on("request", observe)
        try:
            try:
                if page.url == self.list_url:
                    response = page.reload(wait_until="domcontentloaded", timeout=10000)
                else:
                    response = page.goto(self.list_url, wait_until="domcontentloaded", timeout=10000)
            except Timeout as exc:
                self._check_login(page)
                raise SiteError("page_load_timeout", "访客网页加载超时，请检查前台电脑网络") from exc
            self._check_login(page)
            if response is not None and response.status >= 400:
                raise SiteError("page_http_error", f"访客网页返回 HTTP {response.status}")
            if not captured:
                # The list may wait for a click on its search button.
                button = page.get_by_role("button", name=re.compile("查询|搜索")).first
                try:
                    with page.expect_request(lambda r: QUERY_PATH in r.url and r.method == "POST",
                                             timeout=5000) as request:
                        button.click(timeout=3000)
                    captured.append(request.value)
                except Timeout as exc:
                    self._check_login(page)
                    raise SiteError("query_not_found", "访客网页上没有找到访客查询，请确认填写的是访客列表页地址") from exc
            return auth_headers(captured[-1].all_headers())
        finally:
            page.remove_listener("request", observe)

    def _query(self, day: str) -> tuple[Any, list[Visit]]:
        page = self._browser()
        headers = self._load_list(page)
        body = {"bookingPerson": "", "contacts": "", "orders": [{"asc": False, "column": "visit_start_date"}],
                "pageNum": 1, "pageSize": 100, "reasonId": None, "status": None,
                "visitTimeBegin": f"{day} 00:00:00", "visitTimeEnd": f"{day} 23:59:59"}
        result = page.evaluate("""async ([url, body, headers]) => {
            const r = await fetch(url, {method: 'POST', credentials: 'include',
              headers: {'Accept': 'application/json', 'Content-Type': 'application/json', ...headers},
              body: JSON.stringify(body)});
            return {status: r.status, url: r.url, text: await r.text()};
        }""", [self.query_url, body, headers])
        if result["status"] in (401, 403) or "login" in result["url"].lower():
            raise SiteError("login_required", "访客系统登录已失效，请在前台电脑的 Edge 里重新登录")
        if result["status"] >= 400:
            raise SiteError("query_http_error", f"访客查询返回 HTTP {result['status']}")
        try:
            payload = json.loads(result["text"].lstrip("﻿"))
        except ValueError as exc:
            raise SiteError("query_not_json", "访客查询没有返回有效数据") from exc
        if isinstance(payload, dict) and (str(payload.get("code")).lower() in ("401", "403", "unauthorized")
                                          or "登录" in str(payload.get("msg") or payload.get("message") or "")):
            raise SiteError("login_required", "访客系统登录已失效，请在前台电脑的 Edge 里重新登录")
        return page, extract_visits(payload)

    def visits(self, day: str) -> list[Visit]:
        return self._query(day)[1]

    # ── Printing ─────────────────────────────────────────────────────────────

    @staticmethod
    def _rows_with(page, text: str) -> list:
        rows = page.locator("tr")
        return [rows.nth(i) for i in range(rows.count()) if rows.nth(i).get_by_text(text, exact=True).count()]

    def _guest_row(self, page, visit: Visit):
        rows = [r for r in self._rows_with(page, visit.name)
                if not visit.phone_tail or visit.phone_tail in r.inner_text(timeout=1000)]
        if len(rows) > 1:
            raise SiteError("visitor_ambiguous", "预约里有同名且手机号尾号相同的访客，不能自动打印")
        return rows[0] if rows else None

    def _open_guest_row(self, page, visit: Visit):
        """Booking row → its "查看" detail → the guest's own row."""
        if not visit.booking_code:
            raise SiteError("booking_code_missing", "查询结果缺少预约编号，无法定位访客")
        bookings = self._rows_with(page, visit.booking_code)
        if len(bookings) != 1:
            raise SiteError("booking_row_missing", "访客列表里找不到唯一的对应预约，请检查列表筛选条件")
        bookings[0].get_by_text("查看", exact=True).click(timeout=5000)
        for _ in range(10):
            row = self._guest_row(page, visit)
            if row is not None:
                return row
            page.wait_for_timeout(500)
        raise SiteError("visitor_row_missing", "打开了预约详情，但找不到这位访客")

    def print_badge(self, visit: Visit, printer: str, day: str) -> str:
        """Click "打印访客贴" for `visit`; return its status afterwards."""
        use_printer(printer)
        page, _ = self._query(day)
        row = self._open_guest_row(page, visit)
        if NOT_ADMITTED not in row.inner_text(timeout=1000):
            raise SiteError("status_not_printable", "访客已不是未入场状态，没有打印")
        action = row.get_by_text("打印访客贴", exact=True)
        if not action.count():
            raise SiteError("print_action_missing", "页面上没有“打印访客贴”操作")
        before = set(self._context.pages)
        action.click(timeout=5000)
        # Whether or not the print page reported progress, the refreshed
        # status is what says the site took the print.
        self._wait_print_submitted(page, before)
        status = self._refreshed_status(page, visit)
        for extra in set(self._context.pages) - before:
            try:
                extra.close()
            except Exception:
                pass
        return status

    def _wait_print_submitted(self, page, before: set, seconds: float = 45) -> bool:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            pages = [p for p in self._context.pages if not p.is_closed() and p not in before] + [page]
            for candidate in pages:
                try:
                    if candidate.evaluate("window.__badgePrint || 'idle'") in ("submitted", "complete"):
                        return True
                except Exception:
                    continue
            page.wait_for_timeout(200)
        return False

    def _refreshed_status(self, page, visit: Visit, seconds: float = 15) -> str:
        """The list shows a stale status until reloaded; poll until it moves."""
        status = ""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                page.reload(wait_until="domcontentloaded")
                row = self._guest_row(page, visit) or self._open_guest_row(page, visit)
                text = row.inner_text(timeout=2000)
                status = next((s for s in (ADMITTED, LEFT, NOT_ADMITTED) if s in text), "")
            except Exception:
                status = ""
            if status in (ADMITTED, LEFT):
                return status
            page.wait_for_timeout(1500)
        return status


if __name__ == "__main__":
    # `python visitor_site.py pages <profile>`: the driver lists pages over SSH,
    # outside the desktop session; ASCII JSON, whatever the console code page.
    if sys.argv[1:2] == ["pages"]:
        print("VISITOR_PAGES:" + json.dumps(recent_pages(Path(sys.argv[2]))))
