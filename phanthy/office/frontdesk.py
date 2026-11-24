"""SSH to the front-desk Windows PC — the robot's only channel to it.

Password authentication through SSH_ASKPASS, so the password is never on a
command line. The host key is trusted on first use and pinned in a file kept
under the data volume, so a container restart does not reset the trust.
"""

from __future__ import annotations

import base64
import html
import json
import os
import re
import subprocess
import tempfile
from contextlib import contextmanager
from pathlib import Path


REMOTE_ROOT = "C:/ProgramData/VisitorBadge"
UPLOAD = f"{REMOTE_ROOT}/upload"
RUN_JOB = f"{REMOTE_ROOT}/app/run_job.ps1"
VERSION_FILE = f"{REMOTE_ROOT}/app/version.txt"
PYTHON = f"{REMOTE_ROOT}/venv/Scripts/python.exe"
PROFILE = f"{REMOTE_ROOT}/app/data/edge-profile"   # worker.py's PROFILE
RESULT_MARKER = "VISITOR_RESULT:"


class FrontDeskError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _text(raw: bytes) -> str:
    text = raw.decode("utf-8", errors="replace")
    if text.count("\ufffd") * 4 > len(text):
        text = raw.decode("gbk", errors="replace")   # zh-CN Windows error output
    if "#< CLIXML" in text:
        # PowerShell over SSH serialises its error stream; keep just the text.
        errors = re.findall(r'<S S="Error">(.*?)</S>', text, re.S)
        rest = re.sub(r"#< CLIXML|<Objs.*?</Objs>", " ", text, flags=re.S)
        text = rest + " " + html.unescape("".join(errors)).replace("_x000D__x000A_", " ")
    return " ".join(text.split())


def _explain(detail: str) -> FrontDeskError:
    low = detail.lower()
    if "remote host identification has changed" in low or "host key verification failed" in low:
        return FrontDeskError("host_key_changed", "前台电脑的 SSH 身份变了（换了电脑或重装了系统？）；确认后删除 known_hosts 再试")
    if "permission denied" in low:
        return FrontDeskError("auth_failed", "Windows 用户名或密码不对")
    if "timed out" in low or "no route to host" in low or "connection refused" in low:
        return FrontDeskError("unreachable", "连不上前台电脑：请检查 IP、网络，以及 Windows 是否开启了 OpenSSH 服务")
    if "visitor_not_admin" in low:
        return FrontDeskError("not_admin", "这个 Windows 账户不是管理员，无法安装")
    if "visitor_not_installed" in low:
        return FrontDeskError("not_installed", "前台电脑还没装上这个版本的自动化")
    if "visitor_timeout" in low:
        return FrontDeskError("worker_timeout", "前台电脑没有按时完成：可能锁屏/未登录桌面、Edge 卡住或打印机故障")
    return FrontDeskError("ssh_failed", "前台电脑执行失败：" + detail[-300:])


class FrontDesk:
    def __init__(self, host: str, user: str, password: str, port: int = 22,
                 known_hosts: str = "/opt/phanthy-motus/data/visitor_badge/known_hosts"):
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", host or "") or not re.fullmatch(r"[^\s@:/]+", user or ""):
            raise FrontDeskError("not_configured", "请在卡片配置里填写前台电脑 IP 和 Windows 用户名")
        if not password:
            raise FrontDeskError("not_configured", "请在卡片配置里填写 Windows 密码")
        self.target = f"{user}@{host}"
        self.password = password
        self.port = int(port)
        self.known_hosts = known_hosts

    def _options(self) -> list[str]:
        try:
            Path(self.known_hosts).parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise FrontDeskError("known_hosts_unwritable", f"无法写入 SSH 主机指纹文件：{exc}") from None
        return ["-F", "/dev/null", "-o", "PubkeyAuthentication=no",
                "-o", "PreferredAuthentications=password,keyboard-interactive",
                "-o", "NumberOfPasswordPrompts=1", "-o", "ConnectTimeout=8",
                "-o", "StrictHostKeyChecking=accept-new", "-o", f"UserKnownHostsFile={self.known_hosts}"]

    @contextmanager
    def _askpass(self):
        with tempfile.NamedTemporaryFile("w", suffix=".sh", delete=False) as script:
            script.write('#!/bin/sh\nprintf "%s\\n" "$VISITOR_SSH_PASSWORD"\n')
        os.chmod(script.name, 0o700)
        try:
            yield {**os.environ, "SSH_ASKPASS": script.name, "SSH_ASKPASS_REQUIRE": "force",
                   "DISPLAY": "none", "VISITOR_SSH_PASSWORD": self.password}
        finally:
            os.unlink(script.name)

    def _run(self, argv: list[str], timeout: float) -> bytes:
        with self._askpass() as env:
            try:
                done = subprocess.run(argv, capture_output=True, timeout=timeout, env=env,
                                      stdin=subprocess.DEVNULL, start_new_session=True)
            except subprocess.TimeoutExpired:
                raise FrontDeskError("timeout", "前台电脑响应超时") from None
        if done.returncode:
            # Both streams: a marker such as VISITOR_NOT_INSTALLED is on stdout
            # while PowerShell may still have written something to stderr.
            raise _explain(_text(done.stderr + b" " + done.stdout))
        return done.stdout

    def powershell(self, script: str, timeout: float = 20) -> str:
        # Encoded, so neither cmd.exe nor PowerShell as the remote default shell
        # gets to reinterpret the quoting.
        # No progress records: over SSH PowerShell writes them to stderr as
        # CLIXML ("preparing modules for first use"), which buried the real
        # outcome of a failed command.
        script = "$ProgressPreference = 'SilentlyContinue'\n" + script
        encoded = base64.b64encode(script.encode("utf-16le")).decode()
        out = self._run(["ssh", "-T", "-p", str(self.port), *self._options(), self.target,
                         f"powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass -EncodedCommand {encoded}"],
                        timeout)
        return out.decode("utf-8", errors="replace")

    def job(self, job: dict, version: str, timeout: float) -> dict:
        """Run one job on the desktop-session worker (windows/run_job.ps1).

        Raises not_installed when the front desk lacks this driver's version
        of the Windows side — absent or older alike.
        """
        encoded = base64.b64encode(json.dumps(job, ensure_ascii=False).encode()).decode()
        out = self.powershell(
            f"$v = Get-Content -Raw -LiteralPath '{VERSION_FILE}' -ErrorAction SilentlyContinue\n"
            f"if (-not $v -or $v.Trim() -ne '{version}') {{ 'VISITOR_NOT_INSTALLED'; exit 2 }}\n"
            f"& '{RUN_JOB}' -Job {encoded} -TimeoutSec {int(timeout)}", timeout + 15)
        lines = [line for line in out.splitlines() if line.startswith(RESULT_MARKER)]
        if len(lines) != 1:
            raise FrontDeskError("bad_response", "前台电脑返回了无法识别的结果")
        return json.loads(base64.b64decode(lines[0][len(RESULT_MARKER):]).decode("utf-8-sig"))

    def install(self, package: Path, version: str, timeout: float = 900) -> None:
        """Copy windows/ (with its deps/) over and run install.ps1."""
        self.powershell(f"Remove-Item -Recurse -Force '{UPLOAD}' -ErrorAction SilentlyContinue; "
                        f"New-Item -ItemType Directory -Force -Path '{UPLOAD}' | Out-Null")
        self._run(["scp", "-r", "-P", str(self.port), *self._options(),
                   *[str(p) for p in sorted(package.iterdir())], f"{self.target}:{UPLOAD}/"], timeout)
        out = self.powershell(f"& '{UPLOAD}/install.ps1' -Version '{version}'", timeout)
        if "VISITOR_INSTALL_OK" not in out:
            raise FrontDeskError("install_failed", "安装没有完成：" + " ".join(out.split())[-300:])

    def pages(self, version: str) -> list[dict]:
        """Pages open or recently opened in the dedicated Edge (visitor_site.recent_pages).

        Raises not_installed when the front desk lacks this version, as job()
        does — otherwise a fix to the page lookup never reaches it.
        """
        out = self.powershell(
            f"$v = Get-Content -Raw -LiteralPath '{VERSION_FILE}' -ErrorAction SilentlyContinue\n"
            f"if (-not $v -or $v.Trim() -ne '{version}') {{ 'VISITOR_NOT_INSTALLED'; exit 2 }}\n"
            f"& '{PYTHON}' '{REMOTE_ROOT}/app/visitor_site.py' pages '{PROFILE}'")
        lines = [line for line in out.splitlines() if line.startswith("VISITOR_PAGES:")]
        return json.loads(lines[-1][len("VISITOR_PAGES:"):]) if lines else []

    def printers(self) -> list[dict]:
        out = self.powershell("[Console]::OutputEncoding=[Text.Encoding]::UTF8; "
                              "ConvertTo-Json -Compress @(Get-CimInstance Win32_Printer | "
                              "Select-Object Name,Default,WorkOffline)")
        return [{"name": p["Name"], "default": bool(p["Default"]), "offline": bool(p["WorkOffline"])}
                for p in json.loads(out or "[]")]
