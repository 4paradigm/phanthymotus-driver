"""Run visitor-badge jobs in the front-desk user's desktop session.

An SSH session has no desktop, so Edge cannot be shown or print from it.
`run_job.ps1` (called over SSH) drops a `<id>.request.json` into data/jobs and
starts the "Visitor Badge Worker" scheduled task, which runs this script in the
interactive session.  It answers every pending request with `<id>.result.json`,
then lingers a few seconds for late requests and exits — nothing listens on the
network.

Job: {"action": "list" | "print" | "login", "page_url", "printer", "name", "host"}
Result: {"ok": true, ...} or {"ok": false, "code", "error"}
"""

from __future__ import annotations

import json
import os
import sys
import time
import traceback
from datetime import date
from pathlib import Path

from visitor_site import ADMITTED, LEFT, NOT_ADMITTED, SiteError, Visit, VisitorSite, open_for_login


BASE = Path(__file__).resolve().parent
DATA = BASE / "data"
JOBS = DATA / "jobs"
PROFILE = DATA / "edge-profile"
IDLE_EXIT_S = 10


def choose(visits: list[Visit], name: str, host: str) -> Visit:
    """The single visit `name` (and `host`, if given) refers to, or SiteError."""
    matches = [v for v in visits if v.name == name and (not host or host in v.host)]
    waiting = [v for v in matches if v.status == NOT_ADMITTED]
    if len(waiting) == 1:
        return waiting[0]
    if len(waiting) > 1:
        raise SiteError("visitor_ambiguous", f"今天有多位叫“{name}”的未入场访客，请告诉我被访人是谁")
    if any(v.status in (ADMITTED, LEFT) for v in matches):
        # Also the answer when a retried print already went through.
        raise SiteError("already_admitted", f"“{name}”今天已经入场，访客贴打过了")
    raise SiteError("visitor_not_found", f"今天的未入场访客里没有“{name}”" + (f"（被访人 {host}）" if host else ""))


class Worker:
    def __init__(self):
        self._site: VisitorSite | None = None

    def site(self, page_url: str) -> VisitorSite:
        if self._site is None or self._site.list_url != page_url:
            self.close()
            self._site = VisitorSite(page_url, PROFILE)
        return self._site

    def close(self) -> None:
        if self._site is not None:
            self._site.close()
            self._site = None

    def run(self, job: dict) -> dict:
        action = job.get("action")
        page_url = str(job.get("page_url") or "")
        today = date.today().isoformat()
        if action == "login":
            if self._site is not None:
                # Let the automation Edge release the profile, or the new
                # window is handed to the exiting browser and never shows.
                self.close()
                time.sleep(2)
            open_for_login(PROFILE, page_url)
            return {"ok": True}
        if action == "list":
            visits = [v for v in self.site(page_url).visits(today) if v.status == NOT_ADMITTED]
            visits.sort(key=lambda v: (v.visit_time, v.name))
            return {"ok": True, "date": today, "visits": [v.public() for v in visits]}
        if action == "print":
            site = self.site(page_url)
            visit = choose(site.visits(today), str(job.get("name") or "").strip(), str(job.get("host") or "").strip())
            status = site.print_badge(visit, str(job.get("printer") or ""), today)
            if status not in (ADMITTED, LEFT):
                return {"ok": False, "code": "print_unconfirmed",
                        "error": "已点击打印，但访客状态没有变为已入场；请到前台核对打印机，不要直接重打"}
            return {"ok": True, "name": visit.name, "host": visit.host}
        raise SiteError("bad_action", f"不支持的动作：{action}")


def main() -> int:
    JOBS.mkdir(parents=True, exist_ok=True)
    log = DATA / "worker.log"
    if log.exists() and log.stat().st_size > 5 * 1024 * 1024:
        os.replace(log, log.with_suffix(".log.1"))
    sys.stdout = sys.stderr = log.open("a", encoding="utf-8", buffering=1)
    worker = Worker()
    idle_since = time.monotonic()
    try:
        while time.monotonic() - idle_since < IDLE_EXIT_S:
            pending = sorted(JOBS.glob("*.request.json"), key=lambda p: p.stat().st_mtime)
            if not pending:
                time.sleep(0.1)
                continue
            request = pending[0]
            job_id = request.name.removesuffix(".request.json")
            job: dict = {}
            try:
                job = json.loads(request.read_text(encoding="utf-8-sig"))
                result = worker.run(job)
            except SiteError as exc:
                result = {"ok": False, "code": exc.code, "error": str(exc)}
            except Exception as exc:
                traceback.print_exc()
                result = {"ok": False, "code": "automation_error", "error": str(exc)[:300]}
            finally:
                request.unlink(missing_ok=True)
            print(f"{time.strftime('%F %T')} {job_id[:8]} {job.get('action')} ok={result['ok']} {result.get('code', '')}")
            out = JOBS / f"{job_id}.result.json"
            tmp = out.with_suffix(".tmp")
            tmp.write_text(json.dumps(result, ensure_ascii=False), encoding="utf-8")
            os.replace(tmp, out)
            idle_since = time.monotonic()
    finally:
        worker.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
