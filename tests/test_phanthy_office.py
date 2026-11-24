"""visitor_badge driver: card contract, async print over ACP, visit matching.

No SSH, no Windows and no Edge: the front desk is faked at FrontDesk.job, and
the Windows-side matching/parsing is exercised as plain functions.
"""

from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "phanthy" / "office"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))   # for `common`


def _load(name: str, path: Path):
    # Loaded by file and registered under its import name, without putting the
    # driver directory on sys.path: several drivers have a device.py, and other
    # tests' bare `import device` must not find this one.
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


frontdesk = _load("frontdesk", DRIVER / "frontdesk.py")
visitor_site = _load("visitor_site", DRIVER / "windows" / "visitor_site.py")
device = _load("visitor_badge_device", DRIVER / "device.py")
worker = _load("visitor_badge_worker", DRIVER / "windows" / "worker.py")


CONFIG = {"host": "10.0.0.5", "user": "desk", "password": "pw", "page_url": "https://visit.example/list"}


@pytest.fixture
def plugin(monkeypatch):
    calls, completions = [], []
    done = threading.Event()

    def fake_job(self, job, version, timeout):
        assert version == device.PACKAGE_VERSION
        calls.append(job)
        return plugin.reply(job)

    def fake_acp(action_id, status, result):
        completions.append((action_id, status, result))
        done.set()

    monkeypatch.setattr(frontdesk.FrontDesk, "job", fake_job)
    monkeypatch.setattr(device, "_acp_complete", fake_acp)
    plugin = device.VisitorBadgePlugin({})
    plugin.dispatch("config", dict(CONFIG))
    plugin.calls, plugin.completions, plugin.done = calls, completions, done
    plugin.reply = lambda job: {"ok": True}
    return plugin


def test_card_contract():
    definition = device.VisitorBadgePlugin({}).get_tool()
    assert definition["name"] == "visitor_badge" and definition["type"] == "actuator"
    schema = definition["inputSchema"]
    assert schema["x-completion"]["actions"] == ["print"]
    assert set(schema["x-action-params"]) >= {"list", "print", "detect"}
    assert {o["const"]: o["title"] for o in schema["properties"]["action"]["oneOf"]}["detect"] == "检测前台电脑"
    assert "setup" not in schema["x-action-params"]   # installing is the driver's business
    config = definition["configSchema"]
    assert config["properties"]["password"]["x-sensitive"] is True
    # The page is picked from what the front desk reports, after the connection is saved.
    assert set(config["required"]) == {"host", "user", "password"}
    assert config["properties"]["page_url"]["oneOf"] == config["properties"]["printer"]["oneOf"] == []
    # Nothing is detected on its own; the form says how to get choices.
    assert "检测前台电脑" in config["properties"]["page_url"]["description"]


def _form(plugin):
    props = plugin.get_tool()["configSchema"]["properties"]
    return {k: v["oneOf"] for k, v in props.items() if "oneOf" in v}, props["page_url"]["description"]


def _front_desk(monkeypatch, printers, pages):
    monkeypatch.setattr(frontdesk.FrontDesk, "printers", lambda self: printers)
    monkeypatch.setattr(frontdesk.FrontDesk, "pages", lambda self: pages)


def test_nothing_reaches_the_front_desk_until_detect_is_run(plugin, monkeypatch):
    monkeypatch.setattr(frontdesk.FrontDesk, "printers", lambda self: pytest.fail("contacted"))
    plugin.get_tool()
    assert plugin.calls == []


def test_page_and_printer_are_chosen_from_what_detect_found(plugin, monkeypatch):
    _front_desk(monkeypatch, [{"name": "QL-820NWB", "default": False, "offline": False},
                              {"name": "Microsoft Print to PDF", "default": True, "offline": False}],
                [{"url": "https://visit.example/#/visitor/list", "title": "访客列表"}])
    result = plugin.dispatch("detect", {})
    assert result["status"] == "ok" and result["printers"] == ["QL-820NWB", "Microsoft Print to PDF"]
    choices, note = _form(plugin)
    assert choices["printer"] == [{"const": "QL-820NWB", "title": "QL-820NWB"},
                                  {"const": "Microsoft Print to PDF", "title": "Microsoft Print to PDF（系统默认）"}]
    # The saved page stays selectable even though this round did not see it.
    assert choices["page_url"] == [
        {"const": "https://visit.example/#/visitor/list", "title": "访客列表 — https://visit.example/#/visitor/list"},
        {"const": CONFIG["page_url"], "title": f"{CONFIG['page_url']}（当前设置，本次未找到）"}]
    assert "找到 2 台打印机、1 个网页" in note
    assert plugin.calls == []   # pages exist: no login window

    plugin.dispatch("config", {"host": "10.0.0.6"})
    choices, note = _form(plugin)
    assert choices["printer"] == [] and "还没检测" in note


def test_detect_installs_first_then_opens_the_login_window(plugin, monkeypatch):
    plugin.dispatch("config", {"page_url": ""})
    installed = threading.Event()
    monkeypatch.setattr(frontdesk.FrontDesk, "printers", lambda self: [])
    monkeypatch.setattr(frontdesk.FrontDesk, "install", lambda self, *a, **k: installed.wait(5))

    def pages(self):
        if not installed.is_set():
            raise frontdesk.FrontDeskError("not_installed", "")
        return []

    monkeypatch.setattr(frontdesk.FrontDesk, "pages", pages)
    result = plugin.dispatch("detect", {})
    assert result["status"] == "error" and "正在首次安装" in result["message"]
    assert "正在首次安装" in _form(plugin)[1]
    installed.set()
    plugin._installer.join(5)
    result = plugin.dispatch("detect", {})
    assert result["status"] == "ok" and "已在前台电脑打开 Edge" in result["message"]
    assert plugin.calls == [{"action": "login", "page_url": ""}]


@pytest.mark.parametrize("code", ["auth_failed", "unreachable"])
def test_detect_shows_why_the_front_desk_cannot_be_reached(plugin, monkeypatch, code):
    def fail(self):
        raise frontdesk.FrontDeskError(code, "说明" + code)

    monkeypatch.setattr(frontdesk.FrontDesk, "printers", fail)
    result = plugin.dispatch("detect", {})
    assert result["status"] == "error" and result["message"].endswith("说明" + code)
    assert "说明" + code in _form(plugin)[1]


def test_detect_needs_the_connection_settings():
    result = device.VisitorBadgePlugin({}).dispatch("detect", {})
    assert result["code"] == "not_configured"


def test_list_needs_a_chosen_page(plugin):
    plugin.dispatch("config", {"page_url": ""})
    assert plugin.dispatch("list", {})["code"] == "not_configured" and plugin.calls == []


def test_recent_pages_reads_the_edge_history_newest_first(tmp_path):
    import sqlite3

    (tmp_path / "Default").mkdir()
    db = sqlite3.connect(tmp_path / "Default" / "History")
    db.execute("CREATE TABLE urls (url TEXT, title TEXT, last_visit_time INTEGER)")
    db.executemany("INSERT INTO urls VALUES (?, ?, ?)", [
        ("https://visit.example/#/visitor/list", "访客列表", 3),
        ("https://visit.example/#/login", "登录", 4),
        ("edge://newtab/", "", 5),
        ("https://visit.example/#/home", None, 2)])
    db.commit()
    db.close()
    assert visitor_site.recent_pages(tmp_path) == [
        {"url": "https://visit.example/#/visitor/list", "title": "访客列表"},
        {"url": "https://visit.example/#/home", "title": ""}]
    assert visitor_site.recent_pages(tmp_path / "missing") == []


def test_config_reports_adapter_ok_once_the_connection_is_complete():
    plugin = device.VisitorBadgePlugin({})
    assert plugin.dispatch("config", {"host": "10.0.0.5"})["adapter_ok"] is False
    # No page yet, and still ok: Agent Core blocks every action, detect included,
    # on adapter_ok false — and detect is how the page gets chosen.
    no_page = {k: v for k, v in CONFIG.items() if k != "page_url"}
    assert plugin.dispatch("config", no_page)["adapter_ok"] is True


def test_print_needs_configuration_before_going_async():
    result = device.VisitorBadgePlugin({}).dispatch("print", {"name": "张三"})
    assert result["code"] == "not_configured"


def test_print_completes_over_acp_with_the_settings_it_was_accepted_with(plugin):
    gate = threading.Event()
    plugin.reply = lambda job: (gate.wait(5), {"ok": True, "name": job["name"], "host": "李四"})[1]
    accepted = plugin.dispatch("print", {"name": "张三", "host": "李四"})
    assert accepted["status"] == "printing"
    plugin.dispatch("config", {"printer": "Brother"})   # must not redirect the accepted job
    assert plugin.dispatch("print", {"name": "王五"})["code"] == "busy"
    gate.set()
    assert plugin.done.wait(5)
    action_id, status, result = plugin.completions[0]
    assert action_id == accepted["action_id"] and status == "completed"
    assert result == {"status": "printed", "name": "张三", "host": "李四"}
    assert plugin.calls[0] == {"action": "print", "name": "张三", "host": "李四",
                               "page_url": CONFIG["page_url"], "printer": ""}


def test_print_failure_reaches_the_agent_with_the_worker_reason(plugin):
    plugin.reply = lambda job: {"ok": False, "code": "already_admitted", "error": "已经入场"}
    plugin.dispatch("print", {"name": "张三"})
    assert plugin.done.wait(5)
    _, status, result = plugin.completions[0]
    assert status == "error" and result["code"] == "already_admitted"
    assert plugin.dispatch("print", {"name": "张三"})["status"] == "printing"   # busy flag released


def test_list(plugin):
    plugin.reply = lambda job: {"ok": True, "date": "2026-11-24", "visits": [{"name": "张三"}]}
    assert plugin.dispatch("list", {}) == {"status": "listed", "date": "2026-11-24", "count": 1,
                                           "visits": [{"name": "张三"}]}


def test_a_missing_or_old_front_desk_is_installed_then_the_job_runs(plugin, monkeypatch):
    installs = []
    monkeypatch.setattr(frontdesk.FrontDesk, "install",
                        lambda self, package, version, timeout=900: installs.append(version))
    answers = iter([frontdesk.FrontDeskError("not_installed", "x"),
                    {"ok": True, "date": "2026-11-24", "visits": []}])

    def reply(job):
        answer = next(answers)
        if isinstance(answer, Exception):
            raise answer
        return answer

    plugin.reply = reply
    assert plugin.dispatch("list", {})["status"] == "listed"
    assert installs == [device.PACKAGE_VERSION] and len(plugin.calls) == 2


def test_list_does_not_outwait_a_first_install(plugin, monkeypatch):
    release = threading.Event()
    monkeypatch.setattr(frontdesk.FrontDesk, "install", lambda self, *a, **k: release.wait(5))
    monkeypatch.setattr(plugin, "_list_timeout_s", 20.5)   # leaves 0.5 s to wait on the install
    plugin.reply = lambda job: (_ for _ in ()).throw(frontdesk.FrontDeskError("not_installed", "x"))
    assert plugin.dispatch("list", {})["code"] == "preparing"
    release.set()


def test_login_required_opens_the_login_window_and_says_so(plugin):
    plugin.reply = lambda job: {"ok": True} if job["action"] == "login" else \
        {"ok": False, "code": "login_required", "error": "访客系统需要登录"}
    result = plugin.dispatch("list", {})
    assert result["code"] == "login_required" and "前台" in result["error"]
    assert [c["action"] for c in plugin.calls] == ["list", "login"]


def _visit(name, status, host="李四", tail="1234"):
    return visitor_site.Visit(visit_id=name + status, name=name, host=host, status=status, phone_tail=tail)


def test_choose_visit():
    waiting, admitted = visitor_site.NOT_ADMITTED, visitor_site.ADMITTED
    assert worker.choose([_visit("张三", waiting)], "张三", "").name == "张三"
    with pytest.raises(visitor_site.SiteError) as ambiguous:
        worker.choose([_visit("张三", waiting), _visit("张三", waiting, host="王五")], "张三", "")
    assert ambiguous.value.code == "visitor_ambiguous"
    assert worker.choose([_visit("张三", waiting), _visit("张三", waiting, host="王五")], "张三", "王").host == "王五"
    with pytest.raises(visitor_site.SiteError) as done:
        worker.choose([_visit("张三", admitted)], "张三", "")
    assert done.value.code == "already_admitted"
    with pytest.raises(visitor_site.SiteError) as missing:
        worker.choose([_visit("张三", waiting)], "赵六", "")
    assert missing.value.code == "visitor_not_found"


def test_extract_visits_inherits_booking_fields_and_keeps_only_the_phone_tail():
    payload = {"data": {"list": [{
        "code": "B001", "bookingPerson": "李四", "visitStartDate": "2026-11-24 10:00", "visitorStatus": "0",
        "visitGuestVOList": [{"id": "g1", "visitorName": "张三", "visitorPhone": "13800001234"}],
    }]}}
    [visit] = visitor_site.extract_visits(payload)
    assert (visit.name, visit.host, visit.visit_time, visit.status, visit.booking_code, visit.phone_tail) == \
        ("张三", "李四", "2026-11-24 10:00", "未入场", "B001", "1234")


def test_frontdesk_rejects_incomplete_connection_settings():
    with pytest.raises(frontdesk.FrontDeskError) as error:
        frontdesk.FrontDesk("10.0.0.5; rm -rf /", "desk", "pw")
    assert error.value.code == "not_configured"


def test_frontdesk_job_reads_the_result_marker(monkeypatch, tmp_path):
    desk = frontdesk.FrontDesk("10.0.0.5", "desk", "pw", known_hosts=str(tmp_path / "known_hosts"))
    monkeypatch.setattr(desk, "_run", lambda argv, timeout: b"noise\r\nVISITOR_RESULT:eyJvayI6IHRydWV9\r\n")
    assert desk.job({"action": "list"}, "v1", 10) == {"ok": True}


@pytest.mark.parametrize("stderr,code", [
    ("Permission denied, please try again.", "auth_failed"),
    ("ssh: connect to host 10.0.0.5 port 22: Connection timed out", "unreachable"),
    ("WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!", "host_key_changed"),
    ("VISITOR_NOT_INSTALLED", "not_installed"),
    ("VISITOR_TIMEOUT task=Running At C:/ProgramData/VisitorBadge/app/run_job.ps1:20", "worker_timeout"),
])
def test_ssh_failures_become_actionable_codes(stderr, code):
    assert frontdesk._explain(stderr).code == code


def test_worker_stays_while_the_login_window_is_open(monkeypatch, tmp_path):
    # Edge started from the scheduled task dies with it, so the worker must
    # not exit while the operator is still logging in.
    checks = iter([True, True, False])
    monkeypatch.setattr(worker, "DATA", tmp_path)
    monkeypatch.setattr(worker, "JOBS", tmp_path / "jobs")
    monkeypatch.setattr(worker, "IDLE_EXIT_S", 0)
    monkeypatch.setattr(worker, "login_window_open", lambda profile: next(checks))
    monkeypatch.setattr(worker.time, "sleep", lambda s: None)
    clock = iter(range(0, 10**6, 3))
    monkeypatch.setattr(worker.time, "monotonic", lambda: next(clock))
    stdout, stderr = sys.stdout, sys.stderr
    try:
        assert worker.main() == 0
    finally:
        sys.stdout.close()
        sys.stdout, sys.stderr = stdout, stderr
    assert next(checks, "done") == "done"
