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

    def fake_job(self, job, timeout):
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
    assert set(schema["x-action-params"]) >= {"list", "print", "setup", "status"}
    config = definition["configSchema"]
    assert config["properties"]["password"]["x-sensitive"] is True
    assert set(config["required"]) == {"host", "user", "password", "page_url"}


def test_config_reports_adapter_ok_only_when_complete():
    plugin = device.VisitorBadgePlugin({})
    assert plugin.dispatch("config", {"host": "10.0.0.5"})["adapter_ok"] is False
    assert plugin.dispatch("config", CONFIG)["adapter_ok"] is True


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
    assert desk.job({"action": "list"}, 10) == {"ok": True}


@pytest.mark.parametrize("stderr,code", [
    ("Permission denied, please try again.", "auth_failed"),
    ("ssh: connect to host 10.0.0.5 port 22: Connection timed out", "unreachable"),
    ("WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!", "host_key_changed"),
    ("VISITOR_NOT_INSTALLED", "not_installed"),
    ("VISITOR_TIMEOUT task=Running At C:/ProgramData/VisitorBadge/app/run_job.ps1:20", "worker_timeout"),
])
def test_ssh_failures_become_actionable_codes(stderr, code):
    assert frontdesk._explain(stderr).code == code
