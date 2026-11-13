"""Contract tests for Bumi's clock/reminder card."""

import importlib.util
import json
import sys
import types
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest
import yaml


BUMI = Path(__file__).resolve().parents[1]


def _load_clock(monkeypatch):
    class Node:
        def __init__(self, name):
            self.name = name
            self.publishers = []
            self.published = []

        def create_publisher(self, message_type, topic, qos):
            self.publishers.append(topic)
            return types.SimpleNamespace(
                publish=lambda msg: self.published.append((topic, msg.data)))

    class String:
        def __init__(self, data=""):
            self.data = data

    class Executor:
        def add_node(self, node):
            self.node = node

    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: object()
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    node = types.ModuleType("rclpy.node")
    node.Node = Node
    rclpy = types.ModuleType("rclpy")
    rclpy.node = node
    rclpy.qos = qos
    executors = types.ModuleType("rclpy.executors")
    rclpy.executors = executors
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    modules = {
        "rclpy": rclpy,
        "rclpy.node": node,
        "rclpy.qos": qos,
        "rclpy.executors": executors,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    path = BUMI / "clock_card.py"
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location("bumi_clock_card_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Clock:
    def __init__(self, module, store, now):
        self.card = module.ClockPlugin(
            {"store_path": str(store)}, "robot", types.SimpleNamespace(
                add_node=lambda node: None))
        self.now = now
        self.card._now = lambda: self.now

    def advance(self, **kwargs):
        self.now = self.now + timedelta(**kwargs)

    def __call__(self, action, **args):
        return self.card.dispatch(action, args)


@pytest.fixture
def clock(monkeypatch, tmp_path):
    module = _load_clock(monkeypatch)
    start = datetime(2026, 11, 9, 8, 0, tzinfo=timezone.utc)  # Monday
    return Clock(module, tmp_path / "reminders.json", start), module


def test_now_and_weekday_report_local_time(clock):
    card, _ = clock
    result = card("now")
    assert result["state"] == "ok"
    assert result["iso"] == "2026-11-09T08:00:00+00:00"
    assert (result["year"], result["month"], result["day"]) == (2026, 11, 9)
    assert (result["hour"], result["minute"], result["second"]) == (8, 0, 0)
    assert result["weekday"] == {
        "index": 1, "name_en": "Monday", "name_zh": "星期一"}

    weekday = card("weekday")
    assert weekday["weekday"]["index"] == 1
    assert weekday["weekday"]["name_zh"] == "星期一"


def test_until_reports_signed_interval(clock):
    card, _ = clock
    result = card("until", time="2026-11-09T08:01:30+00:00")
    assert result["state"] == "ok"
    assert result["total_seconds"] == 90
    assert result["in_past"] is False
    assert result["text"] == "1分钟30秒后"

    past = card("until", time="2026-11-09T07:58:00+00:00")
    assert past["total_seconds"] == -120
    assert past["in_past"] is True
    assert past["text"] == "2分钟前"


def test_create_list_and_cancel_reminder(clock):
    card, _ = clock
    created = card("create", content="吃药", time="2026-11-09T09:30:00+00:00")
    assert created["state"] == "created"
    reminder = created["reminder"]
    assert reminder["content"] == "吃药"
    assert reminder["repeat"] == "none"
    assert reminder["next_due"] == "2026-11-09T09:30:00+00:00"
    assert reminder["id"].startswith("rmd_")

    listed = card("list")
    assert listed["count"] == 1
    assert listed["pending_count"] == 1
    assert listed["reminders"][0]["id"] == reminder["id"]

    cancelled = card("cancel", id=reminder["id"])
    assert cancelled["state"] == "cancelled"
    assert card("list")["count"] == 0
    assert card("cancel", id=reminder["id"])["state"] == "not_found"


def test_tick_fires_due_reminder_and_publishes_event(clock):
    card, _ = clock
    created = card("create", content="喝水", time="2026-11-09T08:00:05+00:00")
    reminder_id = created["reminder"]["id"]

    card.advance(seconds=6)
    events = card.card._tick()

    assert len(events) == 1
    assert events[0]["type"] == "reminder"
    assert events[0]["id"] == reminder_id
    assert events[0]["content"] == "喝水"
    assert events[0]["late_seconds"] == 1.0

    topic, payload = card.card._node.published[-1]
    assert topic == "/robot/clock/reminder"
    published = json.loads(payload)
    assert published["type"] == "reminder"
    assert published["content"] == "喝水"

    listed = card("list")
    assert listed["pending_count"] == 0
    assert listed["reminders"][0]["state"] == "fired"
    assert listed["reminders"][0]["fired_count"] == 1

    # A fired one-shot never fires a second time.
    card.advance(minutes=10)
    assert card.card._tick() == []


def test_daily_reminder_reschedules_to_next_day(clock):
    card, _ = clock
    created = card("create", content="晨会", repeat="daily", time_of_day="09:00")
    assert created["state"] == "created"
    assert created["reminder"]["next_due"] == "2026-11-09T09:00:00+00:00"
    assert created["reminder"]["weekday"] is None

    card.now = datetime(2026, 11, 9, 9, 0, 1, tzinfo=timezone.utc)
    card.card._tick()
    reminder = card("list")["reminders"][0]
    assert reminder["state"] == "pending"
    assert reminder["fired_count"] == 1
    assert reminder["next_due"] == "2026-11-10T09:00:00+00:00"


def test_weekly_reminder_reschedules_to_next_week(clock):
    card, _ = clock
    created = card("create", content="周会", repeat="weekly",
                   weekday=1, time_of_day="09:00")
    assert created["state"] == "created"
    assert created["reminder"]["next_due"] == "2026-11-09T09:00:00+00:00"
    assert created["reminder"]["weekday"]["name_zh"] == "星期一"

    card.now = datetime(2026, 11, 9, 9, 0, 1, tzinfo=timezone.utc)
    card.card._tick()
    reminder = card("list")["reminders"][0]
    assert reminder["state"] == "pending"
    assert reminder["next_due"] == "2026-11-16T09:00:00+00:00"


def test_create_daily_accepts_weekday_names_and_iso_time(clock):
    card, _ = clock
    created = card("create", content="健身", repeat="每周",
                   weekday="周三", time="2026-11-11T19:00:00+00:00")
    assert created["state"] == "created"
    reminder = created["reminder"]
    assert reminder["repeat"] == "weekly"
    assert reminder["weekday"]["index"] == 3
    assert reminder["next_due"] == "2026-11-11T19:00:00+00:00"
    assert reminder["time_of_day"] == "19:00:00"


def test_update_changes_content_and_time(clock):
    card, _ = clock
    reminder_id = card("create", content="开会",
                       time="2026-11-09T10:00:00+00:00")["reminder"]["id"]

    updated = card("update", id=reminder_id, content="部门会议",
                   time="2026-11-09T11:00:00+00:00")
    assert updated["state"] == "updated"
    assert updated["reminder"]["content"] == "部门会议"
    assert updated["reminder"]["next_due"] == "2026-11-09T11:00:00+00:00"

    assert card("update", id="rmd_missing", content="x")["state"] == "not_found"


def test_countdown_uses_reminder_id(clock):
    card, _ = clock
    reminder_id = card("create", content="考试",
                       time="2026-11-09T10:30:00+00:00")["reminder"]["id"]
    result = card("countdown", id=reminder_id)
    assert result["state"] == "ok"
    assert result["reminder_id"] == reminder_id
    assert result["total_seconds"] == 9000
    assert result["text"] == "2小时30分钟后"


def test_reminders_persist_across_plugin_restarts(clock):
    card, module = clock
    card("create", content="交作业", time="2026-11-09T18:00:00+00:00")

    restored = module.ClockPlugin(
        {"store_path": str(card.card._store_path)}, "robot",
        types.SimpleNamespace(add_node=lambda node: None))
    listed = restored.dispatch("list", {})
    assert listed["count"] == 1
    assert listed["reminders"][0]["content"] == "交作业"
    assert listed["reminders"][0]["next_due"] == "2026-11-09T18:00:00+00:00"


def test_invalid_arguments_return_errors_not_exceptions(clock):
    card, _ = clock
    assert card("create", time="2026-11-09T09:00:00+00:00")["state"] == "invalid_parameters"
    assert card("create", content="x")["state"] == "invalid_parameters"
    assert card("create", content="x",
                time="2026-11-08T09:00:00+00:00")["state"] == "time_in_past"
    assert card("create", content="x", repeat="daily")["state"] == "invalid_parameters"
    assert card("create", content="x", repeat="weekly",
                time_of_day="09:00")["state"] == "invalid_parameters"
    assert card("update", id="rmd_missing")["state"] == "not_found"
    assert card("until", time="not-a-time")["state"] == "invalid_parameters"
    assert card("nonsense")["state"] == "invalid_action"


def test_start_and_stop_lifecycle(clock):
    card, _ = clock
    assert card("info")["state"] == "idle"
    assert card("start") == {"state": "ready"}
    assert card("info")["state"] == "ready"
    assert card("stop") == {"state": "idle"}
    assert card("info")["state"] == "idle"


def test_bundle_registers_clock_card(monkeypatch, tmp_path):
    module = _load_clock(monkeypatch)
    monkeypatch.setitem(sys.modules, "clock_card", module)
    path = BUMI / "main.py"
    source = path.read_text(encoding="utf-8")
    if "from __future__ import annotations" not in source:
        source = "from __future__ import annotations\n" + source
    main_module = types.ModuleType("bumi_main_clock_test")
    main_module.__file__ = str(path)
    exec(compile(source, main_module.__file__, "exec"), main_module.__dict__)

    bundle = main_module.BumiDeviceBundle(
        {"plugins": {"clock": {"enabled": True, "store_path": str(tmp_path / "r.json")}}},
        "robot", types.SimpleNamespace(add_node=lambda node: None), None, None)
    assert {tool["name"] for tool in bundle.get_all_tools()} == {"clock"}
    result = bundle.dispatch("clock", {"action": "now"})
    assert result["state"] == "ok"
    assert bundle.dispatch("clock", {"action": "info"})["state"] == "idle"

    config = yaml.safe_load((BUMI / "config.yaml").read_text(encoding="utf-8"))
    assert config["plugins"]["clock"]["enabled"] is True
    manifest = yaml.safe_load((BUMI / "driver.yaml").read_text(encoding="utf-8"))
    assert {"name": "clock", "type": "actuator"} in manifest["cards"]
