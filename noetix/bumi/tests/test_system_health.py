"""Host observation, freshness and lifecycle contracts; no ROS or robot needed."""

from __future__ import annotations

import importlib.util
import json
import sys
import threading
import time
import types
from pathlib import Path

import pytest
import yaml


@pytest.fixture(scope="module")
def health():
    path = Path(__file__).resolve().parents[1] / "system_health.py"
    spec = importlib.util.spec_from_file_location("bumi_system_health_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write(root, name, text):
    path = root / name.lstrip("/")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


@pytest.fixture
def files(tmp_path):
    for name, text in {
        "/proc/loadavg": "0.5 0.4 0.3 1/100 23\n",
        "/proc/stat": "cpu 100 0 0 100 0 0 0 0 0 0\ncpu0 50 0 0 50\ncpu1 50 0 0 50\n",
        "/proc/meminfo": "MemTotal: 8000 kB\nMemAvailable: 6000 kB\n",
        "/proc/uptime": "1234.56 1200\n",
        "/proc/sys/kernel/osrelease": "6.8.0-test\n",
        "/proc/self/mounts": "proc /proc proc rw 0 0\n",
        "/sys/class/thermal/thermal_zone0/type": "cpu-thermal\n",
        "/sys/class/thermal/thermal_zone0/temp": "52500\n",
    }.items():
        write(tmp_path, name, text)
    return tmp_path


def fs_stats(**overrides):
    values = dict(f_frsize=4096, f_bsize=4096, f_blocks=1000, f_bfree=400, f_bavail=300)
    values.update(overrides)
    return types.SimpleNamespace(**values)


def probe(health, root, **kwargs):
    return health.SystemProbe(root, statvfs=kwargs.get("statvfs", lambda path: fs_stats()))


def observe(health, root, **policy):
    return probe(health, root).collect(health.parse_policy(policy))


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true before timeout")


@pytest.fixture
def make_card(health, files):
    cards = []

    def make(*, cfg=None, **kwargs):
        messages = []
        kwargs.setdefault("probe", probe(health, files))
        kwargs.setdefault("publisher_factory", lambda: messages.append)
        card = health.SystemHealthPlugin(
            {"sample_interval_s": 0.5, **(cfg or {})}, "test", object(), **kwargs)
        cards.append(card)
        return card, messages

    yield make
    for card in cards:
        card.stop()


def test_default_observation_is_not_an_all_normal_verdict(health, files):
    result = observe(health, files)
    checks = result["checks"]
    temp = checks["temperature"]
    assert temp["status"] == "unrated"
    assert temp["value"]["max_celsius"] == 52.5
    assert temp["value"]["hardware_identity_confirmed"] is False
    assert checks["cpu_load"]["value"] == {
        "load_1_5_15": [0.5, 0.4, 0.3], "cores": 2,
        "load_per_core": 0.25, "usage_percent": None}
    assert checks["memory"]["value"] == {
        "total_bytes": 8000 * 1024, "available_bytes": 6000 * 1024, "used_percent": 25.0}
    assert checks["disk:/opt/phanthy-motus/data"]["value"]["used_percent"] == 66.7
    assert result["host"] == {
        "uptime_s": 1234.6, "kernel": "6.8.0-test", "proc_scope": "host_kernel"}
    assert health._verdict(checks)["overall"] == "unknown"
    assert health._verdict(checks)["complete"] is False
    assert json.loads(json.dumps(result, allow_nan=False)) == result


def test_configured_thermal_limit_allows_a_complete_normal_verdict(health, files):
    checks = observe(health, files, temperature_warn_celsius=70)["checks"]
    verdict = health._verdict(checks)
    assert verdict["overall"] == "ok"
    assert verdict["complete"] is True
    assert verdict["summary"] == "全部已评估项正常"
    assert health._verdict({})["overall"] == "unknown"


@pytest.mark.parametrize(("policy", "name"), [
    ({"cpu_load_per_core_warn": 0.25}, "cpu_load"),
    ({"memory_used_percent_warn": 25}, "memory"),
    ({"disk_used_percent_warn": 66}, "disk:/opt/phanthy-motus/data"),
    ({"temperature_warn_celsius": 52.5}, "temperature"),
])
def test_warning_boundaries_preserve_complete_observations(health, files, policy, name):
    checks = observe(health, files, **({"temperature_warn_celsius": 70, **policy}))["checks"]
    verdict = health._verdict(checks)
    assert checks[name]["status"] == "warning"
    assert verdict["overall"] == "warning"
    assert verdict["complete"] is True
    assert name in verdict["summary"]


@pytest.mark.parametrize("text", [
    "MemTotal: 0 kB\nMemAvailable: 0 kB",
    "MemTotal: 5 kB\nMemAvailable: 6 kB",
    "MemTotal: 5 kB", "MemTotal: 5 kB\nMemAvailable: -1 kB",
    "MemTotal: 5 MB\nMemAvailable: 2 kB",
])
def test_invalid_memory_is_unreadable(health, files, text):
    write(files, "/proc/meminfo", text)
    checks = observe(health, files)["checks"]
    assert checks["memory"]["status"] == "unreadable"
    assert checks["cpu_load"]["status"] == "ok"


@pytest.mark.parametrize(("name", "text"), [
    ("loadavg", "nan 0 0"), ("loadavg", "-1 0 0"),
    ("loadavg", "inf 0 0"), ("loadavg", "0 0"),
    ("stat", "cpu0 1 2 3 4"), ("stat", "cpu 1 2 3 4"),
    ("stat", "cpu 1 -2 3 4\ncpu0 1 2 3 4"),
    ("stat", "cpu 1 2 3\ncpu0 1 2 3"),
])
def test_invalid_cpu_observations_are_unreadable(health, files, name, text):
    write(files, f"/proc/{name}", text)
    assert observe(health, files)["checks"]["cpu_load"]["status"] == "unreadable"


def test_missing_files_are_isolated_and_host_metadata_is_optional(health, files):
    for path in ("proc/meminfo", "proc/uptime", "proc/sys/kernel/osrelease"):
        (files / path).unlink()
    result = observe(health, files)
    assert result["checks"]["memory"]["status"] == "unreadable"
    assert result["checks"]["temperature"]["value"]["max_celsius"] == 52.5
    assert result["host"]["uptime_s"] is None
    assert result["host"]["kernel"] is None


def test_thermal_zone_order_and_unlabeled_fallback(health, files):
    for n in range(1, 12):
        write(files, f"/sys/class/thermal/thermal_zone{n}/temp", str(40000 + n * 1000))
    temp = observe(health, files)["checks"]["temperature"]
    assert [item["zone"] for item in temp["value"]["zones"]] == [
        f"thermal_zone{n}" for n in range(12)]
    assert temp["value"]["zones"][1]["type"] == "unlabeled"


@pytest.mark.parametrize("limit", [None, 70, 50])
def test_partial_thermal_reads_never_become_complete_normal(health, files, limit):
    write(files, "/sys/class/thermal/thermal_zone1/temp", "broken")
    checks = observe(health, files, temperature_warn_celsius=limit)["checks"]
    temp = checks["temperature"]
    assert temp["value"]["max_celsius"] == 52.5
    assert temp["value"]["coverage_complete"] is False
    assert "ValueError" in temp["value"]["zones"][1]["error"]
    assert "1 thermal zone(s) unreadable" in temp["reasons"]
    assert temp["status"] == ("warning" if limit == 50 else "unreadable")
    verdict = health._verdict(checks)
    assert verdict["overall"] == ("warning" if limit == 50 else "unknown")
    assert verdict["complete"] is False
    assert any(item["check"] == "temperature" for item in verdict["problems"])


@pytest.mark.parametrize("value", ["broken", "201000", "-51000"])
def test_no_readable_thermal_zone_is_not_ok(health, files, value):
    write(files, "/sys/class/thermal/thermal_zone0/temp", value)
    temp = observe(health, files)["checks"]["temperature"]
    assert temp["status"] == "unreadable"
    assert temp["value"]["max_celsius"] is None


def test_no_thermal_zones_is_unreadable(health, files):
    (files / "sys/class/thermal/thermal_zone0/temp").unlink()
    (files / "sys/class/thermal/thermal_zone0/type").unlink()
    (files / "sys/class/thermal/thermal_zone0").rmdir()
    assert observe(health, files)["checks"]["temperature"]["status"] == "unreadable"


def test_cpu_usage_deltas_reset_and_guest_time_is_not_double_counted(health, files):
    reader = probe(health, files)
    policy = health.parse_policy({})
    assert reader.collect(policy)["checks"]["cpu_load"]["value"]["usage_percent"] is None
    # Delta total=100, idle=25. Guest time is already included in user time.
    write(files, "/proc/stat", "cpu 175 0 0 125 0 0 0 0 999 0\ncpu0 1 2 3 4\n")
    assert reader.collect(policy)["checks"]["cpu_load"]["value"]["usage_percent"] == 75.0
    write(files, "/proc/stat", "cpu 1 0 0 1\ncpu0 1 0 0 1\n")
    assert reader.collect(policy)["checks"]["cpu_load"]["value"]["usage_percent"] is None
    write(files, "/proc/stat", "cpu 5 0 0 0\ncpu0 5 0 0 0\n")
    assert reader.collect(policy)["checks"]["cpu_load"]["value"]["usage_percent"] is None


def test_lxcfs_scope_is_detected_per_proc_file(health, files):
    write(files, "/proc/self/mounts", "proc /proc proc rw 0 0\n"
          "lxcfs /proc/meminfo fuse.lxcfs rw 0 0\n"
          "other /proc/uptime fuse.lxcfs.extra rw 0 0\n")
    result = observe(health, files)
    assert result["checks"]["memory"]["scope"] == "container_lxcfs"
    assert result["checks"]["cpu_load"]["scope"] == "host_kernel"
    assert result["host"]["proc_scope"] == "container_lxcfs"


def test_missing_mount_table_does_not_claim_host_scope(health, files):
    (files / "proc/self/mounts").unlink()
    result = observe(health, files)
    assert result["checks"]["cpu_load"]["scope"] == "unknown"
    assert result["checks"]["memory"]["scope"] == "unknown"
    assert result["host"]["proc_scope"] == "unknown"
    assert "scope could not be established" in result["checks"]["memory"]["reasons"][0]


def test_virtualized_cpu_counters_do_not_claim_host_scope(health, files):
    write(files, "/proc/self/mounts", "proc /proc proc rw 0 0\n"
          "lxcfs /proc/stat fuse.lxcfs rw 0 0\n")
    assert observe(health, files)["checks"]["cpu_load"]["scope"] == "container_lxcfs"


def test_disk_formula_block_size_fallback_and_normalization(health, files):
    calls = []

    def statvfs(path):
        calls.append(path)
        return fs_stats(f_frsize=0)

    policy = health.parse_policy({"disk_paths": ["/data/photos/.."]})
    result = probe(health, files, statvfs=statvfs).collect(policy)
    assert calls == [files / "data"]
    disk = result["checks"]["disk:/data"]
    assert disk["scope"] == "mounted_filesystem"
    assert disk["value"] == {
        "path": "/data", "total_bytes": 4096000,
        "available_bytes": 1228800, "used_percent": 66.7}


@pytest.mark.parametrize("overrides", [
    {"f_blocks": 0}, {"f_frsize": 0, "f_bsize": 0}, {"f_bfree": 1001},
    {"f_bavail": -1}, {"f_bavail": 500}, {"f_bfree": 1000, "f_bavail": 0},
])
def test_invalid_filesystem_counters_are_unreadable(health, files, overrides):
    result = probe(health, files, statvfs=lambda path: fs_stats(**overrides)).collect(
        health.parse_policy({}))
    assert result["checks"]["disk:/opt/phanthy-motus/data"]["status"] == "unreadable"


def test_missing_disk_path_is_isolated(health, files):
    def missing(path):
        raise FileNotFoundError(str(path))

    result = probe(health, files, statvfs=missing).collect(health.parse_policy({}))
    item = result["checks"]["disk:/opt/phanthy-motus/data"]
    assert item["status"] == "unreadable"
    assert item["reasons"][0].startswith("FileNotFoundError:")
    assert len(item["reasons"][0]) <= 200


@pytest.mark.parametrize("cfg", [
    {"sample_interval_s": 0.49}, {"sample_interval_s": 61},
    {"sample_interval_s": True}, {"sample_interval_s": "2"},
    {"cpu_load_per_core_warn": float("nan")},
    {"cpu_load_per_core_warn": float("inf")},
    {"memory_used_percent_warn": 0}, {"memory_used_percent_warn": 101},
    {"disk_used_percent_warn": False}, {"temperature_warn_celsius": 151},
    {"disk_paths": "/data"}, {"disk_paths": []}, {"disk_paths": ["relative"]},
    {"disk_paths": ["/data", "/data/a/.."]}, {"disk_paths": ["/data"] * 9},
    {"disk_paths": [1]}, {"temperature_warn_c": 70}, [],
])
def test_invalid_policy_fails_loudly(health, cfg):
    with pytest.raises(ValueError):
        health.parse_policy(cfg)


def test_policy_defaults_are_independent_and_enabled_is_not_a_threshold(health):
    first = health.parse_policy({"enabled": True})
    first["disk_paths"].append("/other")
    assert health.parse_policy({})["disk_paths"] == ["/opt/phanthy-motus/data"]
    assert health.parse_policy(None)["temperature_warn_celsius"] is None


def test_schema_lifecycle_and_queries_before_start(health, make_card):
    card, _ = make_card()
    tool = card.get_tool()
    assert (tool["name"], tool["type"], tool["multiInstance"]) == ("system_health", "sensor", False)
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "start", "stop", "info", "robot_info"]
    assert "x-action-params" not in tool["inputSchema"]
    assert not {"x-resource", "x-completion", "configSchema"} & set(tool)
    assert json.loads(json.dumps(tool)) == tool
    info = card.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["topic_out"] == [{"topic": "/test/state/system_health", "format": "data/json"}]
    report = card.dispatch("robot_info", {})
    assert report["timestamp_ms"] is None
    assert report["sample_age_s"] is None
    assert set(report["checks"]) == {"monitoring"}
    assert report["checks"]["monitoring"]["status"] == "not_started"
    assert report["overall"] == "unknown"
    assert report["complete"] is report["fresh"] is False
    assert card.dispatch("diagnose", {}) is None
    assert card.dispatch("dance", {}) is None


def test_immediate_sampling_compact_publication_and_cached_deep_copies(make_card):
    card, messages = make_card(cfg={"sample_interval_s": 60})
    card.start()
    wait_for(lambda: messages)
    compact = json.loads(messages[0])
    full = card.dispatch("robot_info", {})
    assert compact["fresh"] is True
    assert isinstance(compact["timestamp_ms"], int)
    assert compact["timestamp_ms"] == full["timestamp_ms"]
    assert not {"policy", "host", "state", "checks", "zones"} & set(compact)
    assert not any(isinstance(value, dict) for value in compact.values())
    assert compact["temperature_celsius"] == 52.5
    assert compact["temperature_status"] == "unrated"
    assert compact["temperature_coverage_complete"] is True
    assert compact["cpu_load_1_5_15"] == [0.5, 0.4, 0.3]
    assert compact["cpu_load_per_core"] == 0.25
    assert compact["cpu_usage_percent"] is None  # First sample has no CPU delta.
    assert compact["memory_used_percent"] == 25.0
    assert compact["memory_available_bytes"] == 6000 * 1024
    assert compact["disk:/opt/phanthy-motus/data:used_percent"] == 66.7
    assert compact["disk:/opt/phanthy-motus/data:available_bytes"] == 1228800
    assert compact["monitoring_status"] == "ok"
    assert compact["samples"] == 1
    assert "zones" in full["checks"]["temperature"]["value"]
    assert "host" in full and "policy" not in full
    assert json.loads(json.dumps(full, allow_nan=False)) == full

    def no_collect(policy):
        raise AssertionError("query must never collect")

    card._probe.collect = no_collect
    full["checks"]["temperature"]["value"]["zones"].clear()
    assert card.dispatch("robot_info", {})["checks"]["temperature"]["value"]["zones"]
    policy = card.dispatch("info", {})["policy"]
    policy["disk_paths"].clear()
    assert card.dispatch("info", {})["policy"]["disk_paths"]
    card.stop()
    assert card.dispatch("info", {})["state"] == "idle"
    assert card.dispatch("robot_info", {})["fresh"] is False


def test_canvas_lifecycle_never_creates_or_revives_a_sampler(make_card, monkeypatch):
    card, messages = make_card()

    def forbidden_start():
        raise AssertionError("canvas start must not create ROS objects or a worker")

    monkeypatch.setattr(card, "start", forbidden_start)
    for _ in range(3):
        assert card.dispatch("start", {}) == {
            "state": "idle", "topic_out": card.get_tool()["topic_out"]}
        assert card.dispatch("stop", {}) == {"state": "idle"}
    assert card._worker is None and card._node is None
    assert messages == []


def test_canvas_stop_keeps_sampler_and_repeated_start_is_idempotent(make_card):
    card, messages = make_card()
    card.start()  # Bundle lifecycle owns actual startup.
    wait_for(lambda: messages)
    worker = card._worker
    for _ in range(3):
        assert card.dispatch("start", {}) == {
            "state": "running", "topic_out": card.get_tool()["topic_out"]}
        card.start()
    assert card._worker is worker
    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert worker.is_alive()
    assert card.dispatch("info", {})["state"] == "running"
    wait_for(lambda: len(messages) >= 2)
    card.stop()
    assert card.dispatch("start", {})["state"] == "idle"
    assert card._worker is None


def test_flat_topic_distinguishes_multiple_filesystems(make_card):
    card, messages = make_card(cfg={"disk_paths": ["/data", "/logs"]})
    card.start()
    wait_for(lambda: messages)
    payload = json.loads(messages[0])
    for path in ("/data", "/logs"):
        assert payload[f"disk:{path}:used_percent"] == 66.7
        assert payload[f"disk:{path}:status"] == "ok"
    assert set(card.dispatch("robot_info", {})["checks"]) >= {"disk:/data", "disk:/logs"}


def test_failed_sample_clears_previous_flat_measurements(health, files, make_card):
    reader = probe(health, files)
    first = [True]

    def collect(policy):
        if first[0]:
            first[0] = False
            return reader.collect(policy)
        raise RuntimeError("read failed after a good sample")

    card, messages = make_card(probe=types.SimpleNamespace(collect=collect))
    card.start()
    wait_for(lambda: len(messages) >= 2)
    good, failed = (json.loads(payload) for payload in messages[:2])
    assert good["memory_used_percent"] == 25.0
    for name in ("temperature_celsius", "cpu_load_1_5_15", "cpu_usage_percent",
                 "memory_used_percent", "disk:/opt/phanthy-motus/data:used_percent"):
        assert name in failed and failed[name] is None
    assert failed["memory_status"] == "unknown"
    assert failed["overall"] == "unknown"
    assert any(item["check"] == "sampler" for item in failed["problems"])


def test_sampling_uses_configured_cadence_without_busy_looping(make_card):
    timestamps = []
    card, _ = make_card(publisher_factory=lambda: lambda payload: timestamps.append(time.monotonic()))
    started = time.monotonic()
    card.start()
    wait_for(lambda: len(timestamps) >= 3)
    assert timestamps[0] - started < 0.5
    assert all(after - before >= 0.35 for before, after in zip(timestamps, timestamps[1:]))


def test_concurrent_starts_create_only_one_sampler_and_publisher(health, files, make_card):
    created = []

    def factory():
        created.append(True)
        return lambda payload: None

    card, _ = make_card(publisher_factory=factory)
    barrier = threading.Barrier(9)

    def start():
        barrier.wait()
        card.start()

    threads = [threading.Thread(target=start) for _ in range(8)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(timeout=2)
        assert not thread.is_alive()
    assert len(created) == 1
    assert card._worker.is_alive()


def blocking_probe(health, files):
    reader = probe(health, files)
    entered, release = threading.Event(), threading.Event()
    calls = []

    def collect(policy):
        calls.append(True)
        if len(calls) == 2:
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test did not release blocked probe")
        return reader.collect(policy)

    return types.SimpleNamespace(collect=collect), entered, release, calls


def test_blocked_probe_queries_stay_fast_and_stop_cannot_overlap_restart(health, files, make_card):
    reader, entered, release, calls = blocking_probe(health, files)
    now = [10.0]
    card, messages = make_card(probe=reader, clock=lambda: now[0])
    card._JOIN_TIMEOUT_S = 0.02
    card.start()
    try:
        assert entered.wait(2)
        original = card.dispatch("robot_info", {})
        now[0] += 1.5
        start = time.monotonic()
        stale = card.dispatch("robot_info", {})
        assert time.monotonic() - start < 0.5
        assert stale["checks"]["monitoring"]["status"] == "stale"
        assert stale["overall"] == "warning"
        assert stale["fresh"] is False
        assert stale["timestamp_ms"] == original["timestamp_ms"]
        assert stale["checks"]["memory"] == original["checks"]["memory"]
        worker = card._worker
        card.stop()
        assert card.dispatch("info", {})["state"] == "stopping"
        card.start()
        assert card._worker is worker
        assert len(calls) == 2
    finally:
        release.set()
        card.stop()
        wait_for(lambda: card._worker is None or not card._worker.is_alive())
        card.stop()
    assert card.dispatch("info", {})["state"] == "idle"
    assert card.dispatch("info", {})["samples_collected"] == 1
    assert len(messages) == 1  # The sample finishing after stop is discarded.
    card.start()
    wait_for(lambda: card.dispatch("info", {})["samples_collected"] == 2)


def test_unexpected_probe_error_is_reported_and_sampling_continues(make_card):
    calls = []

    def broken(policy):
        calls.append(True)
        raise RuntimeError("unexpected probe failure")

    card, messages = make_card(probe=types.SimpleNamespace(collect=broken))
    card.start()
    wait_for(lambda: len(messages) >= 2)
    report = card.dispatch("robot_info", {})
    assert len(calls) >= 2
    assert report["checks"]["sampler"]["status"] == "unreadable"
    assert report["overall"] == "unknown"
    assert report["fresh"] is True


def test_unavailable_publisher_does_not_disable_cache_queries(make_card):
    def missing():
        raise RuntimeError("ROS unavailable")

    card, _ = make_card(publisher_factory=missing)
    card.start()
    wait_for(lambda: card.dispatch("info", {})["samples_collected"] == 1)
    assert card.dispatch("info", {})["publisher"].startswith("unavailable: RuntimeError:")
    assert card.dispatch("robot_info", {})["checks"]["memory"]["status"] == "ok"


def test_publish_error_logs_are_throttled_and_recovery_is_reported(make_card, capsys):
    fail = [True]

    def publish(payload):
        if fail[0]:
            raise OSError("offline")

    card, _ = make_card(cfg={"sample_interval_s": 60}, publisher_factory=lambda: publish)
    card.start()
    wait_for(lambda: card._publish_failures == 1)
    for _ in range(149):
        card._publish_report()
    logs = capsys.readouterr().out
    assert logs.count("publish failed") == 2
    assert logs.count("overall=") == 1
    fail[0] = False
    card._publish_report()
    assert "publisher recovered" in capsys.readouterr().out


def fake_ros(monkeypatch):
    nodes, attached, messages = [], [], []

    class String:
        pass

    class Node:
        def __init__(self, name):
            self.name = name
            self.timers = []
            self.destroyed = False
            nodes.append(self)

        def create_publisher(self, msg_type, topic, depth):
            assert msg_type is String
            self.publisher_args = (topic, depth)
            return types.SimpleNamespace(publish=lambda msg: messages.append(json.loads(msg.data)))

        def create_timer(self, interval, callback):
            self.timers.append((interval, callback))

        def destroy_node(self):
            self.destroyed = True

    node_module = types.ModuleType("rclpy.node")
    node_module.Node = Node
    msg_module = types.ModuleType("std_msgs.msg")
    msg_module.String = String
    monkeypatch.setitem(sys.modules, "rclpy.node", node_module)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", msg_module)
    return nodes, attached, messages


def test_ros_factory_reuses_one_node_and_publishes_stale_while_sampler_is_blocked(
        health, files, monkeypatch):
    nodes, attached, messages = fake_ros(monkeypatch)
    reader, entered, release, _ = blocking_probe(health, files)
    now = [10.0]
    card = health.SystemHealthPlugin(
        {"sample_interval_s": 0.5}, "ros_test", types.SimpleNamespace(add_node=attached.append),
        probe=reader, clock=lambda: now[0])
    card.start()
    try:
        assert entered.wait(2)
        assert len(nodes) == 1 and attached == nodes
        assert nodes[0].name == "bumi_system_health"
        assert nodes[0].publisher_args == ("/ros_test/state/system_health", 10)
        assert nodes[0].timers[0][0] == 0.5
        timestamp = messages[0]["timestamp_ms"]
        now[0] += 1.5
        nodes[0].timers[0][1]()  # The executor can run while the sampler is stuck.
        assert len(messages) == 2
        stale = messages[-1]
        assert stale["fresh"] is False
        assert stale["monitoring_status"] == "stale"
        assert stale["timestamp_ms"] == timestamp
        assert stale["overall"] == "warning"
    finally:
        release.set()
        card.stop()
    count = len(messages)
    nodes[0].timers[0][1]()
    assert len(messages) == count
    card.start()
    try:
        wait_for(lambda: len(messages) > count)
        assert len(nodes) == 1
        assert len(nodes[0].timers) == 1
    finally:
        card.stop()


def test_executor_timer_publishes_unknown_before_first_sample(health, monkeypatch):
    nodes, attached, messages = fake_ros(monkeypatch)
    entered, release = threading.Event(), threading.Event()

    def collect(policy):
        entered.set()
        release.wait(3)
        return {"checks": {}, "host": {}}

    card = health.SystemHealthPlugin({}, "test", types.SimpleNamespace(add_node=attached.append),
                                     probe=types.SimpleNamespace(collect=collect))
    card.start()
    try:
        assert entered.wait(2)
        nodes[0].timers[0][1]()
        assert messages[-1]["fresh"] is False
        assert messages[-1]["monitoring_status"] == "unknown"
    finally:
        release.set()
        card.stop()


def test_ros_setup_failure_cleans_partial_node_but_keeps_sampling(health, files, monkeypatch):
    nodes, _, _ = fake_ros(monkeypatch)

    def unavailable(node):
        raise RuntimeError("executor unavailable")

    card = health.SystemHealthPlugin(
        {}, "test", types.SimpleNamespace(add_node=unavailable), probe=probe(health, files))
    card.start()
    try:
        wait_for(lambda: card.dispatch("info", {})["samples_collected"] == 1)
        assert nodes[0].destroyed is True
        assert "executor unavailable" in card.dispatch("info", {})["publisher"]
        assert card.dispatch("robot_info", {})["checks"]["memory"]["status"] == "ok"
    finally:
        card.stop()


def test_shipped_config_and_manifest_enable_read_only_card(health):
    bumi = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((bumi / "config.yaml").read_text(encoding="utf-8"))
    manifest = yaml.safe_load((bumi / "driver.yaml").read_text(encoding="utf-8"))
    assert cfg["plugins"]["system_health"]["enabled"] is True
    assert health.parse_policy(cfg["plugins"]["system_health"])["temperature_warn_celsius"] is None
    assert [card for card in manifest["cards"] if card["name"] == "system_health"] == [
        {"name": "system_health", "type": "sensor"}]
