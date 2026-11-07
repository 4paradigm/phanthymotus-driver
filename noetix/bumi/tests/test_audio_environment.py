"""Public streaming contract; no robot/ROS dependency."""

import importlib.util
import json
import struct
import math
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / "audio_environment.py"


def load():
    spec = importlib.util.spec_from_file_location("audio_environment_tested", PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pcm(value, n=4000):
    return struct.pack("<" + "h" * n, *([value] * n))


def test_absolute_above_threshold_is_strict_held_and_recovers_once():
    boundary = 20 * math.log10(1000 / 32768)
    monitor = load().AudioMonitor(
        mode="threshold",
        threshold_dbfs=boundary,
        comparison="above",
        hold_s=0.5,
        recover_s=0.5,
        hysteresis_db=3,
    )
    assert monitor.feed(pcm(1000), 0) == []  # Equality is not greater-than.
    assert monitor.read(0)["status"] == "normal"
    assert monitor.feed(pcm(2000), 0.25) == []
    assert [e["event"] for e in monitor.feed(pcm(2000), 0.5)] == ["threshold_entered"]
    assert monitor.read(0.5)["status"] == "triggered"
    assert monitor.feed(pcm(2000), 0.75) == []
    assert monitor.feed(pcm(1000), 1) == []  # Hysteresis keeps it triggered.
    assert monitor.feed(pcm(500), 1.25) == []
    assert [e["event"] for e in monitor.feed(pcm(500), 1.5)] == ["threshold_cleared"]
    assert monitor.read(1.5)["baseline_dbfs"] is None


def test_absolute_below_threshold_silence_and_stale_input():
    monitor = load().AudioMonitor(
        mode="threshold",
        comparison="below",
        threshold_dbfs=-40,
        hold_s=0.5,
        recover_s=0.25,
    )
    monitor.feed(pcm(1000), 0)
    assert monitor.feed(pcm(0), 0.25) == []
    assert [e["event"] for e in monitor.feed(pcm(0), 0.5)] == ["threshold_entered"]
    assert monitor.read(0.5)["rms_dbfs"] is None
    json.dumps(monitor.read(0.5), allow_nan=False)
    assert monitor.read(3)["status"] == "unknown"
    # A gap discards the prior active state and cannot emit a false recovery.
    assert monitor.feed(pcm(1000), 3) == []
    assert monitor.read(3)["status"] == "normal"
    monitor.feed(pcm(100), 3.25)
    assert [e["event"] for e in monitor.feed(pcm(100), 3.5)] == ["threshold_entered"]
    assert [e["event"] for e in monitor.feed(pcm(1000), 3.75)] == ["threshold_cleared"]


def test_below_equality_transient_gap_and_invalid_audio_do_not_trigger():
    boundary = 20 * math.log10(1000 / 32768)
    monitor = load().AudioMonitor(
        mode="threshold", comparison="below", threshold_dbfs=boundary, hold_s=0.5
    )
    assert monitor.feed(pcm(1000), 0) == []
    assert monitor.feed(pcm(500), 0.25) == []
    assert monitor.feed(pcm(1000), 0.5) == []
    assert monitor.feed(pcm(500), 0.75) == []
    assert monitor.feed(pcm(500), 2) == []  # Gap resets pending time.
    with pytest.raises(ValueError):
        monitor.feed(b"bad", 2.25)
    assert monitor.read(2.25)["status"] == "unknown"
    assert monitor.feed(pcm(500), 2.5) == []
    assert monitor.read(2.5)["status"] == "normal"


@pytest.mark.parametrize(
    "settings",
    [
        {"mode": "other"},
        {"comparison": "equal"},
        {"threshold_dbfs": True},
        {"threshold_dbfs": float("nan")},
        {"threshold_dbfs": 1},
        {"threshold_dbfs": -161},
        {"hysteresis_db": -1},
        {"hysteresis_db": float("inf")},
        {"mode": "threshold", "comparison": "below", "threshold_dbfs": -1},
    ],
)
def test_invalid_threshold_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        load().AudioMonitor(**settings)


def test_card_config_is_live_atomic_and_does_not_restart_ros():
    from types import SimpleNamespace as NS

    node = FakeNode()
    card = load().AudioEnvironmentPlugin(
        {},
        "test",
        FakeExecutor(),
        runtime=lambda: (node, object, lambda: NS(data=""), 10),
        clock=lambda: 0,
    )
    schema = card.get_tool()["configSchema"]["properties"]
    assert schema["threshold_dbfs"]["maximum"] == 0
    assert card.dispatch("start", {"input_topic": "/mic"})["state"] == "running"
    result = card.dispatch(
        "config", {"mode": "threshold", "threshold_dbfs": -30, "hold_s": 0.25}
    )
    assert result["state"] == "running" and result["ok"] is True
    assert not node.destroyed
    node.input(NS(format="audio/pcm-16k", data=pcm(2000)))
    assert node.published[-1][1]["event"] == "threshold_entered"
    assert card.dispatch("config", {"mode": "threshold"})["ok"] is True
    assert (
        card.dispatch("read", {})["status"] == "triggered"
    )  # Same settings preserve state.
    old = card.dispatch("info", {})["settings"]
    bad = card.dispatch("config", {"threshold_dbfs": 10, "comparison": "below"})
    assert bad["ok"] is False
    assert card.dispatch("info", {})["settings"] == old
    assert card.dispatch("info", {})["status"] == "triggered"
    result = card.dispatch("config", {"comparison": "below"})
    assert result["settings"]["comparison"] == "below"
    assert card.dispatch("info", {})["status"] == "unknown"
    assert card.dispatch("calibrate", {})["ok"] is False
    card.stop()


def test_yaml_settings_and_config_validation_preserve_event_sequence():
    card = load().AudioEnvironmentPlugin(
        {
            "mode": "threshold",
            "comparison": "below",
            "threshold_dbfs": -40,
            "hold_s": 0.25,
            "hysteresis_db": 0,
        },
        "test",
        FakeExecutor(),
        clock=lambda: 0,
    )
    assert card.dispatch("info", {})["settings"]["threshold_dbfs"] == -40
    assert card.dispatch("config", {"hysteresis_db": False})["ok"] is False
    assert card.dispatch("config", {"typo": -20})["ok"] is False
    assert card.monitor.feed(pcm(100), 0)[0]["sequence"] == 1
    assert card.dispatch("config", {"threshold_dbfs": -30})["ok"] is True
    assert card.monitor.feed(pcm(100), 0.25)[0]["sequence"] == 2


def test_full_scale_statistics_and_json_safe_silence():
    monitor = load().AudioMonitor()
    monitor.feed(pcm(-32768), 0)
    state = monitor.read(0)
    assert state["rms"] == 1
    assert state["peak"] == 1
    assert state["clip_ratio"] == 1
    assert state["rms_dbfs"] == 0
    monitor.feed(pcm(0), 0.25)
    assert monitor.read(0.25)["rms_dbfs"] is None
    json.dumps(monitor.read(0.25), allow_nan=False)


def test_calibration_loud_recovery_and_no_repeated_events():
    monitor = load().AudioMonitor(calibration_s=1, hold_s=0.5, recover_s=0.5)
    now = 0

    def feed(value):
        nonlocal now
        events = monitor.feed(pcm(value), now)
        now += 0.25
        return events

    for _ in range(4):
        assert feed(100) == []
    assert monitor.read(now)["status"] == "normal"
    assert feed(1000) == []
    assert [e["event"] for e in feed(1000)] == ["loud"]
    for _ in range(8):
        assert feed(1000) == []
    assert feed(100) == []
    assert [e["event"] for e in feed(100)] == ["recovered"]
    assert monitor.read(now)["status"] == "normal"


def test_packet_splits_are_sample_weighted_and_stale_is_not_quiet():
    monitor = load().AudioMonitor(calibration_s=0.25)
    for i in range(4):
        monitor.feed(pcm(100, 1000), i * 0.0625)
    assert monitor.read(0.25)["rms"] == pytest.approx(100 / 32768)
    assert monitor.read(3)["status"] == "unknown"
    assert monitor.read(3)["rms"] is None
    monitor.feed(pcm(1000, 1000), 3)
    assert monitor.read(3)["baseline_dbfs"] is None
    assert monitor.read(3)["rms"] is None


@pytest.mark.parametrize("payload", [b"", b"1", b"0" * 16002])
def test_invalid_packet_invalidates_calibration(payload):
    monitor = load().AudioMonitor(calibration_s=0.25)
    monitor.feed(pcm(100), 0)
    with pytest.raises(ValueError):
        monitor.feed(payload, 0.25)
    assert monitor.read(0.25)["status"] == "unknown"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"hold_s": 0},
        {"rise_db": float("nan")},
        {"fall_db": 13},
        {"calibration_s": True},
    ],
)
def test_bad_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        load().AudioMonitor(**kwargs)


class FakeNode:
    def __init__(self):
        self.published = []
        self.destroyed = False

    def create_publisher(self, typ, topic, qos):
        class Publisher:
            def publish(_, message):
                self.published.append((topic, json.loads(message.data)))

        return Publisher()

    def create_subscription(self, typ, topic, callback, qos):
        self.input = callback
        return object()

    def create_timer(self, interval, callback):
        self.timer = callback
        return object()

    def destroy_node(self):
        self.destroyed = True


class FakeExecutor:
    def add_node(self, node):
        self.node = node

    def remove_node(self, node):
        self.node = None


@pytest.mark.parametrize("audio_format", ["audio/pcm-16k", "pcm_16k_16bit_mono"])
def test_card_lifecycle_events_and_invalid_format(audio_format):
    from types import SimpleNamespace

    node = FakeNode()

    def message():
        return SimpleNamespace(data="")

    card = load().AudioEnvironmentPlugin(
        {"calibration_s": 0.25, "hold_s": 0.25},
        "bumi",
        FakeExecutor(),
        runtime=lambda: (node, object, message, 10),
        clock=iter([0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75]).__next__,
    )
    assert card.get_tool()["type"] == "processor"
    assert card.dispatch("start", {})["state"] == "error"
    assert (
        card.dispatch("start", {"input_topic": "/bumi/mic/audio"})["state"] == "running"
    )
    node.input(SimpleNamespace(format=audio_format, data=pcm(100)))
    node.input(SimpleNamespace(format=audio_format, data=pcm(1000)))
    assert [
        data["event"] for topic, data in node.published if topic.endswith("/events")
    ] == ["loud"]
    node.input(SimpleNamespace(format="bad", data=pcm(1000)))
    assert card.dispatch("read", {})["status"] == "unknown"


def test_mixed_samples_are_weighted_and_transient_does_not_alert():
    monitor = load().AudioMonitor(calibration_s=0.25, hold_s=0.5)
    monitor.feed(pcm(100), 0)
    assert monitor.feed(pcm(2000), 0.25) == []
    assert monitor.feed(pcm(100), 0.5) == []
    assert monitor.read(0.5)["status"] == "normal"
    other = load().AudioMonitor()
    other.feed(pcm(32767, 1000), 0)
    other.feed(pcm(0, 3000), 0.1)
    assert other.read(0.1)["rms"] == pytest.approx(32767 / 65536)
    assert other.read(0.1)["clip_ratio"] == 0.25


def test_maximum_packet_cadence_allows_scheduler_jitter():
    monitor = load().AudioMonitor()
    for i in range(24):
        monitor.feed(pcm(100, 8000), i * 0.501)
    assert monitor.read(24 * 0.501)["status"] == "normal"
    assert monitor.read(24 * 0.501)["baseline_dbfs"] is not None


def test_restart_rejects_old_callbacks_and_replayed_stamps():
    from types import SimpleNamespace as NS

    nodes = [FakeNode(), FakeNode()]
    iterator = iter(nodes)
    clock = [0]
    card = load().AudioEnvironmentPlugin(
        {"calibration_s": 0.25},
        "test",
        FakeExecutor(),
        runtime=lambda: (next(iterator), object, lambda: NS(data=""), 10),
        clock=lambda: clock[0],
    )
    assert card.dispatch("start", {"input_topic": "/mic"})["state"] == "running"
    msg = NS(
        format="audio/pcm-16k", data=pcm(100), header=NS(stamp=NS(sec=1, nanosec=0))
    )
    nodes[0].input(msg)
    clock[0] = 0.25
    nodes[0].input(msg)
    assert card.dispatch("read", {})["status"] == "unknown"
    assert "timestamp" in card.dispatch("read", {})["last_error"]
    card.stop()
    card.dispatch("start", {"input_topic": "/other"})
    nodes[0].input(msg)
    nodes[0].timer()
    assert card.dispatch("read", {})["status"] == "unknown"
    assert nodes[1].published == []
    nodes[1].input(msg)
    nodes[1].timer()
    assert card.dispatch("read", {})["status"] == "normal"
    card.dispatch("calibrate", {})
    assert card.dispatch("read", {})["baseline_dbfs"] is None
    card.stop()


def test_node_setup_failure_is_cleaned_up():
    node = FakeNode()

    class FailingExecutor(FakeExecutor):
        def add_node(self, node):
            raise RuntimeError("executor unavailable")

    from types import SimpleNamespace

    card = load().AudioEnvironmentPlugin(
        {},
        "test",
        FailingExecutor(),
        runtime=lambda: (node, object, lambda: SimpleNamespace(data=""), 10),
    )
    assert card.dispatch("start", {"input_topic": "/mic"})["state"] == "error"
    assert node.destroyed
    assert card.dispatch("info", {})["state"] == "idle"
    assert card.dispatch("stop", {})["state"] == "idle"
    assert node.destroyed
    assert card.dispatch("read", {})["status"] == "unknown"
