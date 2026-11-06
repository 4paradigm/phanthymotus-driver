"""Public streaming contract; no robot/ROS dependency."""

import importlib.util
import json
import struct
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
