"""Plugin-level tests with fake ROS/SDK modules: no robot, no rclpy required."""

from __future__ import annotations

import importlib.util
import struct
import sys
import threading
import time
import types
from pathlib import Path

import pytest

BUMI = Path(__file__).resolve().parents[1]
CARD_PATH = BUMI / "beat_performance_card.py"
SAMPLE_RATE = 16000


def _click_track(beat_times: list[float], duration_s: float) -> bytes:
    samples = [0] * int(duration_s * SAMPLE_RATE)
    for beat in beat_times:
        start = int(beat * SAMPLE_RATE)
        for index in range(start, min(start + 640, len(samples))):
            samples[index] = 14000 if (index - start) % 2 == 0 else -14000
    return struct.pack(f"<{len(samples)}h", *samples)


class FakeHighCtrl:
    def __init__(self, mode: int = 2):
        self.mode = mode
        self.forbidden_writes = []

    def get_mode(self):
        return self.mode

    def publish_cmd(self, *_args):
        self.forbidden_writes.append(_args)
        raise AssertionError("card must write commands through LocoPlugin only")


class FakeLoco:
    """Records command edges and enforces the same >=2 ms gap as production."""

    def __init__(self, ctrl: FakeHighCtrl):
        self.ctrl = ctrl
        self.commands: list[tuple] = []
        self._lock = threading.Lock()
        self._last = None
        self.reserved = False

    def _record(self, item) -> None:
        with self._lock:
            now = time.monotonic()
            gap = None if self._last is None else now - self._last
            self.commands.append((item, now, gap))
            self._last = now

    def reserve_body_action(self, owner: str) -> bool:
        if self.reserved:
            return False
        self.reserved = True
        self._record(("reserve", owner))
        return True

    def release_body_action(self, owner: str) -> None:
        if self.reserved:
            self.reserved = False
            self._record(("release_reservation", owner))

    def preempt_for(self, reason: str) -> bool:
        self._record(("preempt", reason))
        return True

    def prepare_walking_mode(self, requested_action: str,
                             cancel_event=None) -> dict:
        if cancel_event is not None and cancel_event.is_set():
            return {"state": "error", "cancelled": True, "error": "cancelled"}
        self._record(("prepare", requested_action))
        if self.ctrl.get_mode() == 26:
            return {"state": "error", "protection": True,
                    "error": "Robot is in protection mode"}
        return {"state": "completed", "workmode": self.ctrl.get_mode()}

    def publish_action_edge(self, command_name: str, index: int = 0,
                            hold_s: float = 0.01) -> None:
        self._record(("edge", command_name))
        time.sleep(0.012)
        self._record(("release", "DEFAULT"))

    def publish_velocity(self, x: float, y: float, z: float) -> None:
        self._record(("velocity", x, y, z))

    @property
    def edges(self) -> list[tuple]:
        return [item for item, _, _ in self.commands if item[0] == "edge"]


class FakeNode:
    def __init__(self, name: str):
        self.name = name
        self.published: list = []
        self.topic = ""

    def create_publisher(self, message_type, topic: str, qos):
        self.topic = topic
        return types.SimpleNamespace(publish=self.published.append)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)


def _load_card(monkeypatch):
    class AudioChunk:
        def __init__(self):
            self.header = types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=0, nanosec=0))
            self.format = ""
            self.data = b""

    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: object()
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    node_module = types.ModuleType("rclpy.node")
    node_module.Node = FakeNode
    modules = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": node_module,
        "rclpy.qos": qos,
        "audio_msgs": types.ModuleType("audio_msgs"),
        "audio_msgs.msg": types.ModuleType("audio_msgs.msg"),
    }
    modules["audio_msgs.msg"].AudioChunk = AudioChunk
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.syspath_prepend(str(BUMI))

    spec = importlib.util.spec_from_file_location("bumi_beat_card_test", CARD_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _make_plugin(module, monkeypatch, tmp_path, *, mode=2, config=None,
                 pcm=None):
    """Build a card with a 16 kHz click track written as a fake upload."""
    ctrl = FakeHighCtrl(mode)
    loco = FakeLoco(ctrl)
    plugin = module.BeatPerformancePlugin(
        config or {}, "robot", types.SimpleNamespace(add_node=lambda _: None),
        ctrl, loco)
    if pcm is None:
        pcm = _click_track([0.0, 0.5, 1.0, 1.5], 2.0)
    upload = tmp_path / "click.pcm"
    upload.write_bytes(pcm)
    plugin._allowed_dirs_for_test = (str(tmp_path),)
    return plugin, ctrl, loco, str(upload)


@pytest.fixture
def card_module(monkeypatch):
    module = _load_card(monkeypatch)
    calls: list[tuple] = []
    monkeypatch.setattr(module, "_acp_notify", lambda *args: calls.append(args))
    module._acp_calls = calls
    return module


# ── declaration / wiring ─────────────────────────────────────────────────────

def test_tool_declares_an_actuator_with_audio_output_and_uploaded_file(card_module):
    ctrl = FakeHighCtrl()
    plugin = card_module.BeatPerformancePlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda _: None),
        ctrl, FakeLoco(ctrl))
    tool = plugin.get_tool()

    assert tool["name"] == "beat_performance"
    assert tool["type"] == "actuator"
    assert tool["topic_out"] == [{
        "topic": "/robot/beat_performance/audio",
        "format": "audio/pcm-16k",
        "message_type": "audio_msgs/msg/AudioChunk",
    }]
    audio_file = tool["inputSchema"]["properties"]["audio_file"]
    assert audio_file["format"] == "file"
    assert audio_file["uploadTo"] == "mcp"
    assert set(audio_file["accept"].split(",")) == {".wav", ".pcm"}
    assert set(tool["inputSchema"]["x-resource"]) == {"base", "arm_l", "arm_r"}
    assert set(tool["inputSchema"]["x-action-params"]["start"]["params"]) == {
        "audio_file", "confirm"}
    assert tool["inputSchema"]["x-completion"] == {
        "actions": ["start"], "timeout": 165}
    assert "confirm" in tool["inputSchema"]["properties"]


def test_both_new_modules_reach_the_image():
    dockerfile = (BUMI / "Dockerfile").read_text(encoding="utf-8")
    assert "COPY beat_performance_spec.py beat_performance_card.py /work/" in dockerfile


def test_loco_action_edge_is_a_command_release_pair_with_the_vendor_gap(monkeypatch):
    """The shared publisher, not just its caller, must respect edge semantics."""
    from test_mic_card import _load_device

    device_module, _ = _load_device(monkeypatch)
    monkeypatch.setattr(device_module, "_get_control_cmd", lambda name: name)

    class RecordingCtrl:
        def __init__(self):
            self.calls = []

        def publish_cmd(self, x, y, z, command, index):
            self.calls.append((time.monotonic(), command, x, y, z, index))

    ctrl = RecordingCtrl()
    loco = device_module.LocoPlugin.__new__(device_module.LocoPlugin)
    loco._high_ctrl = ctrl
    loco._lock = threading.Lock()
    loco._last_cmd_time = 0.0

    loco.publish_action_edge("SWING", index=7)

    assert [call[1] for call in ctrl.calls] == ["SWING", "DEFAULT"]
    assert all(call[2:5] == (0, 0, 0) for call in ctrl.calls)
    assert ctrl.calls[0][5] == 7 and ctrl.calls[1][5] == 0
    assert ctrl.calls[1][0] - ctrl.calls[0][0] >= 0.002


# ── lifecycle / safety ───────────────────────────────────────────────────────

def test_a_full_run_fires_actions_on_the_beat_and_publishes_all_audio_frames(
        card_module, monkeypatch, tmp_path):
    module = card_module
    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    plugin, ctrl, loco, upload = _make_plugin(
        module, monkeypatch, tmp_path,
        config={"action_every_beats": 2, "action_cooldown_s": 0.5,
                "turn_every_beats": 4, "turn_cooldown_s": 0.5})
    started = plugin.dispatch("start", {"audio_file": upload, "confirm": True})
    assert started["state"] == "running"
    assert started["action_id"].startswith("bumi_beat_performance_")
    assert started["expected_duration_s"] == 120.0
    # 2 seconds of audio, 20 ms frames.  Join the worker rather than sleeping.
    thread = plugin._thread
    thread.join(timeout=8.0)
    assert not thread.is_alive()

    info = plugin.dispatch("info", {})
    assert info["beat_source"] == "detected"
    assert info["beat_count"] >= 4
    assert info["frames_published"] == 100
    assert info["state"] == "idle"
    assert info["action_count"] >= 1
    assert info["last_action"] in {"wave", "cheer", "dance_1", "dance_2"}
    assert info["last_action_drift_ms"] is not None
    assert info["last_action_drift_ms"] < 100
    assert ctrl.forbidden_writes == []
    assert [call[1] for call in module._acp_calls] == ["completed"]
    # The body reservation spans preparation and the entire playback.
    assert loco.commands[0][0] == ("reserve", "beat_performance")
    assert any(item == ("preempt", "beat_performance.start")
               for item, _, _ in loco.commands)
    assert ("release_reservation", "beat_performance") in [
        item for item, _, _ in loco.commands]
    assert loco.reserved is False
    edges = [item for item, _, _ in loco.commands if item[0] == "edge"]
    releases = [item for item, _, _ in loco.commands if item[0] == "release"]
    assert len(edges) == info["action_count"]
    assert len(releases) >= len(edges)


def test_stop_cancels_all_later_beats_and_releases_the_command(
        card_module, monkeypatch, tmp_path):
    module = card_module
    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    pcm = _click_track([0.0, 0.4, 0.8, 1.2, 1.6, 2.0], 3.0)
    plugin, _, loco, upload = _make_plugin(
        module, monkeypatch, tmp_path, pcm=pcm,
        config={"action_every_beats": 2, "action_cooldown_s": 0.5,
                "turn_every_beats": 2, "turn_cooldown_s": 0.5})
    plugin.dispatch("start", {"audio_file": upload, "confirm": True})
    time.sleep(0.25)
    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    time.sleep(0.8)  # past several scheduled beats that must never fire

    actions_after_stop = [item for item, at, _ in loco.commands
                          if item[0] in ("edge", "velocity") and at > time.monotonic() - 0.7]
    assert actions_after_stop == []
    assert plugin.dispatch("info", {})["events_fired"] == 0
    assert plugin._thread is None
    assert [call[1] for call in module._acp_calls] == ["cancelled"]


def test_an_invalid_upload_never_preempts_prepares_or_writes_the_body(
        card_module, monkeypatch, tmp_path):
    module = card_module
    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    plugin, _, loco, _ = _make_plugin(module, monkeypatch, tmp_path)
    plugin.dispatch("start", {
        "audio_file": str(tmp_path / "missing.pcm"), "confirm": True})
    plugin._thread.join(timeout=3.0)

    assert loco.commands == []
    assert loco.reserved is False
    info = plugin.dispatch("info", {})
    assert info["state"] == "error"
    assert info["workmode"] == 2
    assert [call[1] for call in module._acp_calls] == ["error"]


def test_a_stop_before_preparation_sends_no_mode_edge_after_release(
        card_module, monkeypatch, tmp_path):
    module = card_module
    entered = threading.Event()
    release = threading.Event()

    def blocked_preempt(reason):
        entered.set()
        release.wait(3.0)
        return True

    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    plugin, _, loco, upload = _make_plugin(module, monkeypatch, tmp_path)
    monkeypatch.setattr(loco, "preempt_for", blocked_preempt)
    plugin.dispatch("start", {"audio_file": upload, "confirm": True})
    assert entered.wait(2.0)
    # Let the join inside stop finish once the worker unblocks.
    threading.Timer(0.1, release.set).start()
    plugin.dispatch("stop", {})

    assert not any(item[0] == "prepare" for item, _, _ in loco.commands)
    assert loco.edges == []
    assert [item for item, _, _ in loco.commands if item[0] == "velocity"] == [
        ("velocity", 0.0, 0.0, 0.0)]
    assert [call[1] for call in module._acp_calls] == ["cancelled"]


def test_a_competing_body_action_is_rejected_while_the_beat_card_holds_the_body(
        monkeypatch):
    from test_mic_card import _load_device

    device_module, _ = _load_device(monkeypatch)
    loco = device_module.LocoPlugin.__new__(device_module.LocoPlugin)
    loco._rotate_lock = threading.Lock()
    loco._rotate_state = {"state": "idle", "last_result": None}
    loco._rotate_stop_event = threading.Event()
    loco._move_stop_event = threading.Event()
    loco._move_thread = None
    loco._body_action = None
    loco._servo = None

    assert loco.reserve_body_action("beat_performance") is True
    refused = loco.dispatch("wave", {"_tool_name": "semantic_action"})
    assert refused["state"] == "error"
    assert "reserved" in refused["error"]
    loco.release_body_action("beat_performance")
    assert loco._body_action is None


def test_an_unconfirmed_start_never_prepares_the_body_or_sends_a_command(
        card_module, monkeypatch, tmp_path):
    module = card_module
    plugin, _, loco, upload = _make_plugin(module, monkeypatch, tmp_path)

    missing = plugin.dispatch("start", {"audio_file": upload})
    rejected = plugin.dispatch("start", {"audio_file": upload, "confirm": False})
    for result in (missing, rejected):
        assert result["state"] == "error"
        assert result["command_sent"] is False
    assert loco.commands == []
    assert plugin._thread is None
    assert plugin.dispatch("info", {})["workmode"] == 2
    assert module._acp_calls == []


def test_protection_mode_sends_no_action_and_reports_it_honestly(
        card_module, monkeypatch, tmp_path):
    module = card_module
    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    plugin, _, loco, upload = _make_plugin(
        module, monkeypatch, tmp_path, mode=26)
    plugin.dispatch("start", {"audio_file": upload, "confirm": True})
    plugin._thread.join(timeout=3.0)

    # Protection is refused before any reservation, preparation or command.
    assert loco.commands == []
    assert loco.reserved is False
    info = plugin.dispatch("info", {})
    assert info["protection"] is True
    assert info["state"] == "error"
    assert "protection mode" in info["last_error"]
    assert info["workmode"] == 26
    assert [call[1] for call in module._acp_calls] == ["error"]
    assert module._acp_calls[0][2]["state"] == "error"


def test_a_superseded_worker_cannot_publish_after_restart(
        card_module, monkeypatch, tmp_path):
    module = card_module
    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    pcm = _click_track([0.0, 0.5, 1.0], 2.0)
    plugin, _, _, upload = _make_plugin(module, monkeypatch, tmp_path, pcm=pcm)
    plugin.dispatch("start", {"audio_file": upload, "confirm": True})
    first_thread = plugin._thread
    plugin.dispatch("stop", {})
    first_thread.join(timeout=2.0)
    frames_at_stop = plugin._node.published.__len__()
    time.sleep(0.05)
    assert len(plugin._node.published) == frames_at_stop


def test_config_is_rejected_while_running(card_module, monkeypatch, tmp_path):
    module = card_module
    parse = module.decode_audio

    def local_decode(path, **kwargs):
        return parse(path, allowed_dirs=(str(tmp_path),), **kwargs)
    monkeypatch.setattr(module, "decode_audio", local_decode)

    plugin, _, _, upload = _make_plugin(module, monkeypatch, tmp_path)
    plugin.dispatch("start", {"audio_file": upload, "confirm": True})
    assert plugin.dispatch("config", {"action_every_beats": 8})["state"] == "error"
    plugin.dispatch("stop", {})
    assert plugin.dispatch("config", {"action_every_beats": 8})["state"] == "configured"
