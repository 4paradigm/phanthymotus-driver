"""Go1 bai_nian card: HOG detection trigger + bow sequence + speech playback."""

import base64
import importlib
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest


JPEG = b"\xff\xd8" + b"frame" * 64 + b"\xff\xd9"


class FakeClient:
    def __init__(self, fresh=True, mode=1):
        self.fresh = fresh
        self.mode = mode
        self.postures = []
        self.stop_move_calls = 0

    def snapshot(self):
        return {"fresh": self.fresh, "mode": self.mode}

    def set_posture(self, mode, euler=(0.0, 0.0, 0.0), body_height=0.0,
                    foot_raise=0.0, speed_level=0):
        self.postures.append((mode, tuple(euler), body_height))
        self.mode = mode

    def stop_move(self):
        self.stop_move_calls += 1


class _SpeakerHandler(BaseHTTPRequestHandler):
    posts = []

    def log_message(self, *a):
        pass

    def do_POST(self):
        payload = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
        type(self).posts.append(payload)
        out = {"ok": True, "card": payload.get("card"), "action": payload.get("action"),
               "state": "playing", "applied": {"played_bytes": 64}}
        raw = json.dumps(out).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


@pytest.fixture
def speaker_server():
    _SpeakerHandler.posts = []
    server = ThreadingHTTPServer(("127.0.0.1", 0), _SpeakerHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield server
    server.shutdown()
    server.server_close()


def make_card(client=None, tmp_path=None, **overrides):
    module = importlib.import_module("unitree.go1.bai_nian")
    cfg = {
        "camera_position": "front",
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": 9201}},
        "speaker_host": "127.0.0.1", "speaker_port": 18083,
        "greet_times": 1, "cooldown_s": 30, "detect_interval_s": 0.2,
        "audio_pcm": str(tmp_path / "voice.pcm") if tmp_path else "resource/missing.pcm",
    }
    cfg.update(overrides)
    return module, module.BaiNianPlugin(cfg, "test", None, client or FakeClient())


def write_audio(tmp_path, frames=8000):
    path = tmp_path / "voice.pcm"
    path.write_bytes(b"\x01\x00" * frames)
    return path


def test_manifest_lists_bai_nian_card():
    bundle_dir = Path(__file__).resolve().parents[1]
    driver = (bundle_dir / "driver.yaml").read_text()
    config = (bundle_dir / "config.yaml").read_text()
    assert "name: bai_nian" in driver
    assert "bai_nian:" in config
    dockerfile = (bundle_dir / "Dockerfile").read_text()
    assert "bai_nian.py" in dockerfile


def test_go1_bundle_exposes_bai_nian_tool():
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle(
        {"plugins": {"bai_nian": {"enabled": True, "greet_times": 1}}},
        "test", None, None)
    tools = {tool["name"]: tool for tool in bundle.get_all_tools()}
    assert tools["bai_nian"]["type"] == "actuator"
    info = bundle.dispatch("bai_nian", {"action": "info"})
    assert info["ok"] is True
    assert info["audio_loaded"] is False


def test_capture_frame_protocol_reads_single_jpeg():
    jpeg = b"\xff\xd8frame\xff\xd9"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def serve():
        connection, _ = server.accept()
        connection.sendall(struct.pack(">I", len(jpeg)) + jpeg)
        connection.close()
        server.close()

    threading.Thread(target=serve, daemon=True).start()
    module, card = make_card()
    card._endpoints["front"] = ("127.0.0.1", port)
    assert card._capture_jpeg("front") == jpeg


def test_detect_once_reports_person(monkeypatch, tmp_path):
    module, card = make_card(tmp_path=tmp_path)
    monkeypatch.setattr(module, "_HAS_CV2", True)
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: JPEG)
    people = [{"box": [0.2, 0.1, 0.3, 0.8], "confidence": 1.2}]
    monkeypatch.setattr(module, "_detect_people", lambda jpeg, c: people)
    result = card.dispatch("detect_once", {})
    assert result["ok"] is True
    assert result["person_detected"] is True
    assert result["count"] == 1
    assert result["close_enough"] is True
    assert result["people"][0]["confidence"] == 1.2
    assert card.dispatch("info", {})["last_detection"]["person_detected"] is True


def test_detect_once_reports_no_person(monkeypatch, tmp_path):
    module, card = make_card(tmp_path=tmp_path)
    monkeypatch.setattr(module, "_HAS_CV2", True)
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: JPEG)
    monkeypatch.setattr(module, "_detect_people", lambda jpeg, c: [])
    result = card.dispatch("detect_once", {})
    assert result["ok"] is True
    assert result["person_detected"] is False
    assert result["count"] == 0
    assert result["close_enough"] is False
    assert result["max_confidence"] == 0.0


def test_detect_once_requires_cv2(monkeypatch, tmp_path):
    module, card = make_card(tmp_path=tmp_path)
    monkeypatch.setattr(module, "_HAS_CV2", False)
    result = card.dispatch("detect_once", {})
    assert result["code"] == "NO_DEPENDENCY"


def test_detect_once_rejects_unknown_position(tmp_path):
    _, card = make_card(tmp_path=tmp_path)
    result = card.dispatch("detect_once", {"position": "tail"})
    assert result["code"] == "INVALID_ARGUMENT"


def test_detect_once_position_occupied(monkeypatch, tmp_path):
    module, card = make_card(tmp_path=tmp_path)
    monkeypatch.setattr(module, "_HAS_CV2", True)
    camera = importlib.import_module("unitree.go1.camera")
    camera._SNAPSHOT_POSITIONS.add("front")
    try:
        result = card.dispatch("detect_once", {})
        assert result["code"] == "RESOURCE_BUSY"
    finally:
        camera._SNAPSHOT_POSITIONS.discard("front")


def test_greet_sequence_and_speech(speaker_server, tmp_path):
    port = speaker_server.server_address[1]
    audio = write_audio(tmp_path)
    client = FakeClient(fresh=True, mode=1)
    _, card = make_card(client=client, tmp_path=tmp_path, speaker_port=port,
                        volume_percent=60, greet_times=1)
    assert card.start()["state"] == "ready"
    result = card.dispatch("greet", {})
    assert result["ok"] is True
    assert result["applied"]["completed"] is True
    assert result["applied"]["speech"]["ok"] is True

    pitches = [euler[1] for (_, euler, _) in client.postures]
    assert any(abs(pitch - 0.4) < 0.05 for pitch in pitches), "bow must pitch forward"
    assert client.postures[-1][0] == 1, "must return to balance stand"

    actions = [post["action"] for post in _SpeakerHandler.posts]
    assert "set_volume" in actions
    assert "play" in actions
    volume = next(p for p in _SpeakerHandler.posts if p["action"] == "set_volume")
    assert volume["volume_percent"] == 60
    play = next(p for p in _SpeakerHandler.posts if p["action"] == "play")
    assert play["sample_rate"] == 16000 and play["channels"] == 1
    assert base64.b64decode(play["pcm_base64"]) == audio.read_bytes()

    info = card.dispatch("info", {})
    assert info["audio_loaded"] is True
    assert info["last_greet"]["completed"] is True
    assert info["last_speak"]["ok"] is True


def test_greet_without_audio_only_bows(speaker_server, tmp_path):
    port = speaker_server.server_address[1]
    client = FakeClient(fresh=True, mode=1)
    _, card = make_card(client=client, tmp_path=tmp_path, speaker_port=port)
    assert card.start()["state"] == "ready"  # audio missing → not loaded
    result = card.dispatch("greet", {})
    assert result["ok"] is True
    assert result["applied"]["speech"]["ok"] is False
    assert result["applied"]["speech"]["code"] == "AUDIO_UNAVAILABLE"
    assert card.dispatch("info", {})["audio_loaded"] is False


def test_greet_rejected_in_stub(tmp_path):
    client = FakeClient(fresh=False)
    _, card = make_card(client=client, tmp_path=tmp_path)
    result = card.dispatch("greet", {})
    assert result["code"] == "STUB_MODE"


def test_greet_busy_when_already_running(tmp_path):
    client = FakeClient(fresh=True, mode=1)
    _, card = make_card(client=client, tmp_path=tmp_path)
    card._lock.acquire()
    try:
        result = card.dispatch("greet", {})
        assert result["code"] == "PRECONDITION_FAILED"
    finally:
        card._lock.release()


def test_arm_requires_confirm(tmp_path):
    client = FakeClient(fresh=True, mode=1)
    _, card = make_card(client=client, tmp_path=tmp_path)
    assert card.dispatch("arm", {})["code"] == "PRECONDITION_FAILED"
    assert card.dispatch("info", {})["auto_armed"] is False


def test_arm_rejected_in_stub(tmp_path):
    client = FakeClient(fresh=False)
    _, card = make_card(client=client, tmp_path=tmp_path)
    result = card.dispatch("arm", {"confirm": True})
    assert result["code"] == "STUB_MODE"


def test_auto_loop_greets_once_then_cooldown(monkeypatch, tmp_path):
    client = FakeClient(fresh=True, mode=1)
    module, card = make_card(client=client, tmp_path=tmp_path,
                             detect_interval_s=0.2, cooldown_s=60, greet_times=1)
    card.start()
    greets = []

    def fake_perform_greet(source):
        greets.append(source)
        card._last_greet_monotonic = time.monotonic()
        return {"ok": True, "source": source}

    monkeypatch.setattr(card, "_perform_greet", fake_perform_greet)
    monkeypatch.setattr(card, "_detect_internal",
                        lambda position: {"ok": True, "person_detected": True, "count": 1})
    assert card.dispatch("arm", {"confirm": True})["ok"] is True
    assert card.dispatch("info", {})["auto_armed"] is True
    time.sleep(1.0)
    card.dispatch("disarm", {})
    assert greets == ["auto"], "must greet exactly once then respect cooldown"
    assert card.dispatch("info", {})["auto_armed"] is False


def test_auto_loop_without_person_does_not_greet(monkeypatch, tmp_path):
    client = FakeClient(fresh=True, mode=1)
    _, card = make_card(client=client, tmp_path=tmp_path, detect_interval_s=0.2)
    card.start()
    greets = []

    def fake_perform_greet(source):
        greets.append(source)
        return {"ok": True}

    monkeypatch.setattr(card, "_perform_greet", fake_perform_greet)
    monkeypatch.setattr(card, "_detect_internal",
                        lambda position: {"ok": True, "person_detected": False, "count": 0})
    assert card.dispatch("arm", {"confirm": True})["ok"] is True
    time.sleep(0.8)
    card.dispatch("disarm", {})
    assert greets == []


def test_stop_disarms_and_recovers(monkeypatch, tmp_path):
    client = FakeClient(fresh=True, mode=1)
    _, card = make_card(client=client, tmp_path=tmp_path, detect_interval_s=0.2)
    card.start()
    monkeypatch.setattr(card, "_perform_greet", lambda source: {"ok": True})
    monkeypatch.setattr(card, "_detect_internal",
                        lambda position: {"ok": True, "person_detected": False, "count": 0})
    card.dispatch("arm", {"confirm": True})
    result = card.dispatch("stop", {})
    assert result["ok"] is True
    assert result["state"] == "idle"
    assert result["applied"]["auto_disarmed"] is True
    assert client.stop_move_calls >= 1
    assert client.postures[-1][0] == 1
    assert card.dispatch("info", {})["auto_armed"] is False
