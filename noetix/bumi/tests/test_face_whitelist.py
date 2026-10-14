"""Contract tests for the Bumi face_whitelist card without camera hardware."""

import time
from pathlib import Path

from noetix.bumi.face_whitelist import FaceWhitelistPlugin


class FakeBackend:
    def __init__(self, result=None, ready=True):
        self.result = result or {"result": "matched", "person_id": "person_001"}
        self.ready = ready
        self.reload_count = 0

    def reload(self):
        self.reload_count += 1
        return self.summary()

    def summary(self):
        return {
            "ready": self.ready,
            "identity_count": 1 if self.ready else 0,
            "enrollment_image_count": 3 if self.ready else 0,
            "warnings": [],
        }

    def recognize(self, jpeg):
        assert jpeg == b"jpeg"
        return dict(self.result)


class FakeCamera:
    def __init__(self, running=True):
        self.running = running

    def is_running(self):
        return self.running

    def wait_for_color_frame(self, timeout_s):
        assert timeout_s == 5.0
        return {
            "data": b"jpeg",
            "timestamp_ms": time.time() * 1000,
        }, 1


def make_plugin(backend=None, camera=None):
    return FaceWhitelistPlugin(
        {}, camera or FakeCamera(), backend=backend or FakeBackend()
    )


def test_tool_contract_is_a_processor_with_explicit_actions():
    tool = make_plugin().get_tool()
    assert tool["name"] == "face_whitelist"
    assert tool["type"] == "processor"
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "recognize", "reload", "info",
    ]


def test_start_loads_the_private_whitelist_without_exposing_a_path():
    backend = FakeBackend()
    result = make_plugin(backend=backend).dispatch("start", {})
    assert result["state"] == "ready"
    assert result["identity_count"] == 1
    assert backend.reload_count == 1
    assert "whitelist_dir" not in result


def test_empty_whitelist_is_unconfigured_not_a_bundle_crash():
    result = make_plugin(backend=FakeBackend(ready=False)).dispatch("start", {})
    assert result["state"] == "unconfigured"
    assert result["identity_count"] == 0


def test_recognize_returns_backend_result_from_a_fresh_camera_frame():
    plugin = make_plugin()
    plugin.dispatch("start", {})
    result = plugin.dispatch("recognize", {})
    assert result == {
        "state": "completed",
        "result": "matched",
        "person_id": "person_001",
    }


def test_recognize_refuses_before_start_or_without_camera():
    plugin = make_plugin()
    assert plugin.dispatch("recognize", {})["state"] == "error"

    plugin = make_plugin(camera=FakeCamera(running=False))
    plugin.dispatch("start", {})
    result = plugin.dispatch("recognize", {})
    assert result["state"] == "error"
    assert "camera" in result["error"]


def test_stop_prevents_later_recognition():
    plugin = make_plugin()
    plugin.dispatch("start", {})
    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    assert plugin.dispatch("recognize", {})["state"] == "error"


def test_bundle_manifest_config_and_image_all_ship_the_card():
    driver_dir = Path(__file__).resolve().parents[1]
    assert "face_whitelist:" in (driver_dir / "config.yaml").read_text()
    assert "name: face_whitelist" in (driver_dir / "driver.yaml").read_text()
    dockerfile = (driver_dir / "Dockerfile").read_text()
    assert "COPY face_whitelist.py /work/face_whitelist.py" in dockerfile
    assert "opencv-contrib-python-headless==4.11.0.86" in dockerfile
