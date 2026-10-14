"""Contract tests for the Bumi face_whitelist card without camera hardware."""

import time
from pathlib import Path

import pytest

from noetix.bumi.face_whitelist import (
    FaceWhitelistError,
    FaceWhitelistPlugin,
    OpenCvLbphBackend,
)


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


def test_tool_contract_is_an_on_demand_actuator_with_explicit_actions():
    tool = make_plugin().get_tool()
    assert tool["name"] == "face_whitelist"
    assert tool["type"] == "actuator"
    assert "topic_in" not in tool
    assert "topic_out" not in tool
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "start", "stop", "recognize", "reload", "info",
    ]
    assert set(tool["inputSchema"]["x-action-params"]) == {
        "start", "stop", "recognize", "reload", "info",
    }


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


def test_unknown_action_uses_bundle_unsupported_action_handling():
    assert make_plugin().dispatch("not_an_action", {}) is None


class FakeRecognizer:
    def __init__(self, label=0, distance=20.0):
        self.result = label, distance

    def predict(self, face):
        del face
        return self.result


def make_loaded_backend(crops, distance=20.0):
    backend = OpenCvLbphBackend(Path("/unused"), 60.0, 80, 20)
    backend._recognizer = FakeRecognizer(distance=distance)
    backend._identities = {
        0: {"person_id": "person_001", "display_name": "Test Person"},
    }
    backend._decode = lambda data: data
    backend._face_crops = lambda image: list(crops)
    return backend


@pytest.mark.parametrize(
    ("crops", "distance", "expected"),
    [
        ([], 20.0, "no_face"),
        ([object(), object()], 20.0, "multiple_faces"),
        ([object()], 60.01, "unknown"),
        ([object()], 59.99, "matched"),
    ],
)
def test_backend_outcomes_do_not_need_camera_hardware(crops, distance, expected):
    result = make_loaded_backend(crops, distance).recognize(b"jpeg")
    assert result["result"] == expected
    if expected == "matched":
        assert result["person_id"] == "person_001"
    else:
        assert "person_id" not in result


def test_invalid_enrollment_metadata_is_rejected(tmp_path):
    person_dir = tmp_path / "person_001"
    person_dir.mkdir()
    (person_dir / "person.json").write_text("not-json", encoding="utf-8")
    with pytest.raises(FaceWhitelistError, match="invalid metadata"):
        OpenCvLbphBackend._identity(person_dir)


def test_stale_camera_frame_is_rejected():
    class StaleCamera(FakeCamera):
        def wait_for_color_frame(self, timeout_s):
            assert timeout_s == 5.0
            return {
                "data": b"jpeg",
                "timestamp_ms": (time.time() - 4.0) * 1000,
            }, 1

    plugin = make_plugin(camera=StaleCamera())
    plugin.dispatch("start", {})
    result = plugin.dispatch("recognize", {})
    assert result["state"] == "error"
    assert "stale" in result["error"]


def test_reload_failure_is_visible_in_info_state():
    class FailingBackend(FakeBackend):
        def reload(self):
            raise RuntimeError("model load failed")

    plugin = make_plugin(backend=FailingBackend(ready=False))
    assert plugin.dispatch("start", {}) == {
        "state": "error", "error": "model load failed",
    }
    info = plugin.dispatch("info", {})
    assert info["state"] == "error"
    assert info["error"] == "model load failed"


def test_bundle_manifest_config_and_image_all_ship_the_card():
    driver_dir = Path(__file__).resolve().parents[1]
    assert "face_whitelist:" in (driver_dir / "config.yaml").read_text()
    assert "{ name: face_whitelist,    type: actuator }" in (
        driver_dir / "driver.yaml"
    ).read_text()
    dockerfile = (driver_dir / "Dockerfile").read_text()
    assert "COPY requirements.txt /tmp/bumi-requirements.txt" in dockerfile
    assert "-r /tmp/bumi-requirements.txt" in dockerfile
    assert "COPY face_whitelist.py /work/face_whitelist.py" in dockerfile
    requirements = (driver_dir / "requirements.txt").read_text()
    assert "opencv-contrib-python-headless==4.11.0.86" in requirements
