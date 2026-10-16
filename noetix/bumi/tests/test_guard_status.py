"""Hardware-free tests for the read-only Bumi guard status card."""

import json
import re
from pathlib import Path

from noetix.bumi.auto_guard import GuardStatusPlugin


class FakeGuard:
    def __init__(self, info=None, error=None):
        self.info = info or {}
        self.error = error
        self.info_calls = 0

    def _info(self):
        self.info_calls += 1
        if self.error:
            raise self.error
        return self.info


class FakeWhitelist:
    def __init__(self, ready=True, error=None):
        self.ready = ready
        self.error = error
        self.calls = 0

    def is_ready(self):
        self.calls += 1
        if self.error:
            raise self.error
        return self.ready


def test_status_card_advertises_only_read_only_action():
    card = GuardStatusPlugin(FakeGuard(), FakeWhitelist())
    tool = card.get_tool()
    assert tool["name"] == "guard_status"
    assert tool["type"] == "actuator"
    schema = tool["inputSchema"]
    assert set(schema["properties"]) == {"action"}
    assert schema["properties"]["action"]["enum"] == ["status"]
    assert schema["required"] == ["action"]
    # One x-action-params entry is mishandled by Agent Core's split registry;
    # with a plain schema, guard_status remains bindable to decision_core.
    assert "x-action-params" not in schema


def test_status_redacts_biometric_data_and_error_text():
    guard = FakeGuard({
        "state": "running", "pipeline_ready": True, "checked_frames": 7,
        "last_error": "/private/face_whitelist/employee_007/01.jpg",
        "last_event": "known_visitor_arrived", "person_id": "employee_007",
    })
    whitelist = FakeWhitelist(ready=True)
    card = GuardStatusPlugin(guard, whitelist)

    result = card.dispatch("status", {})
    assert result == {
        "state": "ready", "guard_state": "running", "pipeline_ready": True,
        "whitelist_ready": True, "checked_frames": 7,
        "guard_has_error": True,
    }
    encoded = json.dumps(result)
    assert "employee_007" not in encoded
    assert "face_whitelist" not in encoded
    assert "last_event" not in result
    assert guard.info_calls == 1
    assert whitelist.calls == 1
    assert card.dispatch("info", {}) == result


def test_status_fails_closed_without_disclosing_exception():
    card = GuardStatusPlugin(
        FakeGuard(error=RuntimeError("person_123 image read failed")),
        FakeWhitelist(error=RuntimeError("private enroll metadata")),
    )
    result = card.dispatch("status", {})
    assert result["guard_state"] == "unavailable"
    assert result["pipeline_ready"] is False
    assert result["whitelist_ready"] is False
    assert result["guard_has_error"] is True
    assert "person_123" not in json.dumps(result)


def test_canvas_lifecycle_is_no_op_and_unknown_action_is_none():
    guard = FakeGuard()
    whitelist = FakeWhitelist()
    card = GuardStatusPlugin(guard, whitelist)
    card.start()
    assert card.dispatch("start", {"instance_id": "guard-status"}) == {"state": "ready"}
    assert card.dispatch("stop", {"instance_id": "guard-status"}) == {"state": "idle"}
    card.stop()
    assert card.dispatch("activate_motion", {}) is None
    assert guard.info_calls == 0
    assert whitelist.calls == 0


def test_unknown_monitor_state_is_not_echoed():
    card = GuardStatusPlugin(
        FakeGuard({"state": "employee_007", "checked_frames": True}),
        FakeWhitelist(ready=False),
    )
    result = card.dispatch("status", {})
    assert result["guard_state"] == "unavailable"
    assert result["checked_frames"] == 0


def test_image_and_marketplace_wire_status_card():
    driver_dir = Path(__file__).resolve().parents[1]
    main = (driver_dir / "main.py").read_text()
    assert "GuardStatusPlugin(" in main
    assert re.search(
        r"\{\s*name:\s*guard_status,\s*type:\s*actuator\s*\}",
        (driver_dir / "driver.yaml").read_text(),
    )
    # The class shares auto_guard.py, which is already copied into the image.
    assert "COPY auto_guard.py /work/auto_guard.py" in (
        driver_dir / "Dockerfile"
    ).read_text()
