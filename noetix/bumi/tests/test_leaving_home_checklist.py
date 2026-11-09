"""ROS-free tests for the checklist card's contract and tool calls."""

from pathlib import Path

import pytest


@pytest.fixture
def card(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    from leaving_home_checklist import LeavingHomeChecklistPlugin

    return LeavingHomeChecklistPlugin({"enabled": True}, "robot", None)


def test_plugin_contract_and_lifecycle(card):
    assert card.get_tool()["name"] == card.PREFIX == "leaving_home_checklist"
    card.start()
    assert card.dispatch("info", {})["default_required_items"] == ["钥匙", "手机", "雨伞"]
    assert card.dispatch("start", {}) == {"state": "ready"}
    assert card.dispatch("stop", {}) == {"state": "idle"}
    card.stop()


def test_default_list(card):
    assert card.dispatch("check", {"carried_items": "钥匙、手机"}) == {
        "missing_items": ["雨伞"], "all_ready": False, "message": "还没带：雨伞",
    }
    assert card.dispatch("check", {})["missing_items"] == ["钥匙", "手机", "雨伞"]


def test_delimiters_and_duplicate_items(card):
    result = card.dispatch("check", {
        "required_items": " 钥匙、手机，雨伞,雨伞\n 手机 ",
        "carried_items": "手机",
    })
    assert result["missing_items"] == ["钥匙", "雨伞"]
    assert card.dispatch("check", {"required_items": "水杯", "carried_items": "水杯"})["all_ready"]


@pytest.mark.parametrize("field", ["required_items", "carried_items"])
def test_rejects_non_text_input(card, field):
    assert card.dispatch("check", {field: ["钥匙"]})["state"] == "invalid_parameters"
