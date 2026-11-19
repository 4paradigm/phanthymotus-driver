"""Contract and standalone query tests for the object_locator card."""

import json
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _memory(now=None):
    now = now or time.time()
    return {
        "version": 1,
        "updated_at": now,
        "objects": {
            "keys": {
                "label": "keys", "last_seen": now, "sightings": 2,
                "position": {"forward": 0.8, "left": 0.2, "up": -0.1},
                "pixel": [160, 240], "scene_width": 640, "distance_m": 0.8,
                "scene_id": "s1", "evidence": "",
                "relations": [{"relation": "left_of", "target": "cup"}],
            },
            "cup": {
                "label": "cup", "last_seen": now, "sightings": 1,
                "position": {"forward": 0.9, "left": -0.2, "up": -0.1},
                "pixel": [480, 240], "scene_width": 640, "distance_m": 0.9,
                "scene_id": "s1", "evidence": "", "relations": [],
            },
        },
        "scenes": {
            "s1": {
                "id": "s1", "captured_at": now,
                "captured_at_text": "2026-11-16 03:30:12",
                "width": 640, "height": 480,
                "nodes": [
                    {"id": "obj1", "label": "keys", "label_key": "keys",
                     "confidence": 0.9, "position": [-0.5, 0.0],
                     "pixel": [160, 240], "distance_m": 0.8,
                     "robot": {"forward": 0.8, "left": 0.2, "up": -0.1}},
                    {"id": "obj2", "label": "cup", "label_key": "cup",
                     "confidence": 0.8, "position": [0.5, 0.0],
                     "pixel": [480, 240], "distance_m": 0.9,
                     "robot": {"forward": 0.9, "left": -0.2, "up": -0.1}},
                ],
                "edges": [{"source": "obj1", "relation": "left_of",
                           "target": "obj2"}],
            },
        },
    }


def _card(storage):
    from object_locator_card import ObjectLocatorPlugin
    return ObjectLocatorPlugin({"storage_dir": str(storage)}, "robot", None)


def _write_memory(storage):
    storage.mkdir(parents=True, exist_ok=True)
    (storage / "memory.json").write_text(json.dumps(_memory()), encoding="utf-8")


# ── Contract ────────────────────────────────────────────────────────────────

def test_tool_contract(tmp_path):
    tool = _card(tmp_path).get_tool()
    assert tool["name"] == "object_locator"
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "ask", "list", "last", "info", "start"]
    assert tool["inputSchema"]["required"] == ["action"]
    assert tool["topic_in"] == [{
        "topic": "/robot/object_memory/state", "format": "data/json",
        "message_type": "std_msgs/msg/String",
        "desc": "object_memory 的完整记忆（可选）"}]
    outputs = {item["format"]: item["topic"] for item in tool["topic_out"]}
    assert outputs == {
        "data/json": "/robot/object_locator/answer",
        "image/jpeg": "/robot/object_locator/evidence",
    }
    assert _card(tmp_path).start()["state"] == "unavailable"


# ── Standalone query from memory.json ───────────────────────────────────────

def test_ask_reads_memory_file_and_answers(tmp_path):
    _write_memory(tmp_path)
    card = _card(tmp_path)
    result = card.dispatch("ask", {"question": "我的钥匙在哪？"})
    assert result["state"] == "found"
    assert result["label"] == "keys"
    assert "钥匙" in result["answer"]
    assert "杯子" in result["answer"]
    assert "左边" in result["answer"]
    assert result["memory_source"] == "file"
    assert result["memory_count"] == 2
    assert result["evidence_topic"] == "/robot/object_locator/evidence"

    assert card.dispatch("list", {})["count"] == 2
    assert card.dispatch("last", {})["answer"]["label"] == "keys"
    assert card.dispatch("info", {})["objects"] == 2


def test_ask_prefers_live_memory_topic(tmp_path, monkeypatch):
    _write_memory(tmp_path)
    card = _card(tmp_path)
    card._memory = _memory(now=time.time() - 1)
    card._memory_at = time.time()
    result = card.dispatch("ask", {"question": "我的钥匙在哪？"})
    assert result["memory_source"] == "topic"


def test_ask_not_found_and_no_last_answer(tmp_path):
    _write_memory(tmp_path)
    card = _card(tmp_path)
    assert card.dispatch("last", {})["state"] == "no_answer"
    result = card.dispatch("ask", {"question": "我的雨伞在哪？"})
    assert result["state"] == "not_found"
    assert card.dispatch("last", {})["state"] == "ok"


def test_ask_requires_question(tmp_path):
    card = _card(tmp_path)
    result = card.dispatch("ask", {})
    assert result["state"] == "error"
    assert "question" in result["message"]
