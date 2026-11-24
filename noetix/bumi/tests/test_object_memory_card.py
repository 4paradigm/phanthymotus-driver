"""Contract and standalone persistence tests for the object_memory card."""

import json
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _card(storage):
    from object_memory_card import ObjectMemoryPlugin
    return ObjectMemoryPlugin({"storage_dir": str(storage)}, "robot", None)


def _scene(scene_id="s1", now=None):
    return {
        "id": scene_id,
        "captured_at": now or time.time(),
        "captured_at_text": "2026-11-16 03:30:12",
        "width": 640,
        "height": 480,
        "depth_available": False,
        "nodes": [
            {"id": "obj1", "label": "keys", "label_key": "keys", "confidence": 0.9,
             "position": [-0.5, 0.0], "pixel": [160, 240], "distance_m": 0.8,
             "robot": {"forward": 0.8, "left": 0.2, "up": -0.1}},
            {"id": "obj2", "label": "cup", "label_key": "cup", "confidence": 0.8,
             "position": [0.5, 0.0], "pixel": [480, 240], "distance_m": 0.9,
             "robot": {"forward": 0.9, "left": -0.2, "up": -0.1}},
        ],
        "edges": [{"source": "obj1", "relation": "left_of", "target": "obj2"}],
    }


# ── Contract ────────────────────────────────────────────────────────────────

def test_tool_contract(tmp_path):
    tool = _card(tmp_path).get_tool()
    assert tool["name"] == "object_memory"
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "remember", "list", "get", "forget", "info", "start"]
    assert tool["inputSchema"]["required"] == ["action"]
    inputs = {item["format"]: item["topic"] for item in tool["topic_in"]}
    assert inputs == {
        "data/json": "/robot/scene_graph/state",
        "image/jpeg": "/robot/scene_graph/evidence",
    }
    assert tool["topic_out"] == [{"topic": "/robot/object_memory/state",
                                  "format": "data/json"}]
    assert _card(tmp_path).start()["state"] == "unavailable"


# ── Standalone persistence ──────────────────────────────────────────────────

def test_remember_persists_memory_and_evidence(tmp_path):
    card = _card(tmp_path)
    card._scene = _scene()
    card._scene_at = time.time()
    card._evidence = b"jpeg-bytes"
    card._evidence_at = time.time()

    result = card.dispatch("remember", {})
    assert result["state"] == "remembered"
    assert result["count"] == 2
    assert set(result["labels"]) == {"keys", "cup"}
    assert result["edges"][0]["relation"] == "left_of"
    assert Path(result["evidence_path"]).is_file()
    assert Path(result["memory_path"]).is_file()

    memory = json.loads(Path(result["memory_path"]).read_text(encoding="utf-8"))
    assert set(memory["objects"]) == {"keys", "cup"}
    assert memory["scenes"]["s1"]["id"] == "s1"

    # A fresh card (e.g. after a restart) reads the same memory file.
    restored = _card(tmp_path)
    listed = restored.dispatch("list", {})
    assert listed["count"] == 2
    assert {item["label"] for item in listed["objects"]} == {"keys", "cup"}

    got = restored.dispatch("get", {"label": "钥匙"})
    assert got["state"] == "found"
    assert got["label"] == "keys"
    assert got["entry"]["distance_m"] == 0.8
    assert got["entry"]["relations"][0]["target"] == "cup"


def test_get_and_forget_are_scoped_to_memory(tmp_path):
    card = _card(tmp_path)
    assert card.dispatch("get", {"label": "keys"})["state"] == "not_found"
    assert card.dispatch("forget", {"label": "keys"})["state"] == "not_found"

    card._memory = {
        "version": 1,
        "updated_at": 1.0,
        "objects": {
            "keys": {"label": "keys", "last_seen": 1.0, "sightings": 1},
            "cup": {"label": "cup", "last_seen": 1.0, "sightings": 1},
        },
        "scenes": {},
    }
    assert card.dispatch("get", {"label": "钥匙"})["label"] == "keys"
    assert card.dispatch("forget", {"label": "钥匙"})["removed"] == 1
    assert card.dispatch("list", {})["count"] == 1
    assert card.dispatch("forget", {})["removed"] == 1
    assert card.dispatch("list", {})["count"] == 0


def test_remember_without_fresh_scene_is_refused(tmp_path):
    card = _card(tmp_path)
    result = card.dispatch("remember", {})
    assert result["state"] == "scene_unavailable"
    assert "scene_graph.snapshot" in result["message"]
