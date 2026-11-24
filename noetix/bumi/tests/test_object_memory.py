"""Object-memory core tests: scene validation, persistence, merging, matching."""

import importlib.util
import json
import time
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "object_memory_core.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("bumi_object_memory_core", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def module():
    return _load_module()


def _scene(scene_id="s1", now=1700000000.0):
    return {
        "id": scene_id,
        "captured_at": now,
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


# ── Scene JSON ───────────────────────────────────────────────────────────────

def test_decode_scene_accepts_a_scene(module):
    scene = _scene()
    assert module.decode_scene(json.dumps(scene))["id"] == "s1"


@pytest.mark.parametrize("payload", [
    "{not json",
    json.dumps({"id": "x"}),
    json.dumps({"id": "x", "captured_at": 1.0, "edges": []}),
    json.dumps({"id": "x", "captured_at": 1.0, "nodes": [{}], "edges": []}),
])
def test_decode_scene_rejects_malformed_payload(module, payload):
    with pytest.raises(ValueError):
        module.decode_scene(payload)


# ── Memory merge / matching ─────────────────────────────────────────────────

def test_merge_memory_keeps_objects_relations_and_sightings(module):
    memory = module.empty_memory()
    module.merge_memory(memory, _scene(), "/tmp/one.jpg")
    assert set(memory["objects"]) == {"keys", "cup"}
    keys = memory["objects"]["keys"]
    assert keys["sightings"] == 1
    assert keys["evidence"] == "/tmp/one.jpg"
    assert keys["relations"] == [{"relation": "left_of", "target": "cup"}]
    assert memory["scenes"]["s1"]["id"] == "s1"

    module.merge_memory(memory, _scene(scene_id="s2", now=1700000100.0),
                        "/tmp/two.jpg")
    keys = memory["objects"]["keys"]
    assert keys["sightings"] == 2
    assert keys["first_seen"] == 1700000000.0
    assert keys["last_seen"] == 1700000100.0
    assert keys["evidence"] == "/tmp/two.jpg"


def test_match_memory_uses_aliases_and_reports_ambiguity(module):
    memory = module.empty_memory()
    module.merge_memory(memory, _scene(), "/tmp/e.jpg")
    assert module.match_memory("钥匙", memory)["key"] == "keys"

    memory["objects"]["mug"] = {"label": "mug", "last_seen": 1.0}
    result = module.match_memory("杯子", memory)
    assert result["state"] == "ambiguous"
    assert len(result["candidates"]) == 2


# ── Persistence / evidence ──────────────────────────────────────────────────

def test_memory_round_trips_through_disk(module, tmp_path):
    memory = module.empty_memory()
    module.merge_memory(memory, _scene(), "/tmp/e.jpg")
    path = tmp_path / "memory.json"
    module.save_memory(path, memory)
    loaded = module.load_memory(path)
    assert loaded["objects"]["keys"]["label"] == "keys"
    assert loaded["scenes"]


def test_corrupt_memory_degrades_to_empty(module, tmp_path):
    path = tmp_path / "memory.json"
    path.write_text("{not json", encoding="utf-8")
    assert module.load_memory(path) == module.empty_memory()
    path.write_text(json.dumps({"version": 999, "objects": {"x": {}}}), encoding="utf-8")
    assert module.load_memory(path) == module.empty_memory()


def test_save_evidence_writes_a_jpeg(module, tmp_path):
    path = module.save_evidence(tmp_path / "evidence", "scene", b"jpeg-bytes")
    assert Path(path).is_file()
    assert Path(path).read_bytes() == b"jpeg-bytes"
