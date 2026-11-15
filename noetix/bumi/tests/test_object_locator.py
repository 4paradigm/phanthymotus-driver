"""Object-locator core tests: query parsing, matching, answers, evidence targets."""

import importlib.util
import json
import time
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "object_locator_core.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("bumi_object_locator_core", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def module():
    return _load_module()


def _memory(now=1700000000.0):
    return {
        "version": 1,
        "updated_at": now,
        "objects": {
            "keys": {
                "label": "keys", "last_seen": now, "sightings": 2,
                "position": {"forward": 0.8, "left": 0.2, "up": -0.1},
                "pixel": [160, 240], "scene_width": 640, "distance_m": 0.8,
                "scene_id": "s1", "evidence": "/tmp/scene.jpg",
                "relations": [{"relation": "left_of", "target": "cup"}],
            },
            "cup": {
                "label": "cup", "last_seen": now, "sightings": 1,
                "position": {"forward": 0.9, "left": -0.2, "up": -0.1},
                "pixel": [480, 240], "scene_width": 640, "distance_m": 0.9,
                "scene_id": "s1", "evidence": "/tmp/scene.jpg", "relations": [],
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


# ── Question parsing ─────────────────────────────────────────────────────────

@pytest.mark.parametrize("question,expected", [
    ("我的钥匙在哪？", "钥匙"),
    ("我的钥匙在哪里", "钥匙"),
    ("我的钥匙呢", "钥匙"),
    ("钥匙在哪", "钥匙"),
    ("你看到我的钱包了吗", "钱包"),
    ("帮我找一下我的眼镜", "眼镜"),
    ("where is my phone?", "phone"),
])
def test_extract_target(module, question, expected):
    assert module.extract_target(question) == expected


@pytest.mark.parametrize("question", ["", "你好", "今天天气怎么样"])
def test_non_where_question_has_no_target(module, question):
    assert module.extract_target(question) == ""


# ── Matching / answers ──────────────────────────────────────────────────────

def test_chinese_question_matches_english_label(module):
    memory = _memory()
    assert module.match_memory("钥匙", memory)["key"] == "keys"


def test_match_memory_reports_ambiguity(module):
    memory = _memory()
    memory["objects"]["mug"] = {"label": "mug", "last_seen": 1.0}
    result = module.match_memory("杯子", memory)
    assert result["state"] == "ambiguous"
    assert len(result["candidates"]) == 2


def test_answer_found_with_relation_and_evidence(module):
    result = module.answer_question(_memory(), "我的钥匙在哪？")
    assert result["state"] == "found"
    assert result["label"] == "keys"
    assert "钥匙" in result["answer"]
    assert "杯子" in result["answer"]
    assert "左边" in result["answer"]
    assert result["evidence_path"] == "/tmp/scene.jpg"
    assert result["scene_id"] == "s1"


def test_answer_not_found(module):
    result = module.answer_question(_memory(), "我的雨伞在哪？")
    assert result["state"] == "not_found"
    assert result["candidates"] == []


def test_answer_without_target(module):
    result = module.answer_question(_memory(), "今天天气怎么样")
    assert result["state"] == "error"
    assert result["code"] == "NO_TARGET"


def test_annotation_target_marks_only_the_asked_object(module):
    scene = _memory()["scenes"]["s1"]
    targets = module.annotation_targets(scene, "keys")
    assert [flag for _, flag in targets] == [True, False]


def test_load_memory_reads_file(module, tmp_path):
    path = tmp_path / "memory.json"
    path.write_text(json.dumps(_memory()), encoding="utf-8")
    loaded = module.load_memory(path)
    assert loaded["objects"]["keys"]["label"] == "keys"
    assert loaded["scenes"]["s1"]["id"] == "s1"
