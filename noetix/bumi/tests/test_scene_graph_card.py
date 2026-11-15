"""Contract and standalone-output tests for the scene_graph card."""

import json
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _card(config=None, camera_plugin=None):
    from scene_graph_card import SceneGraphPlugin
    return SceneGraphPlugin(config or {}, "robot", None,
                            camera_plugin=camera_plugin)


class FakeCamera:
    def get_tools(self):
        return [
            {"name": "camera", "topic_out": [{"topic": "/base/cam/color"}]},
            {"name": "depth", "topic_out": [{"topic": "/base/cam/depth"}]},
        ]

    def dispatch(self, action, args):
        return {"camera_info": [{
            "K": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]}]}


def _scene():
    now = time.time()
    return {
        "id": "abc123",
        "captured_at": now,
        "captured_at_text": "2026-11-16 03:30:12",
        "width": 640,
        "height": 480,
        "depth_available": False,
        "intrinsics": None,
        "depth_scale": 0.001,
        "source": {"rgb_topic": "/robot/camera/color"},
        "nodes": [{
            "id": "obj1", "label": "keys", "label_key": "keys",
            "confidence": 0.9, "position": [0.0, 0.0],
            "pixel": [320, 240], "distance_m": None, "robot": None,
        }],
        "edges": [],
    }


# ── Contract ────────────────────────────────────────────────────────────────

def test_tool_contract():
    tool = _card().get_tool()
    assert tool["name"] == "scene_graph"
    assert tool["type"] == "processor"
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "snapshot", "last", "start", "info"]
    assert tool["inputSchema"]["properties"]["label"]["maxLength"] == 100
    assert tool["inputSchema"]["required"] == ["action"]
    inputs = {item["format"]: item["topic"] for item in tool["topic_in"]}
    assert inputs == {
        "image/jpeg": "/robot/camera/color",
        "image/depth-zlib": "/robot/camera/depth",
        "data/json": "/robot/camera/color/objects",
    }
    outputs = {item["format"]: item["topic"] for item in tool["topic_out"]}
    assert outputs == {
        "data/json": "/robot/scene_graph/state",
        "image/jpeg": "/robot/scene_graph/evidence",
    }
    assert _card().start()["state"] == "unavailable"


def test_card_follows_base_camera_ports_and_intrinsics():
    card = _card(camera_plugin=FakeCamera())
    inputs = {item["format"]: item["topic"] for item in card.get_tool()["topic_in"]}
    assert inputs["image/jpeg"] == "/base/cam/color"
    assert inputs["image/depth-zlib"] == "/base/cam/depth"
    assert card._camera_intrinsics(640, 480) == {
        "fx": 500.0, "fy": 500.0, "ppx": 320.0, "ppy": 240.0}


# ── Standalone output completeness ──────────────────────────────────────────

def test_snapshot_artifacts_are_complete_and_reloadable(tmp_path):
    card = _card({"storage_dir": str(tmp_path)})
    scene = _scene()
    scene_path = card._save_snapshot(scene, b"\xff\xd8fake-jpeg")

    assert Path(scene_path).is_file()
    assert Path(scene["evidence_path"]).is_file()
    saved = json.loads(Path(scene_path).read_text(encoding="utf-8"))
    assert saved["scene_path"] == scene_path
    assert saved["evidence_path"] == scene["evidence_path"]
    assert saved["nodes"][0]["label"] == "keys"
    assert saved["source"]["rgb_topic"] == "/robot/camera/color"
    latest = json.loads((tmp_path / "latest.json").read_text(encoding="utf-8"))
    assert latest["scene_id"] == "abc123"
    assert latest["scene_path"] == scene_path

    # A fresh card (e.g. after a restart) can still answer `last`.
    restored = _card({"storage_dir": str(tmp_path)})
    result = restored.dispatch("last", {})
    assert result["state"] == "ok"
    assert result["scene"]["id"] == "abc123"
    assert result["scene_path"] == scene_path
    assert result["evidence_path"] == scene["evidence_path"]
