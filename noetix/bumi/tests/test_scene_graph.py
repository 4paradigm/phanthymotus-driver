"""Scene-graph core tests: VOP parsing, geometry, relations, scene JSON.

The card keeps ROS out of module scope, so these tests exercise the shared core
without rclpy or the camera.
"""

import importlib.util
import json
import math
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "scene_graph_core.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("bumi_scene_graph_core", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def module():
    return _load_module()


KEYS = {"name": "keys", "position": [-0.5, 0.0], "confidence": 0.9}
CUP = {"name": "cup", "position": [0.5, 0.0], "confidence": 0.8}


def _vop_payload(now, objects):
    return json.dumps({"timestamp": now, "count": len(objects), "latency_ms": 12,
                       "objects": objects})


def _scene(module, objects, now=1700000000.0, depth=True):
    import numpy as np
    image_depth = np.full((8, 8), 1000, dtype=np.uint16) if depth else None
    intrinsics = {"fx": 4.0, "fy": 4.0, "ppx": 3.5, "ppy": 3.5}
    return module.build_scene(objects, 8, 8, image_depth, 0.001, intrinsics, now)


# ── VOP payload ──────────────────────────────────────────────────────────────

def test_decode_vop_keeps_only_documented_fields(module):
    now = 1700000000.0
    payload = _vop_payload(now, [dict(KEYS, extra={"text": "x" * 1000})])
    decoded = module.decode_vop(payload, now)
    assert decoded["count"] == 1
    assert decoded["objects"] == [KEYS]


@pytest.mark.parametrize("mutate", [
    lambda payload: payload.update(count=2),
    lambda payload: payload["objects"][0].update(position=[2.0, 0.0]),
    lambda payload: payload["objects"][0].update(confidence=1.5),
    lambda payload: payload.update(timestamp=0),
    lambda payload: payload.pop("objects"),
])
def test_decode_vop_rejects_invalid_payload(module, mutate):
    now = 1700000000.0
    payload = json.loads(_vop_payload(now, [KEYS]))
    mutate(payload)
    with pytest.raises(ValueError):
        module.decode_vop(json.dumps(payload), now)


# ── Scene graph ──────────────────────────────────────────────────────────────

def test_scene_projects_pixels_and_robot_frame(module):
    scene = _scene(module, [KEYS, CUP])
    keys, cup = scene["nodes"]
    assert keys["pixel"] == [2, 4]
    assert cup["pixel"] == [5, 4]
    assert math.isclose(keys["distance_m"], 1.0)
    assert math.isclose(keys["robot"]["forward"], 1.0)
    assert keys["robot"]["left"] > 0 > cup["robot"]["left"]


def test_scene_builds_directional_edges(module):
    scene = _scene(module, [KEYS, CUP])
    assert {"source": "obj1", "relation": "left_of", "target": "obj2"} in scene["edges"]


def test_scene_without_depth_falls_back_to_image_relations(module):
    scene = _scene(module, [KEYS, CUP], depth=False)
    assert all(node["distance_m"] is None and node["robot"] is None
               for node in scene["nodes"])
    assert any(edge["relation"] == "left_of" for edge in scene["edges"])


def test_scene_pixels_stay_inside_the_frame(module):
    scene = _scene(module, [
        {"name": "left", "position": [-1.0, -1.0], "confidence": 0.5},
        {"name": "right", "position": [1.0, 1.0], "confidence": 0.5},
    ])
    assert scene["nodes"][0]["pixel"] == [0, 0]
    assert scene["nodes"][1]["pixel"] == [7, 7]


# ── Scene JSON / evidence targets ───────────────────────────────────────────

def test_decode_scene_accepts_a_scene_graph(module):
    scene = _scene(module, [KEYS])
    assert module.decode_scene(json.dumps(scene))["id"] == scene["id"]


@pytest.mark.parametrize("payload", [
    "{not json",
    json.dumps({"id": "x"}),
    json.dumps({"id": "x", "captured_at": 1.0, "edges": []}),
    json.dumps({"id": "x", "captured_at": 1.0, "nodes": [{}], "edges": []}),
])
def test_decode_scene_rejects_malformed_payload(module, payload):
    with pytest.raises(ValueError):
        module.decode_scene(payload)


def test_annotation_target_marks_only_the_asked_object(module):
    scene = _scene(module, [KEYS, CUP])
    targets = module.annotation_targets(scene, "keys")
    assert [flag for _, flag in targets] == [True, False]
