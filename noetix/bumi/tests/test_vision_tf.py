"""Offline tests for the `vision_tf` processor card — no rclpy, no robot.

Naming states intent: each test pins one of the silent-failure rules that makes
a vision-to-tf converter produce plausible but wrong transforms.
"""

from __future__ import annotations

import json
import math
import sys
import types
from pathlib import Path

import pytest

BUMI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUMI))

import vision_tf_spec as spec  # noqa: E402


def _cfg(**overrides):
    base = {"source_name": "ar", "adapter": "pose"}
    base.update(overrides)
    return spec.validate_config(base)


def _run(raw, cfg, receive_ms=1000.0, targets=None):
    return spec.transforms_from_json(
        raw, cfg, receive_ms=receive_ms,
        targets={} if targets is None else targets)


# ── optical input needs no rotation ──────────────────────────────────────────

def test_an_optical_input_needs_no_rotation():
    batch = _run({"position": [1, 2, 3], "orientation": [0, 0, 0, 1],
                  "marker_id": 7}, _cfg())
    edge = batch["transforms"][0]
    assert edge["translation"] == [1.0, 2.0, 3.0]
    assert edge["parent"] == "camera_optical_frame"
    assert edge["rotation"] == [0.0, 0.0, 0.0, 1.0]


def test_a_body_axis_input_attaches_to_the_camera_link_not_base_link():
    cfg = _cfg(input_frame="body")
    batch = _run({"position": [1, 2, 3], "orientation": [0, 0, 0, 1],
                  "marker_id": 7}, cfg)
    assert batch["transforms"][0]["parent"] == "camera_link"


def test_a_base_frame_input_attaches_to_base_link():
    batch = _run({"position": [1, 2, 3], "orientation": [0, 0, 0, 1],
                  "marker_id": 7}, _cfg(input_frame="base"))
    assert batch["transforms"][0]["parent"] == "base_link"


# ── card-side optical->body rotation, when asked for ─────────────────────────

def test_output_body_rotates_optical_axes_and_uses_the_camera_link():
    batch = _run({"position": [1, 2, 3], "orientation": [0, 0, 0, 1],
                  "marker_id": 7}, _cfg(output_frame="body"))
    edge = batch["transforms"][0]
    # (x right, y down, z depth) -> (z, -x, -y) = (3, -1, -2)
    assert edge["translation"] == [3.0, -1.0, -2.0]
    assert edge["parent"] == "camera_link"
    assert edge["rotation"] == [pytest.approx(v, abs=1e-6)
                                for v in spec.OPTICAL_TO_BODY_QUAT]


# ── pose direction / inversion ───────────────────────────────────────────────

def test_inverting_a_pose_twice_returns_the_same_pose():
    t = (0.4, -0.1, 1.3)
    root2 = math.sqrt(0.5)
    q = (0.0, root2, 0.0, root2)  # exactly normalised
    t_inv, q_inv = spec.invert_transform(t, q)
    t_back, q_back = spec.invert_transform(t_inv, q_inv)
    assert all(abs(a - b) < 1e-12 for a, b in zip(t, t_back))
    assert all(abs(a - b) < 1e-12 for a, b in zip(q, q_back))


def test_the_same_scene_in_both_directions_lands_on_the_same_camera_pose():
    # Object 1 m down the optical z axis, no rotation.
    t = (0.0, 0.0, 1.0)
    q = (0.0, 0.0, 0.0, 1.0)
    forward = _run({"position": list(t), "orientation": list(q), "marker_id": 7},
                   _cfg(pose_direction="object_in_camera"))["transforms"][0]
    t_inv, q_inv = spec.invert_transform(t, q)
    inverse = _run({"position": list(t_inv), "orientation": list(q_inv),
                    "marker_id": 7},
                   _cfg(pose_direction="camera_in_object"))["transforms"][0]
    for key in ("translation", "rotation"):
        assert all(abs(a - b) < 1e-9 for a, b in zip(forward[key], inverse[key]))


# ── rvec / quaternion ────────────────────────────────────────────────────────

def test_rodrigues_rotation_matches_quaternion_for_a_90_degree_yaw():
    q = spec.rodrigues_to_quaternion([0.0, math.pi / 2, 0.0])
    assert [round(v, 7) for v in q] == [0.0, 0.7071068, 0.0, 0.7071068]


def test_a_zero_rvec_is_the_identity_rotation():
    assert spec.rodrigues_to_quaternion([0.0, 0.0, 0.0]) == (0.0, 0.0, 0.0, 1.0)


def test_an_ambiguous_rvec_and_quaternion_is_rejected():
    batch = _run({"marker_id": 7, "tvec": [0, 0, 1],
                  "rvec": [0, 0, 0], "quaternion": [0, 0, 0, 1]},
                 _cfg(adapter="ar_marker"))
    assert batch["transforms"] == []
    assert batch["rejected"] == 1
    assert "ambiguous" in batch["last_error"]


def test_an_ar_marker_with_rvec_produces_a_rotation():
    batch = _run({"marker_id": 7, "tvec": [0, 0, 1], "rvec": [0, math.pi / 2, 0]},
                 _cfg(adapter="ar_marker"))
    assert [round(v, 7) for v in batch["transforms"][0]["rotation"]] == [
        0.0, 0.7071068, 0.0, 0.7071068]


# ── "don't guess" rules ──────────────────────────────────────────────────────

def test_a_missing_position_is_rejected_rather_than_zero_filled():
    batch = _run({"orientation": [0, 0, 0, 1], "marker_id": 7}, _cfg())
    assert batch["transforms"] == []
    assert batch["rejected"] == 1
    assert "missing position" in batch["last_error"]


def test_mm_units_are_converted_to_metres():
    batch = _run({"position": [100, 200, 300], "orientation": [0, 0, 0, 1],
                  "marker_id": 7}, _cfg(units="mm"))
    assert batch["transforms"][0]["translation"] == [0.1, 0.2, 0.3]


def test_pixel_depth_backprojects_to_optical_axes():
    batch = _run({"u": 320, "v": 240, "depth_m": 2.0, "id": 3},
                 _cfg(adapter="pixel_depth", focal_px=400))
    # image centre -> x=y=0, z=depth
    assert batch["transforms"][0]["translation"] == [0.0, 0.0, 2.0]


# ── naming ───────────────────────────────────────────────────────────────────

def test_a_child_name_with_a_slash_is_sanitized():
    name, changed = spec.sanitize_child_name("ar", "cam/7")
    assert name == "vision_ar_cam_7"
    assert changed is True


def test_a_numeric_object_id_yields_a_valid_tf_frame_name():
    # The "vision_" prefix already guarantees the name starts with a letter;
    # the important thing is that a bare numeric id is still a valid tf name.
    name, changed = spec.sanitize_child_name("ar", "7")
    assert name == "vision_ar_7"
    assert name[0].isalpha()
    assert changed is False  # already legal, so nothing was replaced


# ── TTL / degraded ───────────────────────────────────────────────────────────

def test_an_object_disappears_after_its_ttl():
    cfg = _cfg(ttl_ms=500)
    targets = {}
    _run({"position": [0, 0, 1], "orientation": [0, 0, 0, 1], "marker_id": 7},
         cfg, receive_ms=1000, targets=targets)
    assert "vision_ar_7" in targets
    batch = _run({"position": [0, 0, 1], "orientation": [0, 0, 0, 1],
                  "marker_id": 8}, cfg, receive_ms=1601, targets=targets)
    assert "vision_ar_7" in batch["dropped"]
    assert "vision_ar_7" not in targets


def test_an_input_timestamp_is_kept_verbatim_not_replaced_by_now():
    batch = _run({"position": [0, 0, 1], "orientation": [0, 0, 0, 1],
                  "marker_id": 7, "stamp_ms": 12345},
                 _cfg(), receive_ms=99999)
    assert batch["transforms"][0]["stamp_ms"] == 12345.0
    assert batch["transforms"][0]["stamp_source"] == "input"


def test_all_rejected_input_is_flagged_degraded():
    batch = _run({"marker_id": 7}, _cfg())
    assert batch["degraded"] == "all_rejected"


# ── config validation (raise, don't clamp) ───────────────────────────────────

@pytest.mark.parametrize("bad", [
    {"adapter": "bogus"}, {"ttl_ms": 1}, {"min_confidence": 2},
    {"units": "cm"}, {"pose_direction": "maybe"}, {"output_frame": "nav"},
])
def test_out_of_range_config_is_rejected_at_validation(bad):
    with pytest.raises(ValueError):
        _cfg(**bad)


# ── payload contract ─────────────────────────────────────────────────────────

def test_payload_reports_xyzw_and_the_resolved_parent_frame():
    cfg = _cfg()
    batch = _run({"position": [0, 0, 1], "orientation": [0, 0, 0, 1],
                  "marker_id": 7}, cfg)
    payload = spec.build_payload(cfg, batch)
    assert payload["rotation_order"] == "xyzw"
    assert payload["units"] == "m"
    assert payload["parent_frame"] == "camera_optical_frame"
    assert payload["source"] == "ar"


# ── card: start without input_topic must not claim running ───────────────────

def _load_card(monkeypatch):
    """Import vision_tf_card with rclpy/stub messages injected."""
    fake_rclpy = types.ModuleType("rclpy")
    fake_node = types.ModuleType("rclpy.node")
    fake_qos = types.ModuleType("rclpy.qos")

    class FakeNode:
        def __init__(self, name):
            self.name = name
        def create_publisher(self, *a, **k):
            return types.SimpleNamespace(publish=lambda msg: None)
        def create_subscription(self, *a, **k):
            return object()
        def destroy_subscription(self, sub):
            pass
        def get_logger(self):
            return types.SimpleNamespace(warning=lambda *a: None,
                                         info=lambda *a: None)

    fake_node.Node = FakeNode
    for policy in ("ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(fake_qos, policy,
                types.SimpleNamespace(BEST_EFFORT=1, KEEP_LAST=1, VOLATILE=1))
    fake_qos.QoSProfile = lambda **k: object()

    fake_std = types.ModuleType("std_msgs")
    fake_msg = types.ModuleType("std_msgs.msg")
    class FakeString:
        def __init__(self):
            self.data = ""
    fake_msg.String = FakeString
    fake_std.msg = fake_msg

    for name, mod in (("rclpy", fake_rclpy), ("rclpy.node", fake_node),
                      ("rclpy.qos", fake_qos), ("std_msgs", fake_std),
                      ("std_msgs.msg", fake_msg)):
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.syspath_prepend(str(BUMI))
    sys.modules.pop("vision_tf_card", None)
    import vision_tf_card as card
    return card


class _FakeExecutor:
    def add_node(self, node):
        pass


def test_start_without_an_input_topic_reports_idle_not_running(monkeypatch):
    card = _load_card(monkeypatch)
    plugin = card.VisionTfPlugin({"enabled": True, "source_name": "ar"},
                                 "bumi_test", _FakeExecutor())
    result = plugin.dispatch("start", {})
    assert result == {"state": "idle", "degraded": "no_input_topic"}
    plugin.stop()


def test_one_message_then_silent_does_not_republish_forever(monkeypatch):
    """P1 regression: a silent source must not replay one sample forever, and
    its target must expire after ttl_ms measured from the real observation.
    """
    card = _load_card(monkeypatch)
    plugin = card.VisionTfPlugin(
        {"enabled": True, "source_name": "ar", "adapter": "pose", "ttl_ms": 200},
        "bumi_test", _FakeExecutor())

    published = []
    plugin._pub.publish = lambda msg: published.append(json.loads(msg.data))

    # A controllable clock (seconds). One message arrives at t=0.
    now = [0.0]
    monkeypatch.setattr(card.time, "time", lambda: now[0])

    result = plugin.dispatch("start", {"input_topic": "/bumi_test/ar/poses"})
    assert result["state"] == "running"
    token = plugin._run_token

    # Simulate exactly one callback delivering one AR pose.
    payload = json.dumps(
        {"marker_id": 3, "position": [1, 2, 3], "orientation": [0, 0, 0, 1]})
    with plugin._frame_lock:
        plugin._latest_raw = payload
        plugin._latest_at = 0.0
        plugin._seq += 1

    # First tick converts it -> one publish with the target.
    plugin._tick(token)
    assert len(published) == 1
    assert [t["child"] for t in published[-1]["transforms"]] == ["vision_ar_3"]

    # Many silent ticks inside the TTL -> no republish of the stale transform.
    for _ in range(20):
        now[0] = 0.05  # 50 ms later, well within ttl_ms=200
        plugin._tick(token)
    assert len(published) == 1, "silent source replayed a stale transform"

    # Past the TTL -> one dropped event fires, then silence stays silent.
    now[0] = 0.30  # 300 ms > 200 ms TTL
    plugin._tick(token)
    assert len(published) == 2
    assert "vision_ar_3" in published[-1]["dropped"]
    assert published[-1]["transforms"] == []
    plugin._tick(token)
    assert len(published) == 2, "expired target kept emitting dropped events"
    assert plugin._targets == {}
    plugin.stop()


def test_card_declares_single_instance_honestly(monkeypatch):
    """The bundle creates one plugin and passes no instance id. Claiming
    multiInstance would promise two simultaneous sources that cannot coexist."""
    card = _load_card(monkeypatch)
    plugin = card.VisionTfPlugin({"enabled": True, "source_name": "ar"},
                                 "bumi_test", _FakeExecutor())
    assert plugin.get_tool()["multiInstance"] is False
    plugin.stop()


def test_restart_on_same_instance_reconfigures_without_leaking_threads(monkeypatch):
    """A second start() on the single instance replaces the input cleanly."""
    card = _load_card(monkeypatch)
    plugin = card.VisionTfPlugin({"enabled": True, "source_name": "ar"},
                                 "bumi_test", _FakeExecutor())
    first = plugin.dispatch("start", {"input_topic": "/bumi_test/ar/poses"})
    assert first["state"] == "running"
    gen1, token1 = plugin._generation, plugin._run_token
    second = plugin.dispatch("start", {"input_topic": "/bumi_test/ar/poses2"})
    assert second["state"] == "running"
    assert plugin._generation == gen1 + 1
    assert plugin._run_token is not token1
    assert plugin._input_topic == "/bumi_test/ar/poses2"
    plugin.stop()


def test_a_superseded_publish_thread_publishes_nothing(monkeypatch):
    """After stop(), an in-flight worker must not emit a stale batch."""
    card = _load_card(monkeypatch)
    plugin = card.VisionTfPlugin({"enabled": True, "source_name": "ar"},
                                 "bumi_test", _FakeExecutor())
    published = []
    plugin._pub.publish = lambda msg: published.append(msg.data)
    result = plugin.dispatch("start", {"input_topic": "/bumi_test/ar/poses"})
    assert result["state"] == "running"
    old_token = plugin._run_token
    plugin._stop()  # clears the token
    # Simulate an in-flight worker that survived the bounded join.
    plugin._worker_loop(plugin._generation, old_token)
    assert published == []


# ── only optical input may be rotated to body axes ───────────────────────────

def test_body_input_with_body_output_is_not_rotated_twice():
    """Body-axis input already uses machine convention; output=body must pass it
    through, not apply the optical->body rotation a second time."""
    batch = _run({"position": [1, 2, 3], "orientation": [0, 0, 0, 1],
                  "marker_id": 1},
                 _cfg(input_frame="body", output_frame="body"))
    edge = batch["transforms"][0]
    assert edge["translation"] == [1.0, 2.0, 3.0]
    assert edge["rotation"] == [0.0, 0.0, 0.0, 1.0]
    assert edge["parent"] == "camera_link"


def test_base_to_body_is_rejected_at_config_time():
    """Converting base-axis input to body needs a measured camera-to-base
    extrinsic; it must fail validation rather than emit plausible wrong edges."""
    with pytest.raises(ValueError, match="unsupported"):
        _cfg(input_frame="base", output_frame="body")


# ── non-finite numbers never become a transform ──────────────────────────────

@pytest.mark.parametrize("bad_value", ["NaN", "Infinity", "-Infinity"])
def test_non_finite_position_is_rejected_not_emitted(bad_value):
    raw = '{"marker_id": 1, "position": [' + bad_value + ', 0, 0], "orientation": [0,0,0,1]}'
    batch = spec.transforms_from_json(
        json.loads(raw), _cfg(), receive_ms=1000, targets={})
    assert batch["transforms"] == []
    assert batch["rejected"] == 1


@pytest.mark.parametrize("bad_value", ["NaN", "Infinity", "-Infinity"])
def test_non_finite_quaternion_is_rejected_not_emitted(bad_value):
    raw = '{"marker_id": 1, "position": [1,2,3], "orientation": [' + bad_value + ',0,0,1]}'
    batch = spec.transforms_from_json(
        json.loads(raw), _cfg(), receive_ms=1000, targets={})
    assert batch["transforms"] == []
    assert batch["rejected"] == 1


def test_non_finite_confidence_is_rejected():
    batch = spec.transforms_from_json(
        {"marker_id": 1, "position": [1, 2, 3], "orientation": [0, 0, 0, 1],
         "confidence": float("nan")},
        _cfg(), receive_ms=1000, targets={})
    assert batch["transforms"] == []
    assert batch["rejected"] == 1


def test_non_finite_stamp_falls_back_to_receive_time():
    batch = spec.transforms_from_json(
        {"marker_id": 1, "position": [1, 2, 3], "orientation": [0, 0, 0, 1],
         "stamp_ms": float("inf")},
        _cfg(), receive_ms=1000, targets={})
    assert len(batch["transforms"]) == 1
    assert batch["transforms"][0]["stamp_ms"] == 1000.0


def test_non_finite_pixel_depth_is_rejected():
    cfg = _cfg(adapter="pixel_depth")
    batch = spec.transforms_from_json(
        {"id": 1, "u": float("nan"), "v": 100, "depth_m": 2.0},
        cfg, receive_ms=1000, targets={})
    assert batch["transforms"] == []
    assert batch["rejected"] == 1


def test_no_output_contains_a_non_finite_number():
    """Belt and braces: whatever the input, serialised output stays valid JSON."""
    for raw in (
        {"marker_id": 1, "position": [float("inf"), 0, 0], "orientation": [0, 0, 0, 1]},
        {"marker_id": 2, "position": [1, 2, 3], "orientation": [float("nan"), 0, 0, 1]},
        {"marker_id": 3, "position": [1, 2, 3], "orientation": [0, 0, 0, 1],
         "confidence": float("inf")},
    ):
        batch = spec.transforms_from_json(raw, _cfg(), receive_ms=1000, targets={})
        text = json.dumps(spec.build_payload(_cfg(), batch), allow_nan=False)
        assert "NaN" not in text and "Infinity" not in text


# ── Dockerfile COPY guard: a missing COPY means import fails in the image ───

def test_the_spec_and_card_modules_reach_the_image():
    dockerfile = (BUMI / "Dockerfile").read_text(encoding="utf-8")
    assert "COPY vision_tf_spec.py /work/vision_tf_spec.py" in dockerfile
    assert "COPY vision_tf_card.py /work/vision_tf_card.py" in dockerfile


def test_vision_tf_is_not_exposed_as_a_standalone_card():
    """tf is now an implicit capability of the odometry card, not a canvas card.

    The spec/card modules stay in the image (the spec is the library the
    odometry card builds on), but the driver no longer lists either tf card as
    an independently placeable processor.
    """
    import yaml
    driver = yaml.safe_load((BUMI / "driver.yaml").read_text(encoding="utf-8"))
    cards = {c["name"] for c in driver["cards"]}
    assert "vision_tf" not in cards
    assert "tf_tree" not in cards
    assert "rgbd_visual_odometry" in cards
