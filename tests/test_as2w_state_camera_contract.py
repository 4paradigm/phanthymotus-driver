"""AS2W sensor contracts: no ROS, DDS transport, camera, or robot required."""
from __future__ import annotations

import json
import queue
import struct
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.camera_info import CameraInfoError, parse as parse_camera
from common.odom import is_fresh, parse_interface
from unitree.as2w.camera_specs import CAMERA_ID, declare, jpeg_dimensions
from unitree.as2w.odom_specs import OdomAdapter

EPOCH_MS = 1_760_000_000_000


def sport(**kwargs):
    fields = dict(mode=2, body_height=0.42, velocity=[0.2, -0.1, 0.05],
                  position=[11, 12, 13], yaw_speed=0.3,
                  stamp=SimpleNamespace(sec=123, nanosec=0))
    fields.update(kwargs)
    return SimpleNamespace(**fields)


@pytest.fixture(scope="module")
def cards():
    # Reuse the existing AS2W no-hardware SDK definitions without leaking
    # sys.modules stubs into unrelated tests in the suite.
    from unitree.as2w.test_driver import _install_device_stubs, _load
    with patch.dict(sys.modules):
        _install_device_stubs()
        device = _load("as2w_state_contract_device", ROOT / "unitree/as2w/device.py")
        multimedia = _load("as2w_state_contract_multimedia", ROOT / "unitree/as2w/multimedia.py")
    return SimpleNamespace(device=device, multimedia=multimedia)


def test_default_has_no_verified_axes_or_pose_despite_nonzero_raw_readings():
    adapter = OdomAdapter()
    interface = adapter.interface()
    sample = adapter.sample(sport(), received_ms=EPOCH_MS)
    assert parse_interface(interface).provides == ()
    assert interface["pose_drift"] == "none"
    assert interface["vendor"]["raw_velocity_frame"] == "unknown"
    assert interface["vendor"]["rate_hz_kind"] == "publish_ceiling"
    assert sample["twist"] == [None] * 6
    assert sample["pose"] is None
    assert sample["vendor"]["position"] == [11, 12, 13]
    assert sample["vendor"]["velocity"] == [0.2, -0.1, 0.05]
    assert sample["vendor"]["verification"] == "unverified"


def test_only_explicit_verified_axes_are_exposed_in_fixed_contract_order():
    adapter = OdomAdapter({"frame": "body", "verified_axes": ["wz", "vx"],
                           "verified_on": "test fixture, not a hardware measurement"})
    sample = adapter.sample(sport(), received_ms=EPOCH_MS)
    assert adapter.interface()["provides"] == ["vx", "wz"]
    assert sample["twist"] == [0.2, None, None, None, None, 0.3]
    assert sample["vendor"]["verification"] == "operator-configured"
    assert sample["vendor"]["verified_on"].startswith("test fixture")


def test_world_velocity_is_not_silently_rotated_or_called_body_velocity():
    adapter = OdomAdapter({"frame": "world", "verified_axes": ["vx", "vy"]})
    assert not parse_interface(adapter.interface()).usable_for_control
    sample = adapter.sample(sport(), received_ms=EPOCH_MS)
    assert sample["frame"] == "world"
    assert sample["twist"][:2] == [0.2, -0.1]


@pytest.mark.parametrize("config", [
    {"verified_axes": ["vx"]}, {"frame": "unknown", "verified_axes": ["wz"]},
    {"frame": "base_link"}, {"frame": "body", "verified_axes": ["wx"]},
    {"frame": "body", "verified_axes": ["vx", "vx"]},
    {"frame": "body", "verified_axes": "vx"},
    {"max_age_ms": 0}, {"max_age_ms": -1}, {"max_age_ms": True},
    {"max_age_ms": float("nan")}, {"max_age_ms": float("inf")}, [],
])
def test_invalid_odom_declarations_fail_before_subscription(config):
    with pytest.raises(ValueError):
        OdomAdapter(config)


def test_bad_or_missing_sdk_values_never_become_zero_or_nonfinite_json():
    adapter = OdomAdapter({"frame": "body", "verified_axes": ["vx", "vy", "vz", "wz"]})
    bad = sport(velocity=[float("nan"), True, "fast"], yaw_speed=float("inf"),
                position=[float("-inf")], body_height=float("nan"))
    sample = adapter.sample(bad, received_ms=EPOCH_MS)
    assert sample["twist"] == [None] * 6
    json.dumps(sample, allow_nan=False)
    assert adapter.sample(SimpleNamespace(), received_ms=EPOCH_MS)["twist"] == [None] * 6


def test_zero_is_preserved_only_on_verified_measured_axes():
    adapter = OdomAdapter({"frame": "body", "verified_axes": ["vx", "wz"]})
    sample = adapter.sample(sport(velocity=[0, 0, 0], yaw_speed=0), received_ms=EPOCH_MS)
    assert sample["twist"] == [0.0, None, None, None, None, 0.0]


def test_boot_relative_timestamp_uses_original_receipt_time_with_provenance():
    sample = OdomAdapter().sample(sport(), received_ms=EPOCH_MS)
    assert sample["stamp_ms"] == EPOCH_MS
    assert sample["vendor"]["received_ms"] == EPOCH_MS
    assert sample["vendor"]["stamp_source"] == "received"
    assert not is_fresh(sample, now_ms=EPOCH_MS + 501, max_age_ms=500)


def test_epoch_sdk_timestamp_is_preserved_and_can_already_be_stale_at_receipt():
    stamp = SimpleNamespace(sec=(EPOCH_MS - 1000) // 1000, nanosec=0)
    sample = OdomAdapter().sample(sport(stamp=stamp), received_ms=EPOCH_MS)
    assert sample["stamp_ms"] == EPOCH_MS - 1000
    assert sample["vendor"]["stamp_source"] == "robot"
    assert not is_fresh(sample, now_ms=EPOCH_MS, max_age_ms=500)


@pytest.mark.parametrize("stamp", [None, SimpleNamespace(sec=10),
    SimpleNamespace(sec=EPOCH_MS // 1000, nanosec=1_000_000_000),
    SimpleNamespace(sec=EPOCH_MS // 1000 + 60, nanosec=0)])
def test_missing_invalid_or_clock_skewed_sdk_stamp_falls_back_to_receipt(stamp):
    sample = OdomAdapter().sample(sport(stamp=stamp), received_ms=EPOCH_MS)
    assert sample["stamp_ms"] == EPOCH_MS
    assert sample["vendor"]["stamp_source"] == "received"


def fake_state_node(cards, config=None):
    node = cards.device._StateNode.__new__(cards.device._StateNode)
    node._odom_adapter = OdomAdapter(config)
    node._latest_lock = threading.Lock()
    node._latest_sport = None
    node._latest_sport_received_ms = None
    node._latest_sport_received_monotonic = None
    node._last_odom_sample = None
    node._sport_generation = 0
    node._publisher_thread = object()
    node.raw_messages, node.odom_messages = [], []
    node.loco = SimpleNamespace(publish=lambda msg: node.raw_messages.append(json.loads(msg.data)))
    node.odom = SimpleNamespace(publish=lambda msg: node.odom_messages.append(json.loads(msg.data)))
    return node


def test_delayed_publication_keeps_dds_receipt_stamp_and_legacy_fields(cards):
    node = fake_state_node(cards)
    with patch.object(cards.device.time, "time", return_value=EPOCH_MS / 1000), \
            patch.object(cards.device.time, "monotonic", return_value=20):
        node._on_sport(sport())
    assert node.odom_messages == []  # callback does not publish/serialize
    with patch.object(cards.device.time, "time", return_value=(EPOCH_MS + 1000) / 1000), \
            patch.object(cards.device.time, "monotonic", return_value=21):
        node._publish_sport(node._latest_sport, received_ms=node._latest_sport_received_ms)
        status = node.odom_status()
    assert node.raw_messages == [{"mode": 2, "body_height": 0.42,
        "yaw_speed": 0.3, "timestamp": EPOCH_MS / 1000,
        "velocity_0": 0.2, "velocity_1": -0.1, "velocity_2": 0.05,
        "position_0": 11.0, "position_1": 12.0, "position_2": 13.0}]
    assert node.odom_messages[0]["stamp_ms"] == EPOCH_MS
    assert status["received_samples"] == 1
    assert status["sample_age_ms"] == 1000
    assert status["receive_age_ms"] == 1000
    assert status["fresh"] is False


def test_snapshot_is_detached_and_never_renews_stamp(cards):
    node = fake_state_node(cards)
    assert node.odom_snapshot() is None
    node._publish_sport(sport(), received_ms=EPOCH_MS)
    plugin = cards.device.StatePlugin.__new__(cards.device.StatePlugin)
    plugin._state = node
    first = plugin.odom_snapshot()
    first["twist"][0] = 999
    first["vendor"]["velocity"][0] = 999
    first["stamp_ms"] += 1
    second = plugin.odom_snapshot()
    assert second["twist"] == [None] * 6
    assert second["vendor"]["velocity"][0] == 0.2
    assert second["stamp_ms"] == EPOCH_MS
    plugin._state = None
    assert plugin.odom_snapshot() is None


def test_publish_loop_does_not_repeat_or_refresh_an_unchanged_dds_sample(cards):
    node = fake_state_node(cards)
    node._latest_low = node._latest_bms = None
    node._low_generation = node._bms_generation = 0
    node._published_low_generation = node._published_bms_generation = -1
    node._published_sport_generation = -1
    iterations = []
    node._stop_event = SimpleNamespace(
        is_set=lambda: len(iterations) >= 2,
        wait=lambda _delay: iterations.append(True))
    with patch.object(cards.device.time, "time", return_value=EPOCH_MS / 1000):
        node._on_sport(sport())
    with patch.object(cards.device.time, "time", return_value=(EPOCH_MS + 5000) / 1000):
        node._publish_loop()
    assert len(iterations) == 2
    assert len(node.raw_messages) == len(node.odom_messages) == 1
    assert node.odom_messages[0]["stamp_ms"] == EPOCH_MS


def test_no_received_sample_is_not_fresh(cards):
    status = fake_state_node(cards).odom_status()
    assert status["received_samples"] == 0
    assert status["sample_age_ms"] is None
    assert status["fresh"] is False


def test_state_tool_and_info_keep_old_port_and_add_standard_odom(cards):
    with patch.object(cards.device, "_StateNode", return_value=fake_state_node(cards)):
        plugin = cards.device.StatePlugin({}, "test", object())
    tools = {tool["name"]: tool for tool in plugin.get_tools()}
    expected = [{"topic": "/test/loco/state", "format": "data/json"},
                {"topic": "/test/state/odom", "format": "state/odom"}]
    assert tools["loco_state"]["topic_out"] == expected
    for name in ("imu", "joints", "joint_state", "battery"):
        assert len(tools[name]["topic_out"]) == 1
    info = plugin.dispatch("info", {"_tool_name": "loco_state"})
    assert info["topic_out"] == expected
    assert info["odom_interface"]["provides"] == []
    assert info["odom_status"]["fresh"] is False
    plugin._state = None
    info = plugin.dispatch("info", {"_tool_name": "loco_state"})
    assert info["state"] == "idle"
    assert info["odom_status"]["fresh"] is False


def test_explicit_odom_config_survives_node_recreation(cards):
    config = {"frame": "body", "verified_axes": ["vx"]}
    with patch.object(cards.device, "_StateNode") as state:
        plugin = cards.device.StatePlugin({"odom": config}, "test", "executor")
        plugin.stop()
        plugin.start()
    assert state.call_count == 2
    for call in state.call_args_list:
        assert call.kwargs["odom_config"] == config
    assert plugin._odom_interface()["provides"] == ["vx"]


def test_camera_default_preserves_identity_without_invented_geometry():
    raw = declare("/test/camera/front")[0]
    camera = parse_camera(raw)
    assert camera.id == CAMERA_ID
    assert camera.topic == "/test/camera/front"
    assert camera.format == "image/jpeg"
    assert not camera.known
    assert camera.width is camera.height is camera.K is camera.D is None
    assert camera.half_fov_rad is camera.half_fov_v_rad is None
    assert camera.source == camera.distortion_model == "unknown"


def test_configured_camera_geometry_has_explicit_provenance():
    config = {"width": 1280, "height": 720, "half_fov_rad": 0.6,
              "source": "measured", "measured_on": "synthetic test fixture"}
    raw = declare("/test/camera/front", config)[0]
    assert raw["half_fov_rad"] == 0.6
    assert raw["half_fov_v_rad"] is None  # not inferred from aspect ratio
    assert raw["source"] == "measured"
    assert raw["vendor"]["geometry_verification"] == "operator-configured"
    assert raw["vendor"]["dimensions_source"] == "configured"


@pytest.mark.parametrize("config", [
    {"half_fov_rad": 0.6}, {"source": "measured"},
    {"width": 1.5}, {"height": 0}, {"width": float("inf")},
    {"half_fov_rad": float("nan"), "source": "manual"},
    {"half_fov_rad": 2, "source": "manual"},
    {"K": [1] * 9, "source": "manual"},
    {"width": 640, "height": 480, "K": [0] * 9, "source": "manual"},
    {"D": [float("nan")]}, {"D": "invalid"}, [],
])
def test_invalid_camera_declarations_are_rejected(config):
    with pytest.raises(CameraInfoError):
        declare("/test/camera/front", config)


def test_camera_info_is_available_when_idle_without_starting_capture(cards):
    with patch.object(cards.multimedia._CameraNode, "start_capture",
                      side_effect=AssertionError("must not start camera for metadata")):
        plugin = cards.multimedia.CameraPlugin({}, "test", None)
        tool = plugin.get_tool()
        info = plugin.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["last_frame_ago_ms"] == -1
    assert info["camera_info"] == tool["camera_info"]
    assert info["camera_info"][0]["source"] == "unknown"
    assert info["topic_out"] == [{"topic": "/test/camera/front", "format": "image/jpeg"}]


def jpeg_header(width=1280, height=720, marker=0xC0):
    # SOI, a length-delimited APP segment, and a legal one-component SOF.
    return (b"\xff\xd8\xff\xe1\x00\x06ABCD\xff" + bytes([marker])
            + struct.pack(">HBHHBBBB", 11, 8, height, width, 1, 1, 0x11, 0)
            + b"\xff\xd9")


@pytest.mark.parametrize("marker", [0xC0, 0xC1, 0xC2])
def test_jpeg_dimensions_are_read_from_baseline_and_progressive_headers(marker):
    assert jpeg_dimensions(jpeg_header(1920, 1080, marker)) == (1920, 1080)


def test_unknown_or_truncated_jpeg_never_gets_an_assumed_size():
    header = jpeg_header()
    for end in range(len(header) - 2):
        assert jpeg_dimensions(header[:end]) is None
    assert jpeg_dimensions(b"not a JPEG") is None
    assert jpeg_dimensions(b"\xff\xd8\xff\xda\x00\x02" + header) is None
    assert jpeg_dimensions(jpeg_header(0, 720)) is None
    assert jpeg_dimensions(None) is None


def test_observed_size_fills_unknown_dimensions_but_not_unknown_optics():
    camera = declare("/test/camera/front", observed_dimensions=(1920, 1080))[0]
    assert (camera["width"], camera["height"]) == (1920, 1080)
    assert camera["source"] == "unknown"
    assert camera["half_fov_rad"] is camera["K"] is camera["D"] is None
    assert camera["vendor"]["dimensions_source"] == "jpeg-header"
    configured = declare("/test/camera/front", {"width": 640, "height": 480},
                         observed_dimensions=(1920, 1080))[0]
    assert (configured["width"], configured["height"]) == (640, 480)
    assert configured["vendor"]["dimensions_source"] == "configured"


def test_camera_info_inherits_published_dimensions_and_keeps_frame_age(cards):
    plugin = cards.multimedia.CameraPlugin({}, "test", None)
    plugin._node = SimpleNamespace(status=lambda: {
        "state": "running", "frames": 2, "last_frame_ago_ms": 23,
        "frame_width": 1280, "frame_height": 720,
    })
    info = plugin.dispatch("info", {})
    assert info["last_frame_ago_ms"] == 23
    camera = info["camera_info"][0]
    assert (camera["width"], camera["height"]) == (1280, 720)
    assert camera["source"] == "unknown"
    assert plugin.get_tool()["camera_info"] == info["camera_info"]


def test_camera_worker_reports_dimensions_of_published_frame_without_image_decode(cards):
    control, results = queue.Queue(), queue.Queue()
    frame = jpeg_header(960, 540)
    published = []
    fake_node = SimpleNamespace(
        create_publisher=lambda *_args: SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        destroy_node=lambda: None,
    )

    def fake_capture(frames, statuses, stop, *_args):
        statuses.put(("running", ""))
        frames.put(frame)
        stop.wait(3)

    fake_ros = SimpleNamespace(init=lambda **kwargs: None, shutdown=lambda: None)
    with patch.dict(sys.modules, {"rclpy": fake_ros}), \
            patch.object(cards.multimedia, "_install_logsafe"), \
            patch.object(cards.multimedia, "Node", return_value=fake_node), \
            patch.object(cards.multimedia, "CompressedImage", side_effect=lambda: SimpleNamespace(header=SimpleNamespace())), \
            patch.object(cards.multimedia, "_camera_worker", side_effect=fake_capture):
        worker = threading.Thread(target=cards.multimedia._camera_process,
            args=(control, results, "/test/camera/front", "unused", 10, 0.2, 0.2), daemon=True)
        worker.start()
        try:
            assert results.get(timeout=2)["id"] == "ready"
            control.put((1, "start", None))
            assert results.get(timeout=2)["ok"]
            deadline = time.monotonic() + 2
            status = {}
            while time.monotonic() < deadline:
                control.put((2, "status", None))
                status = results.get(timeout=2)
                if status["frames"]:
                    break
                time.sleep(0.005)
            assert status["frames"] == 1
            assert (status["frame_width"], status["frame_height"]) == (960, 540)
            assert status["last_frame_ago_ms"] >= 0
            assert bytes(published[0].data) == frame
        finally:
            control.put((3, "close", None))
            worker.join(timeout=3)
        assert not worker.is_alive()
