import math
import struct
import threading
import time
import zlib

import pytest

from rokae.helios_sensors.main import SensorCard
from rokae.helios_sensors.orbbec import HEIGHT, WIDTH, encode_depth
from rokae.helios_sensors.orientation import OrientationFilter


def test_depth_encodes_millimetres_and_preserves_invalid_pixels():
    native = struct.pack("<4H", 0, 2000, 65535, 10) * (WIDTH * HEIGHT // 4)
    assert zlib.decompress(encode_depth(native, 1)) == native
    assert zlib.decompress(encode_depth(native, 0.5))[:8] == struct.pack("<4H", 0, 1000, 32768, 5)


@pytest.mark.parametrize("bad_layout", [True, False], ids=["short-frame", "invalid-scale"])
def test_depth_rejects_invalid_frames(bad_layout):
    raw = b"short" if bad_layout else bytes(WIDTH * HEIGHT * 2)
    scale = 1.0 if bad_layout else float("nan")
    with pytest.raises(ValueError):
        encode_depth(raw, scale)


def test_stale_imu_unsets_measurements():
    card = object.__new__(SensorCard)
    card.kind, card.serial = "imu", "test"
    card.lock, card.stop_capture = threading.RLock(), threading.Event()
    card.thread, card.topic, card.count = object(), {}, 1
    card.frame_id, card.device_stamp, card.error = "test_frame", 123, None
    card.values = {"accel_z_m_s2": 9.8}
    card.received = time.monotonic()
    assert card.payload()["accel_z_m_s2"] == 9.8
    assert card.payload()["orientation_w"] is None
    card.values["orientation_w"] = 1.0
    card.values["orientation_source"] = "accel_gyro_fusion"
    assert card.payload()["orientation_w"] == 1.0
    card.received -= 4
    payload = card.payload()
    assert payload["state"] == "stale"
    assert payload["device_stamp_us"] is None and payload["accel_z_m_s2"] is None
    assert payload["orientation_w"] is None and payload["orientation_source"] is None


@pytest.mark.parametrize("kind,name", [("imu", "imu"), ("depth", "camera_depth")])
def test_sensor_exposes_only_canvas_lifecycle_actions(kind, name):
    card = object.__new__(SensorCard)
    card.kind, card.name, card.topic = kind, name, {"topic": "/test/source", "format": "data/json"}
    tool = card.get_tool()
    actions = tool["inputSchema"]["properties"]["action"]["enum"]
    assert tool["type"] == "sensor"
    assert set(actions) <= {"start", "stop", "info", "config"}


def test_sensor_replies_with_framework_lifecycle_states():
    card = object.__new__(SensorCard)
    calls = []
    card.start = lambda: calls.append("start")
    card.stop = lambda: calls.append("stop")

    assert card.dispatch("start", {}) == {"state": "running"}
    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert calls == ["start", "stop"]


def initialize_orientation(fusion, accel=(0, 0, 9.80665)):
    for stamp in range(20_000, 360_001, 20_000):
        result = fusion.update(stamp, accel, (0, 0, 0))
    return result


def test_orientation_aligns_tilt_to_gravity():
    tilt = math.pi / 3
    accel = (0, 9.80665 * math.sin(tilt), 9.80665 * math.cos(tilt))
    fusion = OrientationFilter()

    assert fusion.update(0, accel, (0, 0, 0)) is None
    assert initialize_orientation(fusion, accel) == pytest.approx(
        (math.sin(tilt / 2), 0, 0, math.cos(tilt / 2)))


def test_orientation_integrates_relative_yaw_and_reinitializes_after_gap():
    fusion = OrientationFilter()
    assert initialize_orientation(fusion) == (0, 0, 0, 1)
    for stamp in range(380_000, 1_360_001, 20_000):
        result = fusion.update(stamp, (0, 0, 9.80665), (0, 0, math.pi / 2))
    assert result == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert fusion.update(1_360_000, (0, 0, 9.80665), (0, 0, 0)) == result
    assert fusion.update(2_000_000, (0, 0, 9.80665), (0, 0, 0)) is None


def test_orientation_rejects_unreliable_gravity_and_corrects_tilt_drift():
    fusion = OrientationFilter()
    for stamp in range(20_000, 360_001, 20_000):
        assert fusion.update(stamp, (0, 0, 25), (0, 0, 0)) is None
    assert fusion.update(380_000, (0, 0, 9.80665), (0, 0, 1)) is None
    for stamp in range(400_000, 760_001, 20_000):
        result = fusion.update(stamp, (0, 0, 9.80665), (0, 0, 0))
    assert result == (0, 0, 0, 1)
    for stamp in range(780_000, 980_001, 20_000):
        result = fusion.update(stamp, (25, 0, 0), (1, 0, 0))
    assert result[0] > 0.1 and result[1:3] == (0, 0)
    for stamp in range(1_000_000, 4_000_001, 20_000):
        result = fusion.update(stamp, (0, 0, 9.80665), (0, 0, 0))
    assert abs(result[0]) < 0.002 and math.hypot(*result) == pytest.approx(1)
