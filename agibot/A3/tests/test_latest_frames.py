"""A3 latest-frame cache: the seam between the camera mirror and the cards."""

import importlib
from array import array

import pytest

JPEG = b"\xff\xd8payload\xff\xd9"


class Clock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now


@pytest.fixture
def frames():
    return importlib.import_module("agibot.A3.latest_frames")


def make_cache(frames, **registered):
    clock = Clock()
    cache = frames.LatestFrameCache(clock=clock)
    for key, jpeg in registered.items():
        cache.register(key, jpeg=jpeg)
    return cache, clock


def test_put_normalises_the_payload_shapes_both_writers_produce(frames):
    cache, _ = make_cache(frames, camera_chest=True)
    key = "camera_chest"
    for payload in (JPEG, bytearray(JPEG), memoryview(JPEG),
                    array("B", JPEG)):
        assert cache.put(key, payload) is True
        frame = cache.get(key)
        assert frame.data == JPEG
        assert isinstance(frame.data, bytes)


def test_put_accepts_a_message_with_data_attribute(frames):
    cache, _ = make_cache(frames, camera_chest=True)

    class Msg:
        data = JPEG

    assert cache.put("camera_chest", Msg()) is True
    assert cache.get("camera_chest").data == JPEG


def test_unregistered_streams_are_never_copied(frames):
    cache, _ = make_cache(frames, camera_chest=True)
    assert cache.put("camera_other", JPEG) is False
    assert cache.get("camera_other") is None
    assert cache.age("camera_other") is None


def test_jpeg_admission_rejects_broken_payloads_and_keeps_the_old_frame(frames):
    cache, _ = make_cache(frames, camera_chest=True)
    assert cache.put("camera_chest", JPEG) is True
    assert cache.put("camera_chest", b"\xff\xd8half") is False
    assert cache.put("camera_chest", [0xFF, 0xD8]) is False
    assert cache.get("camera_chest").data == JPEG


def test_non_jpeg_streams_accept_compressed_bytes(frames):
    cache, _ = make_cache(frames, camera_depth=False)
    payload = b"zlib-not-a-jpeg"
    assert cache.put("camera_depth", payload) is True
    assert cache.get("camera_depth").data == payload


def test_get_refuses_stale_frames(frames):
    cache, clock = make_cache(frames, camera_chest=True)
    cache.put("camera_chest", JPEG)
    assert cache.get("camera_chest", 0.25) is not None
    clock.now += 3.0
    assert cache.get("camera_chest", 0.25) is None
    assert cache.get("camera_chest") is not None  # 不设上限时仍然可读
    assert cache.age("camera_chest") == pytest.approx(3.0)


def test_a_timestamp_from_the_future_is_refused(frames):
    cache, clock = make_cache(frames, camera_chest=True)
    cache.put("camera_chest", JPEG)
    clock.now -= 5.0  # 时钟回退
    assert cache.get("camera_chest") is None
    assert cache.age("camera_chest") == pytest.approx(-5.0)


def test_registration_is_visible_for_diagnostics(frames):
    cache, _ = make_cache(frames)
    cache.register("camera_b", jpeg=True)
    cache.register("camera_a", jpeg=False)
    assert cache.registered_keys() == ("camera_a", "camera_b")
    assert cache.is_registered("camera_a") is True
    assert cache.is_registered("camera_c") is False


def test_get_before_any_put_is_none(frames):
    cache, _ = make_cache(frames, camera_chest=True)
    assert cache.get("camera_chest") is None
    assert cache.age("camera_chest") is None
