"""Real RGB output dimensions must be ready before downstream metadata snapshots."""
from __future__ import annotations

import threading
import time

import pytest

from test_camera_lifecycle import (  # same ROS/codec fakes, real worker/lifecycle
    _Camera, _raw_frame, _stubbed_modules, device,
)


def wait_until(predicate, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.002)
    return False


@pytest.fixture
def camera():
    handle = _Camera()
    del handle.plugin._wait_for_first_frame  # exercise the production readiness gate
    handle.plugin._START_FRAME_TIMEOUT_S = 0.3
    yield handle
    handle.close()


def begin_start(camera):
    result = []
    thread = threading.Thread(target=lambda: result.append(camera.plugin.dispatch("start", {})))
    thread.start()
    assert wait_until(lambda: camera.plugin._running)
    return thread, result


def test_first_successful_output_supplies_dimensions_and_preserves_source_header(camera):
    before = camera.plugin.dispatch("info", {})
    assert not before["ready"]
    assert before["camera_info"][0]["width"] is None
    thread, result = begin_start(camera)
    assert thread.is_alive(), "start must not announce ready before output dimensions exist"
    frame = _raw_frame(width=12, height=8)
    camera.feed(frame)
    thread.join(1)
    assert not thread.is_alive()
    info, = result
    assert info["state"] == "running" and info["ready"]
    declaration, = info["camera_info"]
    assert (declaration["width"], declaration["height"]) == (12, 8)
    assert declaration["topic"] == "/host/camera/head"
    assert declaration["id"] == "x-humanoid/tianyi2.0/head/color"
    assert declaration["source"] == "unknown"
    assert declaration["K"] is declaration["D"] is declaration["half_fov_rad"] is None
    assert camera._pub.published[0].header is frame.header
    assert info["source_frame_id"] == "color_optical"


def test_missing_first_frame_is_a_bounded_start_error(camera):
    camera.plugin._START_FRAME_TIMEOUT_S = 0.025
    result = camera.plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert "no RGB frame" in result["error"]
    assert not camera.plugin._running
    assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] is None


def test_stop_cancels_first_frame_wait_without_blocking_for_timeout(camera):
    camera.plugin._START_FRAME_TIMEOUT_S = 5
    thread, result = begin_start(camera)
    camera.plugin.stop()
    thread.join(0.3)
    assert not thread.is_alive()
    assert result[0]["state"] == "idle"
    assert not camera.plugin._running


def test_failed_encoding_cannot_mark_metadata_ready(camera, monkeypatch):
    import cv2
    monkeypatch.setattr(cv2, "imencode", lambda *args: (False, b"bad"))
    thread, result = begin_start(camera)
    camera.feed()
    assert wait_until(lambda: "JPEG encoding failed" in camera.plugin._last_encode_error)
    assert not camera.plugin.dispatch("info", {})["ready"]
    assert not camera._pub.published
    thread.join(1)
    assert result[0]["state"] == "error"


def test_cancelled_starter_does_not_disarm_a_new_session(camera, monkeypatch):
    original = camera.plugin._wait_for_first_frame
    cancelled, release = threading.Event(), threading.Event()

    def wait(generation):
        ready = original(generation)
        if not ready:
            cancelled.set()
            assert release.wait(1)
        return ready

    monkeypatch.setattr(camera.plugin, "_wait_for_first_frame", wait)
    thread, result = begin_start(camera)
    camera.plugin.stop()
    assert cancelled.wait(1)
    camera.plugin.start()
    release.set()
    thread.join(1)
    assert result[0]["state"] == "idle"
    assert camera.plugin._running
    camera.feed(_raw_frame(18, 10))
    assert camera.wait_published(1)


def test_concurrent_starters_share_one_verified_stream(camera):
    first, first_result = begin_start(camera)
    second_result = []
    second = threading.Thread(target=lambda: second_result.append(camera.plugin.dispatch("start", {})))
    second.start()
    camera.feed(_raw_frame(12, 8))
    first.join(1)
    second.join(1)
    assert first_result[0]["ready"] and second_result[0]["ready"]
    assert len(camera.plugin._pub_node.publishers_created) == 1
    assert len(camera.plugin._sub_node.subscriptions_created) == 1


def test_stop_new_start_cannot_interleave_between_arm_and_generation_capture(camera, monkeypatch):
    original_start = camera.plugin.start
    arm_returning, release_arm = threading.Event(), threading.Event()

    def paused_start():
        original_start()
        arm_returning.set()
        assert release_arm.wait(1)

    monkeypatch.setattr(camera.plugin, "start", paused_start)
    old_result = []
    old = threading.Thread(target=lambda: old_result.append(camera.plugin.dispatch("start", {})))
    old.start()
    assert arm_returning.wait(1)
    new_armed = threading.Event()
    new_generation = []

    def replace_session():
        camera.plugin.stop()
        original_start()
        new_generation.append(camera.plugin._frame_generation)
        new_armed.set()

    replacement = threading.Thread(target=replace_session)
    replacement.start()
    interleaved = new_armed.wait(0.05)
    release_arm.set()
    assert new_armed.wait(1)
    old.join(1)
    replacement.join(1)
    assert not interleaved, "arm and generation capture must be one lifecycle operation"
    assert old_result[0]["state"] == "idle"
    assert camera.plugin._running
    assert camera.plugin._frame_generation == new_generation[0]


def test_blocked_bridge_write_does_not_block_stop_or_ready_the_next_session(camera):
    camera.plugin.start()
    publisher, worker = camera.plugin._pub, camera.plugin._encode_thread
    original_publish = publisher.publish
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()

    def blocked_publish(message):
        entered.set()
        assert release.wait(4)
        original_publish(message)

    publisher.publish = blocked_publish
    camera.feed(_raw_frame(12, 8))
    assert entered.wait(1)
    stopping = threading.Thread(target=lambda: (camera.plugin.stop(), stopped.set()))
    stopping.start()
    try:
        assert stopped.wait(0.3), "stop must not wait for a bridge sendall"
        stopping.join(1)
        # start has an existing bounded 2s join, then reuses the blocked worker
        # instead of stacking an encoder or waiting on its network write lock.
        restarting = threading.Thread(target=camera.plugin.start)
        restarting.start()
        restarting.join(2.5)
        assert not restarting.is_alive()
        assert camera.plugin._encode_thread is worker
        assert not camera.plugin.dispatch("info", {})["ready"]
        release.set()
        assert camera.wait_published(1)  # the already-in-flight old output is irrevocable
        assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] is None
        publisher.publish = original_publish
        camera.feed(_raw_frame(18, 10))
        assert camera.wait_published(2)
        assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] == 18
    finally:
        release.set()


def test_restart_requires_new_frame_and_does_not_reuse_previous_dimensions(camera):
    thread, _ = begin_start(camera)
    camera.feed(_raw_frame(12, 8))
    thread.join(1)
    camera.plugin.stop()
    thread, result = begin_start(camera)
    assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] is None
    camera.feed(_raw_frame(18, 10))
    thread.join(1)
    assert result[0]["camera_info"][0]["width"] == 18


def test_inflight_old_frame_cannot_publish_or_ready_a_restarted_stream(camera, monkeypatch):
    import cv2
    original = cv2.imencode
    entered, release = threading.Event(), threading.Event()

    def encode(*args):
        entered.set()
        assert release.wait(2)
        return original(*args)

    monkeypatch.setattr(cv2, "imencode", encode)
    camera.plugin.start()
    camera.feed(_raw_frame(12, 8))
    assert entered.wait(1)
    camera.plugin.stop()
    # Let the previous worker return its old encoding while start is joining it.
    restart = threading.Thread(target=camera.plugin.start)
    restart.start()
    release.set()
    restart.join(1)
    assert not restart.is_alive()
    assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] is None
    assert not camera._pub.published
    camera.feed(_raw_frame(18, 10))
    assert camera.wait_published(1)
    assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] == 18


def test_resolution_change_cannot_silently_invalidate_downstream_snapshot(camera):
    thread, _ = begin_start(camera)
    camera.feed(_raw_frame(12, 8))
    thread.join(1)
    camera.feed(_raw_frame(18, 10))
    assert wait_until(lambda: "resolution changed" in camera.plugin._last_encode_error)
    assert len(camera._pub.published) == 1
    assert camera.plugin.dispatch("info", {})["camera_info"][0]["width"] == 12


@pytest.mark.parametrize("suffix", [
    ' depth_registration:=true color_width:=1280 enable_gyro:=false # keep\n',
    ' enable_point_cloud:=false \\\n  serial_number:="unit with spaces" enable_accel:=false sync_mode:=hardware\n',
    ' "/path/with#inside" enable_accel:="false" & # enable_gyro:=false\n',
    ' label:="unit enable_accel:=false" \'enable_gyro:=false\'&\n',
])
def test_vendor_launch_parameters_and_comments_survive_stream_toggle_update(tmp_path, suffix, capsys):
    launch = "ros2 launch orbbec_camera head_330_ty.launch.py"
    script = tmp_path / "orbbec.sh"
    original = "#!/bin/bash\n# untouched preamble\n  " + launch + suffix
    script.write_text(original)
    updater = device.CameraPlugin._orbbec_startup_updater(str(script), launch)
    exec(updater, {})
    updated = script.read_text()
    assert updated.startswith("#!/bin/bash\n# untouched preamble\n  " + launch)
    for text in ("depth_registration:=true", "color_width:=1280", 'serial_number:="unit with spaces"',
                 "sync_mode:=hardware", '"/path/with#inside"', "& # enable_gyro:=false",
                 'label:="unit enable_accel:=false"'):
        if text in original:
            assert text in updated
    assert "enable_point_cloud:=true" in updated
    assert "enable_accel:=true" in updated
    assert "enable_gyro:=true" in updated
    exec(updater, {})
    assert script.read_text() == updated
    assert capsys.readouterr().out.splitlines() == ["changed", "unchanged"]


def test_unrecognized_vendor_script_is_not_overwritten(tmp_path):
    script = tmp_path / "orbbec.sh"
    original = "# ros2 launch orbbec_camera head_330_ty.launch.py\nother-command\n"
    script.write_text(original)
    with pytest.raises(SystemExit):
        exec(device.CameraPlugin._orbbec_startup_updater(
            str(script), "ros2 launch orbbec_camera head_330_ty.launch.py"), {})
    assert script.read_text() == original
