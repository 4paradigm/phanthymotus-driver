"""Hardware depth diagnostics must describe the native frame without claiming RGB registration."""
import types
import threading
import time
import zlib

import numpy as np
import pytest

from test_camera_depth import depth_plugin, frame


def color_info(width=640, height=480, frame_id="depth_optical"):
    return types.SimpleNamespace(
        width=width, height=height, distortion_model="plumb_bob",
        header=types.SimpleNamespace(frame_id=frame_id,
                                     stamp=types.SimpleNamespace(sec=1, nanosec=0)),
        k=[689.252685546875, 0, width / 2, 0, 689.331298828125, height / 2, 0, 0, 1],
        d=[0.008, -0.044, -0.0005, 0.00003, 0.03, 0, 0, 0],
        binning_x=0, binning_y=0,
        roi=types.SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0))


def arm_hw_depth(plugin):
    plugin.stop()
    plugin._alignment = "vendor_hw_d2c"
    plugin.start()


def test_hw_d2c_keeps_full_view_and_declares_resized_color_geometry(depth_plugin):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    calibration = color_info(1280, 720, "ob_camera_head_color_optical_frame")
    plugin._on_color_camera_info(calibration)
    pixels = np.full((720, 1280), 1234, np.uint16)
    pixels[:, :8] = 800
    pixels[:, -8:] = 2400
    calls = []
    plugin._cv2.INTER_NEAREST = 0

    def resize(data, size, interpolation):
        calls.append(data.shape)
        y = np.arange(size[1]) * data.shape[0] // size[1]
        x = np.arange(size[0]) * data.shape[1] // size[0]
        return data[y[:, None], x[None, :]]

    plugin._cv2.resize = resize
    message = types.SimpleNamespace(
        width=1280, height=720, step=2560, encoding="16UC1", is_bigendian=0,
        data=pixels.tobytes(), header=calibration.header)
    plugin._on_image(message)
    plugin._publish_latest()
    output, = plugin._pub.messages
    decoded = np.frombuffer(zlib.decompress(output.data), dtype="<u2").reshape(480, 640)
    assert calls == [(720, 1280)]
    assert np.all(decoded[:, 0] == 800)
    assert np.all(decoded[:, -1] == 2400)
    assert output.header is message.header
    assert output.array_assignments == 1 and output.sequence_assignments == 0
    info = plugin.dispatch("info", {})
    assert info["ready"]
    assert info["source_geometry"]["crop_xywh"] == [0, 0, 1280, 720]
    declaration, = info["camera_info"]
    assert declaration["id"] == "x-humanoid/tianyi2.0/head/color"
    assert declaration["K"][0] == pytest.approx(calibration.k[0] / 2)
    assert declaration["K"][4] == pytest.approx(calibration.k[4] * 2 / 3)
    assert declaration["D"] == calibration.d
    assert declaration["vendor"]["registration_source"] == "configured-vendor-hw-d2c"
    assert declaration["vendor"]["pixel_alignment_verified"] is False


@pytest.mark.parametrize("problem", ["missing", "frame", "size", "invalid"])
def test_hw_d2c_refuses_missing_or_mismatched_color_calibration(depth_plugin, problem):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    calibration = color_info()
    if problem == "frame":
        calibration.header.frame_id = "another_lens"
    elif problem == "size":
        calibration.width = 1280
    elif problem == "invalid":
        calibration.k[0] = float("nan")
    if problem != "missing":
        plugin._on_color_camera_info(calibration)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    info = plugin.dispatch("info", {})
    assert not plugin._pub.messages
    assert not info["ready"] and info["geometry_problem"]
    assert "camera_info" not in info


def test_hw_d2c_calibration_stamps_do_not_clear_ready_but_geometry_changes_do(depth_plugin):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    calibration = color_info()
    plugin._on_color_camera_info(calibration)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert plugin.dispatch("info", {})["ready"]
    calibration.header.stamp.sec += 1
    plugin._on_color_camera_info(calibration)
    assert plugin.dispatch("info", {})["ready"]
    calibration.k[0] += 10
    plugin._on_color_camera_info(calibration)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert not plugin.dispatch("info", {})["ready"]
    assert "restart" in plugin.dispatch("info", {})["geometry_problem"]
    assert len(plugin._pub.messages) == 1
    plugin.stop()
    plugin.start()
    plugin._on_color_camera_info(calibration)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert plugin.dispatch("info", {})["ready"]


def test_hw_d2c_start_waits_for_matching_first_frame_and_metadata(depth_plugin):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    plugin._publish_loop.__globals__["time"] = time
    plugin._START_GEOMETRY_TIMEOUT_S = 0.5
    result = []
    starter = threading.Thread(target=lambda: result.append(plugin.dispatch("start", {})))
    starter.start()
    time.sleep(0.02)
    assert starter.is_alive()
    plugin._on_color_camera_info(color_info())
    assert starter.is_alive()
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    starter.join(1)
    assert result[0]["ready"] and result[0]["camera_info"][0]["K"]


def test_hw_d2c_changed_calibration_during_compression_is_not_published(
        depth_plugin, monkeypatch):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    calibration = color_info()
    plugin._on_color_camera_info(calibration)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert plugin.dispatch("info", {})["ready"]
    compress = plugin._compress_depth

    def change_geometry_while_compressing(data, level):
        calibration.k[0] += 1
        plugin._on_color_camera_info(calibration)
        return compress(data, level)

    monkeypatch.setattr(plugin, "_compress_depth", change_geometry_while_compressing)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert len(plugin._pub.messages) == 1
    assert not plugin.dispatch("info", {})["ready"]
    assert "restart" in plugin.dispatch("info", {})["geometry_problem"]


def test_hw_d2c_stop_cancels_start_wait_and_restart_drops_previous_metadata(depth_plugin):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    plugin._publish_loop.__globals__["time"] = time
    plugin._START_GEOMETRY_TIMEOUT_S = 2
    result = []
    starter = threading.Thread(target=lambda: result.append(plugin.dispatch("start", {})))
    starter.start()
    time.sleep(0.02)
    plugin.stop()
    plugin.start()
    starter.join(0.3)
    assert not starter.is_alive()
    assert result[0]["state"] == "idle"
    assert plugin._running
    assert not plugin.dispatch("info", {})["ready"]
    assert "camera_info" not in plugin.dispatch("info", {})


def test_hw_d2c_start_timeout_is_bounded_and_returns_geometry_error(depth_plugin):
    plugin, _, _ = depth_plugin
    arm_hw_depth(plugin)
    plugin._publish_loop.__globals__["time"] = time
    plugin._START_GEOMETRY_TIMEOUT_S = 0.02
    result = plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert "geometry" in result["error"]
    assert not plugin._running


def test_native_depth_geometry_is_unknown_until_successful_publication(depth_plugin):
    plugin, _, _ = depth_plugin
    info = plugin.dispatch("info", {})
    assert info["source_geometry"] is None
    assert info["registration_to_color"] == "unknown"
    message, _ = frame()
    plugin._on_image(message)
    plugin._publish_latest()
    info = plugin.dispatch("info", {})
    assert info["source_geometry"] == {
        "width": 640, "height": 480, "frame_id": "depth_optical", "encoding": "16UC1",
        "crop_xywh": [0, 0, 640, 480], "output_size": [640, 480],
    }
    assert "camera_info" not in info, "native depth must not borrow the color lens identity"
    assert info["registration_to_color"] == "unknown"
    plugin.stop()
    plugin.start()
    assert plugin.dispatch("info", {})["source_geometry"] is None


def test_crop_diagnostic_matches_the_actual_native_crop(depth_plugin):
    plugin, _, _ = depth_plugin
    calls = []
    plugin._cv2.INTER_NEAREST = 0

    def resize(data, size, interpolation):
        calls.append((data.shape, size))
        return np.full((size[1], size[0]), 1234, np.uint16)

    plugin._cv2.resize = resize
    message = types.SimpleNamespace(
        width=1280, height=720, step=2560, encoding="16UC1", is_bigendian=0,
        data=np.full((720, 1280), 1234, dtype="<u2").tobytes(),
        header=types.SimpleNamespace(stamp="source", frame_id="native_depth_optical"))
    plugin._on_image(message)
    plugin._publish_latest()
    assert calls == [((720, 960), (640, 480))]
    geometry = plugin.dispatch("info", {})["source_geometry"]
    assert (geometry["width"], geometry["height"]) == (1280, 720)
    assert geometry["crop_xywh"] == [160, 0, 960, 720]
    assert geometry["frame_id"] == "native_depth_optical"
    assert plugin._pub.messages[0].header is message.header


def test_depth_info_uses_one_geometry_snapshot_during_restart(depth_plugin):
    plugin, _, _ = depth_plugin

    class ReplacedDuringCheck(dict):
        def __bool__(self):
            # Deterministically schedule the restart's assignment between the
            # presence check and the copy, without needing thread timing luck.
            plugin._source_geometry = None
            return True

    plugin._source_geometry = ReplacedDuringCheck(width=640, height=480)
    assert plugin.dispatch("info", {})["source_geometry"] == {"width": 640, "height": 480}
    assert plugin._source_geometry is None
