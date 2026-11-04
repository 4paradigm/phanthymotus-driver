"""Hardware depth diagnostics must describe the native frame without claiming RGB registration."""
import types

import numpy as np

from test_camera_depth import depth_plugin, frame


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
