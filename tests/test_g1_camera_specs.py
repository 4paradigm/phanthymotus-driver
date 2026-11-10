"""ROS-free G1 RealSense active-profile -> motus.camera/1 regression tests."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import threading

import pytest

from common.camera_info import parse, resolve_half_fov
from unitree.g1.camera_specs import (
    PROFILES, active_stream_profiles, declare, validated_profiles,
)

DEVICE = Path(__file__).resolve().parents[1] / "unitree/g1/device.py"
# Docker imports camera_specs as a flat /work module; match that in AST tests.
sys.path.insert(0, str(DEVICE.parent))


def profiles():
    return {
        "depth": dict(width=640, height=480, fx=420.1, fy=420.2,
                      ppx=319.5, ppy=239.5, model="none", coeffs=[0.] * 5),
        "color": dict(width=1920, height=1080, fx=1020.1, fy=1020.2,
                      ppx=959.5, ppy=539.5, model="brown_conrady",
                      coeffs=[0.] * 5),
    }


def test_two_real_stream_profiles_not_one_rgb_K():
    class Stream:
        depth = "depth"
        color = "color"

    class Intr:
        def __init__(self, data):
            self.__dict__.update(data)

    class Video:
        def __init__(self, data):
            self._data = data

        def as_video_stream_profile(self):
            return self

        def get_intrinsics(self):
            return Intr(self._data)

    class Active:
        def get_stream(self, key):
            return Video(profiles()[key])

    output = active_stream_profiles(Active(), NS(stream=Stream))
    assert output == profiles()
    assert output["depth"]["fx"] != output["color"]["fx"]
    assert output["depth"]["width"] == 640 and output["color"]["width"] == 1920


def test_missing_rgb_negotiation_keeps_depth_calibration():
    class Stream:
        depth = "depth"
        color = "color"

    class Active:
        def get_stream(self, stream):
            if stream == "color":
                raise RuntimeError("RGB profile unavailable")
            intr = NS(**profiles()["depth"])
            return NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr))

    result = active_stream_profiles(Active(), NS(stream=Stream))
    assert set(result) == {"depth"}
    assert result["depth"]["fx"] == 420.1


def test_depth_declaration_is_measured_unaligned_depth_and_valid_schema():
    entry = declare("camera_depth", "/ubuntu/camera/depth",
                    "image/depth-zlib", profiles())[0]
    parse(entry)
    assert entry["id"] == "unitree/g1/realsense/depth"
    assert entry["topic"] == "/ubuntu/camera/depth"
    assert entry["format"] == "image/depth-zlib"
    assert entry["source"] == "derived-from-K"
    assert (entry["width"], entry["height"]) == (640, 480)
    assert entry["K"] == [420.1, 0., 319.5, 0., 420.2, 239.5, 0., 0., 1.]
    assert entry["D"] == [0.] * 5
    assert entry["distortion_model"] == "pinhole"
    assert entry["vendor"]["depth_unit"] == "mm"
    assert entry["vendor"]["depth_source"] == "measured"
    assert entry["vendor"]["depth_aligned_to"] == "none"
    assert entry["vendor"]["timestamp_semantics"] == "ros_callback_receipt_unverified"
    assert entry["vendor"]["timestamp_clock_relation_verified"] is False
    assert entry["vendor"]["source_frame_id"] == "camera_depth_optical_frame"
    angle, source = resolve_half_fov(parse(entry))
    assert angle == pytest.approx(math.atan((640/2) / 420.1))
    assert source == "derived-from-K"


def test_color_has_independent_identity_dimensions_and_K():
    depth = declare("camera_depth", "/ubuntu/camera/depth",
                    "image/depth-zlib", profiles())[0]
    color = declare("camera_rgb", "/ubuntu/camera/rgb",
                    "image/jpeg", profiles())[0]
    assert color["id"] != depth["id"]
    assert color["K"] == [1020.1, 0., 959.5, 0., 1020.2, 539.5, 0., 0., 1.]
    assert (color["width"], color["height"]) == (1920, 1080)
    assert depth["K"] != color["K"]
    assert color["vendor"]["source_stream"] == "color"


def test_unavailable_intrinsics_are_unknown_not_guessed():
    entry = declare("camera_depth", "/ubuntu/camera/depth",
                    "image/depth-zlib")[0]
    assert entry["source"] == "unknown"
    assert entry["K"] is None and entry["D"] is None
    assert entry["half_fov_rad"] is None and entry["half_fov_v_rad"] is None
    assert entry["vendor"]["timestamp_clock_relation_verified"] is False
    parse(entry)


@pytest.mark.parametrize("change", [
    ("fx", -1.), ("fy", 0.), ("fx", float("nan")),
    ("width", 1920), ("height", 720), ("ppx", 900.),
    ("coeffs", [float("inf"), 0., 0., 0., 0.]),
    ("coeffs", [0., 0.]), ("model", ""),
])
def test_invalid_or_mismatched_calibration_never_exposes_K(change):
    data = profiles()
    key, value = change
    data["depth"][key] = value
    assert "depth" not in validated_profiles(data)
    d = declare("camera_depth", "/ubuntu/camera/depth",
                "image/depth-zlib", data)[0]
    assert d["source"] == "unknown" and d["K"] is None
    assert d["vendor"]["timestamp_clock_relation_verified"] is False
    assert declare("camera_rgb", "/ubuntu/camera/rgb", "image/jpeg", data)[0]["K"] is not None


def test_nonzero_distortion_is_not_falsely_rectified():
    data = profiles()
    data["depth"]["model"] = "inverse_brown_conrady"
    data["depth"]["coeffs"][0] = 0.11
    d = declare("camera_depth", "/ubuntu/camera/depth",
                "image/depth-zlib", data)[0]
    assert d["source"] == "derived-from-K"  # Actual intrinsic matrix remains real.
    assert d["distortion_model"] == "unknown"
    assert d["D"][0] == .11
    assert d["vendor"]["needs_rectification"] is True
    assert d["vendor"]["timestamp_semantics"] != "source_capture_mapped"


def test_each_stream_stands_alone_when_other_is_invalid():
    data = profiles()
    data["color"]["fx"] = float("nan")
    clean = validated_profiles(data)
    assert set(clean) == {"depth"}
    assert declare("camera_depth", "/ubuntu/camera/depth",
                   "image/depth-zlib", clean)[0]["K"] is not None
    assert declare("camera_rgb", "/ubuntu/camera/rgb",
                   "image/jpeg", clean)[0]["K"] is None


def _plugin_instance():
    """Lift the parent plugin only; do not import rclpy/SDK on developer laptops."""
    tree = ast.parse(DEVICE.read_text())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "RealSensePlugin")

    class FrameCache:
        def __init__(self, topic):
            self.topic = topic

        def destroy_node(self):
            return None

    class Executor:
        def add_node(self, node):
            pass

        def remove_node(self, node):
            pass

    namespace = dict(
        threading=threading,
        _CameraFrameNode=FrameCache,
        RS_COLOR_W=1920, RS_COLOR_H=1080, RS_COLOR_FPS=15,
        RS_DEPTH_W=640, RS_DEPTH_H=480, RS_DEPTH_FPS=15,
    )
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(DEVICE), "exec"), namespace)
    return namespace["RealSensePlugin"]({}, "ubuntu", Executor())


def test_g1_card_info_and_tool_list_share_the_child_calibration():
    plugin = _plugin_instance()
    before = {t["name"]: t for t in plugin.get_tools()}
    assert before["camera_depth"]["camera_info"][0]["K"] is None
    assert before["camera_rgb"]["camera_info"][0]["K"] is None

    plugin._adopt_intrinsics(profiles())
    tools = {t["name"]: t for t in plugin.get_tools()}
    depth_info = plugin.dispatch("info", {"_tool_name": "camera_depth"})
    rgb_info = plugin.dispatch("info", {"_tool_name": "camera_rgb"})
    assert tools["camera_depth"]["camera_info"] == depth_info["camera_info"]
    assert tools["camera_rgb"]["camera_info"] == rgb_info["camera_info"]
    assert depth_info["camera_info"][0]["K"][0] == 420.1
    assert rgb_info["camera_info"][0]["K"][0] == 1020.1
    assert tools["camera_depth"]["topic_out"] == [
        {"topic": "/ubuntu/camera/depth", "format": "image/depth-zlib"}]
    assert depth_info["topic_out"] == tools["camera_depth"]["topic_out"]
    assert "camera_info" not in plugin.dispatch("info", {"_tool_name": "camera_distance"})

    # Stop invalidates any previous camera calibration before a restart.
    plugin.stop()
    assert plugin.dispatch("info", {"_tool_name": "camera_depth"})["camera_info"][0]["K"] is None


def test_old_child_calibration_is_not_allowed_to_replace_new_process():
    plugin = _plugin_instance()
    old = object()
    live = object()
    plugin._proc = live
    plugin._adopt_intrinsics(profiles(), old)
    assert plugin.dispatch("info", {"_tool_name": "camera_depth"})["camera_info"][0]["K"] is None
    plugin._adopt_intrinsics(profiles(), live)
    assert plugin.dispatch("info", {"_tool_name": "camera_depth"})["camera_info"][0]["K"] is not None
    plugin._proc = None


def test_pipe_listener_transfers_profiles_to_parent():
    plugin = _plugin_instance()
    proc = NS(is_alive=lambda: True)
    plugin._proc = proc

    class Reader:
        def __init__(self):
            self.closed = False

        def poll(self, _timeout):
            return True

        def recv(self):
            return profiles()

        def close(self):
            self.closed = True

    reader = Reader()
    plugin._receive_intrinsics(reader, proc)
    assert reader.closed is True
    assert plugin.dispatch("info", {"_tool_name": "camera_depth"})["camera_info"][0]["K"][0] == 420.1
    plugin._proc = None
