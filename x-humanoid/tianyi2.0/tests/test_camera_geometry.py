"""Real calibration controls geometry; camera metadata never guesses alignment."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
DRIVER = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "tianyi_camera_geometry", DRIVER / "camera_geometry.py")
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def camera_info():
    return NS(
        width=1280, height=720,
        header=NS(frame_id="ob_camera_head_color_optical_frame",
                  stamp=NS(sec=1793871512, nanosec=582512896)),
        distortion_model="plumb_bob", binning_x=0, binning_y=0,
        roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False),
        k=[689.252685546875, 0, 642.1845703125,
           0, 689.331298828125, 365.81414794921875, 0, 0, 1],
        d=[0.008281971327960491, -0.04400582239031792,
           -0.0005832662573084235, 2.9900180379627272e-05,
           0.030294105410575867, 0, 0, 0])


def declare(snapshot, **updates):
    args = dict(topic="/robot/camera/head/depth/visual_depth",
                format="image/depth-zlib", source_size=(1280, 720),
                output_size=(640, 480),
                frame_id="ob_camera_head_color_optical_frame",
                pipeline=["x-humanoid/tianyi2.0/camera_depth"],
                vendor={"registration": "vendor-hw-d2c"})
    args.update(updates)
    return geometry.color_camera_declaration(snapshot, **args)


def test_full_view_resize_scales_K_per_axis_and_preserves_D():
    original = camera_info()
    snapshot = geometry.camera_info_snapshot(original)
    result = declare(snapshot)
    assert result["id"] == geometry.COLOR_CAMERA_ID
    assert (result["width"], result["height"]) == (640, 480)
    assert result["K"] == pytest.approx([
        original.k[0] / 2, 0, original.k[2] / 2,
        0, original.k[4] * 2 / 3, original.k[5] * 2 / 3, 0, 0, 1])
    assert result["D"] == original.d
    assert result["distortion_model"] == "pinhole"
    assert result["half_fov_rad"] == pytest.approx(
        math.atan(640 / original.k[0]))
    assert result["half_fov_v_rad"] == pytest.approx(
        math.atan(360 / original.k[4]))
    assert result["source"] == "derived-from-K"
    provenance = result["vendor"]
    assert provenance["registration"] == "vendor-hw-d2c"
    assert provenance["calibration_K"] == original.k
    assert provenance["calibration_D"] == original.d
    assert provenance["calibration_size"] == [1280, 720]
    assert provenance["resize_scale_xy"] == [0.5, 2 / 3]
    assert provenance["calibration_stamp_ms"] == 1793871512582


def test_snapshot_and_declaration_do_not_alias_mutable_ROS_fields():
    message = camera_info()
    snapshot = geometry.camera_info_snapshot(message)
    vendor = {"notes": ["original"]}
    first = declare(snapshot, vendor=vendor)
    message.k[0] = 1
    message.d[0] = 99
    first["K"][0] = 2
    first["D"][0] = 98
    first["vendor"]["notes"].append("changed")
    assert declare(snapshot)["K"][0] == pytest.approx(689.252685546875 / 2)
    assert snapshot["D"][0] == pytest.approx(0.008281971327960491)
    assert vendor == {"notes": ["original"]}


@pytest.mark.parametrize("change", [
    {"width": 0}, {"height": -1}, {"width": 1280.5},
    {"k": [1, 2]}, {"k": [0, 0, 1, 0, 5, 1, 0, 0, 1]},
    {"k": [float("nan"), 0, 1, 0, 5, 1, 0, 0, 1]},
    {"d": [float("inf")]}, {"distortion_model": "unsupported"},
    {"binning_x": 2}, {"binning_y": 2},
    {"roi": NS(x_offset=1, y_offset=0, width=0, height=0)},
    {"roi": NS(x_offset=0, y_offset=0, width=640, height=480)},
    {"header": NS(frame_id="")},
])
def test_unusable_or_transformed_source_calibration_stays_unknown(change):
    message = camera_info()
    for name, value in change.items():
        setattr(message, name, value)
    assert geometry.camera_info_snapshot(message) is None


@pytest.mark.parametrize("overrides", [
    {"source_size": (640, 480)}, {"frame_id": "native_depth_optical_frame"},
    {"output_size": (0, 480)}, {"output_size": (640, True)},
])
def test_frame_or_size_mismatch_never_borrows_color_identity(overrides):
    assert declare(geometry.camera_info_snapshot(camera_info()), **overrides) is None
    assert declare(None) is None


def test_explicit_full_ROI_and_unit_binning_are_supported():
    message = camera_info()
    message.roi.width, message.roi.height = 1280, 720
    message.binning_x = message.binning_y = 1
    assert geometry.camera_info_snapshot(message) is not None


def test_normalized_rays_survive_full_view_resize_without_center_crop():
    source = camera_info()
    result = declare(geometry.camera_info_snapshot(source))
    for x, y in [(-0.8, -0.3), (0, 0), (0.7, 0.4)]:
        u = source.k[0] * x + source.k[2]
        v = source.k[4] * y + source.k[5]
        u_out = result["K"][0] * x + result["K"][2]
        v_out = result["K"][4] * y + result["K"][5]
        assert u_out / 640 == pytest.approx(u / 1280)
        assert v_out / 480 == pytest.approx(v / 720)


def test_native_depth_calibration_cannot_pass_as_color_grid():
    snapshot = geometry.camera_info_snapshot(camera_info())
    # Even a depth caller must present the matching source frame. Registration
    # itself is a caller-side contract and is deliberately not inferred here.
    assert declare(snapshot, frame_id="ob_camera_head_depth_optical_frame") is None
