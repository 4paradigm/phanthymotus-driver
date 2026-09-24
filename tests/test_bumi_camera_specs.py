"""Bumi's `motus.camera/1` declaration — the numbers, not the plumbing.

A wrong field of view does not raise anything. It makes the robot refuse
doorways *while the depth map reports clear ahead*, which on r1_sz was diagnosed
as a tracking problem twice before anyone looked at the lens. So these numbers
live in a module that does not import rclpy or pyrealsense2, and are checked
here rather than on a robot.

Two Bumi-specific things this covers that R1's equivalent does not:

  - **the 4:3 crop.** The D435i's RGB sensor is 16:9 and the driver opens it at
    640x480, which keeps the vertical field and cuts the horizontal one. Taking
    Intel's 69.4 deg at face value would overstate the width of the world by
    about a third — an authoritative-looking wrong number.
  - **the runtime intrinsics winning.** The camera knows; the datasheet only
    approximates. The declaration has to prefer the first and still answer
    before the camera has started.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_camera_specs.py -q
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _load():
    bundle = ROOT / "noetix" / "bumi"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "bumi_camera_specs", bundle / "camera_specs.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["bumi_camera_specs"] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(bundle))


camera_specs = _load()

from common.camera_info import (  # noqa: E402
    CameraInfoError, parse, resolve_half_fov, for_topic)

COLOR_TOPIC = "/bumi/camera/color"
DEPTH_TOPIC = "/bumi/camera/depth"


def _color_intrinsics(**overrides):
    """What a D435i reports for the 640x480 colour profile, roughly."""
    data = {"width": 640, "height": 480, "fx": 605.0, "fy": 605.0,
            "ppx": 321.0, "ppy": 241.0,
            "model": "inverse_brown_conrady",
            "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]}
    data.update(overrides)
    return {"color": data}


# ── the fallback ─────────────────────────────────────────────────────────────

def test_both_ports_declare_something_a_consumer_can_parse():
    """`build()` validates on the way out, so a malformed declaration fails
    here rather than at a doorway."""
    for tool, topic, fmt in (("camera", COLOR_TOPIC, "image/jpeg"),
                             ("depth", DEPTH_TOPIC, "image/depth-zlib")):
        declarations = camera_specs.declare(tool, topic, fmt)
        assert len(declarations) == 1
        info = parse(declarations[0])
        assert info.topic == topic
        assert info.known


def test_an_unknown_port_declares_nothing_rather_than_guessing():
    assert camera_specs.declare("lidar", "/bumi/lidar", "sensor/lidar") == []


def test_the_colour_fallback_is_narrower_than_intels_published_angle():
    """The 4:3 crop. The driver opens a 16:9 sensor at 640x480, so the real
    horizontal field is narrower than the datasheet's — and a corridor computed
    from the datasheet value comes out about a third too wide, which is the
    exact shape of the r1_sz doorway bug."""
    info = parse(camera_specs.declare("camera", COLOR_TOPIC, "image/jpeg")[0])
    published_half = math.radians(69.4) / 2.0
    assert info.half_fov_rad < published_half * 0.85


def test_the_colour_fallback_says_it_was_computed_not_published():
    """`source` is what says whether a number can be trusted, and this one is an
    arithmetic step past anything Intel wrote down."""
    info = parse(camera_specs.declare("camera", COLOR_TOPIC, "image/jpeg")[0])
    assert info.source == "manual"


def test_the_depth_fallback_is_intels_own_number_because_that_mode_is_native():
    """The depth stream is natively 4:3 and 640x480 is its own mode, so 87 deg
    describes the picture this driver actually publishes."""
    info = parse(camera_specs.declare("depth", DEPTH_TOPIC, "image/depth-zlib")[0])
    assert info.source == "vendor-spec"
    assert info.half_fov_rad == pytest.approx(math.radians(87.0) / 2.0)


def test_the_two_ports_have_different_ids_and_different_optics():
    """They are two physical lenses. One `id` for both would make a downstream
    calibration table look up the wrong row, silently."""
    color = parse(camera_specs.declare("camera", COLOR_TOPIC, "image/jpeg")[0])
    depth = parse(camera_specs.declare("depth", DEPTH_TOPIC, "image/depth-zlib")[0])
    assert color.id != depth.id
    assert color.half_fov_rad != depth.half_fov_rad


# ── the runtime intrinsics ───────────────────────────────────────────────────

def test_runtime_intrinsics_replace_the_fallback():
    """The camera knows and the datasheet only approximates."""
    declaration = camera_specs.declare(
        "camera", COLOR_TOPIC, "image/jpeg", _color_intrinsics())[0]
    info = parse(declaration)
    assert info.source == "derived-from-K"
    assert info.K is not None and info.K[0] == 605.0


def test_the_angle_is_left_to_the_consumer_to_derive_from_k():
    """Shipping both would create two numbers that can disagree. `K` wins in
    `resolve_half_fov`, so producing an angle beside it is work whose only
    possible effect is to be ignored or to contradict."""
    declaration = camera_specs.declare(
        "camera", COLOR_TOPIC, "image/jpeg", _color_intrinsics())[0]
    info = parse(declaration)
    assert info.half_fov_rad is None
    angle, source = resolve_half_fov(info)
    assert source == "derived-from-K"
    assert angle == pytest.approx(math.atan(320.0 / 605.0))


def test_the_derived_angle_is_close_to_the_cropped_fallback():
    """The two paths must agree to within a lens tolerance, or one of them is
    wrong — and this is the assertion that would have caught the 1.6x error."""
    fallback = parse(camera_specs.declare("camera", COLOR_TOPIC, "image/jpeg")[0])
    runtime = parse(camera_specs.declare(
        "camera", COLOR_TOPIC, "image/jpeg", _color_intrinsics())[0])
    derived, _ = resolve_half_fov(runtime)
    assert derived == pytest.approx(fallback.half_fov_rad, rel=0.15)


def test_intrinsics_for_the_other_stream_do_not_leak_across_ports():
    """Joined by stream name, not by position — the colour port must not adopt
    the depth lens' calibration."""
    declaration = camera_specs.declare(
        "depth", DEPTH_TOPIC, "image/depth-zlib", _color_intrinsics())[0]
    assert parse(declaration).source == "vendor-spec"


def test_rectified_coefficients_are_declared_pinhole():
    """All-zero distortion is the rectified case, which is what a consumer's
    pinhole assumption actually needs to know."""
    info = parse(camera_specs.declare(
        "camera", COLOR_TOPIC, "image/jpeg", _color_intrinsics())[0])
    assert info.distortion_model == "pinhole"


def test_real_distortion_stays_unknown_with_the_vendors_name_recorded():
    """RealSense's model names are not this format's enum, and mapping them by
    guesswork would put a claim in a field consumers act on."""
    intrinsics = _color_intrinsics(coeffs=[0.1, -0.2, 0.0, 0.0, 0.05])
    info = parse(camera_specs.declare(
        "camera", COLOR_TOPIC, "image/jpeg", intrinsics)[0])
    assert info.distortion_model == "unknown"
    assert info.vendor["realsense_distortion_model"] == "inverse_brown_conrady"
    assert info.D == (0.1, -0.2, 0.0, 0.0, 0.05)


def test_unusable_intrinsics_fall_back_rather_than_producing_a_zero_lens():
    """A profile that reported `fx = 0` would otherwise divide the world by
    nothing. Falling back is conservative and says `source` accordingly."""
    for broken in ({"fx": 0.0}, {"width": 0}, {"fx": None}):
        info = parse(camera_specs.declare(
            "camera", COLOR_TOPIC, "image/jpeg", _color_intrinsics(**broken))[0])
        assert info.source == "manual"


def test_the_resolution_is_rewritten_from_the_intrinsics():
    """`width`/`height` describe the *image*, and a consumer joins its pixel
    columns to metres through them — so a profile negotiated at another size
    must be reported at that size."""
    info = parse(camera_specs.declare(
        "camera", COLOR_TOPIC, "image/jpeg",
        _color_intrinsics(width=1280, height=720, ppx=641.0, ppy=361.0))[0])
    assert (info.width, info.height) == (1280, 720)


# ── the subprocess → parent channel ──────────────────────────────────────────

def test_the_marker_line_round_trips():
    import json

    payload = _color_intrinsics()
    line = camera_specs.INTRINSICS_MARKER + json.dumps(payload)
    assert camera_specs.parse_intrinsics_line(line) == payload


def test_a_malformed_marker_line_yields_nothing_rather_than_raising():
    """This runs on the thread forwarding the camera's stdout. An exception
    there takes that thread down and with it every subsequent camera log."""
    for line in (camera_specs.INTRINSICS_MARKER + "{not json",
                 camera_specs.INTRINSICS_MARKER + "[1, 2]",
                 "[camera_subprocess] 300 frames, 29.8 fps", "", None):
        assert camera_specs.parse_intrinsics_line(line) == {}


def test_a_timestamped_log_prefix_does_not_hide_the_marker():
    """The line reaches the parent through a forwarder that may have prefixed
    it. Matching on a bare `startswith` would silently stop adopting intrinsics
    the day anything is added in front."""
    import json

    line = "2026-09-24T10:00:00 " + camera_specs.INTRINSICS_MARKER + json.dumps(
        _color_intrinsics())
    assert "color" in camera_specs.parse_intrinsics_line(line)


# ── the join a consumer performs ─────────────────────────────────────────────

def test_a_consumer_finds_the_declaration_by_the_topic_it_bound():
    """navi knows which topic it bound as its depth input and looks that up —
    not by position in the list, and not by the upstream card's name."""
    declarations = (camera_specs.declare("camera", COLOR_TOPIC, "image/jpeg")
                    + camera_specs.declare("depth", DEPTH_TOPIC, "image/depth-zlib"))
    found = for_topic(declarations, DEPTH_TOPIC)
    assert found is not None and found.id.endswith("camera_depth")


def test_a_full_angle_in_the_half_angle_field_is_refused():
    """The likeliest mistake in this format, and the one that made every
    corridor twice as wide as intended."""
    from common.camera_info import build

    # 102 deg full is 1.78 rad: under pi, so it clears the outer bound and would
    # have been accepted, while making every corridor twice as wide as intended.
    with pytest.raises(CameraInfoError) as excinfo:
        build(topic=COLOR_TOPIC, id="x", half_fov_rad=math.radians(102.0),
              source="measured")
    assert "half" in str(excinfo.value)
