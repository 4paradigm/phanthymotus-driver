"""motus.camera/1 — the declaration, and R1's four lenses.

The bug this format is built against: navi's corridor is metric, so every frame
it converts its half-width back into a column range using the field of view. On
r1_sz that was configured 0.55 rad against a lens that measures 0.888, so the
slice came out 1.6x too wide — a corridor of 1.86 m, wider than any door. Every
doorframe counted as dead ahead and the robot turned away 0.6 m short of a gap it
fitted through, while the depth map reported clear.

The number had been measured the day before. It went into a report and not into
a config file, and nothing noticed. So the tests below are mostly about the ways
a declaration can be *quietly* wrong rather than about the happy path.
"""
from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common import camera_info as C  # noqa: E402


def _built(**kwargs):
    base = dict(topic="/ubuntu/camera/main", id="unitree/r1/camera_main")
    base.update(kwargs)
    return C.build(**base)


# ── rule 1: unknown is null, and it has to be sayable ────────────────────────

def test_a_lens_nobody_measured_reports_null_and_says_so():
    """The whole format in one case.

    A consumer must be able to separate "nobody told me" from "I was told",
    because the two demand opposite behaviour: fall back conservatively and
    report it, versus trust the number.
    """
    info = C.parse(_built())
    assert info.half_fov_rad is None
    assert info.source == "unknown"
    assert not info.known


def test_declaring_geometry_without_saying_where_it_came_from_is_refused():
    """`source` is not paperwork. A tape measure, a vendor datasheet and a
    solved calibration are three different kinds of number, and which one is in
    play is most of what says whether it can be trusted."""
    with pytest.raises(C.CameraInfoError, match="source"):
        _built(half_fov_rad=0.888)


def test_claiming_a_source_while_declaring_nothing_is_refused():
    """The other direction: `source: measured` next to a null angle reads as
    "measured and it came out null", which is not a thing."""
    with pytest.raises(C.CameraInfoError, match="nothing in it"):
        _built(source="measured")


# ── rule 2: this is the *half* angle, horizontal, in radians ─────────────────

def test_passing_the_full_angle_instead_of_the_half_is_refused():
    """The likeliest mistake in the whole format, and it used to slip through.

    r1_sz's lens is 102 deg full. Written as the full angle that is 1.78 rad,
    which is under pi and so cleared the only bound there was — while making
    every corridor twice as wide as intended, i.e. reproducing the original bug
    through a different door. The error names the halved value, because being
    told "that is the full angle" is less useful than being told what to write.
    """
    with pytest.raises(C.CameraInfoError, match=r"\*\*half\*\* angle — 0\.890"):
        _built(half_fov_rad=math.radians(102), source="measured")  # full, not half

    # And the genuinely hemispherical lens is still expressible, by saying what
    # it is rather than by being under a numeric threshold.
    assert C.parse(_built(half_fov_rad=math.pi / 2 + 0.1,
                          distortion_model="equidistant", source="vendor-spec"))


def test_a_pinhole_at_90_degrees_is_refused_because_tan_diverges():
    """Consumers turn this into a lateral offset with `tan()`. At pi/2 that is
    not a wide corridor, it is an infinity produced by a consumer that did
    nothing wrong — so the declaration has to be honest about its model."""
    with pytest.raises(C.CameraInfoError, match="diverges"):
        _built(half_fov_rad=1.58, distortion_model="pinhole", source="measured")

    # The same angle is acceptable when the lens does not claim to be a pinhole.
    assert C.parse(_built(half_fov_rad=1.58, distortion_model="fisheye",
                          source="measured")).half_fov_rad == pytest.approx(1.58)


def test_the_vertical_angle_is_its_own_field():
    """It cannot be derived from the aspect ratio, because a processor that
    resizes 1280x720 into 640x480 stretches the picture — the pixels stop being
    square and the two angles stop being related by the frame's shape."""
    info = C.parse(_built(half_fov_rad=0.888, source="measured"))
    assert info.half_fov_v_rad is None


# ── rule 4: id is the join key, and topic is how a consumer finds its entry ──

def test_topic_and_id_are_both_required():
    with pytest.raises(C.CameraInfoError, match="topic"):
        C.build(topic="", id="unitree/r1/camera_main")
    with pytest.raises(C.CameraInfoError, match="id"):
        C.build(topic="/ubuntu/camera/main", id="")


def test_a_consumer_joins_on_its_own_topic_not_on_position():
    """A card may publish several ports and a consumer may have several inputs,
    so neither side can use list order. Naming the upstream *card* would be
    worse still — navi dispatches inputs by what they carry, on purpose."""
    declarations = [
        _built(topic="/ubuntu/camera/main/objects"),
        _built(topic="/ubuntu/camera/main/visual_depth", half_fov_rad=0.888,
               source="inherited"),
    ]
    found = C.for_topic(declarations, "/ubuntu/camera/main/visual_depth")
    assert found.half_fov_rad == pytest.approx(0.888)
    assert C.for_topic(declarations, "/nothing/here") is None


def test_a_malformed_entry_does_not_hide_the_valid_ones():
    """One bad declaration in the list is a bug in one card, not a reason for
    the consumer to go blind."""
    declarations = [{"schema": "nonsense"},
                    _built(topic="/ubuntu/camera/main", half_fov_rad=0.888,
                           source="measured")]
    assert C.for_topic(declarations, "/ubuntu/camera/main") is not None


# ── K wins over the convenience field ────────────────────────────────────────

def test_a_solved_calibration_beats_the_tape_measure():
    """`K` comes out of many observations; the angle beside it is usually a tape
    measure and some trigonometry. Preferring the coarser number because it sits
    in a handier field would be backwards."""
    fx = 640.0
    info = C.parse(_built(width=1280, height=720, K=[fx, 0, 640, 0, fx, 360, 0, 0, 1],
                          half_fov_rad=0.2, source="manual"))
    angle, source = C.resolve_half_fov(info)
    assert angle == pytest.approx(math.atan(640.0 / fx))
    assert source == "derived-from-K"


def test_without_K_the_declared_angle_is_used_with_its_own_provenance():
    info = C.parse(_built(half_fov_rad=0.888, source="measured"))
    assert C.resolve_half_fov(info) == (pytest.approx(0.888), "measured")


def test_an_empty_declaration_resolves_to_nothing_rather_than_a_default():
    assert C.resolve_half_fov(C.parse(_built())) == (None, "unknown")
    assert C.resolve_half_fov(None) == (None, "unknown")


# ── R1's four lenses ─────────────────────────────────────────────────────────

def _specs():
    """Loaded straight, without `device.py` and therefore without rclpy.

    That separation is the whole reason `camera_specs.py` exists: these numbers
    must be checkable on a laptop. A skipped test here would be the same silent
    failure the format is built against, one level up.
    """
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "unitree" / "r1" / "camera_specs.py"
    spec = importlib.util.spec_from_file_location("r1_camera_specs", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r1_declares_the_lens_it_measured_and_admits_the_three_it_did_not():
    specs = _specs()
    main = C.parse(specs.declare("camera_main", "/ubuntu/camera/main",
                                 "image/jpeg")[0])
    assert main.half_fov_rad == pytest.approx(0.888)
    assert main.source == "measured" and main.measured_on
    assert (main.width, main.height) == (1280, 720)
    assert main.pipeline == ("unitree/r1/camera_main",)

    for tool, topic in (("camera_left", "/ubuntu/camera/left"),
                        ("camera_right", "/ubuntu/camera/right"),
                        ("camera_depth", "/ubuntu/camera/depth")):
        info = C.parse(specs.declare(tool, topic, "image/jpeg")[0])
        assert info.half_fov_rad is None, f"{tool} 报了一个没人量过的数"
        assert info.source == "unknown"
        assert info.id != main.id, "每颗镜头要有自己的 id，下游按它查表"


def test_every_declared_lens_passes_the_shared_validator():
    """A driver must not be able to ship a declaration its own consumers would
    reject. `declare()` runs `build()`, which parses — so this asserts the
    numbers in the table, not just that a table exists."""
    specs = _specs()
    for tool in specs.SPECS:
        entries = specs.declare(tool, f"/ubuntu/{tool}", "image/jpeg")
        assert len(entries) == 1
        C.parse(entries[0])
    assert specs.declare("camera_nonexistent", "/x", "image/jpeg") == []


def test_the_declaration_rides_along_with_info():
    """It has to arrive through the same `info()` call agent-core already makes,
    or the chain has a link that only works when somebody remembers to ask."""
    qos = pytest.importorskip("rclpy.qos")
    if not hasattr(qos, "HistoryPolicy"):
        # `test_tianyi_bridge.py` installs a partial `rclpy` stub into
        # `sys.modules` and leaves it there, so `importorskip` succeeds while the
        # real import fails a line later with a bare ImportError. Named here
        # because "cannot import name 'HistoryPolicy'" gives no hint that the
        # cause is a different test file entirely — and because the failure
        # depends on collection order, so it appears in a full run and vanishes
        # when this file is run alone.
        pytest.skip("sys.modules 里是另一个测试留下的 rclpy 桩")
    from unitree.r1.device import CameraPlugin
    reply = CameraPlugin({}, "ubuntu", None).dispatch(
        "info", {"_tool_name": "camera_main"})
    assert reply["topic_out"][0]["topic"] == "/ubuntu/camera/main"
    assert C.parse(reply["camera_info"][0]).topic == "/ubuntu/camera/main"
