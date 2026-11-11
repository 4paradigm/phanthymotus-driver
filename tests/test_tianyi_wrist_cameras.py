"""ROS-/SDK-free contract tests for Tianyi's four D405 wrist cards."""
import importlib.util
from pathlib import Path
import sys
import zlib

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SOURCE = ROOT / "x-humanoid" / "tianyi2.0" / "wrist_cameras.py"
SPEC = importlib.util.spec_from_file_location("tianyi_wrist_cameras", SOURCE)
wrist = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(wrist)


def session(side="left", serial="261922273335"):
    return wrist.WristCameraSession("test_robot", side, serial)


def test_four_cards_have_independent_names_topics_and_formats():
    cards = [wrist.WristCameraPlugin(session(side), modality).get_tool()
             for side in ("left", "right") for modality in ("rgb", "depth")]
    assert {card["name"] for card in cards} == {
        "camera_wrist_left_rgb", "camera_wrist_left_depth",
        "camera_wrist_right_rgb", "camera_wrist_right_depth",
    }
    assert len({card["topic_out"][0]["topic"] for card in cards}) == 4
    for card in cards:
        assert card["type"] == "sensor"
        modality = card["name"].rsplit("_", 1)[-1]
        assert card["topic_out"][0]["format"] == wrist.FORMATS[modality]


def test_unbound_serial_registers_cards_but_cannot_open_camera():
    left = session(serial="")
    info = left.info("rgb")
    assert info["state"] == "unconfigured"
    assert info["serial"] is None
    assert info["camera_info"][0]["K"] is None
    assert info["camera_info"][0]["source"] == "unknown"
    with pytest.raises(RuntimeError, match="RealSense SDK Serial Number"):
        left.start("rgb")
    assert left._proc is None


def test_wrong_asic_serial_is_not_accepted_implicitly():
    # The ASIC identifier is also 12 digits and cannot be reliably recognized
    # syntactically. It is NOT copied automatically from lsusb into the config.
    main = (ROOT / "x-humanoid" / "tianyi2.0" / "config.yaml").read_text()
    assert 'left_serial: ""' in main and 'right_serial: ""' in main


def test_bad_or_duplicate_binding_fails_closed_without_usb():
    s = session(serial="not-a-sdk-serial")
    with pytest.raises(RuntimeError, match="12-digit"):
        s.start("depth")
    s = session()
    s.blocked_reason = "left and right must bind different devices"
    with pytest.raises(RuntimeError, match="different devices"):
        s.start("rgb")


def test_stopped_card_does_not_report_old_frame_as_fresh():
    s = session()
    s._status = {"last_frame": {"rgb": 1e20}, "frames": {"rgb": 99}}
    info = s.info("rgb")
    assert info["state"] == "idle"
    assert info["fresh"] is False


def test_depth_output_is_zlib_of_scaled_little_endian_millimetres():
    np = pytest.importorskip("numpy")
    data = np.zeros((480, 640), dtype=np.uint16)
    data[0, 0] = 1000
    data[0, 1] = 500
    mm = np.frombuffer(zlib.decompress(wrist.encode_depth_mm(data, 0.0001)),
                       dtype="<u2").reshape(480, 640)
    assert mm[0, 0] == 100  # 1000 SDK units at 0.1 mm/unit
    assert mm[0, 1] == 50
    assert mm[1, 0] == 0
    assert mm.shape == (480, 640)


def test_depth_never_wraps_out_of_range_values():
    np = pytest.importorskip("numpy")
    data = np.zeros((480, 640), dtype=np.uint16)
    data[0, 0] = 1000
    result = np.frombuffer(zlib.decompress(wrist.encode_depth_mm(data, 1.0)),
                           dtype="<u2")
    assert result[0] == 0


@pytest.mark.parametrize("bad_scale", [0, -1, float("nan"), float("inf")])
def test_bad_depth_scale_is_rejected(bad_scale):
    np = pytest.importorskip("numpy")
    data = np.zeros((480, 640), dtype=np.uint16)
    with pytest.raises(ValueError):
        wrist.encode_depth_mm(data, bad_scale)


def test_depth_shape_is_verified_not_resized_and_guessed():
    np = pytest.importorskip("numpy")
    with pytest.raises(ValueError):
        wrist.encode_depth_mm(np.zeros((424, 240), dtype=np.uint16), 0.001)


def test_sdk_calibration_is_reported_with_provenance():
    s = session()
    s._status = {"intrinsics": {"depth": {
        "K": [600., 0., 320., 0., 600., 240., 0., 0., 1.],
        "D": [0., 0., 0., 0., 0.],
        "sdk_distortion": "distortion.brown_conrady",
    }}}
    info = s.info("depth")
    assert info["camera_info"][0]["source"] == "vendor-spec"
    assert info["camera_info"][0]["K"][0] == 600.
    assert info["camera_info"][0]["vendor"]["sdk_distortion"] == "distortion.brown_conrady"
    assert info["depth_aligned_to_rgb"] is False


def test_worker_is_separate_process_and_not_opened_during_construction():
    s = session()
    assert s._proc is None
    assert s._wanted == set()
    assert s.info("rgb")["state"] == "idle"
    assert s.info("depth")["state"] == "idle"
