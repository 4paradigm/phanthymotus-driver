"""Tests for the person-contour display card.

The depth segmentation and drawing are pure OpenCV/numpy helpers; the ROS
plugin is not constructed here.
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load():
    spec = importlib.util.spec_from_file_location(
        "person_contour_card", ROOT / "PersonContour.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def contour():
    return _load()


def _scene():
    cv2 = pytest.importorskip("cv2")
    depth = np.full((480, 640), 5000, dtype=np.uint16)
    depth[100:400, 300:380] = 3000
    return cv2, depth


def test_parse_people_prefers_the_people_list(contour):
    payload = {"state": "fresh", "people": [
        {"bbox": [300, 100, 80, 300], "distance_m": 3.0},
        {"bbox": [50, 100, 70, 280], "distance_m": 4.0},
    ], "person": {"bbox": [1, 2, 3, 4], "distance_m": 9.0}}
    people = contour.parse_people(payload)
    assert len(people) == 2
    assert people[0]["bbox"] == [300, 100, 80, 300]


def test_parse_people_falls_back_to_the_single_person(contour):
    payload = {"state": "fresh",
               "person": {"bbox": [300, 100, 80, 300], "distance_m": 3.0}}
    assert contour.parse_people(payload)[0]["distance_m"] == 3.0


@pytest.mark.parametrize("item", [
    None, "x", {}, {"bbox": [1, 2, 3], "distance_m": 3},
    {"bbox": [1, 2, 3, 4], "distance_m": 0.0},
    {"bbox": [1, 2, 3, 4], "distance_m": 99.0},
])
def test_parse_people_rejects_invalid_items(contour, item):
    payload = {"people": [item]}
    assert contour.parse_people(payload) == []


def test_person_mask_keeps_only_the_depth_band(contour):
    cv2, depth = _scene()
    bbox = [280, 80, 120, 340]
    mask = contour.person_mask(depth, bbox, 3.0, 0.40)
    assert mask is not None
    assert mask[20, 20] == 255          # person area (300..380, 100..400)
    assert mask[20, 0] == 0             # left background column
    assert mask[0, 0] == 0              # above the person


def test_contours_are_shifted_into_full_frame_coordinates(contour):
    cv2, depth = _scene()
    bbox = [280, 80, 120, 340]
    mask = contour.person_mask(depth, bbox, 3.0, 0.40)
    contours = contour.contours_from_mask(mask, bbox)
    assert contours
    x, y, w, h = cv2.boundingRect(contours[0])
    assert 295 <= x <= 305
    assert 95 <= y <= 105


def test_draw_people_paints_green_pixels(contour):
    cv2, depth = _scene()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    contour.draw_people(frame, [{"bbox": [280, 80, 120, 340],
                                 "distance_m": 3.0}], depth, 0.40, 2)
    green = (frame[:, :, 1] > 200) & (frame[:, :, 0] < 60) & (frame[:, :, 2] < 60)
    assert np.any(green)


def test_draw_people_falls_back_to_a_box_without_depth(contour):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    contour.draw_people(frame, [{"bbox": [300, 100, 80, 300],
                                 "distance_m": 3.0}], None, 0.40, 2)
    green = (frame[:, :, 1] > 200) & (frame[:, :, 0] < 60) & (frame[:, :, 2] < 60)
    assert np.any(green)


def test_encode_jpeg_returns_a_jpeg(contour):
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    encoded = contour.encode_jpeg(frame, 70)
    assert encoded is not None
    assert encoded[:2] == b"\xff\xd8"
