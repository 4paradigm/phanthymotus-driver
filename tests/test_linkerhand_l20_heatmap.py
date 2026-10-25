"""Exercise the shipped renderer with valid zeros and missing sensor data."""
import io
import sys
from pathlib import Path

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'linkerhand/l20'))
from tactile_heatmap import FINGER_ORDER, render_heatmap_jpeg


def test_real_renderer_encodes_both_hands_and_does_not_label_missing_data_as_zero():
    matrix = {n: [[0] * 6 for _ in range(12)] for n in FINGER_ORDER}
    scalar = {n: 0 for n in FINGER_ORDER}
    left = render_heatmap_jpeg(matrix, scalar, side='left')
    right = render_heatmap_jpeg(matrix, scalar, side='right')
    missing = render_heatmap_jpeg({n: None for n in FINGER_ORDER},
                                  {n: None for n in FINGER_ORDER}, side='right')
    assert left != right and missing != right
    for data in (left, right, missing):
        with Image.open(io.BytesIO(data)) as image:
            assert image.format == 'JPEG' and image.size == (660, 330)
            image.load()
    # Missing panels have a separate neutral background, not the zero-pressure color.
    with Image.open(io.BytesIO(right)) as valid, Image.open(io.BytesIO(missing)) as invalid:
        assert valid.getpixel((30, 60)) != invalid.getpixel((30, 60))


def test_single_finger_pressure_changes_its_panel_without_changing_other_fingers():
    matrix = {n: [[0] * 6 for _ in range(12)] for n in FINGER_ORDER}
    scalar = {n: 0 for n in FINGER_ORDER}
    before = render_heatmap_jpeg(matrix, scalar)
    matrix['index'][5][3] = 200
    scalar['index'] = 60
    after = render_heatmap_jpeg(matrix, scalar)
    with Image.open(io.BytesIO(before)) as a, Image.open(io.BytesIO(after)) as b:
        assert a.getpixel((177, 127)) != b.getpixel((177, 127))
        assert a.getpixel((30, 60)) == b.getpixel((30, 60))
