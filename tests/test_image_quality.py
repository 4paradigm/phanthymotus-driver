"""Image-quality behavior on actual pixels/JPEGs, without ROS or robot SDK."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.image_quality import QualityConfig, analyze_bgr, analyze_jpeg, jpeg_dimensions


def chart():
    image = np.full((480, 640, 3), 120, dtype=np.uint8)
    for y in range(0, 480, 16):
        for x in range(0, 640, 16):
            if (x//16+y//16) % 2:
                image[y:y+16, x:x+16] = 175
            else:
                image[y:y+16, x:x+16] = 65
    return image


def jpeg(image):
    success, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert success
    return encoded.tobytes()


def test_clear_chart_and_blurred_copy_have_distinct_sharpness():
    image = chart()
    clear = analyze_jpeg(jpeg(image))
    blurred = analyze_jpeg(jpeg(cv2.GaussianBlur(image, (41, 41), 10)))
    assert clear['acceptable'] is True
    assert clear['issues'] == []
    assert blurred['acceptable'] is False
    assert blurred['issues'] == ['low_sharpness']
    assert clear['metrics']['laplacian_variance'] > blurred['metrics']['laplacian_variance']*10


@pytest.mark.parametrize('value,issue', [(0, 'underexposed'), (255, 'overexposed')])
def test_blank_exposure_extremes_report_both_exposure_and_low_texture(value, issue):
    result = analyze_jpeg(jpeg(np.full((480, 640, 3), value, np.uint8)))
    assert result['acceptable'] is False
    assert issue in result['issues']
    assert 'low_sharpness' in result['issues']


def test_mixed_clipped_regions_are_detected_even_with_normal_mean():
    image = chart()
    image[:, :320] = 255
    result = analyze_bgr(image)
    assert 45 < result['metrics']['mean_brightness'] < 220
    assert 'overexposed' in result['issues']


def test_no_upscaling_and_bounded_analysis_size_are_visible():
    large = cv2.resize(chart(), (1280, 960), interpolation=cv2.INTER_NEAREST)
    result = analyze_bgr(large)
    assert result['image_size'] == {'width': 1280, 'height': 960}
    assert result['analysis_size'] == {'width': 640, 'height': 480}
    assert analyze_bgr(chart()[:80, :100])['analysis_size'] == {'width': 100, 'height': 80}


def test_brightness_uses_bgr_not_rgb():
    blue = np.zeros((32, 32, 3), np.uint8)
    blue[:, :, 0] = 255
    assert analyze_bgr(blue)['metrics']['mean_brightness'] == 29


@pytest.mark.parametrize('changes', [
    {'min_brightness': True}, {'min_sharpness': float('nan')}, {'max_rate_hz': 0},
    {'max_dark_fraction': 1.1}, {'unknown': 1}, {'min_brightness': 230},
    {'max_frame_age_s': .1}, {'min_sharpness': '60'},
])
def test_invalid_config_is_rejected(changes):
    with pytest.raises(ValueError):
        QualityConfig().updated(changes)


def test_threshold_change_changes_the_decision_on_the_same_pixels():
    image = np.full((32, 32, 3), 120, np.uint8)
    assert analyze_bgr(image)['issues'] == ['low_sharpness']
    assert analyze_bgr(image, QualityConfig().updated({'min_sharpness': 0}))['acceptable'] is True


@pytest.mark.parametrize('data', [b'', b'not a jpeg', b'\xff\xd8', b'\xff\xd8\xff\xe0\x00\xffBAD'])
def test_invalid_jpeg_does_not_become_a_quality_success(data):
    with pytest.raises(ValueError):
        analyze_jpeg(data)


def test_oversized_dimensions_are_rejected_before_decode(monkeypatch):
    encoded = bytearray(jpeg(chart()))
    marker = encoded.find(b'\xff\xc0')
    assert marker > 0
    encoded[marker+5:marker+9] = (65000).to_bytes(2, 'big')*2
    monkeypatch.setattr(cv2, 'imdecode', lambda *a: pytest.fail('must not allocate image'))
    with pytest.raises(ValueError, match='megapixels'):
        analyze_jpeg(bytes(encoded))


def test_dimensions_and_decoding_are_checked_on_real_jpeg():
    data = jpeg(chart())
    assert jpeg_dimensions(data) == (640, 480)
    assert analyze_jpeg(data)['image_size'] == {'width': 640, 'height': 480}


def test_incomplete_jpeg_is_not_silently_decoded_as_a_good_frame():
    data = jpeg(chart())
    assert data.endswith(b'\xff\xd9')
    with pytest.raises(ValueError, match='end marker'):
        analyze_jpeg(data[:-2])


def test_thin_panorama_is_downscaled_without_stretching_its_short_axis():
    result = analyze_bgr(np.full((8, 4096, 3), 120, np.uint8))
    assert result['image_size'] == {'width': 4096, 'height': 8}
    assert result['analysis_size'] == {'width': 640, 'height': 1}


@pytest.mark.parametrize('image', [None, np.zeros((4, 4, 3), np.uint8),
                                   np.zeros((32, 32), np.uint8), np.zeros((32, 32, 3), np.float32)])
def test_invalid_image_array_is_rejected(image):
    with pytest.raises(ValueError):
        analyze_bgr(image)
