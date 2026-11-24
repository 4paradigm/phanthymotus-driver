"""Camera-owned motus.camera/1 geometry; never borrow another lens's FOV."""
import math
import struct

from common.camera_info import build

INFO_TOPICS = {
    'camera_left': '/camera_1/color/camera_info',
    'camera_right': '/camera_2/color/camera_info',
    'camera_head': '/camera_head/color/image_rect/camera_info',
    'camera_depth': '/camera_head/depth/image_rect/camera_info',
}


def jpeg_dimensions(data):
    """Read JPEG SOF dimensions without decoding pixels or importing OpenCV."""
    i = 2
    while i + 4 <= len(data) and data[:2] == b'\xff\xd8':
        if data[i] != 255:
            return None
        while i < len(data) and data[i] == 255:
            i += 1
        if i >= len(data):
            return None
        marker = data[i]
        i += 1
        if marker in (0xD9, 0xDA):
            return None
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:
            continue
        if i + 2 > len(data):
            return None
        size = struct.unpack_from('>H', data, i)[0]
        if size < 2 or i + size > len(data):
            return None
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF) and size >= 8:
            height, width = struct.unpack_from('>HH', data, i + 3)
            return (width, height) if width > 0 and height > 0 else None
        i += size
    return None


def calibration(message):
    """Validate ROS CameraInfo, retaining the real distortion model as provenance."""
    width, height = message.get('width'), message.get('height')
    K, D = message.get('K'), message.get('D', [])
    if (type(width) is not int or type(height) is not int or not 0 < width <= 16384
            or not 0 < height <= 16384 or not isinstance(K, list) or len(K) != 9
            or not isinstance(D, list) or len(D) > 16
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in K + D)
            or K[0] <= 0 or K[4] <= 0 or abs(K[8] - 1) > 1e-6):
        raise ValueError('invalid camera calibration')
    frame = message.get('header', {}).get('frame_id', '')
    if not isinstance(frame, str) or not frame or len(frame) > 256:
        raise ValueError('invalid camera calibration frame')
    return {'width': width, 'height': height, 'K': list(K), 'D': list(D),
            'frame_id': frame, 'ros_distortion_model': str(message.get('distortion_model', ''))[:80]}


def registered_pair(color, depth, registration):
    """Software registration alone is insufficient: verify its published geometry."""
    return bool(registration is True and color and depth
                and not any(color['D']) and not any(depth['D'])
                and color['frame_id'] == depth['frame_id'] == 'camera_head_color_optical_frame'
                and (color['width'], color['height']) == (depth['width'], depth['height']) == (640, 480)
                and all(abs(a - b) <= 1e-4 for a, b in zip(color['K'], depth['K'])))


def declare(card, topic, fmt, info=None, dimensions=None, aligned=False):
    lens = 'camera_head' if card == 'camera_depth' and aligned else card
    width, height = dimensions or ((640, 480) if card in ('camera_head', 'camera_depth') else (None, None))
    if info and dimensions and (info['width'], info['height']) != dimensions:
        info = None  # A different-resolution calibration cannot describe these pixels.
    fields = {'width': width, 'height': height, 'source': 'unknown'}
    vendor = {'calibration_topic': INFO_TOPICS[card]}
    if info:
        fields.update(width=info['width'], height=info['height'], K=info['K'], D=info['D'],
                      source='derived-from-K',
                      distortion_model='pinhole' if not any(info['D']) else 'unknown')
        vendor.update(frame_id=info['frame_id'], ros_distortion_model=info['ros_distortion_model'])
    if card == 'camera_depth':
        vendor.update(aligned_to_rgb=aligned, unit='mm', invalid_value=0,
                      registration='Orbbec SDK software D2C' if aligned else 'unverified')
    if card in ('camera_head', 'camera_depth'):
        vendor.update(mount='articulated_head', clock_id='orbbec_head_system_ros_header',
                      timestamp_semantics='sdk_frame_system_timestamp',
                      clock_mapping={'verified': False, 'uncertainty_ns': None},
                      registration='registered_rectified' if aligned else 'unverified',
                      body_mount_status='unavailable: no calibrated body-to-optical transform')
        vendor['navigation_requirement'] = 'Head forward and stationary; navi does not compensate head motion.'
    ident = 'lejurobot/kuavo5-w/' + lens
    return [build(topic=topic, id=ident, format=fmt, pipeline=[ident, card],
                  vendor=vendor, **fields)]
