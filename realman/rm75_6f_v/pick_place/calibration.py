"""Read camera calibration without opening, configuring or starting acquisition."""

import hashlib
import zlib

from .alignment import validate_calibration


def read_calibration(publisher, rgb_shape, depth_payload, session_id):
    import pyrealsense2 as rs

    # ext_camera identifies its shared physical-camera publisher by serial hash.
    # Matching this node also prevents applying a local camera's calibration to
    # an unrelated camera publishing the same image format.
    context = rs.context()
    devices = [d for d in context.query_devices() if publisher.endswith(
        "_realsense_rgbd_" + hashlib.sha256(
            d.get_info(rs.camera_info.serial_number).encode()).hexdigest()[:12])]
    if len(devices) != 1:
        raise ValueError("RGB-D publisher must identify one locally accessible RealSense camera")
    device = devices[0]
    decoder = zlib.decompressobj()
    raw = decoder.decompress(depth_payload, 4096 * 4096 * 2 + 1)
    if not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError("Invalid depth payload for calibration")
    profiles = [p.as_video_stream_profile() for sensor in device.query_sensors()
                for p in sensor.get_stream_profiles() if p.is_video_stream_profile()]

    def select(kind, fmt, matches):
        candidates = [p for p in profiles if p.stream_type() == kind and p.format() == fmt and matches(p)]
        # FPS variants of the same video geometry have the same calibration.
        # Refuse ambiguous geometry/calibration instead of guessing a profile.
        unique = {}
        for profile in candidates:
            intr = profile.get_intrinsics()
            values = {k: getattr(intr, k) for k in ("width", "height", "fx", "fy", "ppx", "ppy")}
            values.update(model=str(intr.model), coeffs=list(intr.coeffs))
            unique[str(values)] = (profile, values)
        if len(unique) != 1:
            raise ValueError("Image dimensions must identify one RealSense calibration")
        return next(iter(unique.values()))

    rgb, rgb_intr = select(rs.stream.color, rs.format.bgr8,
                           lambda p: (p.height(), p.width()) == rgb_shape)
    depth, depth_intr = select(rs.stream.depth, rs.format.z16,
                               lambda p: p.width() * p.height() * 2 == len(raw))
    extr = depth.get_extrinsics_to(rgb)
    result = dict(version=2, serial_number=device.get_info(rs.camera_info.serial_number),
                  session_id=session_id, rgb_intrinsics=rgb_intr, depth_intrinsics=depth_intr,
                  depth_to_color=dict(rotation=list(extr.rotation), translation=list(extr.translation)),
                  depth_scale_m=0.001, depth_aligned_to="depth")
    validate_calibration(result)
    return result
