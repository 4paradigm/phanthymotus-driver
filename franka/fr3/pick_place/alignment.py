"""RealSense alignment of received frames; never opens a physical device."""

import math
import zlib

import numpy as np


def validate_calibration(metadata):
    if metadata.get("version") != 2 or metadata.get("depth_aligned_to") != "depth":
        raise ValueError("Unsupported RGB-D calibration metadata")
    if metadata.get("depth_scale_m") != 0.001:
        raise ValueError("ext_camera depth input must contain uint16 millimetres")
    for name in ("serial_number", "session_id"):
        if not isinstance(metadata.get(name), str) or not metadata[name]:
            raise ValueError(f"RGB-D metadata lacks {name}")
    for name in ("rgb_intrinsics", "depth_intrinsics"):
        intr = metadata[name]
        if any(type(intr[k]) is not int or not 1 <= intr[k] <= 4096 for k in ("width", "height")):
            raise ValueError("Invalid RGB-D dimensions")
        values = [intr[k] for k in ("fx", "fy", "ppx", "ppy")] + intr["coeffs"]
        if len(intr["coeffs"]) != 5 or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError("Invalid RGB-D intrinsics")
        if min(intr["fx"], intr["fy"]) <= 0:
            raise ValueError("Invalid RGB-D focal length")
        model = intr["model"].rsplit(".", 1)[-1]
        if model not in ("none", "brown_conrady", "inverse_brown_conrady", "modified_brown_conrady"):
            raise ValueError("Unsupported RGB-D distortion model")
        if any(intr["coeffs"]) and (
            name == "depth_intrinsics"
            and model == "modified_brown_conrady"
            or name == "rgb_intrinsics"
            and model == "inverse_brown_conrady"
        ):
            raise ValueError("Unsupported RGB-D projection distortion")
    extr = metadata["depth_to_color"]
    rotation = np.asarray(extr["rotation"], dtype=float).reshape(3, 3)
    translation = np.asarray(extr["translation"], dtype=float)
    if (
        translation.shape != (3,)
        or not np.isfinite(translation).all()
        or not np.isfinite(rotation).all()
        or np.linalg.norm(translation) > 1
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
        or not math.isclose(np.linalg.det(rotation), 1, abs_tol=1e-4)
    ):
        raise ValueError("Invalid depth-to-color calibration")


def decode_depth(payload, width, height):
    size = width * height * 2
    decoder = zlib.decompressobj()
    raw = decoder.decompress(payload, size + 1)
    if len(raw) != size or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError("Depth bytes do not match calibrated dimensions")
    return np.frombuffer(raw, dtype="<u2").reshape(height, width).copy()


def align_depth(payload, metadata):
    """Run the same SDK align(color) operation with externally acquired data."""
    validate_calibration(metadata)
    import pyrealsense2 as rs

    depth_intr = metadata["depth_intrinsics"]
    color_intr = metadata["rgb_intrinsics"]
    # Keep numpy buffers alive until every software frame and sensor is released.
    arrays = [
        decode_depth(payload, depth_intr["width"], depth_intr["height"]),
        np.zeros((color_intr["height"], color_intr["width"], 3), dtype=np.uint8),
    ]
    device = rs.software_device()
    device.create_matcher(rs.matchers.default)
    sync = rs.syncer()
    sensors, profiles, frames = [], [], []
    try:
        for index, values in enumerate((depth_intr, color_intr)):
            intr = rs.intrinsics()
            for key in ("width", "height", "fx", "fy", "ppx", "ppy", "coeffs"):
                setattr(intr, key, values[key])
            intr.model = (
                getattr(rs.distortion, values["model"].rsplit(".", 1)[-1])
                if any(values["coeffs"])
                else rs.distortion.none
            )
            stream = rs.video_stream()
            stream.type = rs.stream.depth if index == 0 else rs.stream.color
            stream.index, stream.uid, stream.fps = 0, index, 15
            stream.width, stream.height = intr.width, intr.height
            stream.bpp, stream.fmt = (2, rs.format.z16) if index == 0 else (3, rs.format.bgr8)
            stream.intrinsics = intr
            sensor = device.add_sensor(str(stream.type))
            profile = sensor.add_video_stream(stream).as_video_stream_profile()
            if index == 0:
                sensor.add_read_only_option(rs.option.depth_units, 0.001)
            sensor.open(profile)
            try:
                sensor.start(sync)
            except Exception:
                sensor.close()
                raise
            sensors.append(sensor)
            profiles.append(profile)
        extr = rs.extrinsics()
        extr.rotation = metadata["depth_to_color"]["rotation"]
        extr.translation = metadata["depth_to_color"]["translation"]
        profiles[0].register_extrinsics_to(profiles[1], extr)
        # Software sync may emit a single stream on its first pair. Replay these
        # same immutable buffers to prime it; this does not acquire another image.
        for number in range(1, 5):
            for index, sensor in enumerate(sensors):
                frame = rs.software_video_frame()
                frame.profile, frame.pixels = profiles[index], arrays[index]
                frame.stride, frame.bpp = arrays[index].strides[0], 2 if index == 0 else 3
                frame.timestamp, frame.frame_number = number * 1000 / 15, number
                frame.domain, frame.depth_units = rs.timestamp_domain.hardware_clock, 0.001
                sensor.on_video_frame(frame)
                frames.append(frame)
            pair = sync.wait_for_frames(500)
            if pair.get_color_frame() and pair.get_depth_frame():
                aligned = rs.align(rs.stream.color).process(pair).get_depth_frame()
                return np.asanyarray(aligned.get_data()).astype("<u2", copy=True)
        raise RuntimeError("Received RGB-D frames could not be aligned")
    finally:
        for sensor in sensors:
            try:
                sensor.stop()
            finally:
                sensor.close()
