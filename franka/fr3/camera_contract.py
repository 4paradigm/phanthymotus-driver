"""Camera identity and data semantics; no ROS, image or device SDK imports."""

def optical_frame_id(namespace, serial, channel):
    if channel not in ("rgb", "depth", "infrared"):
        raise ValueError("Unsupported camera channel")
    if not serial:
        return None
    # Distinct encodings prevent punctuation-normalization collisions.
    identity = "serial_" + serial if serial.isascii() and serial.isalnum() else "hex_" + serial.encode("utf-8").hex()
    return f"{namespace}_{identity}_{channel}_optical"


def stream_semantics(namespace, serial, channel):
    result = {
        "frame_id": optical_frame_id(namespace, serial, channel),
        "timestamp_source": "publication_clock",
        "hardware_capture_timestamp_available": False,
        "image_semantics": {
            "rgb": "color", "depth": "native_depth_distance",
            "infrared": "left_near_infrared_intensity_not_temperature",
        }[channel],
    }
    if channel == "depth":
        result.update(depth_aligned_to_rgb=False, depth_unit="mm", invalid_depth_value=0)
    return result
