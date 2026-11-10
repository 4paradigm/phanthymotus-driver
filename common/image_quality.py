"""Reusable image-quality measurements; no ROS, SDK or camera ownership.

Laplacian variance is a sharpness *indicator*. Low texture and noise can affect
it; thresholds are application settings, not a guarantee that OCR will work.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path

SCHEMA = "motus.image_quality/1"
MAX_JPEG_BYTES = 4 * 1024 * 1024
MAX_PIXELS = 16_000_000
ANALYSIS_MAX_SIDE = 640
DARK_PIXEL = 20
BRIGHT_PIXEL = 245


@dataclass(frozen=True)
class QualityConfig:
    min_brightness: float = 45.0
    max_brightness: float = 220.0
    min_sharpness: float = 60.0
    max_dark_fraction: float = 0.8
    max_bright_fraction: float = 0.4
    max_frame_age_s: float = 2.0
    max_rate_hz: float = 2.0

    def updated(self, changes):
        if not isinstance(changes, dict) or set(changes) - set(asdict(self)):
            raise ValueError("Unknown image_quality configuration field")
        values = {**asdict(self), **changes}
        bounds = {"min_brightness": (0, 255), "max_brightness": (0, 255),
                  "min_sharpness": (0, 100000), "max_dark_fraction": (0, 1),
                  "max_bright_fraction": (0, 1), "max_frame_age_s": (0.1, 30),
                  "max_rate_hz": (0.2, 10)}
        for key, value in values.items():
            low, high = bounds[key]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be a finite number in [{low}, {high}]")
        if values["min_brightness"] >= values["max_brightness"]:
            raise ValueError("min_brightness must be less than max_brightness")
        if values["max_frame_age_s"] < 1 / values["max_rate_hz"]:
            raise ValueError("max_frame_age_s must cover at least one processing interval")
        return replace(self, **values)


def jpeg_dimensions(data):
    """Read JPEG SOF dimensions before allocating decoded pixel memory."""
    if not isinstance(data, bytes) or not 4 <= len(data) <= MAX_JPEG_BYTES or data[:2] != b"\xff\xd8":
        raise ValueError("Expected a nonempty JPEG of at most 4 MiB")
    if data.rfind(b"\xff\xd9") < 2:
        raise ValueError("Incomplete JPEG (end marker missing)")
    position = 2
    sof = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while position < len(data):
        if data[position] != 0xFF:
            raise ValueError("Invalid JPEG marker")
        while position < len(data) and data[position] == 0xFF:
            position += 1
        if position >= len(data):
            break
        marker = data[position]
        position += 1
        if marker in (0xD9, 0xDA):
            break
        if marker in {0x01, *range(0xD0, 0xD9)}:
            continue
        if position + 2 > len(data):
            break
        length = int.from_bytes(data[position:position+2], "big")
        if length < 2 or position + length > len(data):
            raise ValueError("Truncated JPEG segment")
        if marker in sof:
            if length < 8:
                raise ValueError("Invalid JPEG size segment")
            height = int.from_bytes(data[position+3:position+5], "big")
            width = int.from_bytes(data[position+5:position+7], "big")
            if min(width, height) < 8 or width * height > MAX_PIXELS:
                raise ValueError("JPEG dimensions must be at least 8x8 and at most 16 megapixels")
            return width, height
        position += length
    raise ValueError("JPEG dimensions are missing")


def analyze_bgr(image, config=None):
    import cv2
    import numpy as np

    config = (config or QualityConfig()).updated({})
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3
            or image.shape[2] != 3 or min(image.shape[:2]) < 8
            or image.shape[0] * image.shape[1] > MAX_PIXELS):
        raise ValueError("Expected a uint8 BGR image, at least 8x8 and at most 16 megapixels")
    height, width = image.shape[:2]
    scale = min(1., ANALYSIS_MAX_SIDE / max(width, height))
    if scale < 1:
        image = cv2.resize(image, (max(1, round(width*scale)), max(1, round(height*scale))),
                           interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean = float(gray.mean())
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    dark = float(np.mean(gray <= DARK_PIXEL))
    bright = float(np.mean(gray >= BRIGHT_PIXEL))
    issues = []
    if mean < config.min_brightness or dark > config.max_dark_fraction:
        issues.append("underexposed")
    if mean > config.max_brightness or bright > config.max_bright_fraction:
        issues.append("overexposed")
    if sharpness < config.min_sharpness:
        issues.append("low_sharpness")
    return {"schema": SCHEMA, "state": "analyzed", "quality": "poor" if issues else "ok",
            "acceptable": not issues, "issues": issues,
            "metrics": {"mean_brightness": round(mean, 4), "laplacian_variance": round(sharpness, 4),
                        "dark_fraction": round(dark, 6), "bright_fraction": round(bright, 6)},
            "image_size": {"width": width, "height": height},
            "analysis_size": {"width": int(gray.shape[1]), "height": int(gray.shape[0])},
            "thresholds": {k: v for k, v in asdict(config).items() if k not in ("max_frame_age_s", "max_rate_hz")},
            "method": "BT.601 grayscale; variance of 3x3-discrete Laplacian (ksize=1); max side 640, no upscaling",
            "limitations": "Low sharpness may also indicate low texture; quality thresholds need camera/scene calibration."}


def decode_jpeg(data):
    import cv2
    import numpy as np

    width, height = jpeg_dimensions(data)
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError("JPEG cannot be decoded or dimensions disagree")
    return image


def analyze_jpeg(data, config=None):
    return analyze_bgr(decode_jpeg(data), config)


def main():
    parser = argparse.ArgumentParser(description="Offline image_quality check; does not connect to a robot")
    parser.add_argument("image", type=Path, help="Local JPEG or PNG test image")
    args = parser.parse_args()
    import cv2
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    try:
        result = analyze_bgr(image)
    except ValueError as error:
        parser.exit(2, str(error)+"\n")
    print(json.dumps({**result, "source": "offline_file", "robot_connected": False}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
