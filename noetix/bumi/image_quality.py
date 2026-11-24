"""ImageQualityGuard metrics and per-reason debounce, without ROS or the SDK.

Defaults are engineering starting points, NOT Bumi hardware calibrations.
Images retain their original resolution: Laplacian variance depends on image
size, texture, noise and JPEG compression. A low value also occurs on a sharp,
uniform wall, so BLURRED means suspected blur / insufficient image detail.

Each analyze call represents one newly sampled image. ``analysis_hz`` is only
validated here; scheduling belongs to the future Card. This module does not
monitor missing frames, infer camera health, save images or read a clock.
"""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import asdict, dataclass, fields
import math
from numbers import Integral, Real
import re

import cv2
import numpy as np


HEALTHY = "HEALTHY"
DEGRADED = "DEGRADED"
UNKNOWN = "UNKNOWN"
LOW_LIGHT = "LOW_LIGHT"
OVEREXPOSED = "OVEREXPOSED"
BLURRED = "BLURRED"
REASONS = (LOW_LIGHT, OVEREXPOSED, BLURRED)


@dataclass(frozen=True)
class QualityConfig:
    """Thresholds for uint8 grayscale values and native-resolution Laplacian.

    Pixel comparisons are inclusive; mean and blur comparisons are strict.
    Ratios are fractions, not percentages. Counts are consecutive samples,
    independent for each reason. Numeric strings and booleans are rejected.
    """

    dark_pixel_threshold: int = 40
    bright_pixel_threshold: int = 240
    low_light_mean_threshold: float = 60.0
    low_light_ratio_threshold: float = 0.6
    overexposed_mean_threshold: float = 220.0
    overexposed_ratio_threshold: float = 0.3
    blur_variance_threshold: float = 100.0
    analysis_hz: float = 2.0
    confirm_samples: int = 3
    recover_samples: int = 5

    def __post_init__(self):
        integer_ranges = {
            "dark_pixel_threshold": (0, 254),
            "bright_pixel_threshold": (1, 255),
            "confirm_samples": (1, 1000),
            "recover_samples": (1, 1000),
        }
        for name, (lower, upper) in integer_ranges.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise ValueError(f"{name} must be an integer")
            if not lower <= value <= upper:
                raise ValueError(f"{name} must be in [{lower}, {upper}]")
            object.__setattr__(self, name, int(value))

        for name in (
            "low_light_mean_threshold", "overexposed_mean_threshold",
            "low_light_ratio_threshold", "overexposed_ratio_threshold",
            "blur_variance_threshold", "analysis_hz",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{name} must be a number")
            try:
                value = float(value)
            except (OverflowError, ValueError) as exc:
                raise ValueError(f"{name} must be finite") from exc
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)

        for name in ("low_light_mean_threshold", "overexposed_mean_threshold"):
            if not 0 <= getattr(self, name) <= 255:
                raise ValueError(f"{name} must be in [0, 255]")
        for name in ("low_light_ratio_threshold", "overexposed_ratio_threshold"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in (0, 1]")
        if self.blur_variance_threshold < 0:
            raise ValueError("blur_variance_threshold must be nonnegative")
        if not 0 < self.analysis_hz <= 30:
            raise ValueError("analysis_hz must be in (0, 30]")
        if self.dark_pixel_threshold >= self.bright_pixel_threshold:
            raise ValueError("dark_pixel_threshold must be below bright_pixel_threshold")
        if self.low_light_mean_threshold >= self.overexposed_mean_threshold:
            raise ValueError("low_light_mean_threshold must be below overexposed_mean_threshold")

    @classmethod
    def from_mapping(cls, values: Mapping | None = None) -> QualityConfig:
        if values is None:
            return cls()
        if not isinstance(values, Mapping):
            raise ValueError("configuration must be a mapping")
        unknown = set(values) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError("unknown configuration fields: " + ", ".join(sorted(map(str, unknown))))
        return cls(**dict(values))


class ImageQualityError(ValueError):
    """A sample that cannot honestly produce image quality metrics."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _configuration(config):
    return config if isinstance(config, QualityConfig) else QualityConfig.from_mapping(config)


def compute_metrics(image: np.ndarray, config=None) -> dict[str, float]:
    """Measure a nonempty uint8 grayscale or BGR image, without modifying it.

    No resizing or normalization is performed. In particular, floating-point
    arrays (including NaN/Inf), uint16 depth and four-channel images are refused.
    """
    config = _configuration(config)
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
        raise ImageQualityError("INVALID_IMAGE", "image must be a uint8 NumPy array")
    if image.size == 0 or image.ndim not in (2, 3):
        raise ImageQualityError("INVALID_IMAGE", "image must be nonempty grayscale or BGR")
    if image.ndim == 3 and image.shape[2] != 3:
        raise ImageQualityError("INVALID_IMAGE", "BGR image must have exactly three channels")
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        metrics = {
            "mean_luminance": float(gray.mean(dtype=np.float64)),
            "dark_pixel_ratio": float(np.mean(gray <= config.dark_pixel_threshold)),
            "bright_pixel_ratio": float(np.mean(gray >= config.bright_pixel_threshold)),
            "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F, ksize=1).var()),
        }
    except cv2.error as exc:
        raise ImageQualityError("INVALID_IMAGE", "OpenCV could not analyze the image") from exc
    if not all(math.isfinite(value) for value in metrics.values()):
        raise ImageQualityError("INVALID_METRICS", "image metrics must be finite")
    return metrics


def decode_jpeg(data, image_format: str = "jpeg") -> np.ndarray:
    """Decode JPEG bytes, checking both the advertised format and the payload.

    Bumi advertises ``jpeg``. Also accept MIME and standard ROS compressed-image
    spellings. Require SOI/EOI markers so a truncated JPEG that OpenCV happens to
    partially recover is not silently accepted as a complete sample. This is an
    input check, not a complete JPEG conformance validator.
    """
    if not isinstance(image_format, str):
        raise ImageQualityError("UNSUPPORTED_FORMAT", "image format must declare JPEG")
    image_format = image_format.strip().lower()
    if image_format not in {"jpeg", "jpg", "image/jpeg"} and not re.fullmatch(
        r"(?:bgr8|rgb8|mono8);\s*jpeg\s+compressed(?:\s+(?:bgr8|rgb8))?", image_format
    ):
        raise ImageQualityError("UNSUPPORTED_FORMAT", "only JPEG input is supported")
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise ImageQualityError("INVALID_JPEG", "JPEG input must be bytes")
    try:
        data = bytes(data)
    except (TypeError, ValueError) as exc:
        raise ImageQualityError("INVALID_JPEG", "JPEG bytes are unavailable") from exc
    if not data.startswith(b"\xff\xd8") or b"\xff\xd9" not in data[2:]:
        raise ImageQualityError("INVALID_JPEG", "JPEG start/end markers are missing")
    try:
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    except cv2.error as exc:
        raise ImageQualityError("DECODE_FAILED", "OpenCV could not decode JPEG") from exc
    if image is None or image.size == 0:
        raise ImageQualityError("DECODE_FAILED", "OpenCV could not decode JPEG")
    return image


class ImageQualityAnalyzer:
    """Clock-free state machine; callers serialize access and sample scheduling.

    Initial health requires ``recover_samples`` completely good images. Once
    healthy, unconfirmed anomalies do not change that state. Each reason has its
    own confirmation/recovery latch; clearing one cannot clear another.

    Invalid input overrides the displayed state with UNKNOWN, clears metrics and
    consecutive counters, but keeps confirmed reasons visible as unresolved
    alarms. Only a full run of valid recovery samples can clear those reasons;
    failure to observe an image is not evidence that an anomaly has recovered.
    If no latch remains but a new anomaly is pending, health stays UNKNOWN until
    either that anomaly is confirmed or a completely good run establishes health.
    """

    def __init__(self, config=None):
        self._config = _configuration(config)
        self.reset()

    @property
    def config(self) -> QualityConfig:
        return self._config

    def reset(self) -> None:
        self._confirmed = set()
        self._reset_streaks()
        self._latest = self._result(UNKNOWN, None, [], [], None)

    def _reset_streaks(self):
        self._onset = dict.fromkeys(REASONS, 0)
        self._recovery = dict.fromkeys(REASONS, 0)
        self._good_streak = 0
        self._healthy_confirmed = False

    @staticmethod
    def _result(state, metrics, reasons, candidates, error):
        return {"state": state, "metrics": metrics, "reasons": reasons,
                "candidate_reasons": candidates, "error": error}

    def snapshot(self) -> dict:
        return deepcopy(self._latest)

    def configure(self, changes: Mapping) -> None:
        """Atomically apply a partial configuration; changed settings reset state."""
        if not isinstance(changes, Mapping):
            raise ValueError("configuration changes must be a mapping")
        candidate = QualityConfig.from_mapping({**asdict(self._config), **dict(changes)})
        if candidate != self._config:
            self._config = candidate
            self.reset()

    def _invalid(self, error: ImageQualityError) -> dict:
        self._reset_streaks()
        self._latest = self._result(
            UNKNOWN, None, [reason for reason in REASONS if reason in self._confirmed],
            [], {"code": error.code, "message": str(error)})
        return self.snapshot()

    def analyze_jpeg(self, data, image_format: str = "jpeg") -> dict:
        try:
            image = decode_jpeg(data, image_format)
        except ImageQualityError as exc:
            return self._invalid(exc)
        return self.analyze_image(image)

    def analyze_image(self, image: np.ndarray) -> dict:
        try:
            metrics = compute_metrics(image, self._config)
        except ImageQualityError as exc:
            return self._invalid(exc)
        config = self._config
        flags = {
            LOW_LIGHT: (metrics["mean_luminance"] < config.low_light_mean_threshold
                        and metrics["dark_pixel_ratio"] >= config.low_light_ratio_threshold),
            OVEREXPOSED: (metrics["mean_luminance"] > config.overexposed_mean_threshold
                          and metrics["bright_pixel_ratio"] >= config.overexposed_ratio_threshold),
            BLURRED: metrics["laplacian_variance"] < config.blur_variance_threshold,
        }
        for reason, present in flags.items():
            if present:
                self._onset[reason] = min(self._onset[reason] + 1, config.confirm_samples)
                self._recovery[reason] = 0
                if self._onset[reason] >= config.confirm_samples:
                    self._confirmed.add(reason)
            else:
                self._onset[reason] = 0
                self._recovery[reason] = min(self._recovery[reason] + 1, config.recover_samples)
                if self._recovery[reason] >= config.recover_samples:
                    self._confirmed.discard(reason)

        self._good_streak = (0 if any(flags.values()) else
                             min(self._good_streak + 1, config.recover_samples))
        if self._confirmed:
            self._healthy_confirmed = False
            state = DEGRADED
        else:
            if self._good_streak >= config.recover_samples:
                self._healthy_confirmed = True
            state = HEALTHY if self._healthy_confirmed else UNKNOWN
        self._latest = self._result(
            state, metrics, [r for r in REASONS if r in self._confirmed],
            [r for r in REASONS if flags[r]], None)
        return self.snapshot()
