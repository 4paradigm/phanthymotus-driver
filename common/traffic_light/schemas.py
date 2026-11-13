"""ROS-free configuration and evidence contracts for traffic-light observations."""
from dataclasses import asdict, dataclass, field
import math


@dataclass(frozen=True)
class ColorThresholds:
    # Keep the hue wrap and saturation gate of the existing perception prototype.
    min_saturation: int = 90
    min_value: int = 90
    red_hue_low: int = 10
    red_hue_high: int = 170
    yellow_hue_low: int = 15
    yellow_hue_high: int = 35
    green_hue_low: int = 36
    green_hue_high: int = 95
    min_pixels: int = 12
    min_symbol_size: int = 16


@dataclass
class ColorEvidence:
    color: str = "unknown"
    color_confidence: float = 0.0
    symbol: str = "unknown"
    symbol_confidence: float = 0.0
    bulbs: list = field(default_factory=list)
    reason: str = "no_emitting_region"

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class Config:
    cache_size: int = 64
    max_image_bytes: int = 1048576
    max_cache_bytes: int = 16777216
    max_age_s: float = 1.0
    max_tracks: int = 32
    track_ttl_s: float = 1.0
    min_detection_confidence: float = 0.3

    @classmethod
    def from_dict(cls, values):
        fields = cls.__dataclass_fields__
        config = cls(**{k: v for k, v in (values or {}).items() if k in fields})
        for name, lo, hi in (("cache_size", 2, 180), ("max_image_bytes", 4096, 4194304),
                             ("max_cache_bytes", 1048576, 67108864),
                             ("max_tracks", 1, 128)):
            value = getattr(config, name)
            if isinstance(value, bool) or not isinstance(value, int) or not lo <= value <= hi:
                raise ValueError(f"{name} must be an integer within {lo}..{hi}")
        for name, lo, hi in (("max_age_s", .05, 5.), ("track_ttl_s", .1, 5.),
                             ("min_detection_confidence", 0., 1.)):
            value = getattr(config, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not lo <= value <= hi):
                raise ValueError(f"{name} must be finite and within {lo}..{hi}")
        return config


def bbox_of(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in value):
        return None
    x1, y1, x2, y2 = map(float, value)
    return [x1, y1, x2, y2] if x2 > x1 and y2 > y1 else None


def stamp_ns(stamp):
    if not isinstance(stamp, dict):
        return None
    sec, nano = stamp.get("sec"), stamp.get("nanosec")
    if (type(sec) is not int or type(nano) is not int or sec < 0
            or not 0 <= nano < 10**9 or sec == nano == 0):
        return None
    return sec * 10**9 + nano
