"""ROS-free data contract for the Bumi food instance-segmentation card.

Bounding boxes use half-open ``[x1, y1, x2, y2]`` pixel coordinates.  Masks
are full-image, row-major matrices whose shape is ``height x width``.  The RLE
format in this module is deliberately named ``binary-rle-row-major-v1``: it is
not COCO's column-major compressed or uncompressed RLE.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping, Sequence


# Verified against Ultralytics' official COCO segmentation dataset metadata.
# Runtime metadata is still checked by ``resolve_food_class_ids``; inference
# never assumes that an arbitrary model uses this ordering.
COCO_FOOD_CLASSES = {
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    50: "broccoli",
    51: "carrot",
    52: "hot dog",
    53: "pizza",
    54: "donut",
    55: "cake",
}
FOOD_CLASS_NAMES = frozenset(COCO_FOOD_CLASSES.values())
RLE_ENCODING = "binary-rle-row-major-v1"


@dataclass(frozen=True)
class RawInstance:
    """One backend result before contract validation."""

    class_id: int
    confidence: float
    bbox: Sequence[float]
    mask: Sequence[Sequence[Any]]


def _model_names(names: Mapping[int, str] | Sequence[str]) -> dict[int, str]:
    if isinstance(names, Mapping):
        result = {}
        for key, value in names.items():
            if isinstance(key, bool) or not isinstance(key, int):
                raise TypeError("model class IDs must be integers")
            if not isinstance(value, str) or not value:
                raise ValueError("model class names must be non-empty strings")
            result[key] = value
        return result
    if isinstance(names, (str, bytes)) or not isinstance(names, Sequence):
        raise TypeError("model names must be an ID mapping or a sequence")
    if not all(isinstance(value, str) and value for value in names):
        raise ValueError("model class names must be non-empty strings")
    return dict(enumerate(names))


def resolve_food_class_ids(
    names: Mapping[int, str] | Sequence[str], *, require_coco_ids: bool = True
) -> dict[int, str]:
    """Resolve food IDs from actual model metadata and optionally verify COCO IDs."""

    actual = _model_names(names)
    selected = {class_id: name for class_id, name in actual.items()
                if name in FOOD_CLASS_NAMES}
    missing = FOOD_CLASS_NAMES.difference(selected.values())
    if missing:
        raise ValueError(f"model is missing food classes: {sorted(missing)}")
    if require_coco_ids:
        mismatched = {
            class_id: (COCO_FOOD_CLASSES[class_id], actual.get(class_id))
            for class_id in COCO_FOOD_CLASSES
            if actual.get(class_id) != COCO_FOOD_CLASSES[class_id]
        }
        if mismatched:
            raise ValueError(f"model metadata does not match COCO class IDs: {mismatched}")
    return selected


def validate_bbox(bbox: Sequence[float], image_size: Sequence[int]) -> list[float]:
    """Validate a finite, non-empty half-open bbox inside ``[width, height]``."""

    if isinstance(bbox, (str, bytes)) or len(bbox) != 4:
        raise ValueError("bbox must contain four coordinates")
    if len(image_size) != 2:
        raise ValueError("image_size must be [width, height]")
    width, height = image_size
    if isinstance(width, bool) or isinstance(height, bool) or not all(
            isinstance(v, int) and v > 0 for v in (width, height)):
        raise ValueError("image dimensions must be positive integers")
    try:
        result = [float(value) for value in bbox]
    except (TypeError, ValueError) as exc:
        raise ValueError("bbox coordinates must be numbers") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError("bbox coordinates must be finite")
    x1, y1, x2, y2 = result
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError("bbox must be non-empty and inside the image")
    return result


def binary_mask(mask: Sequence[Sequence[Any]], image_size: Sequence[int],
                threshold: float = 0.5) -> list[list[int]]:
    """Validate and threshold a full-image mask without using bbox geometry."""

    if not isinstance(threshold, (int, float)) or isinstance(threshold, bool) \
            or not math.isfinite(float(threshold)):
        raise ValueError("mask threshold must be finite")
    if len(image_size) != 2:
        raise ValueError("image_size must be [width, height]")
    width, height = image_size
    if isinstance(mask, (str, bytes)) or not isinstance(mask, Sequence):
        raise TypeError("mask must be a two-dimensional sequence")
    if len(mask) != height:
        raise ValueError(f"mask height {len(mask)} does not match image height {height}")
    output = []
    for row in mask:
        if isinstance(row, (str, bytes)) or not isinstance(row, Sequence):
            raise TypeError("each mask row must be a sequence")
        if len(row) != width:
            raise ValueError(f"mask width {len(row)} does not match image width {width}")
        converted = []
        for value in row:
            if isinstance(value, bool):
                number = float(value)
            elif isinstance(value, (int, float)):
                number = float(value)
            else:
                raise TypeError("mask values must be numeric")
            if not math.isfinite(number):
                raise ValueError("mask values must be finite")
            converted.append(1 if number >= threshold else 0)
        output.append(converted)
    return output


def mask_bounds(mask: Sequence[Sequence[int]]) -> list[int] | None:
    """Return the tight half-open bbox of positive pixels, or ``None``."""

    points = [(x, y) for y, row in enumerate(mask) for x, value in enumerate(row)
              if value]
    if not points:
        return None
    xs, ys = zip(*points)
    return [min(xs), min(ys), max(xs) + 1, max(ys) + 1]


def bbox_encloses_mask(bbox: Sequence[float], mask: Sequence[Sequence[int]]) -> bool:
    bounds = mask_bounds(mask)
    if bounds is None:
        return False
    x1, y1, x2, y2 = bbox
    mx1, my1, mx2, my2 = bounds
    return x1 <= mx1 and y1 <= my1 and x2 >= mx2 and y2 >= my2


def encode_binary_rle(mask: Sequence[Sequence[int]]) -> dict[str, Any]:
    """Encode row-major pixels as alternating zero/one run lengths.

    ``size`` is ``[height, width]``. ``counts[0]`` is the initial zero run and
    may be zero when the first pixel is one. All following runs are positive.
    """

    height = len(mask)
    if height == 0 or not isinstance(mask[0], Sequence) or len(mask[0]) == 0:
        raise ValueError("mask must be non-empty")
    width = len(mask[0])
    flat = []
    for row in mask:
        if len(row) != width:
            raise ValueError("mask rows must have equal width")
        for value in row:
            if value not in (0, 1, False, True):
                raise ValueError("RLE input mask must be binary")
            flat.append(int(bool(value)))
    counts = []
    expected = 0
    run = 0
    for value in flat:
        if value == expected:
            run += 1
        else:
            counts.append(run)
            run = 1
            expected = value
    counts.append(run)
    return {"encoding": RLE_ENCODING, "size": [height, width], "counts": counts}


def decode_binary_rle(rle: Mapping[str, Any]) -> list[list[int]]:
    """Decode ``binary-rle-row-major-v1`` and reject ambiguous/malformed runs."""

    if not isinstance(rle, Mapping) or rle.get("encoding") != RLE_ENCODING:
        raise ValueError(f"RLE encoding must be {RLE_ENCODING}")
    size = rle.get("size")
    if not isinstance(size, Sequence) or isinstance(size, (str, bytes)) or len(size) != 2:
        raise ValueError("RLE size must be [height, width]")
    height, width = size
    if not all(isinstance(v, int) and not isinstance(v, bool) and v > 0
               for v in (height, width)):
        raise ValueError("RLE dimensions must be positive integers")
    counts = rle.get("counts")
    if not isinstance(counts, Sequence) or isinstance(counts, (str, bytes)) or not counts:
        raise ValueError("RLE counts must be a non-empty sequence")
    for index, count in enumerate(counts):
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError("RLE counts must be non-negative integers")
        if index > 0 and count == 0:
            raise ValueError("only the initial zero run may be empty")
    if sum(counts) != height * width:
        raise ValueError("RLE run lengths do not match its size")
    flat = []
    value = 0
    for count in counts:
        flat.extend([value] * count)
        value = 1 - value
    return [flat[offset:offset + width] for offset in range(0, len(flat), width)]


def build_instance(raw: RawInstance, names: Mapping[int, str] | Sequence[str],
                   image_size: Sequence[int], confidence_threshold: float = 0.25,
                   mask_threshold: float = 0.5) -> dict[str, Any] | None:
    """Validate one backend instance; return ``None`` when intentionally filtered."""

    model_names = _model_names(names)
    if isinstance(raw.class_id, bool) or not isinstance(raw.class_id, int):
        raise TypeError("class_id must be an integer")
    if raw.class_id not in model_names:
        raise ValueError(f"unknown model class ID {raw.class_id}")
    class_name = model_names[raw.class_id]
    if class_name not in FOOD_CLASS_NAMES:
        return None
    if isinstance(raw.confidence, bool) or not isinstance(raw.confidence, (int, float)):
        raise TypeError("confidence must be numeric")
    confidence = float(raw.confidence)
    if not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError("confidence must be finite and in [0, 1]")
    if not isinstance(confidence_threshold, (int, float)) \
            or isinstance(confidence_threshold, bool) \
            or not 0 <= float(confidence_threshold) <= 1:
        raise ValueError("confidence_threshold must be in [0, 1]")
    if confidence < float(confidence_threshold):
        return None
    bbox = validate_bbox(raw.bbox, image_size)
    mask = binary_mask(raw.mask, image_size, threshold=mask_threshold)
    area = sum(sum(row) for row in mask)
    if area == 0:
        raise ValueError("instance mask must contain at least one positive pixel")
    if not bbox_encloses_mask(bbox, mask):
        raise ValueError("bbox does not enclose the positive mask pixels")
    width, height = image_size
    return {
        "class_id": raw.class_id,
        "class_name": class_name,
        "confidence": confidence,
        "bbox_xyxy": bbox,
        "mask": encode_binary_rle(mask),
        "mask_area_px": area,
        "mask_area_ratio": area / (width * height),
    }


def build_result(raw_instances: Iterable[RawInstance],
                 names: Mapping[int, str] | Sequence[str], image_size: Sequence[int],
                 confidence_threshold: float = 0.25,
                 mask_threshold: float = 0.5) -> dict[str, Any]:
    """Build a frame result, isolating malformed instances as reported errors."""

    resolve_food_class_ids(names)
    if isinstance(raw_instances, (str, bytes)) or not isinstance(raw_instances, Iterable):
        raise TypeError("raw_instances must be iterable")
    width, height = image_size
    # Validate dimensions even for an empty result.
    validate_bbox([0, 0, width, height], image_size)
    instances, errors = [], []
    for index, raw in enumerate(raw_instances):
        try:
            if not isinstance(raw, RawInstance):
                raise TypeError("backend item must be RawInstance")
            instance = build_instance(raw, names, image_size, confidence_threshold,
                                      mask_threshold)
            if instance is not None:
                instances.append(instance)
        except (TypeError, ValueError) as exc:
            errors.append({"index": index, "error": str(exc)})
    return {
        "image_size": [width, height],
        "instances": instances,
        "errors": errors,
    }
