"""Ultralytics YOLO instance-segmentation adapter.

Ultralytics is imported only by :meth:`UltralyticsSegmentationRuntime.load`.
The caller must provide an existing local model file; this module never asks
Ultralytics to resolve or download a model name.
"""

from __future__ import annotations

import importlib
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

try:  # package import in tests/development
    from .food_segmentation_contract import (
        RawInstance,
        mask_bounds,
        resolve_food_class_ids,
    )
except ImportError:  # flat /work layout used by the Bumi Docker image
    from food_segmentation_contract import (
        RawInstance,
        mask_bounds,
        resolve_food_class_ids,
    )


def _to_numpy(value: Any, label: str) -> np.ndarray:
    """Detach a tensor, move it to CPU, and return a NumPy array."""

    if value is None:
        raise RuntimeError(f"Ultralytics result is missing {label}")
    try:
        detached = value.detach() if callable(getattr(value, "detach", None)) else value
        cpu_value = detached.cpu() if callable(getattr(detached, "cpu", None)) else detached
        array = cpu_value.numpy() if callable(getattr(cpu_value, "numpy", None)) else cpu_value
        return np.asarray(array)
    except Exception as exc:  # backend objects can raise device-specific errors
        raise RuntimeError(f"could not convert {label} to a CPU NumPy array: {exc}") from exc


def _image_shape(image: Any) -> tuple[int, int]:
    if image is None or isinstance(image, (str, bytes, Path)):
        raise ValueError("image must be an in-memory HxW or HxWxC array")
    shape = getattr(image, "shape", None)
    if not isinstance(shape, Sequence) or len(shape) not in (2, 3):
        raise ValueError("image must expose shape (height, width[, channels])")
    height, width = shape[:2]
    if not all(isinstance(value, (int, np.integer)) and not isinstance(value, bool)
               and int(value) > 0 for value in (height, width)):
        raise ValueError("image height and width must be positive integers")
    if len(shape) == 3 and int(shape[2]) not in (1, 3, 4):
        raise ValueError("image must have 1, 3, or 4 channels")
    return int(height), int(width)


def _orig_shape(result: Any) -> tuple[int, int]:
    shape = getattr(result, "orig_shape", None)
    if not isinstance(shape, Sequence) or len(shape) != 2:
        raise RuntimeError("Ultralytics result.orig_shape must be (height, width)")
    height, width = shape
    if not all(isinstance(value, (int, np.integer)) and not isinstance(value, bool)
               and int(value) > 0 for value in (height, width)):
        raise RuntimeError("Ultralytics result.orig_shape must contain positive integers")
    return int(height), int(width)


def _resize_nearest(mask: np.ndarray, output_shape: tuple[int, int]) -> np.ndarray:
    """Resize one 2-D mask with deterministic nearest-neighbour sampling."""

    output_h, output_w = output_shape
    input_h, input_w = mask.shape
    y = np.minimum((np.arange(output_h) * input_h / output_h).astype(int), input_h - 1)
    x = np.minimum((np.arange(output_w) * input_w / output_w).astype(int), input_w - 1)
    return mask[y[:, None], x[None, :]]


def restore_mask(mask: np.ndarray, original_shape: tuple[int, int]) -> np.ndarray:
    """Restore an inference/letterbox mask to the original image dimensions.

    With ``retina_masks=True`` this is normally a no-op.  A mismatched mask is
    interpreted as a YOLO letterbox canvas: symmetric padding is cropped using
    the same gain geometry, then the content is resized with nearest-neighbour
    sampling so binary mask labels are not interpolated.
    """

    array = np.asarray(mask)
    if array.ndim != 2 or not all(size > 0 for size in array.shape):
        raise RuntimeError("each segmentation mask must be a non-empty 2-D array")
    original_h, original_w = original_shape
    if array.shape == original_shape:
        return array

    mask_h, mask_w = array.shape
    gain = min(mask_h / original_h, mask_w / original_w)
    content_h, content_w = original_h * gain, original_w * gain
    pad_h, pad_w = (mask_h - content_h) / 2, (mask_w - content_w) / 2
    top = max(0, round(pad_h - 0.1))
    bottom = min(mask_h, round(mask_h - pad_h + 0.1))
    left = max(0, round(pad_w - 0.1))
    right = min(mask_w, round(mask_w - pad_w + 0.1))
    cropped = array[top:bottom, left:right]
    if cropped.size == 0:
        raise RuntimeError("letterbox padding leaves an empty segmentation mask")
    return _resize_nearest(cropped, original_shape)


def align_bbox_to_mask(bbox: Sequence[float], binary: np.ndarray,
                       tolerance_px: float = 1.0) -> list[float]:
    """Expand a model bbox for at most one pixel of mask quantization spill.

    Ultralytics boxes are floating point while masks occupy complete pixels.
    Thresholding can therefore place an edge pixel just outside a fractional
    box.  The box is expanded, never contracted, to the mask's tight half-open
    bounds.  A larger disagreement indicates a coordinate restoration bug or
    unsupported output and is rejected explicitly.
    """

    values = [float(value) for value in bbox]
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise RuntimeError("Ultralytics bbox must contain four finite coordinates")
    bounds = mask_bounds(binary.tolist())
    if bounds is None:
        raise RuntimeError("Ultralytics returned an empty instance mask")
    x1, y1, x2, y2 = values
    mx1, my1, mx2, my2 = bounds
    expansion = (max(0.0, x1 - mx1), max(0.0, y1 - my1),
                 max(0.0, mx2 - x2), max(0.0, my2 - y2))
    if any(value > tolerance_px + 1e-6 for value in expansion):
        raise RuntimeError(
            f"bbox/mask coordinate mismatch exceeds {tolerance_px:g}px: {expansion}"
        )
    height, width = binary.shape
    return [max(0.0, min(x1, mx1)), max(0.0, min(y1, my1)),
            min(float(width), max(x2, mx2)), min(float(height), max(y2, my2))]


class UltralyticsSegmentationRuntime:
    """A lazy implementation of the existing ``SegmentationRuntime`` protocol."""

    def __init__(self) -> None:
        self._model: Any = None
        self._names: Mapping[int, str] | Sequence[str] | None = None
        self._model_path: Path | None = None
        self._predict_options: dict[str, Any] = {}
        self._max_instances: int | None = None
        self._last_diagnostics: dict[str, Any] = {}

    @property
    def names(self) -> Mapping[int, str] | Sequence[str]:
        if self._names is None:
            raise RuntimeError("Ultralytics segmentation model is not loaded")
        return self._names

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        """Return a copy of the last successful result-parsing diagnostics."""

        return dict(self._last_diagnostics)

    def load(self, model_path: str, **options: Any) -> None:
        options = dict(options)
        max_instances = options.pop("max_instances", None)
        if max_instances is not None and (
                isinstance(max_instances, bool)
                or not isinstance(max_instances, int) or max_instances <= 0):
            raise ValueError("max_instances must be a positive integer or None")
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"segmentation model file does not exist: {path}")
        try:
            ultralytics = importlib.import_module("ultralytics")
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed; install it in an isolated environment"
            ) from exc
        yolo = getattr(ultralytics, "YOLO", None)
        if yolo is None:
            raise RuntimeError("installed ultralytics package does not expose YOLO")
        try:
            model = yolo(str(path))
        except Exception as exc:
            raise RuntimeError(f"failed to load segmentation model {path}: {exc}") from exc
        task = getattr(model, "task", None)
        if task != "segment":
            raise ValueError(f"model task must be 'segment', got {task!r}")
        names = getattr(model, "names", None)
        if names is None:
            raise ValueError("segmentation model does not expose class names metadata")
        resolve_food_class_ids(names)

        # Publish state only after every validation succeeds.
        self._model = model
        self._names = names
        self._model_path = path
        self._max_instances = max_instances
        if max_instances is not None:
            options["max_det"] = max_instances
        self._predict_options = options

    def predict(self, image: Any) -> list[RawInstance]:
        if self._model is None:
            raise RuntimeError("load() must succeed before predict()")
        input_shape = _image_shape(image)
        options = dict(self._predict_options)
        options.update({"source": image, "retina_masks": True, "verbose": False})
        try:
            results = self._model.predict(**options)
        except Exception as exc:
            raise RuntimeError(f"Ultralytics segmentation prediction failed: {exc}") from exc
        if not isinstance(results, Sequence) or len(results) != 1:
            count = len(results) if isinstance(results, Sequence) else "unknown"
            raise RuntimeError(f"single-image prediction must return one result, got {count}")
        return self._parse_result(results[0], input_shape)

    def _parse_result(self, result: Any,
                      input_shape: tuple[int, int]) -> list[RawInstance]:
        original_shape = _orig_shape(result)
        if original_shape != input_shape:
            raise RuntimeError(
                f"result.orig_shape {original_shape} does not match input image {input_shape}"
            )
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            raise RuntimeError("Ultralytics result does not contain boxes")
        xyxy = _to_numpy(getattr(boxes, "xyxy", None), "boxes.xyxy")
        confidence = _to_numpy(getattr(boxes, "conf", None), "boxes.conf")
        classes = _to_numpy(getattr(boxes, "cls", None), "boxes.cls")
        if xyxy.ndim != 2 or xyxy.shape[1:] != (4,):
            raise RuntimeError(f"boxes.xyxy must have shape (N, 4), got {xyxy.shape}")
        confidence = confidence.reshape(-1)
        classes = classes.reshape(-1)
        count = len(xyxy)
        if len(confidence) != count or len(classes) != count:
            raise RuntimeError("box, confidence and class counts do not match")
        if self._max_instances is not None and count > self._max_instances:
            raise RuntimeError(
                f"model returned more than {self._max_instances} detections")
        if count == 0:
            self._last_diagnostics = {
                "input_shape": list(input_shape),
                "orig_shape": list(original_shape),
                "detection_count": 0,
                "raw_mask_shape": None,
                "mask_restoration_count": 0,
                "bbox_adjustments": [],
            }
            return []

        masks_object = getattr(result, "masks", None)
        if masks_object is None:
            raise RuntimeError("segmentation result has detections but no masks")
        masks = _to_numpy(getattr(masks_object, "data", None), "masks.data")
        if masks.ndim != 3:
            raise RuntimeError(f"masks.data must have shape (N, H, W), got {masks.shape}")
        if len(masks) != count:
            raise RuntimeError(
                f"box count {count} does not match mask count {len(masks)}"
            )

        instances = []
        raw_mask_shape = list(masks.shape[1:])
        restoration_count = 0
        bbox_adjustments = []
        detection_diagnostics = []
        for index in range(count):
            if tuple(masks[index].shape) != original_shape:
                restoration_count += 1
            restored = restore_mask(masks[index], original_shape)
            if not np.isfinite(restored).all():
                raise RuntimeError(f"mask {index} contains non-finite values")
            binary = (restored >= 0.5).astype(np.uint8)
            bbox = align_bbox_to_mask(xyxy[index], binary)
            original_bbox = [float(value) for value in xyxy[index]]
            bbox_adjustments.append([
                bbox[edge] - original_bbox[edge] for edge in range(4)
            ])
            class_value = float(classes[index])
            if not math.isfinite(class_value) or not class_value.is_integer():
                raise RuntimeError(f"class ID at index {index} is not an integer")
            instances.append(RawInstance(
                class_id=int(class_value),
                confidence=float(confidence[index]),
                bbox=bbox,
                mask=binary,
            ))
            detection_diagnostics.append({
                "class_id": int(class_value),
                "confidence": float(confidence[index]),
                "original_bbox_xyxy": original_bbox,
                "aligned_bbox_xyxy": bbox,
                "bbox_adjustment_xyxy": bbox_adjustments[-1],
            })
        self._last_diagnostics = {
            "input_shape": list(input_shape),
            "orig_shape": list(original_shape),
            "detection_count": count,
            "raw_mask_shape": raw_mask_shape,
            "mask_restoration_count": restoration_count,
            "bbox_adjustments": bbox_adjustments,
            "detections": detection_diagnostics,
        }
        return instances
