"""Lazy, replaceable backend boundary for food segmentation.

This module is ROS-free and imports neither Ultralytics nor TensorRT.  A future
backend owns model loading in ``load()``; importing this module never loads or
downloads weights.
"""

from __future__ import annotations

from typing import Any, Mapping, Protocol, Sequence

try:  # package import in tests/development
    from .food_segmentation_contract import RawInstance, build_result
except ImportError:  # flat /work layout used by the Bumi Docker image
    from food_segmentation_contract import RawInstance, build_result


class SegmentationRuntime(Protocol):
    @property
    def names(self) -> Mapping[int, str] | Sequence[str]:
        """Class metadata read from the configured model."""

    def load(self, model_path: str, **options: Any) -> None:
        """Explicitly load a local model; implementations must not auto-download."""

    def predict(self, image: Any) -> Sequence[RawInstance]:
        """Return full-image instance masks for one image."""


class FoodSegmentationPipeline:
    """Backend-neutral contract adapter used by fake, Ultralytics and TRT runtimes."""

    def __init__(self, runtime: SegmentationRuntime, image_size: Sequence[int],
                 confidence_threshold: float = 0.25, mask_threshold: float = 0.5):
        self.runtime = runtime
        self.image_size = image_size
        self.confidence_threshold = confidence_threshold
        self.mask_threshold = mask_threshold

    def process(self, image: Any) -> dict[str, Any]:
        raw = self.runtime.predict(image)
        return build_result(raw, self.runtime.names, self.image_size,
                            self.confidence_threshold, self.mask_threshold)
