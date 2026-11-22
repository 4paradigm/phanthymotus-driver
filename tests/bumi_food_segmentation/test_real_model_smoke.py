"""Opt-in real YOLO smoke test; never downloads weights or images.

Set all three variables to run:

* FOOD_SEGMENTATION_MODEL_PATH: local yolo11n-seg.pt
* FOOD_SEGMENTATION_TEST_IMAGE: local image containing a known food instance
* FOOD_SEGMENTATION_EXPECTED_CLASS: expected COCO food name in that image
"""

import math
import os
from pathlib import Path

import pytest

from noetix.bumi.food_segmentation_backend import FoodSegmentationPipeline
from noetix.bumi.food_segmentation_contract import decode_binary_rle
from noetix.bumi.food_segmentation_ultralytics import UltralyticsSegmentationRuntime


MODEL = os.environ.get("FOOD_SEGMENTATION_MODEL_PATH")
IMAGE = os.environ.get("FOOD_SEGMENTATION_TEST_IMAGE")
EXPECTED = os.environ.get("FOOD_SEGMENTATION_EXPECTED_CLASS")
pytestmark = pytest.mark.skipif(
    not all((MODEL, IMAGE, EXPECTED)),
    reason="set local model, test image, and expected food class variables",
)


def test_real_yolo_food_instance_mask_is_not_bbox_fill():
    np = pytest.importorskip("numpy")
    pil_image = pytest.importorskip("PIL.Image")
    pytest.importorskip("ultralytics")
    assert Path(MODEL).is_file(), f"model path does not exist: {MODEL}"
    assert Path(IMAGE).is_file(), f"test image path does not exist: {IMAGE}"

    image = np.asarray(pil_image.open(IMAGE).convert("RGB"))
    height, width = image.shape[:2]
    runtime = UltralyticsSegmentationRuntime()
    runtime.load(MODEL, device="cpu")
    result = FoodSegmentationPipeline(runtime, [width, height]).process(image)
    matches = [item for item in result["instances"] if item["class_name"] == EXPECTED]
    assert matches, f"fixture must contain a detectable {EXPECTED!r}: {result}"
    instance = matches[0]
    decoded = decode_binary_rle(instance["mask"])
    assert sum(map(sum, decoded)) == instance["mask_area_px"] > 0
    x1, y1, x2, y2 = instance["bbox_xyxy"]
    integer_bbox_area = (math.ceil(x2) - math.floor(x1)) * \
                        (math.ceil(y2) - math.floor(y1))
    assert instance["mask_area_px"] < integer_bbox_area
