import sys
import types

import numpy as np
import pytest

from noetix.bumi.food_segmentation_contract import COCO_FOOD_CLASSES
from noetix.bumi.food_segmentation_ultralytics import (
    UltralyticsSegmentationRuntime,
    align_bbox_to_mask,
    restore_mask,
)


NAMES = {index: f"other-{index}" for index in range(80)}
NAMES.update(COCO_FOOD_CLASSES)


class FakeTensor:
    def __init__(self, value):
        self.value = np.asarray(value)
        self.detached = False
        self.on_cpu = False

    def detach(self):
        self.detached = True
        return self

    def cpu(self):
        self.on_cpu = True
        return self

    def numpy(self):
        assert self.detached and self.on_cpu
        return self.value


class FakeBoxes:
    def __init__(self, xyxy, confidence, classes):
        self.xyxy = FakeTensor(xyxy)
        self.conf = FakeTensor(confidence)
        self.cls = FakeTensor(classes)


class FakeMasks:
    def __init__(self, data):
        self.data = FakeTensor(data)


class FakeResult:
    def __init__(self, boxes, masks, orig_shape=(6, 8)):
        self.boxes = boxes
        self.masks = masks
        self.orig_shape = orig_shape


class FakeModel:
    task = "segment"
    names = NAMES

    def __init__(self, results):
        self.results = results
        self.calls = []

    def predict(self, **options):
        self.calls.append(options)
        return self.results


def install_fake_ultralytics(monkeypatch, model):
    module = types.ModuleType("ultralytics")
    module.YOLO = lambda path: model
    monkeypatch.setitem(sys.modules, "ultralytics", module)


def make_runtime(monkeypatch, tmp_path, result):
    model = FakeModel([result])
    install_fake_ultralytics(monkeypatch, model)
    model_path = tmp_path / "yolo11n-seg.pt"
    model_path.write_bytes(b"fake, never parsed by FakeModel")
    runtime = UltralyticsSegmentationRuntime()
    runtime.load(str(model_path), device="cpu", imgsz=640)
    return runtime, model


def shaped_mask(points, shape=(6, 8)):
    output = np.zeros(shape, dtype=np.float32)
    for x, y in points:
        output[y, x] = 1
    return output


def test_load_is_explicit_reads_metadata_and_import_is_lazy(monkeypatch, tmp_path):
    monkeypatch.delitem(sys.modules, "ultralytics", raising=False)
    runtime = UltralyticsSegmentationRuntime()
    with pytest.raises(RuntimeError, match="not loaded"):
        _ = runtime.names
    model = FakeModel([])
    install_fake_ultralytics(monkeypatch, model)
    path = tmp_path / "yolo11n-seg.pt"
    path.write_bytes(b"fake")
    runtime.load(str(path))
    assert runtime.names[46] == "banana" and runtime.names[55] == "cake"


def test_load_rejects_missing_file_before_import(monkeypatch, tmp_path):
    monkeypatch.delitem(sys.modules, "ultralytics", raising=False)
    with pytest.raises(FileNotFoundError, match="does not exist"):
        UltralyticsSegmentationRuntime().load(str(tmp_path / "missing.pt"))


@pytest.mark.parametrize("task,names,message", [
    ("detect", NAMES, "task must be"),
    ("segment", {0: "person"}, "missing food classes"),
])
def test_load_rejects_wrong_task_or_metadata(monkeypatch, tmp_path, task, names, message):
    model = FakeModel([])
    model.task, model.names = task, names
    install_fake_ultralytics(monkeypatch, model)
    path = tmp_path / "model.pt"
    path.write_bytes(b"fake")
    with pytest.raises(ValueError, match=message):
        UltralyticsSegmentationRuntime().load(str(path))


def test_load_wraps_model_loader_failure(monkeypatch, tmp_path):
    module = types.ModuleType("ultralytics")
    module.YOLO = lambda path: (_ for _ in ()).throw(OSError("bad checkpoint"))
    monkeypatch.setitem(sys.modules, "ultralytics", module)
    path = tmp_path / "model.pt"
    path.write_bytes(b"bad")
    with pytest.raises(RuntimeError, match="failed to load.*bad checkpoint"):
        UltralyticsSegmentationRuntime().load(str(path))


def test_predict_requires_load_and_valid_in_memory_image():
    runtime = UltralyticsSegmentationRuntime()
    with pytest.raises(RuntimeError, match=r"load\(\)"):
        runtime.predict(np.zeros((6, 8, 3), np.uint8))
    runtime._model, runtime._names = FakeModel([]), NAMES
    for bad in (None, "https://example/image.jpg", np.zeros((6, 8, 2))):
        with pytest.raises(ValueError):
            runtime.predict(bad)


def test_parse_real_masks_keeps_order_and_same_class_instances(monkeypatch, tmp_path):
    masks = np.stack([
        shaped_mask([(1, 1), (1, 2), (2, 2)]),
        shaped_mask([(5, 3), (6, 3)]),
        shaped_mask([(3, 4)]),
    ])
    result = FakeResult(FakeBoxes(
        [[1, 1, 3, 3], [5, 3, 7, 4], [3, 4, 4, 5]],
        [.9, .8, .7], [47, 47, 46],
    ), FakeMasks(masks))
    runtime, model = make_runtime(monkeypatch, tmp_path, result)
    instances = runtime.predict(np.zeros((6, 8, 3), np.uint8))
    assert [item.class_id for item in instances] == [47, 47, 46]
    assert [sum(map(sum, item.mask)) for item in instances] == [3, 2, 1]
    call = model.calls[0]
    assert call["retina_masks"] is True and call["verbose"] is False
    assert call["device"] == "cpu" and call["imgsz"] == 640
    diagnostics = runtime.last_diagnostics
    assert diagnostics["raw_mask_shape"] == [6, 8]
    assert diagnostics["mask_restoration_count"] == 0
    assert diagnostics["detection_count"] == 3
    assert diagnostics["bbox_adjustments"] == [[0, 0, 0, 0]] * 3


def test_letterbox_mask_is_cropped_then_nearest_resized():
    # Original 2x4 image letterboxed into 4x4: one padding row above and below.
    padded = np.zeros((4, 4), dtype=np.uint8)
    padded[1, 1] = 1
    padded[2, 2] = 1
    restored = restore_mask(padded, (2, 4))
    assert restored.shape == (2, 4)
    assert restored.tolist() == [[0, 1, 0, 0], [0, 0, 1, 0]]


def test_fractional_bbox_expands_at_most_one_pixel_for_mask_quantization():
    binary = shaped_mask([(1, 1), (2, 2)]).astype(np.uint8)
    assert align_bbox_to_mask([1.2, 1.1, 2.8, 2.9], binary) == [1, 1, 3, 3]
    with pytest.raises(RuntimeError, match="exceeds"):
        align_bbox_to_mask([3.1, 3.1, 4, 4], binary)


def test_empty_result_does_not_require_masks(monkeypatch, tmp_path):
    result = FakeResult(FakeBoxes(np.empty((0, 4)), [], []), None)
    runtime, _ = make_runtime(monkeypatch, tmp_path, result)
    assert runtime.predict(np.zeros((6, 8, 3), np.uint8)) == []
    assert runtime.last_diagnostics["raw_mask_shape"] is None


@pytest.mark.parametrize("masks,message", [
    (None, "no masks"),
    (FakeMasks(np.zeros((1, 6, 8))), "box count 2 does not match mask count 1"),
])
def test_missing_or_mismatched_masks_are_rejected(monkeypatch, tmp_path, masks, message):
    boxes = FakeBoxes([[1, 1, 2, 2], [3, 3, 4, 4]], [.8, .7], [47, 46])
    runtime, _ = make_runtime(monkeypatch, tmp_path, FakeResult(boxes, masks))
    with pytest.raises(RuntimeError, match=message):
        runtime.predict(np.zeros((6, 8, 3), np.uint8))


def test_result_shape_must_match_input(monkeypatch, tmp_path):
    result = FakeResult(FakeBoxes(np.empty((0, 4)), [], []), None, orig_shape=(5, 8))
    runtime, _ = make_runtime(monkeypatch, tmp_path, result)
    with pytest.raises(RuntimeError, match="does not match input"):
        runtime.predict(np.zeros((6, 8, 3), np.uint8))


def test_non_finite_mask_and_unsupported_output_are_rejected(monkeypatch, tmp_path):
    bad = shaped_mask([(1, 1)])
    bad[2, 2] = np.nan
    result = FakeResult(FakeBoxes([[1, 1, 3, 3]], [.8], [47]), FakeMasks([bad]))
    runtime, _ = make_runtime(monkeypatch, tmp_path, result)
    with pytest.raises(RuntimeError, match="non-finite"):
        runtime.predict(np.zeros((6, 8, 3), np.uint8))
