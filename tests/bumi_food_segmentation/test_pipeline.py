from noetix.bumi.food_segmentation_backend import FoodSegmentationPipeline
from noetix.bumi.food_segmentation_contract import COCO_FOOD_CLASSES, RawInstance


NAMES = {index: f"non-food-{index}" for index in range(80)}
NAMES.update(COCO_FOOD_CLASSES)


def mask(points):
    output = [[0] * 8 for _ in range(6)]
    for x, y in points:
        output[y][x] = 1
    return output


APPLE_L = mask([(1, 1), (1, 2), (1, 3), (2, 3), (3, 3)])
APPLE_DOT = mask([(6, 4)])
BANANA = mask([(3, 1), (4, 1), (4, 2), (5, 2)])
PERSON = mask([(0, 0), (0, 1)])


class FakeRuntime:
    """Deterministic fake: no ROS, CUDA, model load, filesystem or network."""

    names = NAMES

    def __init__(self, scenario="mixed"):
        self.scenario = scenario
        self.loaded = False

    def load(self, model_path, **options):
        self.loaded = True
        self.model_path = model_path
        self.options = options

    def predict(self, image):
        if self.scenario == "empty":
            return []
        if self.scenario == "broken":
            return [RawInstance(47, .9, [1, 1, 4, 4], [[1, 0]])]
        return [
            RawInstance(47, .95, [1, 1, 4, 4], APPLE_L),
            RawInstance(47, .75, [6, 4, 7, 5], APPLE_DOT),
            RawInstance(46, .90, [3, 1, 6, 3], BANANA),
            RawInstance(0, .99, [0, 0, 1, 2], PERSON),
        ]


def test_fake_pipeline_filters_non_food_and_preserves_same_class_instances():
    runtime = FakeRuntime()
    pipeline = FoodSegmentationPipeline(runtime, [8, 6])
    result = pipeline.process(object())
    assert runtime.loaded is False
    assert [item["class_name"] for item in result["instances"]] == [
        "apple", "apple", "banana",
    ]
    assert [item["mask_area_px"] for item in result["instances"]] == [5, 1, 4]
    assert result["errors"] == []


def test_fake_runtime_empty_detection_is_stable():
    result = FoodSegmentationPipeline(FakeRuntime("empty"), [8, 6]).process(None)
    assert result["instances"] == [] and result["errors"] == []


def test_broken_mask_is_reported_and_next_call_still_works():
    runtime = FakeRuntime("broken")
    pipeline = FoodSegmentationPipeline(runtime, [8, 6])
    broken = pipeline.process(None)
    assert broken["instances"] == []
    assert len(broken["errors"]) == 1 and "height" in broken["errors"][0]["error"]
    runtime.scenario = "mixed"
    assert len(pipeline.process(None)["instances"]) == 3


def test_model_loading_is_explicit_and_path_is_caller_supplied():
    runtime = FakeRuntime("empty")
    assert runtime.loaded is False
    runtime.load("/models/yolo11n-seg.pt", device="cpu")
    assert runtime.loaded is True
    assert runtime.model_path == "/models/yolo11n-seg.pt"
    assert runtime.options == {"device": "cpu"}
