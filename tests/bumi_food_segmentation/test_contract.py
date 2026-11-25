import math

import pytest

from noetix.bumi.food_segmentation_contract import (
    COCO_FOOD_CLASSES,
    RLE_ENCODING,
    RawInstance,
    SegmentationLimitError,
    binary_mask,
    build_instance,
    build_result,
    decode_binary_rle,
    encode_binary_rle,
    mask_bounds,
    resolve_food_class_ids,
    validate_bbox,
)


NAMES = {index: f"other-{index}" for index in range(80)}
NAMES.update(COCO_FOOD_CLASSES)


def l_mask():
    return [
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0],
    ]


def test_verified_coco_food_names_and_ids():
    assert COCO_FOOD_CLASSES == {
        46: "banana", 47: "apple", 48: "sandwich", 49: "orange",
        50: "broccoli", 51: "carrot", 52: "hot dog", 53: "pizza",
        54: "donut", 55: "cake",
    }
    assert resolve_food_class_ids(NAMES) == COCO_FOOD_CLASSES
    wrong = dict(NAMES)
    wrong[47] = "pear"
    with pytest.raises(ValueError, match="missing food classes"):
        resolve_food_class_ids(wrong)


def test_non_rectangular_mask_area_ratio_rle_and_bounds():
    result = build_instance(RawInstance(47, 0.9, [1, 0, 4, 3], l_mask()),
                            NAMES, [6, 4])
    assert result["mask_area_px"] == 5
    assert result["mask_area_ratio"] == pytest.approx(5 / 24)
    assert result["mask"]["encoding"] == RLE_ENCODING
    assert decode_binary_rle(result["mask"]) == l_mask()
    assert mask_bounds(l_mask()) == [1, 0, 4, 3]
    # A bbox fill would have area 9, proving this is the actual L-shaped mask.
    assert result["mask_area_px"] != 3 * 3


@pytest.mark.parametrize("mask", [
    [[0, 0, 0], [0, 0, 0]],
    [[1, 1, 1], [1, 1, 1]],
    [[1, 0, 1], [0, 1, 0]],
])
def test_row_major_rle_round_trip(mask):
    encoded = encode_binary_rle(mask)
    assert encoded["size"] == [2, 3]
    assert decode_binary_rle(encoded) == mask


def test_rle_rejects_wrong_size_runs_and_encoding():
    with pytest.raises(ValueError, match="run lengths"):
        decode_binary_rle({"encoding": RLE_ENCODING, "size": [2, 3], "counts": [5]})
    with pytest.raises(ValueError, match="encoding"):
        decode_binary_rle({"encoding": "coco-rle", "size": [1, 1], "counts": [1]})
    with pytest.raises(ValueError, match="initial zero run"):
        decode_binary_rle({"encoding": RLE_ENCODING, "size": [1, 1], "counts": [0, 0, 1]})


def test_threshold_filter_non_food_filter_and_empty_result():
    low = RawInstance(47, 0.2, [1, 0, 4, 3], l_mask())
    person = RawInstance(0, 0.99, [1, 0, 4, 3], l_mask())
    assert build_instance(low, NAMES, [6, 4], confidence_threshold=0.25) is None
    assert build_instance(person, NAMES, [6, 4]) is None
    assert build_result([], NAMES, [6, 4]) == {
        "image_size": [6, 4], "instances": [], "errors": [],
    }


def test_mask_binarization_and_size_validation():
    values = [[0.1, 0.5], [0.9, 0.49]]
    assert binary_mask(values, [2, 2]) == [[0, 1], [1, 0]]
    with pytest.raises(ValueError, match="height"):
        binary_mask([[1, 0]], [2, 2])
    with pytest.raises(ValueError, match="width"):
        binary_mask([[1], [0]], [2, 2])
    with pytest.raises(ValueError, match="finite"):
        binary_mask([[0, math.nan], [0, 0]], [2, 2])


@pytest.mark.parametrize("bbox", [
    [0, 0, 0, 1], [-1, 0, 1, 1], [0, 0, 7, 1], [0, 0, math.inf, 1], [1, 2, 3],
])
def test_invalid_bbox_is_rejected(bbox):
    with pytest.raises(ValueError):
        validate_bbox(bbox, [6, 4])


def test_bbox_must_enclose_real_mask_not_a_synthetic_rectangle():
    with pytest.raises(ValueError, match="does not enclose"):
        build_instance(RawInstance(47, 0.9, [1, 0, 2, 2], l_mask()), NAMES, [6, 4])


def test_malformed_instances_are_isolated_without_losing_valid_ones():
    good = RawInstance(47, 0.9, [1, 0, 4, 3], l_mask())
    broken = RawInstance(46, 0.9, [0, 0, 2, 2], [[1]])
    result = build_result([good, broken, object()], NAMES, [6, 4])
    assert [item["class_name"] for item in result["instances"]] == ["apple"]
    assert [error["index"] for error in result["errors"]] == [1, 2]


def test_rle_budget_stops_before_scanning_full_mask():
    class AlternatingRow:
        def __init__(self, owner, y):
            self.owner = owner
            self.y = y

        def __len__(self):
            return self.owner.width

        def __getitem__(self, x):
            self.owner.accesses += 1
            return (self.y * self.owner.width + x) % 2

    class AlternatingMask:
        height = 100
        width = 100

        def __init__(self):
            self.accesses = 0

        def __len__(self):
            return self.height

        def __getitem__(self, y):
            return AlternatingRow(self, y)

    mask = AlternatingMask()
    with pytest.raises(SegmentationLimitError, match="run-count limit"):
        build_result(
            [RawInstance(47, 0.9, [0, 0, 100, 100], mask)],
            NAMES, [100, 100], max_total_rle_runs=5)
    # A post-materialization check would touch all 10,000 pixels. The streaming
    # encoder stops as soon as appending the sixth run would exceed the budget.
    assert mask.accesses == 7
