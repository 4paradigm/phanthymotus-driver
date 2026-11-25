"""Local-only visual verifier for the Food Segmentation adapter and pipeline.

The caller supplies local model/images and an output directory.  This script
does not download anything and does not use Ultralytics' built-in ``plot()``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import sys
import time

import cv2
import numpy as np
import torch
import ultralytics

from noetix.bumi.food_segmentation_backend import FoodSegmentationPipeline
from noetix.bumi.food_segmentation_contract import decode_binary_rle
from noetix.bumi.food_segmentation_ultralytics import UltralyticsSegmentationRuntime


COLORS = [
    (40, 180, 240), (240, 120, 40), (80, 210, 100), (210, 80, 180),
    (180, 210, 40), (80, 100, 230),
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def draw(image: np.ndarray, result: dict, output: Path) -> None:
    overlay = image.copy()
    boxed = image.copy()
    for index, instance in enumerate(result["instances"]):
        binary = np.asarray(decode_binary_rle(instance["mask"]), dtype=np.uint8)
        color = COLORS[index % len(COLORS)]
        color_layer = np.zeros_like(image)
        color_layer[:] = color
        selected = binary.astype(bool)
        overlay[selected] = (0.55 * overlay[selected] +
                             0.45 * color_layer[selected]).astype(np.uint8)
        cv2.imwrite(str(output / f"mask_{index:02d}_{instance['class_name'].replace(' ', '_')}.png"),
                    binary * 255)

        x1, y1, x2, y2 = instance["bbox_xyxy"]
        p1 = (max(0, int(np.floor(x1))), max(0, int(np.floor(y1))))
        p2 = (min(image.shape[1] - 1, int(np.ceil(x2)) - 1),
              min(image.shape[0] - 1, int(np.ceil(y2)) - 1))
        cv2.rectangle(boxed, p1, p2, color, 2)
        label = f"{instance['class_name']} {instance['confidence']:.3f}"
        cv2.putText(boxed, label, (p1[0], max(15, p1[1] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
    cv2.imwrite(str(output / "mask_overlay.jpg"), overlay)
    cv2.imwrite(str(output / "boxes_and_labels.jpg"), boxed)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("images", nargs="+", type=Path)
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f"model does not exist: {args.model}")
    args.output.mkdir(parents=True, exist_ok=True)

    runtime = UltralyticsSegmentationRuntime()
    runtime.load(str(args.model), device="cpu", imgsz=640, conf=args.confidence)
    environment = {
        "python": platform.python_version(),
        "ultralytics": ultralytics.__version__,
        "torch": torch.__version__,
        "numpy": np.__version__,
        "opencv": cv2.__version__,
        "cuda_available": torch.cuda.is_available(),
        "model_path": str(args.model.resolve()),
        "model_size_bytes": args.model.stat().st_size,
        "model_sha256": sha256(args.model),
        "model_task": runtime._model.task,
        "food_names": {str(i): runtime.names[i] for i in range(46, 56)},
    }
    records = []
    for source in args.images:
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"could not read image: {source}")
        height, width = image.shape[:2]
        pipeline = FoodSegmentationPipeline(
            runtime, [width, height], confidence_threshold=args.confidence,
        )
        started = time.perf_counter()
        result = pipeline.process(image)
        elapsed_ms = (time.perf_counter() - started) * 1000
        destination = args.output / source.stem
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination / f"original{source.suffix.lower()}")
        draw(image, result, destination)

        instances = []
        food_diagnostics = [
            item for item in runtime.last_diagnostics.get("detections", [])
            if 46 <= item["class_id"] <= 55 and
            item["confidence"] >= args.confidence
        ]
        if len(food_diagnostics) != len(result["instances"]):
            raise AssertionError("adapter diagnostics do not match filtered instances")
        for instance, diagnostic in zip(result["instances"], food_diagnostics):
            decoded = decode_binary_rle(instance["mask"])
            decoded_area = sum(map(sum, decoded))
            if decoded_area != instance["mask_area_px"]:
                raise AssertionError("RLE area differs from the pipeline mask area")
            item = dict(instance)
            item["rle_round_trip_area_px"] = decoded_area
            item["bbox_adjustment_xyxy"] = diagnostic["bbox_adjustment_xyxy"]
            instances.append(item)
        record = {
            "source": str(source.resolve()),
            "image_size": [width, height],
            "inference_and_contract_ms": elapsed_ms,
            "all_detection_count": runtime.last_diagnostics["detection_count"],
            "food_instance_count": len(result["instances"]),
            "adapter_diagnostics": runtime.last_diagnostics,
            "instances": instances,
            "errors": result["errors"],
        }
        (destination / "result.json").write_text(
            json.dumps({"environment": environment, "result": record}, indent=2),
            encoding="utf-8",
        )
        records.append(record)

    summary = {"environment": environment, "images": records}
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
