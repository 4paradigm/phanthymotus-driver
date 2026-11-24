"""YOLOv8n inference wrapper — traffic-light detection.

`ultralytics` (and its torch/torchvision dependencies) is imported lazily so
the card schema and the pure classifier/speech modules can be imported and
tested on a laptop without torch installed — the same lazy-import discipline
`common/vendor_runtime.py` uses for ROS.

The official COCO weights (`yolov8n.pt`) are used directly — no training or
fine-tuning. COCO's "traffic light" class (index 9) is all we need for
localization: the color (red/yellow/green) is *not* a COCO class, so
`light_classifier` reads it from the box's pixels. ultralytics downloads the
weight on first use and caches it under ``~/.config/Ultralytics``; set
`model_path` to an absolute path for a pre-downloaded copy on an offline robot.
"""

from __future__ import annotations

# COCO-80 index of the "traffic light" class (yolov8n.pt).
TRAFFIC_LIGHT_CLASS_ID = 9


class DetectionEngine:
    def __init__(self, model_path="yolov8n.pt", device=None, conf=0.35,
                 imgsz=640, traffic_light_class_id=TRAFFIC_LIGHT_CLASS_ID):
        self.model_path = model_path
        # None -> ultralytics auto-selects (cuda if available, else cpu).
        self.device = device
        self.conf = conf
        self.imgsz = imgsz
        self.traffic_light_class_id = traffic_light_class_id
        self._model = None
        self._load_error = None

    @property
    def loaded(self) -> bool:
        return self._model is not None

    def load(self) -> None:
        """Load the model (blocking; call once, or rely on detect()'s lazy load)."""
        if self._model is not None:
            return
        from ultralytics import YOLO
        self._model = YOLO(self.model_path)
        self._load_error = None

    def detect(self, bgr):
        """Run traffic-light detection on a BGR image (numpy array).

        Returns a list of boxes, each::

            {"bbox": [x1, y1, x2, y2], "confidence": float}

        in original image pixel coordinates (ultralytics rescales back).
        """
        if self._model is None:
            self.load()

        results = self._model.predict(
            bgr, imgsz=self.imgsz, conf=self.conf, device=self.device,
            classes=[self.traffic_light_class_id], verbose=False, stream=False,
        )

        boxes = []
        for r in results:
            r_boxes = r.boxes
            if r_boxes is None or len(r_boxes) == 0:
                continue
            xyxy = r_boxes.xyxy.cpu().tolist()
            conf = r_boxes.conf.cpu().tolist()
            for i in range(len(xyxy)):
                boxes.append({
                    "bbox": [float(v) for v in xyxy[i]],
                    "confidence": float(conf[i]),
                })
        return boxes
