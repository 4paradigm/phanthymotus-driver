"""YOLOv11-pose inference wrapper.

`ultralytics` (and its torch/torchvision dependencies) is imported lazily so
the card schema and the pure fall-detection/speech modules can be imported and
tested on a laptop without torch installed — the same lazy-import discipline
`common/vendor_runtime.py` uses for ROS.

The official pose weights (`yolo11n-pose.pt`, `yolo11s-pose.pt`, ...) are used
directly — no training or fine-tuning. ultralytics downloads the file on first
use and caches it under ``~/.config/Ultralytics``; set ``model_path`` to an
absolute path for a pre-downloaded copy on an offline robot.
"""

from __future__ import annotations

COCO_KEYPOINT_COUNT = 17


class PoseEngine:
    def __init__(self, model_path="yolo11n-pose.pt", device=None, conf=0.5,
                 imgsz=640):
        self.model_path = model_path
        # None -> ultralytics auto-selects (cuda if available, else cpu).
        self.device = device
        self.conf = conf
        self.imgsz = imgsz
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
        """Run pose detection on a BGR image (numpy array).

        Returns a list of persons, each::

            {
              "keypoints": [[x, y, conf], ... 17 ...],
              "bbox": [x1, y1, x2, y2],
              "confidence": float,
            }
        """
        if self._model is None:
            self.load()

        results = self._model.predict(
            bgr, imgsz=self.imgsz, conf=self.conf, device=self.device,
            verbose=False, stream=False,
        )

        persons = []
        for r in results:
            boxes = r.boxes
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.cpu().tolist()
            box_conf = boxes.conf.cpu().tolist()

            kpts = r.keypoints
            kp_xy = None
            kp_conf = None
            if kpts is not None:
                kp_xy = kpts.xy.cpu().tolist() if getattr(kpts, "xy", None) is not None else None
                kp_conf = kpts.conf.cpu().tolist() if getattr(kpts, "conf", None) is not None else None

            for i in range(len(xyxy)):
                keypoints = []
                for j in range(COCO_KEYPOINT_COUNT):
                    x, y = (kp_xy[i][j][0], kp_xy[i][j][1]) if kp_xy is not None else (0.0, 0.0)
                    c = kp_conf[i][j] if kp_conf is not None else 0.0
                    if isinstance(c, (list, tuple)):
                        c = c[0]  # some versions add a visibility dimension
                    keypoints.append([float(x), float(y), float(c)])
                persons.append({
                    "keypoints": keypoints,
                    "bbox": [float(v) for v in xyxy[i]],
                    "confidence": float(box_conf[i]),
                })
        return persons
