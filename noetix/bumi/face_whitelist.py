"""Local, opt-in face whitelist recognition for the Bumi RGB camera.

The card deliberately performs recognition only.  It never stores visitor
frames, speaks, moves the robot, or decides whether somebody may enter.
Enrollment photos live on the robot host under the configured data directory
and are not copied into the driver image.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any


_IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp"})


class FaceWhitelistError(RuntimeError):
    """A user-actionable enrollment or inference error."""


class OpenCvLbphBackend:
    """Haar face detection followed by an OpenCV LBPH identity model."""

    def __init__(
        self,
        whitelist_dir: Path,
        distance_threshold: float,
        min_face_size: int,
        max_images_per_person: int,
    ) -> None:
        self._whitelist_dir = whitelist_dir
        self._threshold = distance_threshold
        self._min_face_size = min_face_size
        self._max_images_per_person = max_images_per_person
        self._cv2 = None
        self._np = None
        self._detector = None
        self._recognizer = None
        self._identities: dict[int, dict[str, str]] = {}
        self._image_count = 0
        self._warnings: list[str] = []

    @property
    def ready(self) -> bool:
        return self._recognizer is not None and bool(self._identities)

    def summary(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "identity_count": len(self._identities),
            "enrollment_image_count": self._image_count,
            "warnings": list(self._warnings),
        }

    def _load_runtime(self) -> None:
        if self._cv2 is not None:
            return
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise FaceWhitelistError(
                "OpenCV runtime unavailable; rebuild the Bumi driver image"
            ) from exc
        if not hasattr(cv2, "face") or not hasattr(
            cv2.face, "LBPHFaceRecognizer_create"
        ):
            raise FaceWhitelistError(
                "OpenCV face module unavailable; install "
                "opencv-contrib-python-headless and rebuild the image"
            )
        cascade_path = Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"
        detector = cv2.CascadeClassifier(str(cascade_path))
        if detector.empty():
            raise FaceWhitelistError(f"cannot load face detector: {cascade_path}")
        self._cv2 = cv2
        self._np = np
        self._detector = detector

    def _decode(self, data: bytes):
        encoded = self._np.frombuffer(data, dtype=self._np.uint8)
        image = self._cv2.imdecode(encoded, self._cv2.IMREAD_COLOR)
        if image is None:
            raise FaceWhitelistError("cannot decode JPEG image")
        return image

    def _read_image(self, path: Path):
        try:
            return self._decode(path.read_bytes())
        except OSError as exc:
            raise FaceWhitelistError(f"cannot read enrollment image {path.name}") from exc

    def _face_crops(self, image) -> list[Any]:
        gray = self._cv2.cvtColor(image, self._cv2.COLOR_BGR2GRAY)
        gray = self._cv2.equalizeHist(gray)
        boxes = self._detector.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(self._min_face_size, self._min_face_size),
        )
        crops = []
        for x, y, width, height in boxes:
            crop = gray[y:y + height, x:x + width]
            crops.append(self._cv2.resize(crop, (160, 160)))
        return crops

    @staticmethod
    def _identity(person_dir: Path) -> dict[str, str]:
        person_id = person_dir.name.strip()
        display_name = person_id
        metadata_path = person_dir / "person.json"
        if metadata_path.is_file():
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise FaceWhitelistError(
                    f"invalid metadata for identity {person_id}"
                ) from exc
            configured_name = metadata.get("display_name", person_id)
            if not isinstance(configured_name, str) or not configured_name.strip():
                raise FaceWhitelistError(
                    f"display_name for identity {person_id} must be a non-empty string"
                )
            display_name = configured_name.strip()
        return {"person_id": person_id, "display_name": display_name}

    def reload(self) -> dict[str, Any]:
        self._load_runtime()
        self._recognizer = None
        self._identities = {}
        self._image_count = 0
        self._warnings = []
        self._whitelist_dir.mkdir(parents=True, exist_ok=True)

        faces = []
        labels = []
        for person_dir in sorted(self._whitelist_dir.iterdir()):
            if not person_dir.is_dir() or person_dir.name.startswith("."):
                continue
            identity = self._identity(person_dir)
            label = len(self._identities)
            accepted = 0
            images = [
                path for path in sorted(person_dir.iterdir())
                if path.is_file() and path.suffix.lower() in _IMAGE_SUFFIXES
            ][:self._max_images_per_person]
            for image_path in images:
                crops = self._face_crops(self._read_image(image_path))
                if len(crops) != 1:
                    self._warnings.append(
                        f"{identity['person_id']}/{image_path.name}: "
                        f"expected one face, found {len(crops)}"
                    )
                    continue
                faces.append(crops[0])
                labels.append(label)
                accepted += 1
            if accepted:
                self._identities[label] = identity
                self._image_count += accepted

        if not faces:
            return self.summary()
        recognizer = self._cv2.face.LBPHFaceRecognizer_create()
        recognizer.train(faces, self._np.asarray(labels, dtype=self._np.int32))
        self._recognizer = recognizer
        return self.summary()

    def recognize(self, jpeg: bytes) -> dict[str, Any]:
        if not self.ready:
            raise FaceWhitelistError(
                "whitelist has no usable enrollment photos; add one-person photos and reload"
            )
        faces = self._face_crops(self._decode(jpeg))
        if not faces:
            return {"result": "no_face", "face_count": 0}
        if len(faces) > 1:
            return {"result": "multiple_faces", "face_count": len(faces)}

        label, distance = self._recognizer.predict(faces[0])
        distance = float(distance)
        if not math.isfinite(distance):
            raise FaceWhitelistError("recognizer returned a non-finite distance")
        base = {
            "face_count": 1,
            "distance": round(distance, 3),
            "distance_threshold": self._threshold,
        }
        identity = self._identities.get(int(label))
        if identity is None or distance > self._threshold:
            return {"result": "unknown", **base}
        return {"result": "matched", **identity, **base}


class FaceWhitelistPlugin:
    """Read one recent Bumi camera frame and compare it with local enrollment."""

    PREFIX = "face_whitelist"

    def __init__(self, plugin_config: dict, camera_plugin, backend=None) -> None:
        config = plugin_config or {}
        self._whitelist_dir = Path(str(config.get(
            "whitelist_dir", "/opt/phanthy-motus/data/face_whitelist"
        ))).expanduser()
        threshold = float(config.get("distance_threshold", 60.0))
        min_face_size = int(config.get("min_face_size", 80))
        max_images = int(config.get("max_images_per_person", 20))
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError("distance_threshold must be a positive finite number")
        if not 40 <= min_face_size <= 400:
            raise ValueError("min_face_size must be between 40 and 400")
        if not 1 <= max_images <= 100:
            raise ValueError("max_images_per_person must be between 1 and 100")

        self._camera = camera_plugin
        self._backend = backend or OpenCvLbphBackend(
            self._whitelist_dir, threshold, min_face_size, max_images
        )
        self._started = False
        self._last_error: str | None = None
        self._reload_error: str | None = None
        self._lock = threading.Lock()

    def get_tool(self) -> dict:
        return {
            "name": "face_whitelist",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Compare one recent Bumi RGB frame with a private local face "
                "whitelist. It does not store visitor images or control hardware."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "recognize", "reload", "info"],
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {
                        "params": [],
                        "description": "Load the private whitelist and make recognition ready.",
                    },
                    "stop": {
                        "params": [],
                        "description": "Transition the card to idle without stopping the camera.",
                    },
                    "recognize": {
                        "params": [],
                        "description": "Recognize exactly one face in the latest RGB frame.",
                    },
                    "reload": {
                        "params": [],
                        "description": "Reload enrollment photos after the whitelist changes.",
                    },
                    "info": {
                        "params": [],
                        "description": "Show readiness and enrollment counts without exposing images.",
                    },
                },
            },
        }

    def start(self) -> dict:
        self._started = True
        return self._reload()

    def stop(self) -> dict:
        with self._lock:
            self._started = False
        return {"state": "idle"}

    def _reload(self) -> dict:
        with self._lock:
            try:
                summary = self._backend.reload()
                self._last_error = None
                self._reload_error = None
            except Exception as exc:
                self._last_error = str(exc)
                self._reload_error = self._last_error
                return {"state": "error", "error": self._last_error}
        state = "ready" if summary.get("ready") else "unconfigured"
        return {"state": state, **summary}

    def _info(self) -> dict:
        with self._lock:
            summary = self._backend.summary()
            if not self._started:
                state = "idle"
            elif self._last_error:
                state = "error"
            elif summary.get("ready"):
                state = "ready"
            else:
                state = "unconfigured"
            result = {"state": state, **summary}
            if self._last_error:
                result["error"] = self._last_error
            return result

    def is_ready(self) -> bool:
        """Return readiness under the same lock used by model reload/predict."""
        with self._lock:
            return self._started and self._reload_error is None and self._backend.ready

    def recognize_jpeg(self, jpeg: bytes) -> dict:
        """Recognize one supplied frame without opening another camera stream."""
        with self._lock:
            if not self._started:
                return {"state": "error", "error": "face_whitelist is not started"}
            if self._reload_error:
                return {"state": "error", "error": self._reload_error}
            if not self._backend.ready:
                return {"state": "error", "error": "face whitelist is unconfigured"}
            try:
                result = self._backend.recognize(jpeg)
                self._last_error = None
                return {"state": "completed", **result}
            except Exception as exc:
                self._last_error = str(exc)
                return {"state": "error", "error": self._last_error}

    def _recognize(self) -> dict:
        if self._camera is None or not self._camera.is_running():
            return {"state": "error", "error": "Bumi camera worker is unavailable"}
        try:
            frame, _ = self._camera.wait_for_color_frame(timeout_s=5.0)
            if not isinstance(frame, dict) or not frame.get("data"):
                raise FaceWhitelistError("no RGB frame has arrived")
            timestamp_ms = float(frame.get("timestamp_ms", 0) or 0)
            if timestamp_ms and time.time() - timestamp_ms / 1000 > 3.0:
                raise FaceWhitelistError("latest RGB frame is stale")
            return self.recognize_jpeg(bytes(frame["data"]))
        except Exception as exc:
            self._last_error = str(exc)
            return {"state": "error", "error": self._last_error}

    def dispatch(self, action: str, args: dict) -> dict | None:
        del args
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        if action == "reload":
            return self._reload()
        if action == "recognize":
            return self._recognize()
        return None
