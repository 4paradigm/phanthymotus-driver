"""Robot-independent JPEG QR decoding and bounded content deduplication."""
from __future__ import annotations

class QrDecoder:
    """Decode JPEG frames. Pixel coordinates refer to the original image."""

    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.detector = cv2.QRCodeDetector()

    def decode(self, jpeg: bytes) -> list[dict]:
        import numpy as np
        if not jpeg or len(jpeg) > 5_000_000:
            raise ValueError("empty or oversized JPEG")
        image = self.cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), self.cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("invalid JPEG")
        ok, texts, points, _ = self.detector.detectAndDecodeMulti(image)
        # The single-code path can succeed where multi-code detection fails.
        if not ok or not any(texts):
            text, corners, _ = self.detector.detectAndDecode(image)
            texts, points = ([text], corners) if text else ([], None)
        if points is None:
            return []
        codes = []
        for text, corners in zip(texts, points):
            if not text:
                continue
            codes.append({
                "text": text,
                "corners_px": [[float(x), float(y)] for x, y in corners],
                "image_width": int(image.shape[1]),
                "image_height": int(image.shape[0]),
            })
        return codes


class QrTracker:
    """Emit once per content until absent for rearm_s. Bounded memory."""

    def __init__(self, rearm_s: float):
        self.rearm_s = rearm_s
        self.last_seen: dict[str, float] = {}
        self.sequence = 0

    def update(self, codes: list[dict], now: float) -> list[dict]:
        self.last_seen = {k: t for k, t in self.last_seen.items()
                          if now - t < self.rearm_s}
        events = []
        for code in codes:
            text = code["text"]
            if text not in self.last_seen:
                self.sequence += 1
                events.append({"sequence": self.sequence, "text": text})
            self.last_seen[text] = now
        # Detection input is physically bounded by image size; also bound history.
        if len(self.last_seen) > 256:
            self.last_seen = dict(sorted(self.last_seen.items(),
                                         key=lambda item: item[1])[-256:])
        return events
