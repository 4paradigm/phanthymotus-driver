"""Bounded, asynchronous reuse of the same-robot OCR orientation action.

No model, files, proxy, remote image URL or recognition request is involved.
Older OCR images are reported as unsupported, never guessed from recognized text.
"""
from __future__ import annotations

import base64
import json
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np

SCHEMA = "motus.document-orientation/1"


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        raise urllib.error.URLError("OCR redirects are not permitted")


def fingerprint(image):
    gray = cv2.cvtColor(cv2.resize(image, (32, 32)), cv2.COLOR_BGR2GRAY).astype(np.float32)
    return np.clip((gray - gray.mean()) / max(5.0, float(gray.std())), -3, 3)


def same_page(first, second):
    return first is not None and float(np.mean(np.abs(first - second))) < 0.35


class OCROrientation:
    def __init__(self, port=15720, interval=1.0, timeout=1.5):
        self._port, self._interval, self._timeout = port, interval, timeout
        self._lock = threading.Lock()
        self._wake, self._cancel = threading.Event(), threading.Event()
        self._thread = None
        self._epoch = 0
        self._reference = self._pending = None
        self._next_request = 0.0
        self._backoff_until = 0.0
        self._result = self._unknown("not_checked")

    @staticmethod
    def _unknown(reason):
        return {"status": "unknown", "reason": reason,
                "rotation_cw_degrees": None, "support": 0.0}

    def start(self):
        if self._thread is not None:
            return
        self._cancel.clear()
        self._thread = threading.Thread(target=self._run, name="planar_ocr_direction", daemon=True)
        self._thread.start()

    def stop(self):
        self._cancel.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("OCR direction request did not stop")
            self._thread = None
        self.invalidate()

    def invalidate(self):
        with self._lock:
            self._epoch += 1
            self._reference = self._pending = None
            self._result = self._unknown("not_checked")
            self._next_request = 0.0

    def configure(self, port):
        with self._lock:
            self._port = port
            self._epoch += 1
            self._reference = self._pending = None
            self._result = self._unknown("not_checked")
            self._next_request = self._backoff_until = 0.0

    def consider(self, image, sequence):
        signature = fingerprint(image)
        now = time.monotonic()
        with self._lock:
            if self._cancel.is_set():
                return self._unknown("stopped")
            if not same_page(self._reference, signature):
                self._epoch += 1
                self._result = self._unknown("page_changed")
            self._reference = signature
            result = dict(self._result)
            if "received_monotonic_s" in result:
                result["age_s"] = max(0.0, now - result["received_monotonic_s"])
                if result["age_s"] > 3:
                    result = self._unknown("orientation_expired")
            if now >= max(self._next_request, self._backoff_until):
                height, width = image.shape[:2]
                scale = min(1.0, 640 / max(width, height))
                small = cv2.resize(image, (round(width * scale), round(height * scale)))
                ok, data = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok and len(data) <= 2 * 1024 * 1024:
                    # One replaceable pending request, one bounded in-flight RPC.
                    self._pending = (self._epoch, signature, int(sequence), data.tobytes())
                    self._next_request = now + self._interval
                    self._wake.set()
            return result

    def _request(self, data):
        body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                           "params": {"name": "ocr", "arguments": {
                               "action": "classify_orientation",
                               "image_base64": base64.b64encode(data).decode("ascii")}}}).encode()
        request = urllib.request.Request(f"http://127.0.0.1:{self._port}/mcp", data=body,
                                         headers={"Content-Type": "application/json"})
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        try:
            with opener.open(request, timeout=self._timeout) as response:
                raw = response.read(65537)
            if len(raw) > 65536:
                return self._unknown("invalid_ocr_response")
            rpc = json.loads(raw)
            if "error" in rpc:
                return self._unknown("unsupported_ocr_api")
            content = rpc["result"]["content"]
            report = json.loads(next(item["text"] for item in content if item.get("type") == "text"))
            if not report.get("ok"):
                reason = report.get("reason")
                allowed = {"adapter_loading", "adapter_error", "adapter_idle", "adapter_busy", "orientation_failed"}
                return self._unknown(reason if reason in allowed else "invalid_ocr_response")
            if report.get("schema") != SCHEMA:
                return self._unknown("unsupported_ocr_api")
            support = report.get("support", 0)
            angle = report.get("rotation_cw_degrees")
            if (report.get("status") == "ready" and type(angle) is int
                    and angle in (0, 90, 180, 270) and type(support) in (int, float)
                    and np.isfinite(support) and 0.8 <= support <= 1):
                return {"status": "ready", "reason": None, "rotation_cw_degrees": angle,
                        "support": float(support), "source": "shared_ocr_det_cls"}
            if report.get("status") == "unknown":
                reason = report.get("reason")
                allowed = {"no_text", "angle_classifier_disabled", "insufficient_directional_text",
                           "ambiguous_text_direction", "invalid_classifier_result"}
                if reason in allowed:
                    return self._unknown(reason)
            return self._unknown("uncertain_text_direction")
        except (OSError, ValueError, KeyError, TypeError, StopIteration):
            return self._unknown("ocr_unavailable")

    def _run(self):
        while not self._cancel.is_set():
            self._wake.wait(0.2)
            self._wake.clear()
            with self._lock:
                job, self._pending = self._pending, None
            if job is None or self._cancel.is_set():
                continue
            epoch, signature, sequence, data = job
            try:
                result = self._request(data)
            except Exception:
                result = self._unknown("ocr_unavailable")
            with self._lock:
                if (not self._cancel.is_set() and self._epoch == epoch
                        and same_page(self._reference, signature)):
                    result.update(based_on_frame_sequence=sequence,
                                  received_monotonic_s=time.monotonic())
                    if not (result.get("reason") == "adapter_busy"
                            and self._result.get("status") == "ready"
                            and time.monotonic() - self._result.get("received_monotonic_s", 0) <= 3):
                        self._result = result
                    if result.get("reason") in ("unsupported_ocr_api", "ocr_unavailable"):
                        self._backoff_until = time.monotonic() + 10
                        self._pending = None
