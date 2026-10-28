"""Deterministic before/after image comparison for the Unitree G1.

The card compares two RGB frames from the same viewpoint and writes an
evidence bundle: the two source images, a before/after/diff triptych, a
binary mask, and a JSON result.  It does not recognise objects; the verdict
is geometric and can be reproduced without a model or network access.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


CARD = "image_diff"
DEFAULT_OUTPUT_DIR = "/opt/phanthy-motus/data/image_diff"
DEFAULT_SOURCE_ROOT = "/opt/phanthy-motus/data"
MAX_INPUT_BYTES = 32 * 1024 * 1024
MAX_INPUT_PIXELS = 24_000_000
WORK_LONG_EDGE = 960
MAX_REGIONS = 8
MAX_FRAME_AGE_S = 3.0

_BASELINE_ID_RE = re.compile(r"^b_[0-9]{8}_[0-9]{6}_[0-9a-f]{4}$")
_RUN_ID_RE = re.compile(r"^r_[0-9]{8}_[0-9]{6}_[0-9a-f]{4}$")

_SENSITIVITY = {
    # threshold is in standard deviations of the grayscale scene.  The low
    # profile tolerates exposure changes; the high profile catches small
    # object movement at the cost of more false positives.
    "low": {"threshold": 1.6, "min_region": 0.0010, "changed": 0.0030},
    "medium": {"threshold": 1.0, "min_region": 0.0003, "changed": 0.0008},
    "high": {"threshold": 0.6, "min_region": 0.0001, "changed": 0.0003},
}

_ALIGNMENTS = ("none", "translation")


class ImageDiffError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message

    def as_result(self) -> dict:
        return {"ok": False, "code": self.code, "message": self.message}


class ImageDiffPlugin:
    PREFIX = CARD

    def __init__(self, plugin_config, namespace, executor, camera_plugin=None):
        config = dict(plugin_config or {})
        self._camera = camera_plugin
        self._output_dir = Path(str(config.get(
            "output_dir", DEFAULT_OUTPUT_DIR))).expanduser()
        self._source_root = Path(str(config.get(
            "source_root", DEFAULT_SOURCE_ROOT))).expanduser()
        self._retention_days = max(0, int(config.get("retention_days", 7)))
        self._frame_timeout_s = max(0.0, float(config.get("frame_timeout_s", 5.0)))
        self._baselines_dir = self._output_dir / "baselines"
        self._runs_dir = self._output_dir / "runs"
        self._lock = threading.Lock()
        self._last_run = None

    # ── MCP contract ────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": CARD,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Compare two G1 RGB frames from the same viewpoint and write "
                "a before/after/diff triptych, a mask, and a JSON result "
                "under /opt/phanthy-motus/data/image_diff. Deterministic "
                "pixel/geometry comparison only — no object recognition. "
                "Live capture requires plugins.camera.enabled=true."),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start", "stop", "info", "capture_baseline",
                            "compare", "compare_files", "list", "delete",
                        ],
                    },
                    "label": {
                        "type": "string",
                        "maxLength": 64,
                        "description": "可选，给 baseline 起一个人能看懂的名字。",
                    },
                    "baseline_id": {
                        "type": "string",
                        "description": "缺省使用最新的 baseline。",
                    },
                    "before_path": {
                        "type": "string",
                        "description": "compare_files：改造前照片的绝对路径。",
                    },
                    "after_path": {
                        "type": "string",
                        "description": "compare_files：改造后照片的绝对路径。",
                    },
                    "align": {
                        "type": "string",
                        "enum": list(_ALIGNMENTS),
                        "default": "translation",
                        "description": "translation 会先估计相机平移并裁掉边缘。",
                    },
                    "sensitivity": {
                        "type": "string",
                        "enum": list(_SENSITIVITY),
                        "default": "medium",
                    },
                    "kind": {
                        "type": "string",
                        "enum": ["baselines", "runs"],
                        "default": "baselines",
                    },
                    "id": {
                        "type": "string",
                        "description": "list 返回的 baseline_id 或 run_id。",
                    },
                    "limit": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 100,
                        "default": 20,
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "准备目录并清理过期文件。"},
                    "stop": {"params": [], "description": "停止卡片（不删除证据）。"},
                    "info": {"params": [], "description": "查看目录、baseline 数量和相机状态。"},
                    "capture_baseline": {"params": ["label"],
                                         "description": "保存当前 RGB 帧为 baseline。"},
                    "compare": {"params": ["baseline_id", "align", "sensitivity"],
                                "description": "用当前 RGB 帧和 baseline 对比。"},
                    "compare_files": {"params": ["before_path", "after_path", "align",
                                                 "sensitivity"],
                                      "description": "直接对比两个已有文件。"},
                    "list": {"params": ["kind", "limit"],
                             "description": "列出 baseline 或对比结果。"},
                    "delete": {"params": ["kind", "id"],
                               "description": "删除一个 baseline 或一次对比结果。"},
                },
            },
        }

    def start(self) -> dict:
        self._prepare_dirs()
        self._cleanup_expired()
        return {"ok": True, "state": "ready", "camera_ready": self._camera_ready()}

    def stop(self) -> dict:
        return {"ok": True, "state": "idle"}

    def dispatch(self, action: str, args: dict) -> dict | None:
        args = dict(args or {})
        handlers = {
            "start": lambda _args: self.start(),
            "stop": lambda _args: self.stop(),
            "info": lambda _args: self._info(),
            "capture_baseline": self._capture_baseline,
            "compare": self._compare_to_baseline,
            "compare_files": self._compare_files,
            "list": self._list,
            "delete": self._delete,
        }
        handler = handlers.get(action)
        if handler is None:
            return None
        try:
            if not self._lock.acquire(blocking=False):
                raise ImageDiffError("BUSY", "another image_diff operation is running")
            try:
                return handler(args)
            finally:
                self._lock.release()
        except ImageDiffError as exc:
            return exc.as_result()
        except Exception as exc:  # noqa: BLE001 - MCP must always answer
            return {"ok": False, "code": "INTERNAL_ERROR",
                    "message": f"{type(exc).__name__}: {exc}"}

    # ── Actions ─────────────────────────────────────────────────────────────

    def _info(self) -> dict:
        self._prepare_dirs()
        baselines = self._baseline_files()
        runs = len(list(self._runs_dir.glob("*/result.json")))
        return {
            "ok": True,
            "state": "ready" if self._camera_ready() else "camera_unavailable",
            "camera_ready": self._camera_ready(),
            "output_dir": str(self._output_dir),
            "baselines_dir": str(self._baselines_dir),
            "runs_dir": str(self._runs_dir),
            "source_root": str(self._source_root),
            "retention_days": self._retention_days,
            "baseline_count": len(baselines),
            "latest_baseline": self._latest_baseline_meta(baselines),
            "run_count": runs,
            "last_run": self._last_run,
            "alignments": list(_ALIGNMENTS),
            "sensitivities": list(_SENSITIVITY),
        }

    def _capture_baseline(self, args: dict) -> dict:
        self._prepare_dirs()
        data = self._capture_jpeg()
        image = self._decode(data, "camera_rgb")
        baseline_id = self._new_id("b")
        path = self._baselines_dir / f"{baseline_id}.jpg"
        self._atomic_write(path, data)
        created_at = datetime.now().astimezone()
        meta = {
            "baseline_id": baseline_id,
            "label": self._label(args.get("label")),
            "file_path": str(path),
            "created_at": created_at.isoformat(timespec="seconds"),
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
            "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
        }
        self._atomic_json(self._baselines_dir / f"{baseline_id}.json", meta)
        self._cleanup_expired()
        return {"ok": True, **meta}

    def _compare_to_baseline(self, args: dict) -> dict:
        self._prepare_dirs()
        before_bytes, meta = self._load_baseline(args.get("baseline_id"))
        before = self._decode(before_bytes, meta["file_path"])
        after_bytes = self._capture_jpeg()
        after = self._decode(after_bytes, "camera_rgb")
        return self._run_compare(
            before, before_bytes, after, after_bytes, args,
            before_source=meta["file_path"], after_source="camera_rgb",
            baseline_id=meta["baseline_id"])

    def _compare_files(self, args: dict) -> dict:
        self._prepare_dirs()
        before, before_bytes, before_path = self._load_input(
            args.get("before_path"), "before_path")
        after, after_bytes, after_path = self._load_input(
            args.get("after_path"), "after_path")
        return self._run_compare(
            before, before_bytes, after, after_bytes, args,
            before_source=before_path, after_source=after_path,
            baseline_id=None)

    def _list(self, args: dict) -> dict:
        kind = str(args.get("kind", "baselines"))
        try:
            limit = int(args.get("limit", 20))
        except (TypeError, ValueError):
            raise ImageDiffError("INVALID_ARGUMENT", "limit must be an integer") from None
        limit = max(1, min(100, limit))
        if kind == "baselines":
            items = [meta for meta in (
                self._read_json(path) for path in self._baseline_files()) if meta]
            items.sort(key=lambda item: item.get("created_at", ""), reverse=True)
            return {"ok": True, "kind": kind, "items": items[:limit]}
        if kind == "runs":
            items = []
            for path in self._runs_dir.glob("*/result.json"):
                result = self._read_json(path)
                if result:
                    items.append(self._run_summary(result))
            items.sort(key=lambda item: item.get("created_at", ""), reverse=True)
            return {"ok": True, "kind": kind, "items": items[:limit]}
        raise ImageDiffError("INVALID_ARGUMENT", f"unknown kind: {kind}")

    def _delete(self, args: dict) -> dict:
        kind = str(args.get("kind", "baselines"))
        item_id = str(args.get("id", "")).strip()
        deleted = []
        if kind == "baselines":
            if not _BASELINE_ID_RE.match(item_id):
                raise ImageDiffError("INVALID_ARGUMENT", "invalid baseline id")
            for suffix in (".jpg", ".json"):
                path = self._baselines_dir / f"{item_id}{suffix}"
                if path.exists():
                    path.unlink()
                    deleted.append(str(path))
            if not deleted:
                raise ImageDiffError("NOT_FOUND", f"baseline not found: {item_id}")
            return {"ok": True, "deleted": deleted}
        if kind == "runs":
            if not _RUN_ID_RE.match(item_id):
                raise ImageDiffError("INVALID_ARGUMENT", "invalid run id")
            path = self._runs_dir / item_id
            if not path.is_dir():
                raise ImageDiffError("NOT_FOUND", f"run not found: {item_id}")
            shutil.rmtree(path)
            return {"ok": True, "deleted": [str(path)]}
        raise ImageDiffError("INVALID_ARGUMENT", f"unknown kind: {kind}")

    # ── Camera and input files ──────────────────────────────────────────────

    def _camera_ready(self) -> bool:
        camera = self._camera
        return camera is not None and bool(camera.is_running())

    def _capture_jpeg(self) -> bytes:
        if not self._camera_ready():
            raise ImageDiffError(
                "CAMERA_UNAVAILABLE",
                "image_diff live capture requires plugins.camera.enabled=true")
        frame, sequence = self._camera.wait_for_color_frame(
            None, self._frame_timeout_s)
        if not isinstance(frame, dict) or not frame.get("data"):
            raise ImageDiffError("NO_FRESH_FRAME", "no RGB frame has arrived yet")
        age = time.monotonic() - float(frame.get("received_monotonic", 0.0))
        if age > MAX_FRAME_AGE_S:
            frame, sequence = self._camera.wait_for_color_frame(
                sequence, self._frame_timeout_s)
            if (not isinstance(frame, dict) or not frame.get("data")
                    or time.monotonic() - float(frame.get("received_monotonic", 0.0))
                    > MAX_FRAME_AGE_S):
                raise ImageDiffError(
                    "NO_FRESH_FRAME", "no fresh RGB frame has arrived yet")
        return bytes(frame["data"])

    def _load_input(self, raw_path, field: str):
        if not isinstance(raw_path, str) or not raw_path.strip():
            raise ImageDiffError("INVALID_ARGUMENT", f"{field} is required")
        path = Path(raw_path).expanduser()
        if not path.is_absolute():
            raise ImageDiffError(
                "PATH_NOT_ALLOWED", f"{field} must be an absolute path")
        path = path.resolve()
        try:
            path.relative_to(self._source_root.resolve())
        except ValueError:
            raise ImageDiffError(
                "PATH_NOT_ALLOWED",
                f"{field} must be under {self._source_root}") from None
        if not path.is_file():
            raise ImageDiffError("IMAGE_NOT_FOUND", f"{field} not found: {path}")
        if path.stat().st_size > MAX_INPUT_BYTES:
            raise ImageDiffError("IMAGE_TOO_LARGE", f"{field} exceeds 32 MiB")
        data = path.read_bytes()
        return self._decode(data, str(path)), data, str(path)

    def _decode(self, data: bytes, source: str):
        if not data:
            raise ImageDiffError("IMAGE_UNREADABLE", f"empty image: {source}")
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ImageDiffError("IMAGE_UNREADABLE", f"cannot decode image: {source}")
        height, width = image.shape[:2]
        if height * width > MAX_INPUT_PIXELS:
            raise ImageDiffError(
                "IMAGE_TOO_LARGE", f"image has too many pixels: {source}")
        return image

    def _load_baseline(self, baseline_id):
        files = self._baseline_files()
        if not files:
            raise ImageDiffError("BASELINE_NOT_FOUND", "no baseline has been captured")
        if baseline_id in (None, ""):
            meta_path = files[-1]
        else:
            baseline_id = str(baseline_id)
            if not _BASELINE_ID_RE.match(baseline_id):
                raise ImageDiffError("INVALID_ARGUMENT", "invalid baseline id")
            meta_path = self._baselines_dir / f"{baseline_id}.json"
        meta = self._read_json(meta_path)
        if not meta:
            raise ImageDiffError("BASELINE_NOT_FOUND", "baseline metadata is missing")
        image_path = Path(str(meta.get("file_path", "")))
        if not image_path.is_file():
            raise ImageDiffError("BASELINE_NOT_FOUND", "baseline image is missing")
        return image_path.read_bytes(), meta

    # ── Comparison ──────────────────────────────────────────────────────────

    def _run_compare(self, before, before_bytes, after, after_bytes, args,
                     before_source, after_source, baseline_id):
        started = time.monotonic()
        align = str(args.get("align", "translation"))
        sensitivity = str(args.get("sensitivity", "medium"))
        if align not in _ALIGNMENTS:
            raise ImageDiffError("INVALID_ARGUMENT", f"unknown align: {align}")
        if sensitivity not in _SENSITIVITY:
            raise ImageDiffError(
                "INVALID_ARGUMENT", f"unknown sensitivity: {sensitivity}")
        profile = _SENSITIVITY[sensitivity]

        before_h, before_w = before.shape[:2]
        after_h, after_w = after.shape[:2]
        ratio_before = before_w / max(1, before_h)
        ratio_after = after_w / max(1, after_h)
        if abs(ratio_before - ratio_after) / max(ratio_before, ratio_after) > 0.01:
            raise ImageDiffError(
                "SIZE_MISMATCH", "images have different aspect ratios")

        before_work = self._resize_work(before)
        after_work = self._resize_work(after)
        if before_work.shape[:2] != after_work.shape[:2]:
            # Same aspect ratio, different resolution: fit the after image to
            # the before image's work size so the two masks stay comparable.
            target = (before_work.shape[1], before_work.shape[0])
            source_area = after_work.shape[0] * after_work.shape[1]
            target_area = target[0] * target[1]
            after_work = cv2.resize(
                after_work, target,
                interpolation=cv2.INTER_AREA if source_area > target_area
                else cv2.INTER_LINEAR)

        before_color, after_color, alignment = self._align(
            before_work, after_work, align)
        before_gray = self._gray(before_color)
        after_gray = self._gray(after_color)
        before_norm, std_before = self._standardize(before_gray)
        after_norm, std_after = self._standardize(after_gray)
        low_texture = std_before < 1.0 or std_after < 1.0
        if low_texture:
            diff = np.zeros_like(before_norm)
        else:
            diff = np.abs(before_norm - after_norm)

        mask = (diff > profile["threshold"]).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        height, width = mask.shape[:2]
        min_area = max(64, int(round(height * width * profile["min_region"])))
        regions = self._regions(mask, diff, min_area)
        changed_ratio = float(cv2.countNonZero(mask)) / max(1, height * width)
        if changed_ratio >= profile["changed"]:
            verdict = "changed"
        elif changed_ratio >= profile["changed"] / 3.0:
            verdict = "uncertain"
        else:
            verdict = "no_change"
        if low_texture and verdict == "no_change":
            verdict = "uncertain"

        ssim = self._ssim(before_gray, after_gray)
        raw_diff = cv2.absdiff(before_gray, after_gray)
        metrics = {
            "changed_ratio": round(changed_ratio, 6),
            "ssim": round(ssim, 6),
            "mean_abs_diff": round(float(raw_diff.mean()), 3),
            "regions": regions,
            "threshold": profile["threshold"],
            "min_region_ratio": profile["min_region"],
            "low_texture": low_texture,
        }

        run_id = self._new_id("r")
        run_dir = self._runs_dir / run_id
        run_dir.mkdir(parents=True, exist_ok=False)
        before_path = run_dir / "before.jpg"
        after_path = run_dir / "after.jpg"
        diff_path = run_dir / "diff.jpg"
        mask_path = run_dir / "mask.png"
        result_path = run_dir / "result.json"
        self._write_jpeg(before_path, before)
        self._write_jpeg(after_path, after)
        self._write_jpeg(diff_path, self._triptych(
            before_color, after_color, mask, regions, verdict, metrics,
            alignment, sensitivity))
        self._write_png(mask_path, mask)
        created_at = datetime.now().astimezone().isoformat(timespec="seconds")
        result = {
            "ok": True,
            "run_id": run_id,
            "created_at": created_at,
            "verdict": verdict,
            "baseline_id": baseline_id,
            "before_source": before_source,
            "after_source": after_source,
            "before_path": str(before_path),
            "after_path": str(after_path),
            "diff_path": str(diff_path),
            "mask_path": str(mask_path),
            "result_path": str(result_path),
            "image_size": [width, height],
            "source_size": [before_w, before_h],
            "alignment": alignment,
            "sensitivity": sensitivity,
            "metrics": metrics,
            "elapsed_ms": int(round((time.monotonic() - started) * 1000)),
            "input_sha256": {
                "before": hashlib.sha256(before_bytes).hexdigest(),
                "after": hashlib.sha256(after_bytes).hexdigest(),
            },
        }
        self._atomic_json(result_path, result)
        self._last_run = self._run_summary(result)
        self._cleanup_expired()
        return result

    def _align(self, before, after, mode: str):
        gray_before = self._gray(before)
        gray_after = self._gray(after)
        if mode == "none":
            return before, after, {
                "mode": "none", "shift_px": [0.0, 0.0],
                "response": None, "overlap_ratio": 1.0,
            }
        height, width = gray_before.shape[:2]
        window = cv2.createHanningWindow((width, height), cv2.CV_32F)
        (dx, dy), response = cv2.phaseCorrelate(
            np.float32(gray_before), np.float32(gray_after), window)
        if not math.isfinite(dx) or not math.isfinite(dy):
            raise ImageDiffError("ALIGNMENT_FAILED", "translation estimate is invalid")
        if abs(dx) > width * 0.08 or abs(dy) > height * 0.08:
            raise ImageDiffError(
                "ALIGNMENT_FAILED", "camera moved too far for translation alignment")
        if response < 0.05:
            raise ImageDiffError(
                "ALIGNMENT_FAILED", "not enough texture to estimate translation")
        if abs(dx) < 0.75:
            dx = 0.0
        if abs(dy) < 0.75:
            dy = 0.0
        if dx == 0.0 and dy == 0.0:
            return before, after, {
                "mode": "translation", "shift_px": [0.0, 0.0],
                "response": round(float(response), 5), "overlap_ratio": 1.0,
            }
        matrix = np.float32([[1, 0, -dx], [0, 1, -dy]])
        moved = cv2.warpAffine(
            after, matrix, (width, height),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        crop_x = int(math.ceil(abs(dx))) + 1
        crop_y = int(math.ceil(abs(dy))) + 1
        if crop_x * 2 >= width * 0.5 or crop_y * 2 >= height * 0.5:
            raise ImageDiffError(
                "ALIGNMENT_FAILED", "translation leaves too little overlap")
        before_crop = before[crop_y:height - crop_y, crop_x:width - crop_x]
        after_crop = moved[crop_y:height - crop_y, crop_x:width - crop_x]
        overlap = (width - 2 * crop_x) * (height - 2 * crop_y) / (width * height)
        return before_crop, after_crop, {
            "mode": "translation",
            "shift_px": [round(float(dx), 3), round(float(dy), 3)],
            "response": round(float(response), 5),
            "overlap_ratio": round(float(overlap), 5),
            "crop_px": [crop_x, crop_y],
        }

    @staticmethod
    def _resize_work(image):
        height, width = image.shape[:2]
        longest = max(height, width)
        if longest <= WORK_LONG_EDGE:
            return image
        scale = WORK_LONG_EDGE / float(longest)
        return cv2.resize(
            image, (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
            interpolation=cv2.INTER_AREA)

    @staticmethod
    def _gray(image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _standardize(gray):
        """Local z-score: removes exposure and low-frequency lighting changes.

        A global mean/std also works on textured scenes, but flat regions keep
        flipping sign for tiny global shifts because their local variance is
        near zero.  Dividing by a local variance with a small floor keeps
        flat areas quiet while leaving object edges loud.
        """
        scene_std = float(gray.std())
        gray = cv2.GaussianBlur(gray.astype(np.float32), (5, 5), 0)
        local_mean = cv2.GaussianBlur(gray, (0, 0), 15)
        local_var = cv2.GaussianBlur(gray * gray, (0, 0), 15) - local_mean * local_mean
        return (gray - local_mean) / np.sqrt(np.maximum(local_var, 0.0) + 25.0), scene_std

    @staticmethod
    def _regions(mask, diff, min_area):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        regions = []
        height, width = mask.shape[:2]
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            x = int(stats[index, cv2.CC_STAT_LEFT])
            y = int(stats[index, cv2.CC_STAT_TOP])
            w = int(stats[index, cv2.CC_STAT_WIDTH])
            h = int(stats[index, cv2.CC_STAT_HEIGHT])
            region_mask = labels == index
            regions.append({
                "bbox": [x, y, w, h],
                "bbox_norm": [
                    round(x / width, 4), round(y / height, 4),
                    round(w / width, 4), round(h / height, 4),
                ],
                "area_px": area,
                "area_ratio": round(area / (width * height), 6),
                "mean_delta": round(float(diff[region_mask].mean()), 4),
            })
        regions.sort(key=lambda item: item["area_px"], reverse=True)
        return regions[:MAX_REGIONS]

    @staticmethod
    def _ssim(first, second):
        first = first.astype(np.float32)
        second = second.astype(np.float32)
        c1 = (0.01 * 255) ** 2
        c2 = (0.03 * 255) ** 2
        mu1 = cv2.GaussianBlur(first, (11, 11), 1.5)
        mu2 = cv2.GaussianBlur(second, (11, 11), 1.5)
        mu1_sq, mu2_sq, mu12 = mu1 * mu1, mu2 * mu2, mu1 * mu2
        sigma1 = cv2.GaussianBlur(first * first, (11, 11), 1.5) - mu1_sq
        sigma2 = cv2.GaussianBlur(second * second, (11, 11), 1.5) - mu2_sq
        sigma12 = cv2.GaussianBlur(first * second, (11, 11), 1.5) - mu12
        score = ((2 * mu12 + c1) * (2 * sigma12 + c2)) / (
            (mu1_sq + mu2_sq + c1) * (sigma1 + sigma2 + c2))
        return float(score.mean())

    @staticmethod
    def _triptych(before, after, mask, regions, verdict, metrics,
                  alignment, sensitivity):
        panel_w = 640
        gap = 8

        def panel(image):
            height, width = image.shape[:2]
            scale = panel_w / float(width)
            return cv2.resize(
                image, (panel_w, max(1, int(round(height * scale)))),
                interpolation=cv2.INTER_AREA)

        left = panel(before)
        middle = panel(after)
        right = middle.copy()
        height = max(left.shape[0], middle.shape[0], right.shape[0])
        panels = []
        for image in (left, middle, right):
            if image.shape[0] < height:
                image = cv2.copyMakeBorder(
                    image, 0, height - image.shape[0], 0, 0,
                    cv2.BORDER_CONSTANT, value=(0, 0, 0))
            panels.append(image)
        left, middle, right = panels
        work_h, work_w = mask.shape[:2]
        scale_x = panel_w / float(work_w)
        scale_y = height / float(work_h)
        mask_small = cv2.resize(mask, (panel_w, height), interpolation=cv2.INTER_NEAREST)
        red = np.zeros_like(right)
        red[:, :, 2] = 255
        right = np.where(mask_small[:, :, None] > 0,
                         cv2.addWeighted(right, 0.55, red, 0.45, 0), right)
        for region in regions:
            x, y, w, h = region["bbox"]
            cv2.rectangle(
                right,
                (int(round(x * scale_x)), int(round(y * scale_y))),
                (int(round((x + w) * scale_x)), int(round((y + h) * scale_y))),
                (0, 255, 0), 2)
        header = 30
        canvas = np.zeros((height + header, panel_w * 3 + gap * 2, 3), np.uint8)
        canvas[header:, 0:panel_w] = left[:height]
        canvas[header:, panel_w + gap:panel_w * 2 + gap] = middle[:height]
        canvas[header:, panel_w * 2 + gap * 2:] = right[:height]
        color = {"changed": (0, 0, 255), "uncertain": (0, 200, 255)}.get(
            verdict, (0, 200, 0))
        shift = alignment.get("shift_px") or [0, 0]
        text = (f"{verdict}  changed={metrics['changed_ratio'] * 100:.2f}%  "
                f"ssim={metrics['ssim']:.4f}  {sensitivity}  "
                f"shift=({shift[0]:.1f},{shift[1]:.1f})")
        cv2.putText(canvas, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 1, cv2.LINE_AA)
        return canvas

    # ── Storage helpers ─────────────────────────────────────────────────────

    def _prepare_dirs(self):
        self._baselines_dir.mkdir(parents=True, exist_ok=True)
        self._runs_dir.mkdir(parents=True, exist_ok=True)

    def _baseline_files(self):
        if not self._baselines_dir.is_dir():
            return []
        return sorted(self._baselines_dir.glob("b_*.json"))

    def _latest_baseline_meta(self, files):
        if not files:
            return None
        return self._read_json(files[-1])

    @staticmethod
    def _new_id(prefix: str) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{prefix}_{stamp}_{uuid.uuid4().hex[:4]}"

    @staticmethod
    def _label(value):
        if value is None:
            return ""
        return str(value).strip()[:64]

    @staticmethod
    def _read_json(path):
        try:
            return json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    @staticmethod
    def _atomic_write(path: Path, data: bytes):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
        with temporary.open("wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)

    def _atomic_json(self, path: Path, value):
        self._atomic_write(
            path, (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))

    @staticmethod
    def _write_jpeg(path: Path, image):
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not ok:
            raise ImageDiffError("ENCODE_FAILED", f"cannot encode {path.name}")
        ImageDiffPlugin._atomic_write(path, buffer.tobytes())

    @staticmethod
    def _write_png(path: Path, image):
        ok, buffer = cv2.imencode(".png", image)
        if not ok:
            raise ImageDiffError("ENCODE_FAILED", f"cannot encode {path.name}")
        ImageDiffPlugin._atomic_write(path, buffer.tobytes())

    def _cleanup_expired(self):
        if self._retention_days <= 0:
            return
        cutoff = time.time() - self._retention_days * 86400
        for directory in (self._baselines_dir, self._runs_dir):
            if not directory.is_dir():
                continue
            for entry in directory.iterdir():
                try:
                    if entry.stat().st_mtime >= cutoff:
                        continue
                    if entry.is_dir():
                        shutil.rmtree(entry)
                    else:
                        entry.unlink()
                except FileNotFoundError:
                    continue
                except OSError as exc:
                    print(f"[image_diff] cleanup skipped {entry}: {exc}", flush=True)

    @staticmethod
    def _run_summary(result):
        return {
            "run_id": result.get("run_id"),
            "created_at": result.get("created_at"),
            "verdict": result.get("verdict"),
            "changed_ratio": (result.get("metrics") or {}).get("changed_ratio"),
            "diff_path": result.get("diff_path"),
        }
