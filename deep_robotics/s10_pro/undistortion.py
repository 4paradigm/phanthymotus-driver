"""Per-camera lens undistortion for the S10 Pro RTSP cameras.

Each camera card carries an `undistort` option (on by default) and an optional
`calibration` override. When undistortion is active the lane's FFmpeg command
gains a `remap` stage, so the JPEG published on the camera topic is already
rectified and the card declares the rectified geometry (`pinhole`, the
calibrated K, zero D). Downstream cards need nothing extra: they read K from the
declaration as they would for any camera.

Why remap inside FFmpeg rather than OpenCV in Python: the frames never leave
FFmpeg today (the driver only splits the MJPEG pipe on JPEG markers), the image
carries no OpenCV, and decoding/re-encoding in this process would add CPU to the
driver that also runs the robot ROS callbacks. The maps are computed once per
calibration at the 1024x576 source resolution and handed to FFmpeg as 16-bit
PGM images; the rectified frame is then scaled to the 640x360 the cards publish.
`remap` samples nearest-neighbour, and that downscale smooths it.

Enabling rectification never silently falls back to distorted frames: if maps
cannot be prepared, that lane retries without publishing misregistered images.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import threading
from pathlib import Path

MODELS = ("plumb_bob", "rational_polynomial", "equidistant")
OUTPUT_SIZE = (640, 360)                 # what the camera cards publish
DEFAULT_CONFIG_PATH = "/opt/phanthy-motus/data/s10_pro/camera_undistort.json"
DEFAULT_MAP_DIR = "/tmp/s10_pro_undistort"
PIPELINE_STAGE = "deeprobotics/s10_pro/undistort"

# Chessboard calibrations of the four S10 Pro cameras, keyed on camera id.
#
# Calibrated on the 1024x576 RTSP streams (10x13 inner corners, 18 mm squares,
# robot standing) with tools/calibrate_chessboard.py and scaled to the 640x360
# the driver publishes (K x 0.625, D unchanged). The id is per model, not per
# unit, so these are the default for every S10 Pro; a camera card's
# `calibration` override replaces its row for one robot.
CALIBRATIONS = {
    "deeprobotics/s10_pro/camera_front": {
        "id": "deeprobotics/s10_pro/camera_front",
        "width": 640, "height": 360, "distortion_model": "plumb_bob",
        "K": [314.963248, 0, 336.650604, 0, 314.934089, 182.285271, 0, 0, 1],
        "D": [-0.264916006540722, 0.08280010720226033, -0.00018047856465669117,
              3.14558084286742e-05, -0.013244463692919645],
        "measured_on": "2026-11-07", "unit": "S10 Pro CD1 (host CA9B_NO.0)",
        "source_resolution": [1024, 576], "frames_used": 103, "rms_px": 0.213,
    },
    "deeprobotics/s10_pro/camera_rear": {
        "id": "deeprobotics/s10_pro/camera_rear",
        "width": 640, "height": 360, "distortion_model": "plumb_bob",
        "K": [312.750288, 0, 308.919092, 0, 312.683048, 183.364774, 0, 0, 1],
        "D": [-0.2622894409938081, 0.08153733337528488, 0.00012089478431258563,
              0.00040150868161954143, -0.013044993729573065],
        "measured_on": "2026-11-08", "unit": "S10 Pro CD1 (host CA9B_NO.0)",
        "source_resolution": [1024, 576], "frames_used": 103, "rms_px": 0.208,
    },
    "deeprobotics/s10_pro/camera_left": {
        "id": "deeprobotics/s10_pro/camera_left",
        "width": 640, "height": 360, "distortion_model": "plumb_bob",
        "K": [314.208908, 0, 311.686392, 0, 314.175661, 192.549502, 0, 0, 1],
        "D": [-0.25929876926166817, 0.07786005715673677, 0.00035419731549147994,
              0.0005598844289782525, -0.011875881953120242],
        "measured_on": "2026-11-08", "unit": "S10 Pro CD1 (host CA9B_NO.0)",
        "source_resolution": [1024, 576], "frames_used": 106, "rms_px": 0.222,
    },
    "deeprobotics/s10_pro/camera_right": {
        "id": "deeprobotics/s10_pro/camera_right",
        "width": 640, "height": 360, "distortion_model": "plumb_bob",
        "K": [314.351144, 0, 327.541354, 0, 314.28902, 177.870404, 0, 0, 1],
        "D": [-0.2617530191348165, 0.0801398926074851, 9.564678897937886e-05,
              -0.0001381378356953391, -0.012449424682861728],
        "measured_on": "2026-11-08", "unit": "S10 Pro CD1 (host CA9B_NO.0)",
        "source_resolution": [1024, 576], "frames_used": 98, "rms_px": 0.219,
    },
}


def camera_id(lane):
    return f"deeprobotics/s10_pro/{lane}"


def validate(calibration, lane=None):
    """A clean copy of a calibration object, or ValueError saying what is wrong."""
    if not isinstance(calibration, dict):
        raise ValueError("calibration must be an object")
    cal = copy.deepcopy(calibration)
    if lane is not None:
        cal.setdefault("id", camera_id(lane))
        if cal["id"] != camera_id(lane):
            raise ValueError(f"calibration id {cal['id']!r} belongs to another camera, "
                             f"expected {camera_id(lane)!r}")
    for key in ("width", "height"):
        value = cal.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"calibration {key} must be a positive integer")
    if (cal["width"], cal["height"]) != OUTPUT_SIZE:
        raise ValueError(f"calibration must describe the {OUTPUT_SIZE[0]}x{OUTPUT_SIZE[1]} "
                         f"stream the camera card publishes, got {cal['width']}x{cal['height']}")
    model = cal.get("distortion_model")
    if model not in MODELS:
        raise ValueError(f"distortion_model must be one of {', '.join(MODELS)}")
    for key in ("K", "D"):
        values = cal.get(key)
        if not isinstance(values, list) or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in values):
            raise ValueError(f"calibration {key} must be a list of finite numbers")
    k = cal["K"]
    if (len(k) != 9 or k[0] <= 0 or k[4] <= 0 or k[1] != 0 or k[3] != 0
            or list(k[6:]) != [0, 0, 1]
            or not 0 <= k[2] < cal["width"] or not 0 <= k[5] < cal["height"]):
        raise ValueError("K must be nine row-major values with positive focal lengths, "
                         "zero skew and an in-frame principal point")
    lengths = {"plumb_bob": (4, 5), "rational_polynomial": (8,), "equidistant": (4,)}
    if len(cal["D"]) not in lengths[model]:
        raise ValueError(f"D has {len(cal['D'])} values; {model} takes "
                         f"{' or '.join(map(str, lengths[model]))}")
    source = cal.get("source_resolution", [1024, 576])
    if (not isinstance(source, (list, tuple)) or len(source) != 2
            or any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in source)):
        raise ValueError("source_resolution must be [width, height] of the RTSP stream")
    cal["source_resolution"] = [int(source[0]), int(source[1])]
    return cal


def half_fov(k, width, height):
    """Pinhole half angles (horizontal, vertical) of a K at width x height."""
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return ((math.atan(cx / fx) + math.atan((width - cx) / fx)) / 2,
            (math.atan(cy / fy) + math.atan((height - cy) / fy)) / 2)


def _distort(cal, x, y):
    """Forward lens model on normalized coordinates (scalars or numpy arrays)."""
    d = list(cal["D"])
    model = cal["distortion_model"]
    if model == "equidistant":
        k1, k2, k3, k4 = d
        r = (x * x + y * y) ** 0.5
        theta = _atan(r)
        t2 = theta * theta
        theta_d = theta * (1 + k1 * t2 + k2 * t2 * t2 + k3 * t2 ** 3 + k4 * t2 ** 4)
        scale = _where(r > 1e-12, theta_d / _maximum(r, 1e-12), 1.0)
        return x * scale, y * scale
    if model == "rational_polynomial":
        k1, k2, p1, p2, k3, k4, k5, k6 = d
    else:
        k1, k2, p1, p2 = d[:4]
        k3 = d[4] if len(d) == 5 else 0.0
        k4 = k5 = k6 = 0.0
    r2 = x * x + y * y
    radial = ((1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3)
              / (1 + k4 * r2 + k5 * r2 * r2 + k6 * r2 ** 3))
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return xd, yd


def _atan(v):
    try:
        import numpy as np
        if isinstance(v, np.ndarray):
            return np.arctan(v)
    except ImportError:
        pass
    return math.atan(v)


def _where(cond, a, b):
    try:
        import numpy as np
        if isinstance(cond, np.ndarray):
            return np.where(cond, a, b)
    except ImportError:
        pass
    return a if cond else b


def _maximum(a, b):
    try:
        import numpy as np
        if isinstance(a, np.ndarray):
            return np.maximum(a, b)
    except ImportError:
        pass
    return max(a, b)


def source_matrix(cal):
    """K at the RTSP source resolution (inverse of the tool's 1024->640 scale)."""
    src_w, src_h = cal["source_resolution"]
    sx, sy = src_w / cal["width"], src_h / cal["height"]
    k = cal["K"]
    return [k[0] * sx, 0.0, (k[2] + 0.5) * sx - 0.5,
            0.0, k[4] * sy, (k[5] + 0.5) * sy - 0.5, 0.0, 0.0, 1.0]


INVALID = 65535


def compute_maps(cal):
    """(xmap, ymap, invalid_fraction) at the source resolution, row-major lists
    or numpy uint16 arrays. Each output pixel holds the source pixel it samples,
    rounded to nearest; pixels that sample outside the source hold 65535, which
    FFmpeg's remap fills black. The new camera matrix equals the calibrated one,
    so the rectified image keeps the declared K."""
    src_w, src_h = cal["source_resolution"]
    k = source_matrix(cal)
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    try:
        import numpy as np
    except ImportError:
        np = None
    if np is not None:
        u, v = np.meshgrid(np.arange(src_w, dtype=np.float64),
                           np.arange(src_h, dtype=np.float64))
        xd, yd = _distort(cal, (u - cx) / fx, (v - cy) / fy)
        us = np.rint(fx * xd + cx)
        vs = np.rint(fy * yd + cy)
        valid = (us >= 0) & (us <= src_w - 1) & (vs >= 0) & (vs <= src_h - 1)
        xmap = np.where(valid, us, INVALID).astype(np.uint16)
        ymap = np.where(valid, vs, INVALID).astype(np.uint16)
        return xmap, ymap, float(1.0 - valid.mean())
    xmap, ymap, invalid = [], [], 0
    xs = [(u - cx) / fx for u in range(src_w)]
    for v in range(src_h):
        y = (v - cy) / fy
        for x in xs:
            xd, yd = _distort(cal, x, y)
            us, vs = round(fx * xd + cx), round(fy * yd + cy)
            if 0 <= us <= src_w - 1 and 0 <= vs <= src_h - 1:
                xmap.append(us)
                ymap.append(vs)
            else:
                xmap.append(INVALID)
                ymap.append(INVALID)
                invalid += 1
    return xmap, ymap, invalid / float(src_w * src_h)


def _pgm16(values, width, height):
    header = f"P5\n{width} {height}\n65535\n".encode("ascii")
    try:
        import numpy as np
        if isinstance(values, np.ndarray):
            return header + values.astype(">u2").tobytes()
    except ImportError:
        pass
    import struct
    return header + struct.pack(f">{len(values)}H", *values)


def calibration_key(cal):
    fields = {k: cal[k] for k in ("width", "height", "distortion_model", "K", "D",
                                  "source_resolution")}
    return hashlib.sha256(json.dumps(fields, sort_keys=True).encode()).hexdigest()[:16]


def write_maps(cal, directory):
    """Write (or reuse) the PGM maps for a calibration; returns (xpath, ypath, invalid)."""
    directory = Path(directory)
    key = calibration_key(cal)
    xpath, ypath = directory / f"{key}_x.pgm", directory / f"{key}_y.pgm"
    meta = directory / f"{key}.json"
    if xpath.is_file() and ypath.is_file() and meta.is_file():
        try:
            return str(xpath), str(ypath), float(json.loads(meta.read_text())["invalid_fraction"])
        except (OSError, ValueError, KeyError):
            pass
    directory.mkdir(parents=True, exist_ok=True)
    xmap, ymap, invalid = compute_maps(cal)
    src_w, src_h = cal["source_resolution"]
    for path, values in ((xpath, xmap), (ypath, ymap)):
        tmp = path.with_suffix(f".{os.getpid()}.{threading.get_ident()}.tmp")
        tmp.write_bytes(_pgm16(values, src_w, src_h))
        os.replace(tmp, path)
    meta.write_text(json.dumps({"invalid_fraction": invalid}))
    return str(xpath), str(ypath), invalid


def raw_command(url):
    return ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay",
            "-i", str(url), "-an", "-vf", "fps=15,scale='min(640,iw)':-2",
            "-q:v", "5", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1"]


def remap_command(url, xpath, ypath, source_resolution, output_size=OUTPUT_SIZE):
    """Raw command with a remap stage: fps first (remap only what is published),
    force the calibrated source size, remap, then scale to the published size."""
    src_w, src_h = source_resolution
    out_w, out_h = output_size
    graph = (f"[0:v]fps=15,scale={src_w}:{src_h},format=gbrp[src];"
             f"[src][1:v][2:v]remap,scale={out_w}:{out_h}[out]")
    return ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay",
            "-i", str(url), "-i", str(xpath), "-i", str(ypath),
            "-filter_complex", graph, "-map", "[out]", "-an",
            "-q:v", "5", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1"]


class CameraUndistortion:
    """Settings, persistence and runtime state of undistortion for all lanes.

    `rtsp_command` is the MediaWorkers hook; `configure`,
    `status` and `declaration` serve the camera cards. `on_change(lane)` is
    called after a setting changes so the lane can reconnect with its new
    command.
    """

    def __init__(self, lanes, config_path=None, map_dir=None, on_change=None):
        self.lanes = tuple(lanes)
        self.config_path = Path(config_path or os.environ.get(
            "S10_CAMERA_UNDISTORT_CONFIG", DEFAULT_CONFIG_PATH))
        self.map_dir = Path(map_dir or os.environ.get("S10_CAMERA_UNDISTORT_MAPS", DEFAULT_MAP_DIR))
        self.on_change = on_change
        self._lock = threading.RLock()
        self._settings = {lane: {"undistort": True, "calibration": {}} for lane in self.lanes}
        self._runtime = {lane: self._fresh_runtime() for lane in self.lanes}
        self.persist_error = None
        self._generations = {lane: 0 for lane in self.lanes}
        self._load()

    @staticmethod
    def _fresh_runtime():
        return {"mode": None, "last_error": None, "invalid_fraction": None}

    # ── settings ───────────────────────────────────────────────────────────

    def _load(self):
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, ValueError) as exc:
            self.persist_error = f"cannot read {self.config_path}: {exc}"
            print(f"[camera-undistort] {self.persist_error}; using defaults", flush=True)
            return
        cameras = data.get("cameras", {}) if isinstance(data, dict) else {}
        if not isinstance(cameras, dict):
            self.persist_error = "saved camera settings must be an object; using defaults"
            return
        for lane, saved in cameras.items():
            if lane not in self._settings or not isinstance(saved, dict):
                continue
            try:
                self._settings[lane] = self._normalize(lane, saved)
            except ValueError as exc:
                print(f"[camera-undistort] ignoring saved setting for {lane}: {exc}", flush=True)

    def _save(self):
        payload = {"version": 1, "cameras": self._settings}
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.config_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            os.replace(tmp, self.config_path)
            self.persist_error = None
        except OSError as exc:
            self.persist_error = f"cannot write {self.config_path}: {exc}"
            print(f"[camera-undistort] {self.persist_error}; setting kept in memory", flush=True)

    def _normalize(self, lane, values):
        enabled = values.get("undistort", True)
        if not isinstance(enabled, bool):
            raise ValueError("undistort must be true or false")
        calibration = values.get("calibration", {})
        if not isinstance(calibration, dict):
            raise ValueError("calibration must be an object; {} selects the built-in calibration")
        if calibration:
            calibration = validate(calibration, lane)
        return {"undistort": enabled, "calibration": calibration}

    def settings(self, lane):
        with self._lock:
            return copy.deepcopy(self._settings[lane])

    def configure(self, lane, values):
        """Apply `undistort` / `calibration` from a card call. Returns True when
        something changed. Invalid values raise ValueError and change nothing."""
        with self._lock:
            current = self._settings[lane]
            merged = {key: values[key] if key in values else current[key]
                      for key in ("undistort", "calibration")}
            updated = self._normalize(lane, merged)
            if updated == current:
                return False
            self._settings[lane] = updated
            self._generations[lane] += 1
            self._runtime[lane] = self._fresh_runtime()
            self._save()
        if self.on_change is not None:
            self.on_change(lane)
        return True

    def generation(self, lane):
        with self._lock:
            return self._generations[lane]

    def publish_if_current(self, lane, generation, publish):
        # Configuration and publication cannot interleave: an old FFmpeg pipe
        # may still contain a complete JPEG after terminate() was requested.
        with self._lock:
            if self._generations[lane] == generation:
                publish()
                self._runtime[lane]["last_error"] = None

    def record_error(self, lane, generation, text):
        with self._lock:
            if self._generations[lane] == generation:
                self._runtime[lane]["last_error"] = text[-500:]

    def calibration(self, lane):
        """(calibration, source) or (None, reason)."""
        with self._lock:
            override = self._settings[lane]["calibration"]
        if override:
            return copy.deepcopy(override), "manual"
        row = CALIBRATIONS.get(camera_id(lane))
        if row is None:
            return None, f"no calibration for {camera_id(lane)}"
        return validate(row, lane), f"table:{camera_id(lane)}"

    def active(self, lane):
        """Whether the lane is (or is about to be) serving rectified frames."""
        with self._lock:
            settings = self._settings[lane]
            if not settings["undistort"]:
                return False
        cal, _ = self.calibration(lane)
        return cal is not None

    # ── MediaWorkers hooks ─────────────────────────────────────────────────

    def rtsp_command(self, lane, url):
        if not self.active(lane):
            with self._lock:
                self._runtime[lane]["mode"] = "raw"
            return raw_command(url)
        cal, _ = self.calibration(lane)
        try:
            xpath, ypath, invalid = write_maps(cal, self.map_dir)
        except (OSError, ValueError) as exc:
            with self._lock:
                self._runtime[lane].update(
                    last_error=f"cannot prepare undistortion maps: {exc}")
            raise ValueError(f"cannot prepare undistortion maps for {lane}: {exc}") from exc
        with self._lock:
            self._runtime[lane].update(mode="undistort", invalid_fraction=invalid)
        return remap_command(url, xpath, ypath, cal["source_resolution"])

    # ── card views ─────────────────────────────────────────────────────────

    def status(self, lane):
        cal, source = self.calibration(lane)
        with self._lock:
            settings, runtime = dict(self._settings[lane]), dict(self._runtime[lane])
        result = {
            "enabled": settings["undistort"],
            "active": self.active(lane),
            "calibration_source": source if cal is not None else None,
            "error": runtime["last_error"] or (None if cal is not None else source),
        }
        if runtime["invalid_fraction"]:
            result["invalid_border_fraction"] = round(runtime["invalid_fraction"], 4)
        if self.persist_error:
            result["persist_error"] = self.persist_error
        return result

    def declaration(self, lane):
        """Keyword arguments for build_camera_info describing what is served."""
        cal, source = self.calibration(lane)
        if cal is None:
            return {"half_fov_rad": None, "source": "unknown"}
        width, height = OUTPUT_SIZE
        if self.active(lane):
            h, v = half_fov(cal["K"], width, height)
            return {
                "distortion_model": "pinhole",
                "K": [float(x) for x in cal["K"]],
                "D": [0.0] * 5,
                "half_fov_rad": h, "half_fov_v_rad": v,
                "source": "measured",
                "measured_on": cal.get("measured_on", ""),
                "pipeline": [camera_id(lane), PIPELINE_STAGE],
                "undistortion": {
                    "state": "rectified", "calibration_source": source,
                    "input_calibration": {k: cal[k] for k in
                                          ("distortion_model", "K", "D", "source_resolution")},
                    "method": "FFmpeg remap at the source resolution, then scaled to 640x360",
                },
            }
        # Raw, barrel-distorted frames. The driver spec has no plumb_bob model,
        # so only the pinhole angle of the calibrated K is declared: right near
        # the image centre, narrower than the true edge rays.
        h, v = half_fov(cal["K"], width, height)
        return {
            "half_fov_rad": h, "half_fov_v_rad": v,
            "source": "measured", "measured_on": cal.get("measured_on", ""),
            "undistortion": {
                "state": "raw", "calibration_source": source,
                "note": ("raw stream is barrel-distorted; half_fov_rad is the pinhole angle "
                         "of the calibrated K and understates the edge rays. Enable the "
                         "camera card's undistort option for exact geometry."),
            },
        }
