#!/usr/bin/env python3
"""Measure the G1 D435I callback-arrival offset from RealSense GLOBAL_TIME.

The released G1 Driver timestamps RGB messages when the librealsense callback
arrives.  This probe opens the same color profile exclusively, compares host
delivery time with each frame's GLOBAL_TIME timestamp, and emits the fixed
offset that FAST-LIVO2 should add to those callback-arrival ROS headers.

It does not measure camera-to-LiDAR spatial extrinsics or prove end-to-end
cross-sensor synchronization.  The result is intentionally preview evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any


PROBE_TYPE = "g1_realsense_callback_latency"
MAX_OFFSET_ABS_S = 0.5
MAX_P95_RESIDUAL_MS = 20.0
MIN_SAMPLES = 30


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        raise ValueError("cannot calculate a percentile from an empty sample")
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def canonical_id(data: dict[str, Any]) -> str:
    encoded = json.dumps(
        data,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def build_result(
    *,
    serial: str,
    boot_id: str,
    width: int,
    height: int,
    fps: int,
    timestamp_domain: str,
    latencies_ms: list[float],
    exposures_us: list[float],
) -> dict[str, Any]:
    if len(latencies_ms) < MIN_SAMPLES:
        raise ValueError(f"need at least {MIN_SAMPLES} latency samples")
    median_ms = statistics.median(latencies_ms)
    residuals_ms = [abs(value - median_ms) for value in latencies_ms]
    data: dict[str, Any] = {
        "schema_version": 1,
        "probe_type": PROBE_TYPE,
        "captured_at_epoch_ns": time.time_ns(),
        "boot_id": boot_id,
        "d435i": {
            "serial": serial,
            "color": {
                "width": width,
                "height": height,
                "fps": fps,
                "format": "bgr8",
            },
        },
        "measurement": {
            "method": "realsense_global_time_to_host_delivery_proxy",
            "timestamp_domain": timestamp_domain,
            "sample_count": len(latencies_ms),
            "callback_latency_ms": {
                "min": min(latencies_ms),
                "median": median_ms,
                "p95": percentile(latencies_ms, 0.95),
                "max": max(latencies_ms),
            },
            "recommended_img_time_offset_s": -median_ms / 1000.0,
            "p95_abs_residual_ms": percentile(residuals_ms, 0.95),
            "exposure_us": {
                "median": statistics.median(exposures_us),
                "p95": percentile(exposures_us, 0.95),
            }
            if exposures_us
            else None,
        },
    }
    return {**data, "probe_id": canonical_id(data)}


def validate_result(result: Any) -> dict[str, Any]:
    if not isinstance(result, dict):
        raise ValueError("time-offset probe root must be an object")
    if result.get("schema_version") != 1 or result.get("probe_type") != PROBE_TYPE:
        raise ValueError("unsupported time-offset probe schema/type")

    payload = {key: value for key, value in result.items() if key != "probe_id"}
    if result.get("probe_id") != canonical_id(payload):
        raise ValueError("time-offset probe_id does not match canonical SHA256")

    boot_id = str(result.get("boot_id", "")).strip()
    d435i = result.get("d435i")
    measurement = result.get("measurement")
    if not boot_id or not isinstance(d435i, dict) or not isinstance(measurement, dict):
        raise ValueError("time-offset probe identity/measurement is incomplete")
    serial = str(d435i.get("serial", "")).strip()
    color = d435i.get("color")
    if not serial or not isinstance(color, dict):
        raise ValueError("time-offset probe D435I identity/profile is incomplete")
    profile = (int(color.get("width", 0)), int(color.get("height", 0)), int(color.get("fps", 0)))
    if any(value <= 0 for value in profile):
        raise ValueError("time-offset probe color profile is invalid")

    if measurement.get("timestamp_domain") != "global_time":
        raise ValueError("RealSense frame timestamp domain must be global_time")
    sample_count = int(measurement.get("sample_count", 0))
    if sample_count < MIN_SAMPLES:
        raise ValueError(f"time-offset probe requires at least {MIN_SAMPLES} samples")
    offset_s = float(measurement.get("recommended_img_time_offset_s"))
    residual_ms = float(measurement.get("p95_abs_residual_ms"))
    if not math.isfinite(offset_s) or abs(offset_s) > MAX_OFFSET_ABS_S:
        raise ValueError("recommended img_time_offset is outside +/-0.5 s")
    if not math.isfinite(residual_ms) or not 0.0 <= residual_ms <= MAX_P95_RESIDUAL_MS:
        raise ValueError("callback latency residual p95 exceeds 20 ms")

    return {
        "probe_id": result["probe_id"],
        "boot_id": boot_id,
        "serial": serial,
        "profile": profile,
        "offset_s": offset_s,
        "p95_residual_ms": residual_ms,
        "sample_count": sample_count,
    }


def capture(args: argparse.Namespace) -> dict[str, Any]:
    import pyrealsense2 as rs

    context = rs.context()
    devices = context.query_devices()
    if len(devices) != 1:
        raise RuntimeError(f"expected exactly one RealSense device, found {len(devices)}")
    device = devices[0]
    serial = device.get_info(rs.camera_info.serial_number)

    color_sensor = next(
        sensor
        for sensor in device.query_sensors()
        if sensor.get_info(rs.camera_info.name) == "RGB Camera"
    )
    if not color_sensor.supports(rs.option.global_time_enabled):
        raise RuntimeError("D435I RGB sensor does not support global_time_enabled")
    color_sensor.set_option(rs.option.global_time_enabled, 1.0)

    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )

    latencies_ms: list[float] = []
    exposures_us: list[float] = []
    timestamp_domains: set[str] = set()
    pipeline.start(config)
    try:
        total = args.warmup + args.samples
        for index in range(total):
            frameset = pipeline.wait_for_frames(5000)
            arrival_epoch_ns = time.time_ns()
            color_frame = frameset.get_color_frame()
            if not color_frame:
                raise RuntimeError("frameset did not contain a color frame")
            if index < args.warmup:
                continue

            domain = str(color_frame.get_frame_timestamp_domain()).split(".")[-1]
            timestamp_domains.add(domain)
            capture_epoch_ns = int(round(float(color_frame.get_timestamp()) * 1_000_000.0))
            latency_ms = (arrival_epoch_ns - capture_epoch_ns) / 1_000_000.0
            if not -5.0 <= latency_ms <= 500.0:
                raise RuntimeError(
                    "RealSense GLOBAL_TIME is not comparable with host CLOCK_REALTIME: "
                    f"latency_ms={latency_ms:.3f}"
                )
            latencies_ms.append(latency_ms)
            try:
                if color_frame.supports_frame_metadata(
                    rs.frame_metadata_value.actual_exposure
                ):
                    exposures_us.append(
                        float(
                            color_frame.get_frame_metadata(
                                rs.frame_metadata_value.actual_exposure
                            )
                        )
                    )
            except RuntimeError:
                pass
    finally:
        pipeline.stop()

    if timestamp_domains != {"global_time"}:
        raise RuntimeError(
            "unexpected RealSense timestamp domains: " + ",".join(sorted(timestamp_domains))
        )
    return build_result(
        serial=serial,
        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip(),
        width=args.width,
        height=args.height,
        fps=args.fps,
        timestamp_domain="global_time",
        latencies_ms=latencies_ms,
        exposures_us=exposures_us,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--validate", type=Path)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--warmup", type=int, default=30)
    args = parser.parse_args()

    try:
        if args.validate is not None:
            with args.validate.open(encoding="utf-8") as handle:
                result = json.load(handle)
            summary = validate_result(result)
            print(
                "rgb_time_probe=PASS "
                f"serial={summary['serial']} "
                f"offset_s={summary['offset_s']:.9f} "
                f"p95_residual_ms={summary['p95_residual_ms']:.3f} "
                f"samples={summary['sample_count']}"
            )
            return 0
        if args.samples < MIN_SAMPLES:
            raise ValueError(f"--samples must be at least {MIN_SAMPLES}")
        if args.warmup < 1:
            raise ValueError("--warmup must be positive")
        result = capture(args)
        validate_result(result)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        return 0
    except (
        OSError,
        RuntimeError,
        StopIteration,
        TypeError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        print(f"rgb_time_probe=FAIL error={error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
