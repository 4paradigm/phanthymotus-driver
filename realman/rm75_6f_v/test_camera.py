#!/usr/bin/env python3
"""Check the upper-computer RealSense RGB-D camera without controlling the arm."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list connected RealSense cameras and exit")
    parser.add_argument("--serial", help="camera serial number (required when multiple cameras are connected)")
    parser.add_argument("--frames", type=int, default=30, help="number of RGB-D frames to check (default: 30)")
    parser.add_argument("--output", type=Path, default=Path("camera_test_output"),
                        help="directory for saved RGB and depth samples")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be at least 1")
    return args


def device_info(device, field, default=""):
    try:
        return str(device.get_info(field)) if device.supports(field) else default
    except RuntimeError:
        return default


def main():
    args = parse_args()
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as exc:
        print(f"Missing camera dependency: {exc}. Install pyrealsense2, numpy and opencv-python-headless.",
              file=sys.stderr)
        return 2

    devices = list(rs.context().query_devices())
    if not devices:
        print("No RealSense camera found. Check USB connection and device permissions.", file=sys.stderr)
        return 1
    for device in devices:
        serial = device_info(device, rs.camera_info.serial_number)
        name = device_info(device, rs.camera_info.name, "RealSense")
        usb = device_info(device, rs.camera_info.usb_type_descriptor, "unknown")
        print(f"Camera: {name}, serial={serial}, USB={usb}")
    if args.list:
        return 0

    matches = [device for device in devices
               if device_info(device, rs.camera_info.serial_number) == args.serial] if args.serial else devices
    if len(matches) != 1:
        print("Select exactly one camera with --serial.", file=sys.stderr)
        return 1
    device = matches[0]
    serial = device_info(device, rs.camera_info.serial_number)
    usb = device_info(device, rs.camera_info.usb_type_descriptor)
    usb3 = usb.startswith("3")
    fps = 15 if usb3 else 6
    color_width, color_height = (1280, 720) if usb3 else (640, 480)
    depth_width, depth_height = 640, 480

    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)
    pipeline = rs.pipeline()
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        if not np.isfinite(scale) or scale <= 0:
            raise RuntimeError(f"Invalid depth scale: {scale}")
        print(f"Streaming RGB {color_width}x{color_height}, depth {depth_width}x{depth_height} "
              f"at {fps} fps; depth scale={scale:.8f} m/unit")

        color = depth = None
        first_time = last_time = None
        for index in range(args.frames):
            frames = pipeline.wait_for_frames(5000)
            color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError(f"Frame {index + 1} has no RGB or depth image")
            color = np.asanyarray(color_frame.get_data()).copy()
            depth = np.asanyarray(depth_frame.get_data()).copy()
            if color.shape != (color_height, color_width, 3) or color.dtype != np.uint8:
                raise RuntimeError(f"Unexpected RGB frame: {color.shape}, {color.dtype}")
            if depth.shape != (depth_height, depth_width) or depth.dtype != np.uint16:
                raise RuntimeError(f"Unexpected depth frame: {depth.shape}, {depth.dtype}")
            now = time.monotonic()
            if first_time is None:
                first_time = now
            last_time = now
            if (index + 1) % max(1, args.frames // 5) == 0 or index + 1 == args.frames:
                print(f"Received {index + 1}/{args.frames} RGB-D pairs")

        elapsed = last_time - first_time
        measured_fps = (args.frames - 1) / elapsed if elapsed > 0 else 0.0
        depth_mm_float = np.rint(depth.astype(np.float64) * scale * 1000)
        valid = (depth > 0) & (depth_mm_float >= 1) & (depth_mm_float <= 65535)
        depth_mm = np.zeros(depth.shape, dtype=np.uint16)
        depth_mm[valid] = depth_mm_float[valid].astype(np.uint16)
        valid_percent = 100 * np.count_nonzero(valid) / valid.size
        center = depth_mm[depth_height // 2 - 10:depth_height // 2 + 11,
                          depth_width // 2 - 10:depth_width // 2 + 11]
        center_valid = center[center > 0]
        center_mm = int(np.median(center_valid)) if center_valid.size else None

        args.output.mkdir(parents=True, exist_ok=True)
        color_path = args.output / "color.jpg"
        depth_path = args.output / "depth_mm.png"
        preview_path = args.output / "depth_preview.png"
        preview_gray = np.clip((depth_mm.astype(np.float32) - 200) / 1800 * 255, 0, 255).astype(np.uint8)
        preview = cv2.applyColorMap(preview_gray, cv2.COLORMAP_TURBO)
        preview[~valid] = 0
        for path, image in ((color_path, color), (depth_path, depth_mm), (preview_path, preview)):
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"Could not save {path}")
        print(f"Measured rate: {measured_fps:.1f} fps; valid depth: {valid_percent:.1f}%; "
              f"center depth: {center_mm if center_mm is not None else 'invalid'} mm")
        print(f"Saved: {color_path}, {depth_path}, {preview_path}")
        print("RGB and raw depth are from separate sensors; these files are not pixel-aligned.")
        return 0
    except RuntimeError as exc:
        print(f"Camera check failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if started:
            pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
