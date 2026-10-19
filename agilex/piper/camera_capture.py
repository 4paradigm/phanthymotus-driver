#!/usr/bin/env python3
"""Capture one RealSense color frame: V4L2 on Ubuntu, AVFoundation on Mac."""

import argparse
import glob
import json
import platform
import time

import cv2


def capture_color(output, device=None, warmup=5.0, min_contrast=0.0):
    """Save a warmed-up D435 color frame and return capture metadata.

    A single RealSense camera is auto-detected unless ``device`` is provided.
    ``min_contrast`` rejects an obstructed or nearly uniform view.
    """
    if not 0 < warmup <= 10:
        raise ValueError("warmup 必须在 (0, 10] 秒内")
    on_mac = platform.system() == "Darwin"
    if on_mac:
        import AVFoundation
        authorization = AVFoundation.AVCaptureDevice.authorizationStatusForMediaType_(AVFoundation.AVMediaTypeVideo)
        if authorization != AVFoundation.AVAuthorizationStatusAuthorized:
            raise RuntimeError("Mac 尚未允许相机访问，请在系统设置 → 隐私与安全性 → 相机中允许当前终端或应用，再重试")
        devices = list(AVFoundation.AVCaptureDevice.devicesWithMediaType_(AVFoundation.AVMediaTypeVideo))
        matches = [(i,d) for i,d in enumerate(devices) if "realsense" in d.localizedName().lower()]
        if device is not None:
            matches = [(i,d) for i,d in matches if str(i) == str(device) or d.uniqueID() == device]
        if len(matches) != 1:
            raise RuntimeError(f"Mac 找到 {len(matches)} 个匹配的 RealSense 视频设备；不会打开内置摄像头")
        index, camera = matches[0]
        selected = camera.localizedName()
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    else:
        matches = glob.glob("/dev/v4l/by-id/*RealSense*video-index0")
        if device:
            selected = device
        elif len(matches) == 1:
            selected = matches[0]
        else:
            raise RuntimeError(f"找到 {len(matches)} 个候选相机，请指定彩色设备: {matches}")
        cap = cv2.VideoCapture(selected, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 {selected}")
    try:
        if not on_mac:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        deadline = time.monotonic() + warmup
        count = 0
        last = None
        while time.monotonic() < deadline:
            ok, frame = cap.read()
            if ok:
                count += 1
                last = frame
        if last is None:
            raise RuntimeError("等待期间没有收到相机画面")
    finally:
        cap.release()

    contrast = float(last.std(axis=(0, 1)).mean())
    if last.ndim != 3 or last.shape[2] != 3:
        raise RuntimeError("没有收到三通道 RGB 画面，请检查 RealSense 彩色接口")
    if on_mac:
        chroma = float(abs(last[:,:,0].astype("float32")-last[:,:,2]).mean())
        if chroma < 1:
            raise RuntimeError("Mac 画面未确认包含彩色信息，请检查是否只开放了深度/红外接口")
    if contrast < min_contrast:
        raise RuntimeError(f"画面近乎纯色（对比度 {contrast:.1f}），请检查朝向、遮挡或保护膜")
    if not cv2.imwrite(str(output), last):
        raise RuntimeError(f"无法保存 {output}")
    return {"device": selected, "output": str(output), "frames": count,
            "width": last.shape[1], "height": last.shape[0], "contrast": contrast,
            "backend": "avfoundation" if on_mac else "v4l2"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", help="V4L2 彩色设备；只有一台 RealSense 时自动识别")
    parser.add_argument("--output", default="realsense_color.jpg")
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--min-contrast", type=float, default=0.0)
    args = parser.parse_args()
    info = capture_color(args.output, args.device, args.warmup, args.min_contrast)
    print(json.dumps(info, ensure_ascii=False))


if __name__ == "__main__":
    main()
