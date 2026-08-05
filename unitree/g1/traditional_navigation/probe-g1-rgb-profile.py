#!/usr/bin/env python3
"""Read the live G1 model and D435i factory profile without starting a stream."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import threading
import time


def main() -> int:
    network_interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    expected_width = int(sys.argv[2]) if len(sys.argv) > 2 else 1920
    expected_height = int(sys.argv[3]) if len(sys.argv) > 3 else 1080
    expected_fps = int(sys.argv[4]) if len(sys.argv) > 4 else 15

    import pyrealsense2 as rs

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, network_interface)
    lowstate: dict[str, int] = {}
    received = threading.Event()

    def on_lowstate(message) -> None:
        lowstate["mode_machine"] = int(message.mode_machine)
        lowstate["mode_pr"] = int(message.mode_pr)
        received.set()

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(on_lowstate, 10)
    if not received.wait(8):
        raise RuntimeError("timed out waiting for rt/lowstate")

    context = rs.context()
    devices = context.query_devices()
    if len(devices) != 1:
        raise RuntimeError(f"expected exactly one RealSense device, found {len(devices)}")
    device = devices[0]
    profiles = [
        profile
        for sensor in device.query_sensors()
        for profile in sensor.get_stream_profiles()
    ]

    def video_profile(stream, width, height, fps, pixel_format):
        return next(
            profile
            for profile in profiles
            if profile.stream_type() == stream
            and profile.as_video_stream_profile().width() == width
            and profile.as_video_stream_profile().height() == height
            and profile.fps() == fps
            and profile.format() == pixel_format
        )

    color = video_profile(
        rs.stream.color,
        expected_width,
        expected_height,
        expected_fps,
        rs.format.bgr8,
    )
    depth = video_profile(rs.stream.depth, 640, 480, 15, rs.format.z16)
    intrinsics = color.as_video_stream_profile().get_intrinsics()
    depth_to_color = depth.get_extrinsics_to(color)
    global_time = [
        {
            "sensor": sensor.get_info(rs.camera_info.name),
            "enabled": bool(sensor.get_option(rs.option.global_time_enabled)),
        }
        for sensor in device.query_sensors()
        if sensor.supports(rs.option.global_time_enabled)
    ]

    print(
        json.dumps(
            {
                "schema_version": 1,
                "captured_at_epoch_ns": time.time_ns(),
                "boot_id": Path("/proc/sys/kernel/random/boot_id")
                .read_text(encoding="utf-8")
                .strip(),
                "mode_machine": lowstate["mode_machine"],
                "mode_pr": lowstate["mode_pr"],
                "network_interface": network_interface,
                "d435i": {
                    "serial": device.get_info(rs.camera_info.serial_number),
                    "model": device.get_info(rs.camera_info.name),
                    "color": {
                        "width": intrinsics.width,
                        "height": intrinsics.height,
                        "fps": color.fps(),
                        "format": "bgr8",
                        "intrinsics": {
                            "fx": intrinsics.fx,
                            "fy": intrinsics.fy,
                            "ppx": intrinsics.ppx,
                            "ppy": intrinsics.ppy,
                            "distortion_model": str(intrinsics.model).removeprefix(
                                "distortion."
                            ),
                            "coeffs": list(intrinsics.coeffs),
                        },
                    },
                    "depth_to_color_optical": {
                        # librealsense exposes this matrix in column-major order.
                        "rotation_column_major": list(depth_to_color.rotation),
                        "translation_m": list(depth_to_color.translation),
                    },
                    "global_time": global_time,
                },
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        # stdout is a pipe through ssh/docker exec.  Flush before native DDS and
        # librealsense destructors run so a fast interpreter teardown cannot
        # leave the caller with a successful-but-empty probe file.
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
