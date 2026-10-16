"""Enumerate RealSense capabilities without opening streams or changing options."""
import json
from pathlib import Path


def inspect_device(rs_module):
    required = ("color", "depth", "gyro", "accel")
    devices = []
    for device in rs_module.context().query_devices():
        profiles = [p for s in device.query_sensors() for p in s.get_stream_profiles()]
        available = {name for name in required
                     if any(p.stream_type() == getattr(rs_module.stream, name) for p in profiles)}
        devices.append(dict(
            name=device.get_info(rs_module.camera_info.name),
            serial=device.get_info(rs_module.camera_info.serial_number),
            firmware=device.get_info(rs_module.camera_info.firmware_version),
            profiles=[dict(stream=str(p.stream_type()), fps=p.fps(), format=str(p.format()),
                           index=p.stream_index()) for p in profiles],
            missing_streams=[s for s in required if s not in available],
        ))
    return dict(devices=devices, ready=any(not d["missing_streams"] for d in devices),
                streaming_verified=False, calibration_verified=False,
                udev_data_visible=Path("/run/udev/data").is_dir())


def main():
    try:
        import importlib.metadata
        import pyrealsense2 as rs
        result = inspect_device(rs)
        result["pyrealsense2_version"] = importlib.metadata.version("pyrealsense2")
    except (ImportError, RuntimeError) as exc:
        result = dict(ready=False, error=str(exc))
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
