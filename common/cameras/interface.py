"""Backend-only acquisition contract; no MCP, ROS publication or SDK imports.

Descriptors/modes are JSON-compatible dictionaries. SDK imports belong inside
backend discover/open, invoked only in isolated child processes. Frame arrays
must remain valid until the caller completes encoding (copy borrowed buffers).
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
from typing import Any, Protocol

CameraDevice = dict[str, Any]
CaptureMode = dict[str, dict[str, Any]]


class BackendUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class CameraFrame:
    # RGB: HWC uint8 BGR. Depth: HW uint16/float32 with explicit metres/unit.
    data: Any
    intrinsics: dict | None = None
    depth_scale_m: float | None = None
    timestamp_ms: float | None = None
    timestamp_domain: str = 'unknown'


FrameBatch = dict[str, CameraFrame]


@dataclass(frozen=True)
class ActiveCameraInfo:
    profiles: CaptureMode
    device_name: str
    depth_scale_m: float | None = None


class CameraCapture(Protocol):
    def info(self) -> ActiveCameraInfo: ...
    def read(self, timeout_ms: int) -> FrameBatch | None: ...
    def close(self) -> None: ...


class CameraBackend(Protocol):
    id: str
    def discover(self) -> list[CameraDevice]: ...
    def open(self, device: CameraDevice, requested_mode: CaptureMode) -> CameraCapture: ...


def physical_usb_id(path):
    """Match SDK/V4L2 sysfs paths only when a USB device node is known."""
    devices = [p for p in Path(os.path.realpath(path)).parts
               if re.fullmatch(r'\d+-\d+(?:\.\d+)*', p)]
    return f'usb:{devices[-1]}' if devices else None



def validate_device(device, backend_id):
    """Reject broken adapters at their discovery boundary, not in tools/list."""
    if not isinstance(device, dict):
        raise ValueError('Camera descriptor must be an object')
    if device.get('backend') != backend_id or any(
            not isinstance(device.get(key), str) or not device[key] for key in ('id', 'name')):
        raise ValueError('Camera identity/name/backend is invalid')
    physical = device.get('physical_id')
    if physical is not None and (not isinstance(physical, str) or not physical):
        raise ValueError('physical_id must be a nonempty string or null')
    capabilities, profiles = device.get('capabilities'), device.get('profiles')
    if not isinstance(capabilities, dict) or not isinstance(profiles, dict):
        raise ValueError('Camera capabilities and profiles are required objects')
    for channel in ('rgb', 'depth'):
        modes = profiles.get(channel, [])
        if not isinstance(capabilities.get(channel), bool) or not isinstance(modes, list):
            raise ValueError(f'Invalid {channel} capability/profile list')
        if capabilities[channel] != bool(modes):
            raise ValueError(f'{channel} capability does not match its profiles')
        for mode in modes:
            if not isinstance(mode, dict) or any(
                    type(mode.get(key)) is not int or mode[key] <= 0 for key in ('width', 'height')):
                raise ValueError(f'Invalid native {channel} dimensions')
            fps = mode.get('fps')
            if type(fps) not in (int, float) or not math.isfinite(fps) or fps <= 0:
                raise ValueError(f'Invalid native {channel} frame rate')
            if not isinstance(mode.get('format'), str) or not mode['format']:
                raise ValueError(f'Invalid native {channel} format')
