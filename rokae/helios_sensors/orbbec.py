"""Capture Gemini 335Lg sensor frames with the machine's Orbbec SDK 2.9.3."""

from contextlib import ExitStack
import ctypes as C
from functools import lru_cache
import math
import os
import queue
import sys
import time
import zlib
from pathlib import Path

import numpy as np

from common import logsafe

from .orientation import OrientationFilter


WIDTH, HEIGHT, DEPTH_FPS, IMU_HZ = 640, 480, 30, 50
DEPTH_FORMAT = "16UC1; compressedDepth zlib"
# Orbbec SDK 2.9.3 ObTypes.h.
SENSOR_DEPTH, SENSOR_ACCEL, SENSOR_GYRO = 3, 4, 5
FRAME_ACCEL, FRAME_GYRO, FORMAT_Y16, RATE_50_HZ = 4, 7, 8, 6
PTR, ENUM, UINT = C.c_void_p, C.c_int, C.c_uint32


class XYZ(C.Structure):
    _fields_ = [(axis, C.c_float) for axis in "xyz"]

    def values(self):
        return self.x, self.y, self.z


class Intrinsic(C.Structure):
    _fields_ = [(axis, C.c_float) for axis in ("fx", "fy", "cx", "cy")] + [
        ("width", C.c_int16), ("height", C.c_int16)]


class SDK:
    def __init__(self, directory):
        self.lib = C.CDLL(str(directory / "libOrbbecSDK.so.2.9.3"))
        self.lib.ob_error_get_message.argtypes = [PTR]
        self.lib.ob_error_get_message.restype = C.c_char_p
        self.lib.ob_delete_error.argtypes = [PTR]
        self.lib.ob_delete_error.restype = None

    def bind(self, name, result, *arguments):
        fn = getattr(self.lib, "ob_" + name)
        fn.argtypes = [*arguments, C.POINTER(PTR)]
        fn.restype = result

        def call(*values):
            err = PTR()
            value = fn(*values, C.byref(err))
            if err.value:
                message = self.lib.ob_error_get_message(err)
                self.lib.ob_delete_error(err)
                raise RuntimeError(f"{name}: {(message or b'unknown SDK error').decode(errors='replace')}")
            return value

        return call

    def own(self, stack, value, release):
        if not value:
            raise RuntimeError(f"SDK returned a null handle for {release}")
        stack.callback(self.bind(release, None, PTR), value)
        return value

    def device(self, stack, directory, serial):
        create = self.bind("create_context_with_config", PTR, C.c_char_p)
        ctx = self.own(stack, create(os.fsencode(directory / "OrbbecSDKConfig.xml")), "delete_context")
        self.bind("enable_net_device_enumeration", None, PTR, C.c_bool)(ctx, False)
        devices = self.own(stack, self.bind("query_device_list", PTR, PTR)(ctx), "delete_device_list")
        count = self.bind("device_list_get_count", UINT, PTR)(devices)
        name_for = self.bind("device_list_get_device_name", C.c_char_p, PTR, UINT)
        serial_for = self.bind("device_list_get_device_serial_number", C.c_char_p, PTR, UINT)
        for i in range(count):
            if serial_for(devices, i) == serial.encode("ascii"):
                if name_for(devices, i) not in (b"Gemini 335Lg", b"Orbbec Gemini 335Lg"):
                    raise RuntimeError("configured serial is not a Gemini 335Lg")
                return self.own(stack, self.bind("device_list_get_device", PTR, PTR, UINT)(
                    devices, i), "delete_device")
        raise RuntimeError("configured Gemini 335Lg is not available")

    def profile(self, stack, device, kind, sensor_type):
        sensor = self.own(stack, self.bind("device_get_sensor", PTR, PTR, ENUM)(
            device, sensor_type), "delete_sensor")
        profiles = self.own(stack, self.bind("sensor_get_stream_profile_list", PTR, PTR)(
            sensor), "delete_stream_profile_list")
        get_count = self.bind("stream_profile_list_get_count", UINT, PTR)
        get_profile = self.bind("stream_profile_list_get_profile", PTR, PTR, ENUM)
        if kind == "depth":
            fields = [self.bind("video_stream_profile_get_" + name, UINT, PTR)
                      for name in ("width", "height", "fps")]
            field_format = self.bind("stream_profile_get_format", ENUM, PTR)

            def matches(profile):
                return field_format(profile) == FORMAT_Y16 and tuple(
                    field(profile) for field in fields) == (WIDTH, HEIGHT, DEPTH_FPS)
        else:
            rate = self.bind(kind + "_stream_profile_get_sample_rate", ENUM, PTR)

            def matches(profile):
                return rate(profile) == RATE_50_HZ

        for i in range(get_count(profiles)):
            profile = self.own(stack, get_profile(profiles, i), "delete_stream_profile")
            if matches(profile):
                return profile
        raise RuntimeError(f"the selected camera does not support the required {kind} profile")


def encode_depth(raw, scale_mm):
    if len(raw) != WIDTH * HEIGHT * 2 or not math.isfinite(scale_mm) or not 0 < scale_mm <= 100:
        raise ValueError("invalid native depth frame or scale")
    if scale_mm != 1.0 or sys.byteorder != "little":
        pixels = np.frombuffer(raw, dtype=np.uint16)
        if scale_mm != 1.0:
            pixels = depth_scale_table(scale_mm)[pixels]
        raw = pixels.astype("<u2", copy=False).tobytes()
    return zlib.compress(raw, 1)


@lru_cache(maxsize=4)
def depth_scale_table(scale_mm):
    values = np.rint(np.arange(65536, dtype=np.float64) * scale_mm)
    np.clip(values, 1, 65535, out=values)
    values[0] = 0
    return values.astype("<u2")


def get_intrinsic(sdk, profile):
    try:
        i = sdk.bind("video_stream_profile_get_intrinsic", Intrinsic, PTR)(profile)
    except RuntimeError:
        return None
    if (i.width, i.height) == (WIDTH, HEIGHT) and i.fx > 0 and i.fy > 0 and all(
            math.isfinite(value) for value in (i.fx, i.fy, i.cx, i.cy)):
        return [i.fx, 0.0, i.cx, 0.0, i.fy, i.cy, 0.0, 0.0, 1.0]
    return None


def depth_reader(sdk):
    get = sdk.bind("frameset_get_depth_frame", PTR, PTR)
    width = sdk.bind("video_frame_get_width", UINT, PTR)
    height = sdk.bind("video_frame_get_height", UINT, PTR)
    fmt = sdk.bind("frame_get_format", ENUM, PTR)
    size = sdk.bind("frame_get_data_size", UINT, PTR)
    data = sdk.bind("frame_get_data", PTR, PTR)
    scale = sdk.bind("depth_frame_get_value_scale", C.c_float, PTR)
    stamp = sdk.bind("frame_get_timestamp_us", C.c_uint64, PTR)
    count = WIDTH * HEIGHT * 2

    def read(stack, frameset):
        frame = get(frameset)
        if not frame:
            return None
        sdk.own(stack, frame, "delete_frame")
        if (width(frame), height(frame), fmt(frame), size(frame)) != (WIDTH, HEIGHT, FORMAT_Y16, count):
            raise ValueError("unexpected Y16 frame layout")
        address = data(frame)
        if not address:
            raise ValueError("empty depth buffer")
        return stamp(frame), encode_depth(C.string_at(address, count), scale(frame))

    return read


def imu_reader(sdk):
    get = sdk.bind("frameset_get_frame", PTR, PTR, ENUM)
    stamp = sdk.bind("frame_get_timestamp_us", C.c_uint64, PTR)
    accel_values = sdk.bind("accel_frame_get_value", XYZ, PTR)
    gyro_values = sdk.bind("gyro_frame_get_value", XYZ, PTR)
    orientation = OrientationFilter()

    def read(stack, frameset):
        accel, gyro = get(frameset, FRAME_ACCEL), get(frameset, FRAME_GYRO)
        if accel:
            sdk.own(stack, accel, "delete_frame")
        if gyro:
            sdk.own(stack, gyro, "delete_frame")
        if not (accel and gyro):
            return None
        timestamp = stamp(accel)
        if not timestamp or timestamp != stamp(gyro):
            return None
        a, g = accel_values(accel).values(), gyro_values(gyro).values()
        if not all(math.isfinite(value) for value in a + g):
            return None
        # Update from every paired sample before the queue can drop a frame.
        return timestamp, (a, g, orientation.update(timestamp, a, g))

    return read


def capture(kind, directory, serial, stop, output):
    """A child process owns the camera. It returns only data and status."""
    logsafe.install(check_fd=False)
    # If the parent must terminate capture, a full queue must not hang child teardown.
    output.cancel_join_thread()
    try:
        directory = Path(directory)
        sdk = SDK(directory)
        sdk.bind("set_logger_severity", None, ENUM)(6)  # OB_LOG_SEVERITY_OFF
        with ExitStack() as stack:
            device = sdk.device(stack, directory, serial)
            pipe = sdk.own(stack, sdk.bind("create_pipeline_with_device", PTR, PTR)(
                device), "delete_pipeline")
            config = sdk.own(stack, sdk.bind("create_config", PTR)(), "delete_config")
            enable = sdk.bind("config_enable_stream_with_stream_profile", None, PTR, PTR)
            if kind == "depth":
                profile = sdk.profile(stack, device, "depth", SENSOR_DEPTH)
                enable(config, profile)
                geometry = get_intrinsic(sdk, profile)
                read = depth_reader(sdk)
            else:
                enable(config, sdk.profile(stack, device, "accel", SENSOR_ACCEL))
                enable(config, sdk.profile(stack, device, "gyro", SENSOR_GYRO))
                geometry = None
                read = imu_reader(sdk)
            sdk.bind("pipeline_start_with_config", None, PTR, PTR)(pipe, config)
            stack.callback(sdk.bind("pipeline_stop", None, PTR), pipe)
            wait = sdk.bind("pipeline_wait_for_frameset", PTR, PTR, UINT)
            previous = 0
            deadline = time.monotonic() + 6
            while not stop.is_set():
                if time.monotonic() > deadline:
                    raise RuntimeError("no fresh sensor frames")
                with ExitStack() as frames:
                    frame_set = wait(pipe, 500)
                    if not frame_set:
                        continue
                    sdk.own(frames, frame_set, "delete_frame")
                    captured = time.monotonic()
                    sample = read(frames, frame_set)
                    if sample is None or sample[0] <= previous:
                        continue
                    previous, data = sample
                    deadline = time.monotonic() + 6
                    try:
                        output.put_nowait(("frame", captured, previous, data, geometry))
                    except queue.Full:
                        pass
    except (OSError, RuntimeError, ValueError) as exc:
        try:
            output.put(("error", " ".join(str(exc).split())[:300]), timeout=.4)
            # Give the queue feeder a moment to flush before exiting.
            stop.wait(.1)
        except queue.Full:
            pass
