"""Process-isolated camera encoding and point-cloud packing for S10 media."""

from __future__ import annotations

import multiprocessing as mp
import queue
import threading


class MediaWorkers:
    """Run expensive media conversion outside the robot ROS executor."""

    def __init__(self, on_result):
        self._ctx = mp.get_context("spawn")
        self._on_result = on_result
        self._results = {}
        self._jobs = {}
        self._processes = {}
        self._collector_threads = {}
        self._result_failures = 0
        self._closed = False

    def add_camera(self, lane):
        self._add(lane, "camera")

    def add_compressed_camera(self, lane):
        self._add(lane, "compressed_camera")

    def add_lidar(self, lane):
        self._add(lane, "lidar")

    def _add(self, lane, kind):
        # Media is a live view: never queue historical frames.  A single slot
        # plus _put_latest keeps conversion and publication non-blocking while
        # bounding end-to-end latency to roughly one frame.
        jobs = self._ctx.Queue(maxsize=1)
        results = self._ctx.Queue(maxsize=1)
        process = self._ctx.Process(target=_worker, args=(jobs, results, kind, lane),
                                    name=f"s10-pro-media-{lane}", daemon=True)
        process.start()
        self._jobs[lane] = jobs
        self._results[lane] = results
        self._processes[lane] = process
        collector = threading.Thread(target=self._collect_lane, args=(lane, results),
                                     name=f"s10-pro-media-results-{lane}", daemon=True)
        self._collector_threads[lane] = collector
        collector.start()

    def submit(self, lane, job):
        jobs = self._jobs.get(lane)
        if jobs is None or self._closed:
            return False
        return _put_latest(jobs, job)

    def is_alive(self, lane):
        process = self._processes.get(lane)
        return bool(process and process.is_alive())

    def _collect_lane(self, lane, results):
        while not self._closed:
            try:
                result = results.get(timeout=0.05)
            except queue.Empty:
                continue
            # Drain superseded results for this lane before publishing. A slow
            # lane can no longer block the result collector for other cameras.
            while True:
                try:
                    result = results.get_nowait()
                except queue.Empty:
                    break
            self._handle_result(result)

    def _handle_result(self, result):
        try:
            self._on_result(*result)
            self._result_failures = 0
        except Exception as exc:
            self._result_failures += 1
            count = self._result_failures
            if count == 1 or count % 100 == 0:
                print(f"[media-worker] result handling failed count={count}: {exc}", flush=True)

    def close(self):
        self._closed = True
        for jobs in self._jobs.values():
            try:
                jobs.put_nowait(None)
            except queue.Full:
                try:
                    jobs.get_nowait()
                    jobs.put_nowait(None)
                except queue.Empty:
                    pass
        for process in self._processes.values():
            process.join(timeout=2)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)
        for collector in self._collector_threads.values():
            collector.join(timeout=1)
        self._jobs.clear()
        self._results.clear()
        self._processes.clear()
        self._collector_threads.clear()


def _put_latest(target, item):
    """Enqueue a live frame, replacing an older queued frame when full."""
    try:
        target.put_nowait(item)
        return True
    except queue.Full:
        try:
            target.get_nowait()
        except queue.Empty:
            return False
        try:
            target.put_nowait(item)
            return True
        except queue.Full:
            return False


def _worker(jobs, results, kind, lane):
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError:
        pass
    failures = 0
    while True:
        job = jobs.get()
        if job is None:
            return
        try:
            if kind in ("camera", "compressed_camera"):
                if len(job) == 2:
                    header, data = job
                    packet = _reencode_jpeg(data)
                else:
                    header, width, height, step, encoding, data = job
                    packet = _jpeg(width, height, step, encoding, data)
            else:
                header, fields, bigendian, width, height, row_step, point_step, data = job
                packet = encode_pointcloud(fields, bigendian, width, height, row_step,
                                           point_step, data)
            # Each lane has its own one-frame result queue, so camera traffic
            # cannot evict a newer lidar frame (or vice versa).
            _put_latest(results, (lane, header, packet))
        except Exception as exc:
            failures += 1
            if failures == 1 or failures % 100 == 0:
                print(f"[media-worker] {lane} failed ({failures}): {exc!r}", flush=True)


def _jpeg(width, height, step, encoding, data):
    from io import BytesIO
    from PIL import Image
    encoding = str(encoding).lower()
    if encoding in ("jpeg", "jpg", "mjpeg"):
        payload = bytes(data)
        try:
            image = Image.open(BytesIO(payload)).convert("RGB")
        except (OSError, ValueError):
            return None
        stream = BytesIO()
        _save_preview(image, stream)
        return stream.getvalue()
    if encoding in ("nv12", "nv21"):
        try:
            width, height, step = int(width), int(height), int(step)
            raw = bytes(data)
            y_size = step * height
            uv_rows = (height + 1) // 2
            if width <= 0 or height <= 0 or step < width or len(raw) < y_size + step * uv_rows:
                return None
            # Keep this conversion dependency-free. The camera driver commonly
            # publishes NV12/NV21, and adding NumPy just for this path adds a
            # large wheel to the runtime image.
            rgb = bytearray(width * height * 3)
            out = 0
            for y in range(height):
                y_row = y * step
                uv_row = y_size + (y // 2) * step
                for x in range(width):
                    chroma = uv_row + (x // 2) * 2
                    u, v = raw[chroma], raw[chroma + 1]
                    if encoding == "nv21":
                        u, v = v, u
                    luminance = raw[y_row + x]
                    c = luminance - 16
                    d, e = u - 128, v - 128
                    rgb[out] = max(0, min(255, (298 * c + 409 * e + 128) >> 8))
                    rgb[out + 1] = max(0, min(255, (298 * c - 100 * d - 208 * e + 128) >> 8))
                    rgb[out + 2] = max(0, min(255, (298 * c + 516 * d + 128) >> 8))
                    out += 3
            image = Image.frombytes("RGB", (width, height), bytes(rgb))
        except (IndexError, ValueError, TypeError):
            return None
        stream = BytesIO()
        _save_preview(image, stream)
        return stream.getvalue()
    mode, raw_mode = {"rgb8": ("RGB", "RGB"), "bgr8": ("RGB", "BGR"),
                      "rgba8": ("RGBA", "RGBA"), "bgra8": ("RGBA", "BGRA"),
                      "mono8": ("L", "L"), "8uc1": ("L", "L")}.get(encoding, (None, None))
    if mode is None:
        return None
    image = Image.frombytes(mode, (int(width), int(height)), bytes(data), "raw", raw_mode, int(step), 1)
    if image.mode == "RGBA":
        image = image.convert("RGB")
    stream = BytesIO()
    _save_preview(image, stream)
    return stream.getvalue()


def _save_preview(image, stream):
    """Encode a bounded dashboard preview instead of queuing full frames."""
    max_dimension = 640
    if max(image.size) > max_dimension:
        scale = max_dimension / float(max(image.size))
        image = image.resize((max(1, int(image.width * scale)),
                              max(1, int(image.height * scale))))
    image.save(stream, format="JPEG", quality=65, optimize=False)


def _reencode_jpeg(data):
    # The vendor sometimes declares an already encoded JPEG as sensor_msgs/Image
    # with encoding=jpeg. Preserve valid payloads byte-for-byte; decoding these
    # non-standard Image samples can produce black frames or corrupt the image.
    payload = bytes(data)
    if payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9"):
        return payload
    from io import BytesIO
    from PIL import Image
    image = Image.open(BytesIO(payload)).convert("RGB")
    stream = BytesIO()
    _save_preview(image, stream)
    return stream.getvalue()


def encode_pointcloud(fields, bigendian, width, height, row_step, point_step, data,
                      max_points=2000):
    """Pack XYZ only as little-endian float32 records for the cloud renderer."""
    import math
    import struct
    point_formats = {7: "f", 8: "d"}  # sensor_msgs/PointField FLOAT32/FLOAT64
    field_specs = {}
    for field in fields:
        if len(field) == 2:
            name, offset = field
            datatype, count = 7, 1
        else:
            name, offset, datatype, count = field[:4]
        if name in ("x", "y", "z"):
            if datatype not in point_formats or int(count) != 1:
                return None
            field_specs[name] = (int(offset), point_formats[datatype])
    if set(field_specs) != {"x", "y", "z"}:
        return None
    width, height = int(width), int(height)
    row_step, point_step = int(row_step), int(point_step)
    data = memoryview(data)
    if width < 0 or height < 0 or point_step <= 0 or row_step < width * point_step:
        return None
    total = width * height
    if total <= 0:
        return struct.pack("<II", 12, 0)
    if len(data) < row_step * height:
        return None
    for offset, fmt in field_specs.values():
        if offset < 0 or offset + struct.calcsize(fmt) > point_step:
            return None
    limit = max(1, int(max_points))
    sample_count = min(total, limit)
    if sample_count == 1:
        indices = [0]
    else:
        indices = [round(i * (total - 1) / (sample_count - 1))
                   for i in range(sample_count)]
    endian = ">" if bigendian else "<"
    packed = bytearray(struct.pack("<II", 12, 0))
    count = 0
    for index in indices:
        row, col = divmod(index, width)
        offset = row * row_step + col * point_step
        try:
            point = tuple(struct.unpack_from(endian + fmt, data, offset + field_offset)[0]
                          for field_offset, fmt in (field_specs[name] for name in ("x", "y", "z")))
        except struct.error:
            continue
        if all(math.isfinite(value) for value in point):
            # S10 reports +Z down relative to the dashboard convention.
            packed.extend(struct.pack("<fff", point[0], point[1], -point[2]))
            count += 1
    struct.pack_into("<I", packed, 4, count)
    return bytes(packed)
