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
        self._result_failures = 0
        self._closed = False
        self._thread = threading.Thread(target=self._collect, name="lynx-s10-media-results", daemon=True)
        self._thread.start()

    def add_camera(self, lane):
        self._add(lane, "camera")

    def add_compressed_camera(self, lane):
        self._add(lane, "compressed_camera")

    def add_lidar(self, lane):
        self._add(lane, "lidar")

    def _add(self, lane, kind):
        jobs = self._ctx.Queue(maxsize=2)
        # Allow conversion and publication to overlap by a few frames while
        # retaining a bounded queue; old frames are still replaced by latest.
        results = self._ctx.Queue(maxsize=4)
        process = self._ctx.Process(target=_worker, args=(jobs, results, kind, lane),
                                    name=f"lynx-s10-media-{lane}", daemon=True)
        process.start()
        self._jobs[lane] = jobs
        self._results[lane] = results
        self._processes[lane] = process

    def submit(self, lane, job):
        jobs = self._jobs.get(lane)
        if jobs is None or self._closed:
            return False
        return _put_latest(jobs, job)

    def is_alive(self, lane):
        process = self._processes.get(lane)
        return bool(process and process.is_alive())

    def _collect(self):
        while not self._closed:
            found = False
            for results in tuple(self._results.values()):
                try:
                    result = results.get_nowait()
                except queue.Empty:
                    continue
                found = True
                self._handle_result(result)
            if not found:
                threading.Event().wait(0.01)

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
        self._thread.join(timeout=1)
        self._jobs.clear()
        self._results.clear()
        self._processes.clear()


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
    mode, raw_mode = {"rgb8": ("RGB", "RGB"), "bgr8": ("RGB", "BGR"),
                      "rgba8": ("RGBA", "RGBA"), "bgra8": ("RGBA", "BGRA"),
                      "mono8": ("L", "L"), "8uc1": ("L", "L")}.get(str(encoding).lower(), (None, None))
    if mode is None:
        return None
    image = Image.frombytes(mode, (int(width), int(height)), bytes(data), "raw", raw_mode, int(step), 1)
    if image.mode == "RGBA":
        image = image.convert("RGB")
    stream = BytesIO()
    image.save(stream, format="JPEG", quality=85, optimize=False)
    return stream.getvalue()


def _reencode_jpeg(data):
    # ROS compressed-image transport already carries a JPEG payload. Avoid
    # decoding and re-encoding every frame; preserve it byte-for-byte.
    payload = bytes(data)
    if payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9"):
        return payload
    from io import BytesIO
    from PIL import Image
    image = Image.open(BytesIO(payload)).convert("RGB")
    stream = BytesIO()
    image.save(stream, format="JPEG", quality=85, optimize=False)
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
            packed.extend(struct.pack("<fff", *point))
            count += 1
    struct.pack_into("<I", packed, 4, count)
    return bytes(packed)
