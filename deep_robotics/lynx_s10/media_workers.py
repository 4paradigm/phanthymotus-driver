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
        self._results = self._ctx.Queue(maxsize=8)
        self._jobs = {}
        self._processes = {}
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
        process = self._ctx.Process(target=_worker, args=(jobs, self._result_queue, kind, lane),
                                    name=f"lynx-s10-media-{lane}", daemon=True)
        process.start()
        self._jobs[lane] = jobs
        self._processes[lane] = process

    @property
    def _result_queue(self):
        return self._results

    def submit(self, lane, job):
        jobs = self._jobs.get(lane)
        if jobs is None or self._closed:
            return False
        try:
            jobs.put_nowait(job)
            return True
        except queue.Full:
            try:
                jobs.get_nowait()
                jobs.put_nowait(job)
                return False
            except queue.Empty:
                return False

    def _collect(self):
        while not self._closed:
            try:
                result = self._result_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._on_result(*result)
            except Exception as exc:
                print(f"[media-worker] result handling failed: {exc}", flush=True)

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
        self._processes.clear()


def _worker(jobs, results, kind, lane):
    while True:
        job = jobs.get()
        if job is None:
            return
        try:
            if kind == "camera":
                header, width, height, step, encoding, data = job
                packet = _jpeg(width, height, step, encoding, data)
            elif kind == "compressed_camera":
                header, data = job
                packet = _reencode_jpeg(data)
            else:
                header, fields, bigendian, width, height, row_step, point_step, data = job
                packet = _pointcloud(fields, bigendian, width, height, row_step, point_step, data)
            results.put((lane, header, packet), timeout=0.2)
        except Exception as exc:
            print(f"[media-worker] {lane} failed: {exc}", flush=True)


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
    from io import BytesIO
    from PIL import Image
    image = Image.open(BytesIO(bytes(data))).convert("RGB")
    stream = BytesIO()
    image.save(stream, format="JPEG", quality=85, optimize=False)
    return stream.getvalue()


def _pointcloud(fields, bigendian, width, height, row_step, point_step, data):
    import math
    import struct
    offsets = dict(fields)
    if not all(name in offsets for name in ("x", "y", "z")):
        return None
    total = int(width) * int(height)
    if total <= 0:
        return struct.pack("<II", 12, 0)
    stride = max(1, math.ceil(total / 2000))
    endian = ">" if bigendian else "<"
    packed = bytearray(struct.pack("<II", 12, 0))
    count = 0
    for index in range(0, total, stride):
        row, col = divmod(index, int(width))
        offset = row * int(row_step) + col * int(point_step)
        try:
            point = tuple(struct.unpack_from(endian + "f", data, offset + offsets[name])[0]
                          for name in ("x", "y", "z"))
        except struct.error:
            continue
        if all(math.isfinite(value) for value in point):
            packed.extend(struct.pack("<fff", *point))
            count += 1
    struct.pack_into("<I", packed, 4, count)
    return bytes(packed)
