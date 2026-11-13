"""Process-isolated point-cloud packing and RTSP camera pulling for S10 media."""

from __future__ import annotations

import multiprocessing as mp
import queue
import subprocess
import threading
import time


def _rtsp_command(url):
    return ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay",
            "-i", str(url), "-an", "-vf", "fps=15,scale='min(640,iw)':-2",
            "-q:v", "5", "-f", "image2pipe", "-vcodec", "mjpeg", "-threads", "1", "pipe:1"]


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
        self._rtsp_threads = {}
        self._rtsp_processes = {}
        self._rtsp_frame_failures = {}

    def add_lidar(self, lane):
        self._add(lane, "lidar")

    def add_rtsp_camera(self, lane, url):
        thread = threading.Thread(target=self._rtsp_loop, args=(lane, url),
                                  name=f"s10-pro-rtsp-{lane}", daemon=True)
        self._rtsp_threads[lane] = thread
        self._rtsp_frame_failures[lane] = 0
        thread.start()

    def restart_rtsp_camera(self, lane):
        """Reconnect only this lane after its camera settings change."""
        process = self._rtsp_processes.get(lane)
        if process is not None and process.poll() is None:
            process.terminate()

    def _rtsp_loop(self, lane, url):
        while not self._closed:
            process = None
            errors_thread = None
            try:
                controller = getattr(self, "camera_undistortion", None)
                generation = controller.generation(lane) if controller else None
                command = controller.rtsp_command(lane, url) if controller else _rtsp_command(url)
                process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE if controller else subprocess.DEVNULL,
                                            bufsize=0)
                self._rtsp_processes[lane] = process
                if controller:
                    def drain_errors(proc=process, version=generation):
                        # Drain continuously so ffmpeg cannot block on stderr.
                        # Keep diagnostics bounded; camera info exposes the last error.
                        for line in iter(proc.stderr.readline, b""):
                            text = line.decode("utf-8", errors="replace").strip()
                            if text:
                                controller.record_error(lane, version, text)
                    errors_thread = threading.Thread(target=drain_errors, daemon=True,
                                                     name=f"s10-ffmpeg-errors-{lane}")
                    errors_thread.start()
                if controller and generation != controller.generation(lane):
                    process.terminate()
                self._read_mjpeg(lane, process.stdout, generation)
            except (OSError, ValueError) as exc:
                print(f"[rtsp] {lane} pull failed: {exc}", flush=True)
            finally:
                self._rtsp_processes.pop(lane, None)
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1)
                    if process.stdout:
                        process.stdout.close()
                    if errors_thread is not None:
                        errors_thread.join(timeout=1)
                        if process.stderr:
                            process.stderr.close()
            if not self._closed:
                if process is not None and process.returncode not in (None, 0, -15):
                    print(f"[rtsp] {lane} ffmpeg exited with {process.returncode}; reconnecting",
                          flush=True)
                time.sleep(1)

    def _read_mjpeg(self, lane, stream, generation=None):
        buffer = bytearray()
        while not self._closed:
            chunk = stream.read(65536)
            if not chunk:
                return
            buffer.extend(chunk)
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    buffer[:] = buffer[-1:]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start:
                        del buffer[:start]
                    if len(buffer) > 8 * 1024 * 1024:
                        buffer.clear()
                    break
                frame = bytes(buffer[start:end + 2])
                del buffer[:end + 2]
                controller = getattr(self, "camera_undistortion", None)
                if controller and generation != controller.generation(lane):
                    continue
                try:
                    if controller:
                        controller.publish_if_current(lane, generation,
                            lambda: self._on_result(lane, None, frame))
                    else:
                        self._on_result(lane, None, frame)
                except Exception as exc:
                    count = self._rtsp_frame_failures.get(lane, 0) + 1
                    self._rtsp_frame_failures[lane] = count
                    if count == 1 or count % 100 == 0:
                        print(f"[rtsp] {lane} frame publish failed count={count}: {exc}", flush=True)

    def rtsp_alive(self, lane):
        process = self._rtsp_processes.get(lane)
        thread = self._rtsp_threads.get(lane)
        return bool(thread and thread.is_alive() and process and process.poll() is None)

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
        for process in self._rtsp_processes.values():
            if process.poll() is None:
                process.terminate()
        for thread in self._rtsp_threads.values():
            thread.join(timeout=2)
        for lane, process in list(self._rtsp_processes.items()):
            if process.poll() is None:
                process.kill()
                process.wait(timeout=1)
            if process.stdout:
                process.stdout.close()
        self._rtsp_threads.clear()
        self._rtsp_processes.clear()
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
            header, fields, bigendian, width, height, row_step, point_step, data = job
            packet = encode_pointcloud(fields, bigendian, width, height, row_step,
                                       point_step, data)
            _put_latest(results, (lane, header, packet))
        except Exception as exc:
            failures += 1
            if failures == 1 or failures % 100 == 0:
                print(f"[media-worker] {lane} failed ({failures}): {exc!r}", flush=True)


def encode_pointcloud(fields, bigendian, width, height, row_step, point_step, data,
                      max_points=10000):
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
