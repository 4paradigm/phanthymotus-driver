"""Process-isolated point-cloud packing and RTSP camera pulling for S10 media."""

from __future__ import annotations

from contextlib import contextmanager
from functools import partial
import multiprocessing as mp
import queue
import subprocess
import threading
import time

from deep_robotics.s10_pro.undistortion import raw_command


def _rtsp_command(url):
    # Select one actual decoded frame per 15 Hz time bin. The fps filter fills
    # missing timestamps with copies of the previous frame.
    select = "select='isnan(prev_selected_t)+not(eq(floor(t*15),floor(prev_selected_t*15)))'"
    return ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay",
            "-i", str(url), "-map", "0:v:0", "-vf", select, "-vsync", "0",
            "-c:v", "wrapped_avframe", "-pix_fmt", "yuv420p", "-f", "fifo",
            "-fifo_format", "yuv4mpegpipe", "-queue_size", "2", "-drop_pkts_on_overflow", "1",
            "-format_opts", "flush_packets=1", "pipe:1"]


@contextmanager
def _camera_process(command, stdin, report_error):
    process = subprocess.Popen(command, stdin=stdin, stdout=subprocess.PIPE, bufsize=0,
                               stderr=subprocess.PIPE if report_error else subprocess.DEVNULL)
    reader = None
    try:
        if report_error:
            def drain_errors():
                for line in process.stderr:
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        report_error(text)
            reader = threading.Thread(target=drain_errors, daemon=True)
            reader.start()
        yield process
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()
        if reader:
            reader.join()
            process.stderr.close()


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
        for process in self._rtsp_processes.get(lane, ()):
            if process.poll() is None:
                process.terminate()

    def _rtsp_loop(self, lane, url):
        while not self._closed:
            try:
                controller = getattr(self, "camera_undistortion", None)
                generation = controller.generation(lane) if controller else None
                command = controller.converter_command(lane) if controller else raw_command()
                report = partial(controller.record_error, lane, generation) if controller else None

                # Only decoded frames may be discarded: the RTSP reader must
                # keep consuming H.264 reference frames even when remap is slow.
                with _camera_process(_rtsp_command(url), None, report) as capture:
                    with _camera_process(command, capture.stdout, report) as converter:
                        capture.stdout.close()
                        self._rtsp_processes[lane] = (capture, converter)
                        if self._closed or (controller and generation != controller.generation(lane)):
                            self.restart_rtsp_camera(lane)
                        else:
                            self._read_mjpeg(lane, converter.stdout, generation)
                if not self._closed:
                    for stage, process in (("capture", capture), ("converter", converter)):
                        if process.returncode not in (0, -15):
                            print(f"[rtsp] {lane} {stage} exited with {process.returncode}; reconnecting",
                                  flush=True)
            except (OSError, ValueError) as exc:
                print(f"[rtsp] {lane} pull failed: {exc}", flush=True)
            finally:
                self._rtsp_processes.pop(lane, None)
            if not self._closed:
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
        processes = self._rtsp_processes.get(lane)
        thread = self._rtsp_threads.get(lane)
        return bool(thread and thread.is_alive() and processes
                    and all(p.poll() is None for p in processes))

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
        for lane in list(self._rtsp_processes):
            self.restart_rtsp_camera(lane)
        for lane, thread in self._rtsp_threads.items():
            thread.join(timeout=2)
            if thread.is_alive():
                for process in self._rtsp_processes.get(lane, ()):
                    if process.poll() is None:
                        process.kill()
                thread.join(timeout=1)
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
