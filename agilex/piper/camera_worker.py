"""Isolated V4L2 reader with a bounded, latest-frame shared-memory mailbox."""
from dataclasses import dataclass
from datetime import datetime
import multiprocessing as mp
import struct
import time
from zoneinfo import ZoneInfo


MAX_JPEG_BYTES = 4 * 1024 * 1024


@dataclass
class Mailbox:
    lock: object
    values: object
    image: object
    error: object

    @classmethod
    def create(cls, context):
        # sequence, wall timestamp, width, height, capture FPS, JPEG length
        return cls(context.Lock(), context.RawArray('d', 6),
                   context.RawArray('B', MAX_JPEG_BYTES), context.RawArray('B', 1024))

    def write(self, jpeg, stamp, width, height, capture_fps):
        if not jpeg or len(jpeg) > MAX_JPEG_BYTES:
            raise ValueError('JPEG frame exceeds mailbox capacity')
        with self.lock:
            memoryview(self.image).cast('B')[:len(jpeg)] = jpeg
            self.values[1:] = (stamp, width, height, capture_fps, len(jpeg))
            self.values[0] += 1

    def fail(self, error):
        message = str(error).replace('\n', ' ').encode('utf-8')[:1020]
        with self.lock:
            self.error[:] = message + bytes(len(self.error) - len(message))

    def read(self):
        # Never block behind a camera worker, including one killed while writing.
        if not self.lock.acquire(False):
            return None, None
        try:
            sequence, stamp, width, height, fps, length = self.values[:]
            error = bytes(self.error[:]).split(b'\0', 1)[0].decode('utf-8', errors='replace')
            frame = None
            if sequence and 0 < length <= MAX_JPEG_BYTES:
                frame = dict(sequence=int(sequence), timestamp=stamp, width=int(width), height=int(height),
                             capture_fps=fps, jpeg=bytes(memoryview(self.image).cast('B')[:int(length)]))
            return frame, error or None
        finally:
            self.lock.release()


def encode_frame(frame, stamp, quality):
    import cv2
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise RuntimeError('没有收到三通道彩色画面')
    image = frame.copy()
    label = datetime.fromtimestamp(stamp, ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    label += ' +08 LIVE'
    cv2.rectangle(image, (0, 0), (image.shape[1], 29), (0, 0, 0), -1)
    cv2.putText(image, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('相机画面 JPEG 编码失败')
    # Standard JPEG APP15 metadata lets read-only acceptance measure the age
    # of the exact frame received through Core, without OCR or another topic.
    payload = b'PIPER_LIVE_V1 ' + struct.pack('>d', stamp)
    encoded = jpeg.tobytes()
    return encoded[:2] + b'\xff\xef' + struct.pack('>H', len(payload) + 2) + payload + encoded[2:]


def capture_worker(settings, mailbox, stop):
    from common import logsafe
    logsafe.install(check_fd=False)
    import cv2
    cap = None
    try:
        cap = cv2.VideoCapture(settings['device'], cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError('无法打开绑定的彩色相机')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings['height'])
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        started = window = time.monotonic()
        next_frame = started + settings['warmup_seconds']
        frames = 0
        capture_fps = 0.
        while not stop.is_set():
            ok, frame = cap.read()  # A USB stall is terminated by the parent watchdog.
            now = time.monotonic()
            if not ok:
                raise RuntimeError('相机读取失败，请检查彩色接口或 USB 连接')
            frames += 1
            if now - window >= 1:
                capture_fps = frames / (now - window)
                window, frames = now, 0
            if now < next_frame:
                continue
            stamp = time.time()
            jpeg = encode_frame(frame, stamp, settings['jpeg_quality'])
            if stop.is_set():
                break
            mailbox.write(jpeg, stamp, frame.shape[1], frame.shape[0], capture_fps)
            next_frame = now + 1 / settings['fps']
    except Exception as exc:
        mailbox.fail(exc)
    finally:
        if cap is not None:
            cap.release()


class CameraSession:
    def __init__(self, settings, *, worker_target=capture_worker):
        context = mp.get_context('spawn')
        self.mailbox = Mailbox.create(context)
        self.stop_event = context.Event()
        self.process = context.Process(target=worker_target, args=(settings, self.mailbox, self.stop_event),
                                       daemon=True, name='piper-color-camera')
        self.process.start()

    def read(self):
        frame, error = self.mailbox.read()
        if not error and not self.process.is_alive():
            error = '相机采集进程已退出'
        return frame, error

    def close(self):
        self.stop_event.set()
        self.process.join(.5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(1)
        return not self.process.is_alive()

    def dispose(self):
        self.process.close()
