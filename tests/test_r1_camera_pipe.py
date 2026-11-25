"""Run the actual R1 JPEG reader against a live, buffered OS pipe."""
import ast
import os
import queue
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def camera_pipe():
    # Importing device.py needs ROS and the SDK. Execute its unmodified nested
    # reader, substituting only the ROS message, clock and publisher boundary.
    path = Path(__file__).resolve().parents[1] / 'unitree/r1/device.py'
    tree = ast.parse(path.read_text())
    camera = next(n for n in tree.body
                  if isinstance(n, ast.FunctionDef) and n.name == '_run_camera_process')
    reader = next(n for n in camera.body
                  if isinstance(n, ast.FunctionDef) and n.name == 'read_frames')
    stamp = object()
    namespace = {
        '_CompressedImage': lambda: SimpleNamespace(header=SimpleNamespace()),
        'node': SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: stamp))),
    }
    exec(compile(ast.Module(body=[reader], type_ignores=[]), str(path), 'exec'), namespace)
    read_fd, write_fd = os.pipe()
    output = os.fdopen(read_fd, 'rb')
    writer = os.fdopen(write_fd, 'wb')
    messages = queue.Queue()
    errors = []
    progress = threading.Condition()
    consumed = 0

    def observe(read):
        def counted(size):
            nonlocal consumed
            data = read(size)
            with progress:
                consumed += len(data)
                progress.notify_all()
            return data
        return counted

    stream = SimpleNamespace(read=observe(output.read), read1=observe(output.read1))

    def run():
        try:
            namespace['read_frames'](SimpleNamespace(stdout=stream, poll=lambda: None),
                                     SimpleNamespace(publish=messages.put), 'left')
        except Exception as exc:
            errors.append(exc)

    worker = threading.Thread(target=run, daemon=True)
    worker.start()

    def send(data):
        writer.write(data)
        writer.flush()

    def receive():
        try:
            message = messages.get(timeout=1)
        except queue.Empty:
            pytest.fail('complete JPEG was held until more pipe bytes or EOF arrived')
        assert message.format == 'jpeg'
        assert message.header.stamp is stamp
        return message.data

    def wait_read(size):
        with progress:
            assert progress.wait_for(lambda: consumed >= size, timeout=1)

    try:
        yield send, receive, wait_read
    finally:
        writer.close()
        worker.join(timeout=2)
        assert not worker.is_alive(), 'camera reader did not exit at pipe EOF'
        output.close()
        assert not errors


def test_complete_small_jpeg_is_published_while_pipe_stays_open(camera_pipe):
    send, receive, _ = camera_pipe
    frame = b'\xff\xd8' + b'x' * 7290 + b'\xff\xd9'
    send(frame)
    assert receive() == frame


def test_split_start_marker_survives_a_read_boundary(camera_pipe):
    send, receive, wait_read = camera_pipe
    # Consume exactly 64 KiB ending in FF before allowing D8 into the pipe.
    # This forces the marker boundary with both the old and the new reader.
    send(b'noise' + b'\x00' * (65536 - len(b'noise') - 1) + b'\xff')
    wait_read(65536)
    frame = b'\xff\xd8payload\xff\xd9'
    send(frame[1:])
    assert receive() == frame


@pytest.mark.parametrize('payload', [b'x' * 20000, b'x' * 70000])
def test_split_end_marker_and_following_frame_keep_bytes_in_order(camera_pipe, payload):
    send, receive, wait_read = camera_pipe
    frame = b'\xff\xd8' + payload + b'\xff\xd9'
    second = b'\xff\xd8next\xff\xd9'
    send(frame[:-1])
    wait_read(len(frame) - 1)
    send(frame[-1:] + second)
    assert receive() == frame
    assert receive() == second
