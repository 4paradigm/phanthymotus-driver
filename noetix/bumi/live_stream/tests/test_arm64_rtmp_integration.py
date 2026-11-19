"""Opt-in native librtmp integration: isolated local RTMP only, no public streaming."""
import os
import platform
import socket
import subprocess
import threading
import time

import pytest

from live_stream.transport import FLVFramer, IsolatedRTMPPublisher, RTMPTransportError

pytestmark = pytest.mark.skipif(
    os.environ.get("BUMI_LOCAL_RTMP_TEST") != "1" or platform.system() != "Linux",
    reason="Opt-in local-RTMP integration, run on Bumi Linux ARM64",
)


def _generate_flv():
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "lavfi", "-i", "testsrc=size=160x120:rate=15",
        "-f", "lavfi", "-i", "sine=frequency=500:sample_rate=16000",
        "-t", "2", "-c:v", "libx264", "-preset", "ultrafast",
        "-pix_fmt", "yuv420p", "-c:a", "aac",
        "-ar", "44100", "-f", "flv", "pipe:1",
    ]
    return subprocess.check_output(command, timeout=16)


def _free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_native_librtmp_publishes_real_flv_to_local_ffmpeg_server():
    records = FLVFramer().feed(_generate_flv())
    assert len(records) > 5
    port = _free_port()
    url = f"rtmp://127.0.0.1:{port}/live/armtest"
    server = subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        "-listen", "1", "-i", url,
        "-c", "copy", "-f", "null", "-",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    publisher = IsolatedRTMPPublisher(url)
    try:
        # Give the server time to enter its listen loop.
        time.sleep(0.5)
        publisher.connect()
        for record in records:
            publisher.send(record)
        assert server.poll() is None
    finally:
        publisher.close()
        server.terminate()
        try:
            server.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            server.kill()
            server.communicate(timeout=3)


def test_native_librtmp_blocked_handshake_can_be_aborted():
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(6)
        port = listener.getsockname()[1]
        publisher = IsolatedRTMPPublisher(f"rtmp://127.0.0.1:{port}/live/armtest")
        outcome = []
        def connect():
            try:
                publisher.connect()
            except RTMPTransportError as error:
                outcome.append(str(error))
        worker = threading.Thread(target=connect)
        worker.start()
        conn, _ = listener.accept()
        try:
            started = time.monotonic()
            publisher.abort()
            worker.join(timeout=3)
            assert time.monotonic() - started < 3
            assert not worker.is_alive()
            assert outcome and outcome[0] in ("RTMP_CANCELLED", "RTMP_PROCESS_EXITED")
        finally:
            conn.close()
            publisher.close()
