import random
import shutil
import subprocess
import threading
import time

import pytest

from deep_robotics.s10_pro import media_workers as media


@pytest.fixture
def ffmpeg():
    exe = shutil.which("ffmpeg")
    if not exe:
        pytest.skip("FFmpeg executable required for the camera stream regression")
    return exe


def local_input(command, exe, options):
    # Replace only the RTSP input. Exercise the production filters, muxers and pipes.
    end = command.index("-i") + 2
    return [exe, *command[1:command.index("-rtsp_transport")], *options, *command[end:]]


@pytest.mark.parametrize("rectified", [False, True])
def test_timestamp_gap_does_not_turn_one_camera_frame_into_many(ffmpeg, monkeypatch, tmp_path, rectified):
    from deep_robotics.s10_pro.undistortion import CameraUndistortion

    frames = []
    worker = media.MediaWorkers(lambda lane, stamp, frame: frames.append(frame))
    worker.camera_undistortion = CameraUndistortion(
        ("camera_front",), config_path=tmp_path / "settings", map_dir=tmp_path / "maps")
    worker.camera_undistortion.configure("camera_front", {"undistort": rectified})
    popen = subprocess.Popen
    source = "testsrc2=s=320x180:r=15:d=2.2,select='lt(n,3)+gte(n,30)'"

    def open_local(command, **options):
        if "-rtsp_transport" in command:
            command = local_input(command, ffmpeg, ["-re", "-f", "lavfi", "-i", source])
        return popen(command, **options)

    read = worker._read_mjpeg

    def read_one_session(*args):
        read(*args)
        worker._closed = True

    monkeypatch.setattr(subprocess, "Popen", open_local)
    monkeypatch.setattr(worker, "_read_mjpeg", read_one_session)
    worker._rtsp_loop("camera_front", "rtsp://camera/front")
    # Six real input frames with a two-second gap. A paced output duplicates
    # the third frame to fill the gap. A live view may drop frames under load.
    assert 2 <= len(frames) <= 6
    assert len(set(frames)) == len(frames)


def test_capture_drains_input_while_the_converter_is_blocked(ffmpeg):
    command = local_input(media._rtsp_command("rtsp://camera/front"), ffmpeg,
                          ["-f", "rawvideo", "-pixel_format", "yuv420p", "-video_size",
                           "320x240", "-framerate", "15", "-i", "pipe:0"])
    frame = random.Random(0).randbytes(320 * 240 * 3 // 2)
    fed = threading.Event()
    with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL) as capture:
        def feed():
            try:
                for _ in range(100):
                    capture.stdin.write(frame)
                capture.stdin.flush()
                fed.set()
            except BrokenPipeError:
                pass

        writer = threading.Thread(target=feed)
        writer.start()
        try:
            assert fed.wait(4), "RTSP input remained blocked behind unread video output"
            # Release the slow consumer, then feed distinct live frames. At most
            # the in-flight frame and the small decoded queue may predate release.
            output = bytearray()
            draining = threading.Event()

            def drain():
                output.extend(capture.stdout.readline())  # Y4M header
                output.extend(capture.stdout.read(len(frame) + len(b"FRAME\n")))
                draining.set()
                output.extend(capture.stdout.read())

            reader = threading.Thread(target=drain)
            reader.start()
            try:
                assert draining.wait(3)
                live = bytes([190]) * len(frame)
                for _ in range(8):
                    capture.stdin.write(live)
                    capture.stdin.flush()
                    time.sleep(.05)
                capture.stdin.close()
                assert capture.wait(timeout=4) == 0
            finally:
                capture.kill()
                reader.join(timeout=3)
            frames = output.split(b"FRAME\n")[1:]
            assert frames[-1] == live
            assert sum(f == frame for f in frames) <= 4
        finally:
            capture.kill()
            writer.join(timeout=3)
