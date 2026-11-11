"""Run the real wrist capture loop against controlled SDK/ROS boundaries."""
import importlib.util
from pathlib import Path
import queue
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SPEC = importlib.util.spec_from_file_location(
    "tianyi_wrist_worker_under_test", ROOT / "x-humanoid/tianyi2.0/wrist_cameras.py")
wrist = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(wrist)


@pytest.fixture
def capture(monkeypatch, tmp_path):
    import numpy as np
    from common import logsafe

    monkeypatch.setattr(sys, "stdout", sys.stdout)
    monkeypatch.setattr(sys, "stderr", sys.stderr)
    monkeypatch.setattr(logsafe, "_installed", False)
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
                "RCUTILS_COLORIZED_OUTPUT"):
        monkeypatch.setenv(key, wrist.os.environ.get(key, ""))
    profile = tmp_path / "dds.xml"
    profile.write_text("<dds/>")
    monkeypatch.setattr(wrist, "DDS_PROFILE", str(profile))
    s = wrist.WristCameraSession("test_robot", "left", "261922273335")
    s._proc = NS(is_alive=lambda: True)
    s._channels = s._ctx.Value("B", 3)
    s._wanted = {"rgb", "depth"}
    s._requested_at = {name: wrist.time.monotonic() for name in s._wanted}
    s._queue = queue.Queue()
    stop = threading.Event()
    hooks, published = {}, []

    intr = NS(width=640, height=480, fx=600, fy=600, ppx=320, ppy=240,
              coeffs=[0.] * 5, model="none")
    frame_profile = NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr))

    def frame(channel):
        raw = np.zeros((480, 640, 3) if channel == "rgb" else (480, 640),
                       dtype=np.uint8 if channel == "rgb" else np.uint16)

        def data():
            if "encode" in hooks:
                hooks["encode"](channel)
            return raw

        return NS(profile=frame_profile, get_data=data, get_timestamp=lambda: 1.0)

    frames = NS(get_color_frame=lambda: frame("rgb"), get_depth_frame=lambda: frame("depth"))
    empty = NS(get_color_frame=lambda: None, get_depth_frame=lambda: None)
    device = NS(get_info=lambda key: "261922273335" if key == "serial" else "Intel D405",
                first_depth_sensor=lambda: NS(get_depth_scale=lambda: .001))

    class Pipeline:
        calls = 0

        def start(self, config):
            return NS(get_device=lambda: device)

        def wait_for_frames(self, timeout_ms):
            self.calls += 1
            if self.calls == 1:
                if "wait" in hooks:
                    hooks["wait"]()
                return frames
            stop.set()
            return empty

        def stop(self):
            pass

    rs = ModuleType("pyrealsense2")
    rs.camera_info = NS(serial_number="serial", name="name")
    rs.stream = NS(color="color", depth="depth")
    rs.format = NS(rgb8="rgb8", z16="z16")
    rs.context = lambda: NS(query_devices=lambda: [device])
    rs.config = lambda: NS(enable_device=lambda *args: None, enable_stream=lambda *args: None)
    rs.pipeline = Pipeline
    monkeypatch.setitem(sys.modules, "pyrealsense2", rs)

    class Publisher:
        def __init__(self, topic):
            self.channel = topic.rsplit("/", 1)[-1]

        def publish(self, msg):
            if "publish" in hooks:
                hooks["publish"](self.channel)
            published.append(self.channel)

    node = NS(create_publisher=lambda cls, topic, qos: Publisher(topic),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS())),
              destroy_node=lambda: None)
    ros = ModuleType("rclpy")
    ros.init = lambda **kwargs: None
    ros.create_node = lambda name: node
    ros.shutdown = lambda: None
    qos = ModuleType("rclpy.qos")
    qos.qos_profile_sensor_data = object()
    messages = ModuleType("sensor_msgs.msg")
    messages.CompressedImage = lambda: NS(header=NS())
    for name, mod in (("rclpy", ros), ("rclpy.qos", qos),
                      ("sensor_msgs", ModuleType("sensor_msgs")), ("sensor_msgs.msg", messages)):
        monkeypatch.setitem(sys.modules, name, mod)

    def run():
        wrist._capture_worker(s.namespace, s.side, s.serial, 85, s._channels, stop, s._queue)

    return NS(session=s, hooks=hooks, published=published, run=run)


@pytest.mark.parametrize("channel", ["rgb", "depth"])
@pytest.mark.parametrize("phase", ["wait", "encode"])
def test_stop_during_wait_or_encoding_does_not_publish_stopped_channel(capture, channel, phase):
    if phase == "wait":
        capture.hooks["wait"] = lambda: capture.session.stop(channel)
    else:
        capture.hooks["encode"] = lambda current: capture.session.stop(channel) if current == channel else None
    capture.run()
    sibling = "depth" if channel == "rgb" else "rgb"
    assert capture.published == [sibling]


def test_start_during_wait_uses_the_next_available_frame(capture):
    capture.session._channels.value = 1
    capture.session._wanted = {"rgb"}
    capture.hooks["wait"] = lambda: capture.session.start("depth")
    capture.run()
    assert capture.published == ["rgb", "depth"]


@pytest.mark.parametrize("channel", ["rgb", "depth"])
def test_stop_waits_for_an_inflight_publish(capture, channel):
    entered, release, stop_started, stopped = (threading.Event() for _ in range(4))

    def publishing(current):
        if current == channel:
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test did not release publisher")

    def stopping():
        stop_started.set()
        capture.session.stop(channel)
        stopped.set()

    capture.hooks["publish"] = publishing
    worker = threading.Thread(target=capture.run)
    stopper = threading.Thread(target=stopping)
    worker.start()
    try:
        assert entered.wait(3)
        stopper.start()
        assert stop_started.wait(3)
        assert not stopped.wait(.1), "stop returned before its in-flight publish completed"
    finally:
        release.set()
        worker.join(timeout=3)
        if stopper.ident is not None:
            stopper.join(timeout=3)
    assert not worker.is_alive()
    assert not stopper.is_alive()
    assert stopped.is_set()
    assert capture.published.count(channel) == 1


def _exit_holding_selection_lock(selection):
    selection.get_lock().acquire()
    # A native crash can exit without releasing the publication semaphore.


def _wait_until_stopped(namespace, side, serial, quality, channels, stop_event, status_queue):
    stop_event.wait(3)


def test_stop_recovers_when_worker_exits_holding_publish_lock(monkeypatch):
    s = wrist.WristCameraSession("test_robot", "left", "261922273335")
    s._channels = s._ctx.Value("B", 3)
    s._wanted = {"rgb", "depth"}
    s._requested_at = {name: wrist.time.monotonic() for name in s._wanted}
    s._quit = s._ctx.Event()
    s._proc = s._ctx.Process(target=_exit_holding_selection_lock, args=(s._channels,))
    s._proc.start()
    s._proc.join(timeout=3)
    assert s._proc.exitcode == 0
    result = []
    stopped = threading.Thread(target=lambda: result.append(s.stop("depth")), daemon=True)
    stopped.start()
    stopped.join(timeout=2.5)
    assert not stopped.is_alive(), "dead worker left stop blocked on the shared lock"
    assert result[0]["state"] == "idle"
    assert s._proc is None
    assert s._channels is None
    assert s.info("rgb")["state"] == "error"
    # Restart allocates an independent lock rather than reusing the poisoned one.
    monkeypatch.setattr(wrist, "_capture_worker", _wait_until_stopped)
    try:
        assert s.start("depth")["state"] == "loading"
        assert s._channels.value == 3
    finally:
        s.stop("rgb")
        s.stop("depth")
