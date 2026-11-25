"""RGB identity, real worker lifecycle and independent multi-card ownership."""

import struct
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from common.camera_info import parse
from common.vendor_runtime import DriverBundle
from rokae.helios.camera import HeliosCamera, RosCameraStream
from rokae.helios.camera_capture import (
    MAX_JPEG_BYTES, SOURCES, CameraCapture, JpegFramer, capture_command,
    jpeg_dimensions, resolve_device,
)


def header_jpeg(width=640, height=400, channels=3):
    # Deliberately just a SOF envelope for parser/IPC tests, not a colour sample.
    return b"\xff\xd8\xff\xc0\x00\x08\x08" + struct.pack(">HHB", height, width, channels) + b"\xff\xd9"


def wait_for(predicate, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


def test_device_identity_survives_renumbering_and_rejects_ir_metadata(tmp_path):
    sysfs, dev = tmp_path / "sys", tmp_path / "dev"
    dev.mkdir()
    for name, label in [("video24", SOURCES["chest_rgb"]["device_name"]),
                        ("video4", "vi-output, G300 ir_r 9-0071"),
                        ("video5", "tegra-capture-vi-metadata-2")]:
        (sysfs / name).mkdir(parents=True)
        (sysfs / name / "name").write_text(label)
        (dev / name).touch()
    assert resolve_device(SOURCES["chest_rgb"], sysfs, dev) == str(dev / "video24")
    (dev / "video24").unlink()
    with pytest.raises(ValueError, match="found 0"):
        resolve_device(SOURCES["chest_rgb"], sysfs, dev)
    assert set(SOURCES) == {"chest_rgb", "head_rgb", "head_front_fisheye", "head_rear_left_fisheye", "head_rear_right_fisheye"}


@pytest.mark.parametrize("source_id,dimensions", [
    ("chest_rgb", "width=640,height=400"), ("head_front_fisheye", "width=640,height=512")])
def test_capture_drops_before_conversion_without_shell_or_vendor_controls(source_id, dimensions):
    command = capture_command(SOURCES[source_id], "/dev/video4", 5)
    assert command[0:2] == ["gst-launch-1.0", "-q"]
    assert "device=/dev/video4" in command
    assert command.index("videorate") < command.index("videoscale") < command.index("videoconvert")
    assert "video/x-raw," + dimensions in command
    assert not any(word in " ".join(command) for word in ("sudo", "gpio", "jetson_clocks", "reset"))


def test_jpeg_pipe_chunk_boundaries_and_bounded_overflow():
    frame = header_jpeg()
    framer = JpegFramer()
    assert framer.feed(b"noise" + frame[:1]) == []
    assert framer.feed(frame[1:6]) == []
    assert framer.feed(frame[6:] + frame) == [frame, frame]
    assert jpeg_dimensions(frame) == (640, 400)
    with pytest.raises(ValueError, match="limit"):
        framer.feed(b"\xff\xd8" + b"x" * MAX_JPEG_BYTES)
    assert len(framer.buffer) == 0


@pytest.mark.parametrize("frame", [b"not JPEG", header_jpeg(channels=1),
                                        header_jpeg(width=0), header_jpeg()[:-2],
                                        b"\xff\xd8\xff\xe0\xff\xff\xff\xd9"])
def test_invalid_or_non_colour_images_are_never_published(frame):
    with pytest.raises(ValueError):
        jpeg_dimensions(frame)


def test_capture_real_subprocess_continuous_frames_and_restart():
    received = []
    frame = header_jpeg()
    command = [sys.executable, "-u", "-c",
               f"import sys,time\nwhile True:\n sys.stdout.buffer.write(bytes.fromhex('{frame.hex()}')); "
               "sys.stdout.buffer.flush(); time.sleep(.03)"]
    worker = CameraCapture(SOURCES["chest_rgb"], lambda jpeg, ts: received.append((jpeg, ts)),
                           device_resolver=lambda _: "/dev/fake", command_builder=lambda *_: command)
    try:
        worker.start(timeout=2)
        wait_for(lambda: worker.snapshot()["frame_count"] >= 3)
        assert worker.snapshot()["available"]
        assert worker.snapshot()["width"] == 640
        assert received[-1][1] > received[0][1]
        process = worker._process
        worker.stop()
        assert process.poll() is not None
        count = len(received)
        time.sleep(.08)
        assert len(received) == count and worker.snapshot()["state"] == "idle"
        worker.start(timeout=2)
        wait_for(lambda: len(received) > count)
    finally:
        worker.stop()


def test_no_frame_timeout_reaps_process_and_fails_start():
    command = [sys.executable, "-u", "-c", "import time; time.sleep(30)"]
    worker = CameraCapture(SOURCES["chest_rgb"], lambda *_: pytest.fail("unexpected frame"),
                           device_resolver=lambda _: "/dev/fake", command_builder=lambda *_: command,
                           stall_timeout_s=.15)
    with pytest.raises(ValueError, match="stopped producing frames"):
        worker.start(timeout=.4)
    assert worker._thread is None and worker._process is None


def test_ros_fanout_receive_stamp_and_detaching_one_subscriber(monkeypatch):
    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=SimpleNamespace(), frame_id="")

    class Publisher:
        def __init__(self, topic):
            self.topic = topic
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Node:
        def __init__(self, *_args, **_kwargs):
            self.destroyed_publishers = []
            self.destroyed = False

        def create_publisher(self, _type, topic, _qos):
            return Publisher(topic)

        def destroy_publisher(self, publisher):
            self.destroyed_publishers.append(publisher)

        def destroy_node(self):
            self.destroyed = True

    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1),
        HistoryPolicy=SimpleNamespace(KEEP_LAST=1)))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", SimpleNamespace(CompressedImage=Message))
    nodes = []
    ros = SimpleNamespace(ctx_core=object(), executor_core=SimpleNamespace(
        add_node=nodes.append, remove_node=nodes.remove))
    stream = RosCameraStream(ros, "rokae_helios", "chest_rgb", SOURCES["chest_rgb"], 5)
    stream.attach("a", "/camera/a/rgb")
    stream.attach("b", "/camera/b/rgb")
    first, second = stream._publishers["a"], stream._publishers["b"]
    stream._publish(header_jpeg(), 1_234_567_890)
    assert len(first.messages) == len(second.messages) == 1
    assert first.messages[0].header.stamp.sec == 1
    assert first.messages[0].header.stamp.nanosec == 234_567_890
    assert first.messages[0].format == "jpeg"
    stream.detach("a")
    stream._publish(header_jpeg(), 2_345_678_901)
    assert len(first.messages) == 1 and len(second.messages) == 2
    assert first in stream.node.destroyed_publishers
    stream.stop()
    assert not nodes and stream.node.destroyed


class FakeStream:
    def __init__(self, source_id):
        self.source = source_id
        self.started = self.stopped = 0
        self.attached = {}

    def attach(self, instance, topic):
        self.attached[instance] = topic

    def detach(self, instance):
        self.attached.pop(instance, None)

    def start(self):
        self.started += 1
        if self.source == "head_rear_right_fisheye":
            raise ValueError("device busy")

    def stop(self):
        self.stopped += 1

    def snapshot(self):
        return {"state": "online", "available": True, "width": 640, "height": 400,
                "frame_count": 10, "received_at_ms": 123}


def plugin():
    created = []

    def factory(source_id):
        stream = FakeStream(source_id)
        created.append(stream)
        return stream

    return HeliosCamera({}, stream_factory=factory), created


def test_camera_contract_five_upper_body_slots_and_unknown_optics():
    camera, created = plugin()
    definition = camera.get_tools()[0]
    assert definition["name"] == "camera_rgb" and definition["multiInstance"]
    assert definition["topic_out"][0]["format"] == "image/jpeg"
    assert len(definition["configSchema"]["properties"]["camera"]["oneOf"]) == 5
    camera.start()
    info = camera.dispatch("info", {})
    assert info["state"] == "idle" and not created
    optics = parse(info["camera_info"][0])
    assert not optics.known and optics.width is None and optics.source == "unknown"
    assert info["position_mapping_confirmed"]


def test_shared_producer_stopping_one_card_preserves_the_other():
    camera, created = plugin()
    bundle = DriverBundle([camera])
    for instance in ("card-a", "card-b"):
        response = bundle.dispatch("camera_rgb", {"action": "start", "instance_id": instance})
        assert response["state"] == "running"
    assert len(created) == 1 and created[0].started == 1
    camera.dispatch("stop", {"instance_id": "card-a"})
    assert created[0].stopped == 0
    assert camera.dispatch("info", {"instance_id": "card-b"})["available"]
    camera.dispatch("stop", {"instance_id": "card-b"})
    assert created[0].stopped == 1
    camera.dispatch("stop", {"instance_id": "card-b"})
    assert created[0].stopped == 1


def test_source_config_switch_keeps_topic_and_failed_switch_keeps_original():
    camera, created = plugin()
    camera.dispatch("config", {"instance_id": "a", "camera": "head_rgb"})
    assert not created
    first = camera.dispatch("start", {"instance_id": "a"})
    assert first["camera"] == "head_rgb"
    switched = camera.dispatch("config", {"instance_id": "a", "camera": "head_front_fisheye"})
    assert switched["camera"] == "head_front_fisheye"
    assert switched["topic_out"] == first["topic_out"]
    assert created[0].stopped == 1
    with pytest.raises(ValueError, match="device busy"):
        camera.dispatch("config", {"instance_id": "a", "camera": "head_rear_right_fisheye"})
    assert camera.dispatch("info", {"instance_id": "a"})["camera"] == "head_front_fisheye"
    assert created[-1].stopped == 1 and created[1].stopped == 0
    with pytest.raises(ValueError, match="Unknown Helios camera"):
        camera.dispatch("config", {"instance_id": "a", "camera": "chassis_front"})
    assert camera.dispatch("info", {"instance_id": "a"})["camera"] == "head_front_fisheye"
    camera.stop()
    assert created[1].stopped == 1


def test_switching_stops_the_old_producer_unless_another_card_uses_it():
    camera, created = plugin()
    camera.dispatch("start", {"instance_id": "a", "camera": "chest_rgb"})
    for source in ("head_rgb", "head_front_fisheye", "chest_rgb", "head_rgb"):
        camera.dispatch("config", {"instance_id": "a", "camera": source})
    # Repeated switching leaves exactly one live producer, owned by card a.
    live = [s for s in created if s.started > s.stopped]
    assert [s.source for s in live] == ["head_rgb"] and set(camera._streams) == {"head_rgb"}
    assert all(not s.attached for s in created if s not in live)

    camera.dispatch("start", {"instance_id": "b", "camera": "head_rgb"})
    shared = camera._streams["head_rgb"]
    camera.dispatch("config", {"instance_id": "a", "camera": "chest_rgb"})
    assert shared.stopped == 0 and set(shared.attached) == {"b"}  # still used by card b
    camera.dispatch("stop", {"instance_id": "b"})
    assert shared.stopped == 1 and set(camera._streams) == {"chest_rgb"}


@pytest.mark.parametrize("args", [{}, {"instance_id": "a", "camera": "../../video0"},
                                   {"instance_id": "x" * 129}])
def test_invalid_requests_do_not_open_any_device(args):
    camera, created = plugin()
    with pytest.raises(ValueError):
        camera.dispatch("start", args)
    assert not created


@pytest.mark.parametrize("fps", [0, -1, 16, True, 5.5, "5"])
def test_invalid_publish_rates(fps):
    with pytest.raises(ValueError):
        HeliosCamera({"camera_publish_fps": fps})
