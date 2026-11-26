import json
import math
import os
import shutil
import socket
import struct
import subprocess
import threading
import time
import zlib
from io import BytesIO
from pathlib import Path

import pytest
import yaml
from PIL import Image

from rokae.helios_sensors.depth_preview import depth_preview_jpeg
from rokae.helios_sensors.depth_socket import HEADER, read_depth_frame
from rokae.helios_sensors.main import DEFAULT_PORT, DepthSensorCard, ImuSensorCard
from rokae.helios_sensors.orbbec import HEIGHT, WIDTH, encode_depth
from rokae.helios_sensors.orientation import OrientationFilter

REPO = Path(__file__).resolve().parents[1]
DRIVER = REPO / "rokae/helios_sensors"


def test_driver_descriptor_matches_runtime_and_uses_an_unallocated_port():
    path = DRIVER / "driver.yaml"
    descriptor = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert 15700 <= DEFAULT_PORT <= 15799
    assert descriptor["port"] == DEFAULT_PORT
    assert descriptor["mcp_url"] == f"http://localhost:{DEFAULT_PORT}/mcp"
    collisions = [
        other.relative_to(REPO).as_posix()
        for other in REPO.glob("*/*/driver.yaml")
        if other != path
        and str(yaml.safe_load(other.read_text(encoding="utf-8")).get("port")) == str(DEFAULT_PORT)
    ]
    assert not collisions, f"MCP port {DEFAULT_PORT} is also allocated in {collisions}"


@pytest.mark.parametrize("base_has_dds", [True, False], ids=["standard", "missing-dds"])
def test_core_network_overlay_keeps_dds_contract_in_merged_compose(tmp_path, base_has_dds):
    docker = shutil.which("docker")
    if docker is None or subprocess.run(
        [docker, "compose", "version"], capture_output=True, check=False
    ).returncode:
        pytest.skip("Docker Compose CLI is required to render the deployment")

    deploy = DRIVER / "deploy"
    service = "rokae-helios-sensors"
    profile = "/opt/phanthy-motus/dds-local.xml"
    template = yaml.safe_load((deploy / "service.yml").read_text(encoding="utf-8"))
    generated = template[service]
    generated["image"] = "helios-camera-sensors:test"
    if not base_has_dds:
        generated["volumes"].remove(f"{profile}:{profile}:ro")
        for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"):
            del generated["environment"][key]
    compose = tmp_path / "docker-compose.yml"
    compose.write_text(yaml.safe_dump({"services": template}), encoding="utf-8")
    env = {**os.environ, "HELIOS_CORE_CONTAINER": "helios-test-core"}
    result = subprocess.run(
        [docker, "compose", "-f", str(compose), "-f", str(deploy / "compose.core-network.yml"),
         "config", "--format", "json", "--no-path-resolution"],
        cwd=tmp_path, env=env, capture_output=True, text=True, encoding="utf-8", timeout=20,
    )
    assert result.returncode == 0, result.stderr
    merged = json.loads(result.stdout)["services"][service]
    assert merged["network_mode"] == "container:helios-test-core"
    assert merged["environment"]["ROS_DOMAIN_ID"] == "42"
    assert merged["environment"]["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert merged["environment"]["FASTRTPS_DEFAULT_PROFILES_FILE"] == profile
    assert "FASTDDS_BUILTIN_TRANSPORTS" not in merged["environment"]
    mounts = [mount for mount in merged["volumes"] if mount["target"] == profile]
    assert len(mounts) == 1
    assert mounts[0]["type"] == "bind"
    assert mounts[0]["source"] == profile
    assert mounts[0]["read_only"] is True
    assert merged["logging"] == {"driver": "local", "options": {"max-file": "3", "max-size": "10m"}}


def test_depth_encodes_millimetres_and_preserves_invalid_pixels():
    native = struct.pack("<4H", 0, 2000, 65535, 10) * (WIDTH * HEIGHT // 4)
    assert zlib.decompress(encode_depth(native, 1)) == native
    assert zlib.decompress(encode_depth(native, 0.5))[:8] == struct.pack("<4H", 0, 1000, 32768, 5)


def test_depth_preview_keeps_raw_output_and_encodes_a_smaller_image():
    native = struct.pack("<H", 1700) * (WIDTH * HEIGHT)
    raw = encode_depth(native, 1.0)
    with Image.open(BytesIO(depth_preview_jpeg(raw))) as preview:
        assert preview.size == (WIDTH // 2, HEIGHT // 2)
        assert preview.format == "JPEG"
    assert zlib.decompress(raw) == native


@pytest.mark.parametrize("frame", [b"invalid zlib", zlib.compress(b"incomplete frame")])
def test_invalid_depth_preview_input_is_recoverable(frame):
    with pytest.raises(ValueError):
        depth_preview_jpeg(frame)


def test_depth_socket_reads_fragmented_header_metadata_and_frame():
    captured = time.monotonic()
    metadata = json.dumps({"captured_monotonic": captured, "K": [1.0]}).encode()
    payload = b"depth frame"
    header = HEADER.pack(123, len(metadata), len(payload))
    receiver, sender = socket.socketpair()
    chunks = (header[:3], header[3:], metadata[:5], metadata[5:], payload[:4], payload[4:])

    def send_frame():
        for chunk in chunks:
            sender.sendall(chunk)
            time.sleep(.005)

    with receiver, sender:
        worker = threading.Thread(target=send_frame)
        worker.start()
        try:
            assert read_depth_frame(receiver) == ("frame", captured, 123, payload, [1.0])
        finally:
            worker.join(timeout=1)


def test_depth_socket_reports_eof_with_partial_frame():
    receiver, sender = socket.socketpair()
    with receiver, sender:
        sender.sendall(HEADER.pack(123, 2, 4) + b"{}xx")
        sender.shutdown(socket.SHUT_WR)
        with pytest.raises(OSError, match="disconnected"):
            read_depth_frame(receiver)


@pytest.mark.parametrize("bad_layout", [True, False], ids=["short-frame", "invalid-scale"])
def test_depth_rejects_invalid_frames(bad_layout):
    raw = b"short" if bad_layout else bytes(WIDTH * HEIGHT * 2)
    scale = 1.0 if bad_layout else float("nan")
    with pytest.raises(ValueError):
        encode_depth(raw, scale)


def test_stale_imu_unsets_measurements():
    card = object.__new__(ImuSensorCard)
    card.serial = "test"
    card.preview_topic = None
    card.lock, card.stop_capture = threading.RLock(), threading.Event()
    card.thread, card.topic, card.count = object(), {}, 1
    card.frame_id, card.device_stamp, card.error = "test_frame", 123, None
    card.values = {"accel_z_m_s2": 9.8}
    card.received = time.monotonic()
    assert card.payload()["accel_z_m_s2"] == 9.8
    assert card.payload()["orientation_w"] is None
    card.values["orientation_w"] = 1.0
    card.values["orientation_source"] = "accel_gyro_fusion"
    assert card.payload()["orientation_w"] == 1.0
    card.received -= 4
    payload = card.payload()
    assert payload["state"] == "stale"
    assert payload["device_stamp_us"] is None and payload["accel_z_m_s2"] is None
    assert payload["orientation_w"] is None and payload["orientation_source"] is None


@pytest.mark.parametrize("card_type,name,output_format", [
    (ImuSensorCard, "imu", "data/json"),
    (DepthSensorCard, "camera_depth", "image/depth-zlib"),
])
def test_sensor_exposes_stable_prefix_and_canvas_lifecycle_actions(card_type, name, output_format):
    card = object.__new__(card_type)
    card.topic = {"topic": "/test/source", "format": output_format}
    card.preview_topic = {"topic": "/test/source/preview", "format": "image/jpeg"} if name == "camera_depth" else None
    tool = card.get_tool()
    actions = tool["inputSchema"]["properties"]["action"]["enum"]
    assert card_type.PREFIX == tool["name"] == name
    assert tool["type"] == "sensor"
    assert tool["topic_out"][0] == card.topic
    assert set(actions) <= {"start", "stop", "info", "config"}


def test_sensor_dispatch_follows_framework_contract():
    card = object.__new__(ImuSensorCard)
    calls = []
    card.start = lambda: calls.append("start")
    card.stop = lambda: calls.append("stop")
    card.payload = lambda: {"state": "online"}

    assert card.dispatch("start", {}) == {"state": "running"}
    assert card.dispatch("stop", {}) == {"state": "idle"}
    assert card.dispatch("imu", {}) == card.dispatch("read", {}) == {"state": "online"}
    assert card.dispatch("unknown", {}) is None
    assert calls == ["start", "stop"]


def test_sensor_retries_when_camera_frames_time_out(monkeypatch):
    card = object.__new__(ImuSensorCard)
    card.serial = "test"
    card.preview_topic = None
    card.lock, card.stop_capture = threading.RLock(), threading.Event()
    card.thread, card.topic, card.count = object(), {}, 1
    card.frame_id, card.device_stamp, card.error = "test_frame", 123, None
    card.values = {"accel_z_m_s2": 9.8}
    card.received = time.monotonic()
    attempts, statuses, delays = [], [], []

    def capture():
        attempts.append(card.payload()["state"])
        if len(attempts) == 1:
            raise TimeoutError("no fresh camera frames")
        card.stop_capture.set()

    card.run_capture = capture
    card.publish_status = lambda: statuses.append(card.payload())
    monkeypatch.setattr(card.stop_capture, "wait", delays.append)

    card.run()

    assert attempts == ["online", "reconnecting"]
    assert delays == [3]
    assert len(statuses) == 1
    assert statuses[0]["state"] == "reconnecting"
    assert statuses[0]["error"] == "no fresh camera frames"
    assert statuses[0]["accel_z_m_s2"] is None


def initialize_orientation(fusion, accel=(0, 0, 9.80665)):
    for stamp in range(20_000, 360_001, 20_000):
        result = fusion.update(stamp, accel, (0, 0, 0))
    return result


def test_orientation_aligns_tilt_to_gravity():
    tilt = math.pi / 3
    accel = (0, 9.80665 * math.sin(tilt), 9.80665 * math.cos(tilt))
    fusion = OrientationFilter()

    assert fusion.update(0, accel, (0, 0, 0)) is None
    assert initialize_orientation(fusion, accel) == pytest.approx(
        (math.sin(tilt / 2), 0, 0, math.cos(tilt / 2)))


def test_orientation_integrates_relative_yaw_and_reinitializes_after_gap():
    fusion = OrientationFilter()
    assert initialize_orientation(fusion) == (0, 0, 0, 1)
    for stamp in range(380_000, 1_360_001, 20_000):
        result = fusion.update(stamp, (0, 0, 9.80665), (0, 0, math.pi / 2))
    assert result == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert fusion.update(1_360_000, (0, 0, 9.80665), (0, 0, 0)) == result
    assert fusion.update(2_000_000, (0, 0, 9.80665), (0, 0, 0)) is None


def test_orientation_rejects_unreliable_gravity_and_corrects_tilt_drift():
    fusion = OrientationFilter()
    for stamp in range(20_000, 360_001, 20_000):
        assert fusion.update(stamp, (0, 0, 25), (0, 0, 0)) is None
    assert fusion.update(380_000, (0, 0, 9.80665), (0, 0, 1)) is None
    for stamp in range(400_000, 760_001, 20_000):
        result = fusion.update(stamp, (0, 0, 9.80665), (0, 0, 0))
    assert result == (0, 0, 0, 1)
    for stamp in range(780_000, 980_001, 20_000):
        result = fusion.update(stamp, (25, 0, 0), (1, 0, 0))
    assert result[0] > 0.1 and result[1:3] == (0, 0)
    for stamp in range(1_000_000, 4_000_001, 20_000):
        result = fusion.update(stamp, (0, 0, 9.80665), (0, 0, 0))
    assert abs(result[0]) < 0.002 and math.hypot(*result) == pytest.approx(1)
