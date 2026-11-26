"""Regression checks for the Helios lidar wire format and SDK lifecycle."""

import asyncio
from types import SimpleNamespace

import pytest

from rokae.helios import lidar
from rokae.helios.sdk_lidar_worker import HEADER, MAGIC, XYZ, pack_points


def test_cloud_registration_without_sdk(monkeypatch, tmp_path):
    monkeypatch.setenv("HELIOS_LIDAR_SDK_PYTHON", str(tmp_path / "python"))
    plugin = lidar.HeliosLidar({}, "rokae_helios", None)

    status = plugin.dispatch("status", {})

    assert not status["sdk_installed"]
    assert status["preview_sources"] == []
    assert {source["available"] for source in status["sources"].values()} == {False}
    preview = [{"topic": "/rokae_helios/lidar/preview", "format": "sensor/pointcloud"}]
    assert status["topic_out"] == plugin.get_tool()["topic_out"] == preview
    info = plugin.dispatch("info", {})
    assert info["topic_out"] == preview
    assert info["native_topics"] == [
        {"topic": "/rokae_helios/lidar/left_front/points", "message_type": "sensor_msgs/msg/PointCloud2"},
        {"topic": "/rokae_helios/lidar/right_rear/points", "message_type": "sensor_msgs/msg/PointCloud2"},
    ]
    for port in info["native_topics"]:
        assert port["topic"] in info["description"]
        assert port["message_type"] in info["description"]


def test_preview_sampling_and_timestamp_alignment():
    left = lidar.Cloud(2_000_000_000, 4_000, XYZ.pack(1.0, 2.0, 3.0) * 4_000, 0)
    right = lidar.Cloud(2_200_000_000, 4_000, XYZ.pack(4.0, 5.0, 6.0) * 4_000, 0)

    pair = lidar.preview_frames({-8: right, -4: left})
    preview = lidar.pack_preview(tuple(pair.values()))

    assert tuple(pair) == (-4, -8)
    assert len(preview) == 8 + 6_000 * lidar.XYZI.size
    assert lidar.XYZI.unpack_from(preview, 8) == (1.0, 2.0, 3.0, 0.0)
    assert lidar.XYZI.unpack_from(preview, 8 + 3_000 * lidar.XYZI.size) == (4.0, 5.0, 6.0, 0.0)
    outside_window = lidar.Cloud(1_900_000_000, left.count, left.xyz, 0)
    assert lidar.preview_frames({-4: outside_window, -8: right}) == {-8: right}


def test_worker_emits_only_finite_xyz():
    valid = SimpleNamespace(x=1.0, y=-2.0, z=3.0)
    invalid = SimpleNamespace(x=float("inf"), y=0.0, z=0.0)

    assert pack_points([invalid, valid]).tobytes() == XYZ.pack(1.0, -2.0, 3.0)


def test_read_cloud_rejects_truncated_payload():
    async def read():
        stream = asyncio.StreamReader()
        stream.feed_data(HEADER.pack(MAGIC, -4, 1, 2) + XYZ.pack(1, 2, 3))
        stream.feed_eof()
        with pytest.raises(asyncio.IncompleteReadError):
            await lidar.read_cloud(stream)

    asyncio.run(read())


def test_short_subscription_preserves_latest_complete_cloud(monkeypatch):
    client = lidar.SdkLidarClient({})
    monkeypatch.setattr(lidar, "SDK_SUBSCRIPTION_WINDOW_S", .03)
    started = []
    reaped = []

    class Worker:
        def __init__(self):
            self.stdout = asyncio.StreamReader()

        def kill(self):
            self.stdout.feed_eof()

        async def communicate(self):
            reaped.append(self)

    async def spawn(*args, **kwargs):
        worker = Worker()
        started.append(worker)
        if len(started) == 1:
            packet = HEADER.pack(MAGIC, -4, 1, 1) + XYZ.pack(1, 2, 3)
            incomplete = HEADER.pack(MAGIC, -4, 2, 1) + b"\x00"
            worker.stdout.feed_data(packet + incomplete)
        else:
            clouds = client.snapshot()
            assert clouds[-4].stamp_ns == 1
            assert clouds[-4].xyz == XYZ.pack(1, 2, 3)
            client._stop.set()
        return worker

    monkeypatch.setattr(asyncio, "create_subprocess_exec", spawn)

    async def run():
        await asyncio.wait_for(client._run(), timeout=1)

    asyncio.run(run())
    assert len(started) == 2
    assert reaped == started
    assert client.snapshot() == {}
