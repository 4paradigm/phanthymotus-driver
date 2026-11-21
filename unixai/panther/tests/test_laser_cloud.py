"""Check geometry and transport for a planar Panther lidar frame."""
import math
import struct
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core_bridge import encode_bridge_payload, output_topics
from device import PantherDevice
from plugins import build_plugins


def test_laser_card_publishes_xyz_and_preserves_scan_header_for_ros():
    device = object.__new__(PantherDevice)
    device._topics = output_topics("test")
    device._lock = threading.RLock()
    device._cache = {}
    device._last_card_publish = {}
    device._laser_cloud_type = SimpleNamespace
    device._laser_cloud_fields = object()
    device._laser_cloud_pub = Mock()
    device._bridge = Mock()
    header = SimpleNamespace(frame_id="front_laser", stamp=object())
    device._on_laser(SimpleNamespace(
        header=header, angle_min=-math.pi / 2, angle_increment=math.pi / 2,
        range_min=0.1, range_max=8.0,
        ranges=[2.0, float("nan"), 3.0, float("inf"), 9.0, 0.01],
    ))

    spec = build_plugins({"plugins": {"laser_scan": {"enabled": True}}}, "test", device)[0].get_tool()
    assert spec["topic_out"] == [{"topic": "/test/panther/laser_scan", "format": "sensor/pointcloud"}]
    card, raw = device._bridge.publish.call_args.args
    cloud, = device._laser_cloud_pub.publish.call_args.args
    assert cloud.header is header
    assert (cloud.width, cloud.height, cloud.point_step, cloud.row_step) == (2, 1, 12, 24)
    assert bytes(cloud.data) == raw
    wire = encode_bridge_payload(card, raw)["data"]
    assert struct.unpack_from("<II", wire) == (12, 2)
    assert [tuple(round(v, 6) for v in point) for point in struct.iter_unpack("<fff", wire[8:])] == [
        (0.0, -2.0, 0.0), (0.0, 3.0, 0.0)
    ]
