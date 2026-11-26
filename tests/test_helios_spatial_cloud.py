import io
import json
import math
import struct
from types import SimpleNamespace
from unittest.mock import Mock
from zipfile import ZipFile

import pytest

from rokae.helios.spatial_cloud import XYZ, mapping_frame, read_pcd
from rokae.helios.spatial_map_worker import MappingScan
from rokae.helios.controlled_spatial import ControlledSpatial


def test_saved_native_xyz_matches_core_wire_format():
    data = io.BytesIO()
    with ZipFile(data, "w") as z:
        header = b"FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nPOINTS 2\nDATA binary\n"
        records = struct.pack("<fffIfffI", 1, 2, 3, 0, -1, -2, -3, 123)
        z.writestr("lidmap/lab/map.pcd", header + records)
    points = read_pcd(data, "lab")
    frame = mapping_frame(points, {"x": 1, "y": -3, "theta": 0.5}, [], "lab")
    assert struct.unpack_from("<fffBI", frame) == (1, -3, -0.5, 7, 2)
    assert list(XYZ.iter_unpack(frame[17:41])) == [(1, 2, 3), (-1, -2, -3)]
    length, = struct.unpack_from("<I", frame, 41)
    assert len(frame[45:]) == length and json.loads(frame[45:])["active_map"] == "lab"


def test_building_scan_uses_synchronized_pose_and_real_height():
    pose = Mock()
    pose.fresh.return_value = {"theta": math.pi/2, "x": 1, "y": 3, "timestamp_ns": "2000000000"}
    scans = MappingScan(pose)
    scans.enabled = True
    cloud = SimpleNamespace(timestamp_ns=100, location=1, points=[SimpleNamespace(x=2, y=0, z=-.25)])
    scans.receive(cloud)
    assert not scans.frames
    cloud.timestamp_ns = 2000000000
    scans.receive(cloud)
    assert XYZ.unpack(scans.frames[1][1]) == pytest.approx((1, 5, -.25))


def test_tags_persist_in_map_frame_and_navigation_uses_current_map(tmp_path):
    controller = ControlledSpatial(tmp_path / "tags.db")
    controller.open()
    sensor = Mock()
    sensor.payload.return_value = {"map_name": "lab", "localization": {"x": 1, "y": -2, "theta": .7}}
    sensor.chassis.snapshot.return_value = {"available": True, "chassis": {"located": True, "mapping": False}}
    controller.execute("tag_place", {"name": "取件"}, sensor)
    controller.close()
    controller.open()
    assert not controller.tags("other")
    controller.execute("navigate_to_tag", {"tag_name": "取件"}, sensor)
    sensor.command.assert_called_once_with("navigate", map_name="lab", x=1., y=-2., yaw=.7)
    controller.selected = "other"
    with pytest.raises(ValueError, match="localization"):
        controller.execute("navigate_to_tag", {"tag_name": "取件"}, sensor)
    controller.close()
