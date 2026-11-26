import io
import math
from zipfile import ZipFile

from PIL import Image
import pytest

from rokae.helios.chassis_battery_pb2 import UiDatagram
from rokae.helios.spatial_map_preview import DisplayPoseClient, read_floor_map, render_preview


def raster(yaw=0):
    blob = io.BytesIO()
    with ZipFile(blob, "w") as archive:
        archive.writestr("lab.yaml", f'%YAML:1.0\nimage: lab.pgm\nresolution: 0.1\norigin: [-3, -2, {yaw}]\n')
        archive.writestr("lab.pgm", b"P5\n100 80\n255\n" + b"\x05" + b"\xff" * 7999)
    return read_floor_map(blob.getvalue(), "lab")


def test_calibrated_pixel_and_heading_include_rotation_and_vertical_flip():
    floor = raster(math.pi / 2)
    assert floor.image.getpixel((0, 0)) == 250  # occupied cell on the Helios PGM
    assert floor.image.getpixel((2, 0)) == 0
    pose = {"located": True, "x": -5, "y": 2, "theta": math.pi / 2}
    assert floor.marker(pose) == pytest.approx((80, 120, 0))
    picture = Image.open(io.BytesIO(render_preview(floor, pose))).convert("RGB")
    r, g, b = picture.getpixel((94, 120))
    assert b > r + 70  # arrow points right in image space
    for invisible in (None, {"located": False}, {**pose, "x": 800}):
        picture = Image.open(io.BytesIO(render_preview(floor, invisible))).convert("RGB")
        assert max(picture.getpixel((94, 120))) < 10


def test_display_pose_matches_hmi_validity_and_ignores_replayed_stamp():
    client = DisplayPoseClient("ws://localhost/ws/AGVInfo")
    message = UiDatagram()
    loc = message.robot_base.locate
    loc.update_time, loc.locate, loc.current_state.theta = 100, 1, 0.5
    assert client._parse(message.SerializeToString()) == {"located": True, "x": 0, "y": 0, "theta": 0.5}
    assert client._parse(message.SerializeToString()) is None
    loc.locate = 0
    assert client._parse(message.SerializeToString()) == {"located": False}
