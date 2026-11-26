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


def scan_frame(location, stamp, points):
    message = UiDatagram()
    scan = message.laser_scan
    scan.location, scan.timestamp_ms = location, stamp
    for x, y in points:
        scan.scan.add(x=x, y=y)
    return message.SerializeToString()


def test_independent_scans_clear_on_empty_stale_or_lost_pose(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("rokae.helios.spatial_map_preview.time.monotonic", lambda: now[0])
    client = DisplayPoseClient("ws://localhost/ws/AGVInfo")
    message = UiDatagram()
    loc = message.robot_base.locate
    loc.update_time, loc.locate, loc.current_state.x = 5000, 1, 0
    client._parse(message.SerializeToString())
    for source in (-4, -8, -1):
        client._parse(scan_frame(source, 4990, [(1, 2)]))
    client._parse(scan_frame(-1, 5001, []))
    assert client.fresh_scans() == {-4: ((1, 2),), -8: ((1, 2),), -1: ()}
    now[0] += 1.6
    client._parse(scan_frame(-4, 4990, [(1, 2)]))
    assert client.fresh_scans() == {}
    client._parse(scan_frame(-2, 5400, [(2, 0)]))
    loc.locate = 0
    client._parse(message.SerializeToString())
    assert client.fresh_scans() == {}


def test_sensor_points_follow_robot_heading_and_have_distinct_colors():
    floor = raster()
    pose = {"located": True, "x": 1, "y": 1, "theta": math.pi/2}
    marker = floor.marker(pose)
    assert list(floor.scan_pixels(marker, [(1, 0), (0, 1), (0, -1)])) == [(80, 80), (60, 100), (100, 100)]
    # A small cluster makes JPEG chroma stable enough to compare source colours.
    def cluster(x, y):
        return [(x+i/30, y+j/30) for i in range(-5, 6) for j in range(-5, 6)]
    layers = {-4:cluster(1, 0), -8:cluster(0, 1), -1:cluster(0, -1)}
    image = Image.open(io.BytesIO(render_preview(floor, pose, scans=layers))).convert("RGB")
    r, g, b = image.getpixel((80, 80))
    assert g > r + 80 and b > r + 55
    r, g, b = image.getpixel((60, 100))
    assert r > b + 80 and g > b + 40
    r, g, b = image.getpixel((100, 100))
    assert r > g + 70 and b > g + 60
    hidden = Image.open(io.BytesIO(render_preview(floor, None, scans=layers))).convert("RGB")
    assert max(hidden.getpixel((100, 100))) < 10
