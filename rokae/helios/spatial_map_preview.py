"""Draw the current chassis map using the single-vehicle HMI's raster and pose."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from functools import lru_cache
import io
import json
import math
import posixpath
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from zipfile import ZipFile

from google.protobuf.message import DecodeError
from PIL import Image, ImageDraw, ImageFont
import yaml

from .chassis_battery_pb2 import UiDatagram
from .feedback import StreamFeedbackClient


class DisplayPoseClient(StreamFeedbackClient):
    def __init__(self, url):
        self._last_key = None
        super().__init__(url, parser=self._parse, data_key="pose", binary=True,
                         stale_timeout_s=3, max_msg_size=8 * 1024 * 1024)

    def _parse(self, raw):
        try:
            frame = UiDatagram.FromString(raw)
        except DecodeError as exc:
            raise ValueError("invalid chassis datagram") from exc
        if not frame.robot_base.HasField("locate"):
            return None
        loc = frame.robot_base.locate
        key = (loc.update_time, loc.locate)
        if key == self._last_key:
            return None
        self._last_key = key
        pose = loc.current_state
        if (not loc.update_time or loc.locate != 1 or not loc.HasField("current_state")
                or not all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta))):
            return {"located": False}
        return {"located": True, "x": pose.x, "y": pose.y, "theta": pose.theta}


@dataclass
class FloorMap:
    image: Image.Image
    origin: tuple
    resolution: float
    original_size: tuple

    def marker(self, pose):
        ox, oy, yaw = self.origin
        dx, dy = pose["x"] - ox, pose["y"] - oy
        x = (math.cos(yaw) * dx + math.sin(yaw) * dy) / self.resolution
        y = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / self.resolution
        width, height = self.original_size
        return (x * self.image.width / width, (height - y) * self.image.height / height,
                pose["theta"] - yaw)


def read_floor_map(blob, name):
    with ZipFile(io.BytesIO(blob)) as package:
        entry = next((p for p in package.namelist() if posixpath.basename(p) == name + ".yaml"), None)
        if entry is None:
            raise ValueError("current map YAML is absent")
        text = package.read(entry).decode("utf-8-sig").removeprefix("%YAML:1.0")
        meta = yaml.safe_load(text)
        path = posixpath.normpath(posixpath.join(posixpath.dirname(entry), meta["image"]))
        with Image.open(io.BytesIO(package.read(path))) as grid:
            size = grid.size
            zoom = min(2.0, 800 / max(size))
            raster = grid.convert("RGB").resize(tuple(max(1, round(i * zoom)) for i in size),
                                                Image.Resampling.NEAREST)
    origin = tuple(float(v) for v in meta["origin"])
    resolution = float(meta["resolution"])
    if len(origin) != 3 or not all(map(math.isfinite, origin)) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid map calibration")
    return FloorMap(raster, origin, resolution, size)


def fetch_floor_map(http_origin, name):
    # The industrial monitor uses this read-only POST to download the onboard map.
    request = Request(http_origin + "/tool/rob/getRunMapZip",
                      data=json.dumps({"name": name}).encode(),
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=8) as response:
        return read_floor_map(response.read(), name)


@lru_cache(maxsize=1)
def _font():
    for font in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(font, 15)
        except OSError:
            continue
    return ImageFont.load_default()


def render_preview(floor, pose, status=""):
    width, height = floor.image.size if floor else (480, 180)
    canvas = Image.new("RGB", (width, height + 30), "#edf2f5")
    draw = ImageDraw.Draw(canvas)
    if floor:
        canvas.paste(floor.image)
        status = "No current position"
        if pose and pose.get("located"):
            px, py, heading = floor.marker(pose)
            if 0 <= px < width and 0 <= py < height:
                # The tip shows heading. The centre of the white dot is the reported pose.
                c, s = math.cos(heading), math.sin(heading)
                def point(forward, left):
                    return (px + forward*c - left*s, py - forward*s - left*c)
                arrow = [point(18, 0), point(-8, 9), point(-5, 0), point(-8, -9)]
                draw.ellipse((px-13, py-13, px+13, py+13), fill="#daefff", outline="#176092", width=2)
                draw.polygon(arrow, fill="#0076ca", outline="#083e72", width=2)
                draw.ellipse((px-3, py-3, px+3, py+3), fill="white", outline="#135785")
                status = f'x {pose["x"]:.2f} m    y {pose["y"]:.2f} m    yaw {math.degrees(pose["theta"]):.1f} deg'
            else:
                status = "Position outside map"
    draw.text((10, height+5), status, font=_font(), fill="#15384f")
    output = io.BytesIO()
    canvas.save(output, format="JPEG", quality=90)
    return output.getvalue()


class MapPreview:
    def __init__(self, ws_url):
        endpoint = urlsplit(ws_url)
        self.http_origin = ("https" if endpoint.scheme == "wss" else "http") + "://" + endpoint.netloc
        self.pose_client = DisplayPoseClient(ws_url)
        self._executor = None
        self._pending = None
        self._name = self._floor = None
        self._retry_after = 0

    def start(self):
        if self._executor is None:
            self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="helios-map-image")
            self.pose_client.start()

    def stop(self):
        self.pose_client.stop()
        if self._executor is not None:
            self._executor.shutdown(wait=False, cancel_futures=True)
        self._executor = self._pending = self._name = self._floor = None
        self._retry_after = 0

    def frame(self, name):
        if name != self._name:
            if self._pending is not None:
                self._pending.cancel()
            self._name, self._floor, self._pending, self._retry_after = name, None, None, 0
        if not name or self._executor is None:
            return render_preview(None, None, "No current map")
        if self._pending is not None and self._pending.done():
            try:
                self._floor = self._pending.result()
            except Exception as exc:
                print(f"[helios] map image unavailable: {type(exc).__name__}", flush=True)
                self._retry_after = time.monotonic() + 8
            self._pending = None
        if self._floor is None:
            if self._pending is None and time.monotonic() >= self._retry_after:
                self._pending = self._executor.submit(fetch_floor_map, self.http_origin, name)
            return render_preview(None, None, "Loading map" if self._pending is not None else "Map unavailable")
        snap = self.pose_client.snapshot()
        return render_preview(self._floor, snap["pose"] if snap["available"] else None)
