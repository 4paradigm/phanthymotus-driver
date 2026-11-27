"""Native Helios PCD maps and Motus sensor/mapping frames."""

from concurrent.futures import ThreadPoolExecutor
import io
from itertools import islice
import json
import math
import struct
import time
from urllib.request import Request, urlopen
from zipfile import ZipFile

import yaml

XYZ = struct.Struct("<fff")
MAX_POINTS = 80000


def read_pcd(stream):
    header = {}
    for line in stream:
        cells = line.decode("ascii").strip().split()
        if cells and not cells[0].startswith("#"):
            header[cells[0]] = cells[1:]
            if cells[0] == "DATA":
                break
    fields = header.get("FIELDS", [])
    if (fields not in (["x", "y", "z"], ["x", "y", "z", "rgb"])
            or header.get("DATA") != ["binary"] or header["SIZE"] != ["4"] * len(fields)
            or header["TYPE"][:3] != ["F"] * 3 or header["COUNT"] != ["1"] * len(fields)):
        raise ValueError("Unsupported Helios binary XYZ PCD")
    count = int(header["POINTS"][0])
    width = 4 * len(fields)
    raw = stream.read()
    if len(raw) != count * width:
        raise ValueError("Incomplete PCD point records")
    result = bytearray()
    for i in range(min(count, MAX_POINTS)):
        pos = (i * count // min(count, MAX_POINTS)) * width
        if all(map(math.isfinite, XYZ.unpack_from(raw, pos))):
            result.extend(raw[pos:pos+XYZ.size])
    return bytes(result)


def read_grid(package, name):
    with package.open(name + ".yaml") as meta:
        lines = (line for line in meta if not line.startswith(b"%YAML:"))
        config = yaml.safe_load(b"".join(lines))
    with package.open(config["image"]) as pgm:
        header = []
        while len(header) < 4:
            line = pgm.readline()
            if not line:
                raise ValueError("Incomplete PGM header")
            header.extend(line.split(b"#", 1)[0].split())
        if header[0] != b"P5" or header[3] != b"255":
            raise ValueError("Expected an 8-bit binary PGM map")
        width, height = map(int, header[1:3])
        pixels = pgm.read()
    if len(pixels) != width * height:
        raise ValueError("Incomplete PGM raster")
    threshold = config["occupied_thresh"] * 255
    occupied = bytes(int((v if config["negate"] else 255 - v) > threshold) for v in range(256))
    mask = pixels.translate(occupied)
    stride = max(1, math.ceil(mask.count(1) / MAX_POINTS))
    ox, oy, yaw = config["origin"]
    resolution = config["resolution"]
    c, s = math.cos(yaw), math.sin(yaw)
    result = bytearray()
    for i in islice((i for i, cell in enumerate(mask) if cell), 0, None, stride):
        row, col = divmod(i, width)
        x, y = (col + .5) * resolution, (height - row - .5) * resolution
        result.extend(XYZ.pack(ox + c*x - s*y, oy + s*x + c*y, 0))
    return bytes(result)


def read_map(archive, name):
    with ZipFile(archive) as package:
        pcd = f"lidmap/{name}/map.pcd"
        if pcd in package.namelist():
            with package.open(pcd) as stream:
                return "pcd", read_pcd(stream)
        if name + ".yaml" in package.namelist():
            return "grid", read_grid(package, name)
        raise ValueError(f"No PCD or occupancy map in archive for {name}")


def fetch_map(origin, name):
    request = Request(origin + "/tool/rob/getRunMapZip", data=json.dumps({"name": name}).encode(),
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=10) as response:
        return read_map(io.BytesIO(response.read()), name)


class SavedCloud:
    def __init__(self, origin):
        self.origin = origin
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="helios-pcd")
        self.clear()

    def clear(self):
        self.name = None
        self.points = b""
        self.kind = None
        self.error = ""
        self.retry_at = 0
        self.pending = None

    def read(self, name):
        if name != self.name:
            if self.pending:
                self.pending.cancel()
            self.clear()
            self.name = name
        if self.pending and self.pending.done():
            try:
                self.kind, self.points = self.pending.result()
                self.error = ""
            except Exception as exc:
                self.error = str(exc)
                self.retry_at = time.monotonic() + 5
            self.pending = None
        if name and self.kind is None and not self.pending and time.monotonic() >= self.retry_at:
            self.pending = self.pool.submit(fetch_map, self.origin, name)
        return self.points

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)


def mapping_frame(points, pose, tags, map_name):
    markers = bytearray()
    for tag in tags:
        if len(markers) >= 12000 * XYZ.size:
            break
        x, y, yaw = tag["x"], tag["y"], tag["yaw"]
        for i in range(13):
            for j in range(12):
                angle = math.tau * j / 12
                markers.extend(XYZ.pack(x + .10 * math.cos(angle), y + .10 * math.sin(angle), i * .05))
            markers.extend(XYZ.pack(x + .03*i*math.cos(yaw), y + .03*i*math.sin(yaw), .12))
    capacity = MAX_POINTS - len(markers) // XYZ.size
    count = len(points) // XYZ.size
    if count > capacity:
        points = b"".join(points[(i*count//capacity)*12:(i*count//capacity)*12+12] for i in range(capacity))
    robot = {"x": 0., "y": 0., "yaw": 0., "pose_available": bool(pose)}
    if pose:
        robot.update(x=pose["x"], y=pose["y"], yaw=pose["theta"])
    meta = json.dumps({"version": 3, "active_map": map_name, "robot": robot, "tags": tags,
                       "grid_points": len(points)//12}, ensure_ascii=False, allow_nan=False).encode()
    return (struct.pack("<fffBI", robot["x"], robot["y"], -robot["yaw"], 7,
                        (len(points) + len(markers)) // 12) + points + markers
            + struct.pack("<I", len(meta)) + meta)
