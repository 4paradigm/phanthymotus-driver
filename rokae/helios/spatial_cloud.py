"""Native Helios PCD maps and Motus sensor/mapping frames."""

from concurrent.futures import ThreadPoolExecutor
import io
import json
import math
import struct
import time
from urllib.request import Request, urlopen
from zipfile import ZipFile

XYZ = struct.Struct("<fff")
MAX_POINTS = 80000


def read_pcd(archive, name):
    with ZipFile(archive) as package, package.open(f"lidmap/{name}/map.pcd") as stream:
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


def fetch_pcd(origin, name):
    request = Request(origin + "/tool/rob/getRunMapZip", data=json.dumps({"name": name}).encode(),
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=10) as response:
        return read_pcd(io.BytesIO(response.read()), name)


class SavedCloud:
    def __init__(self, origin):
        self.origin = origin
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="helios-pcd")
        self.clear()

    def clear(self):
        self.name = None
        self.points = b""
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
                self.points = self.pending.result()
                self.error = ""
            except Exception as exc:
                self.error = str(exc)
                self.retry_at = time.monotonic() + 5
            self.pending = None
        if name and not self.points and not self.pending and time.monotonic() >= self.retry_at:
            self.pending = self.pool.submit(fetch_pcd, self.origin, name)
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
