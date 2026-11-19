"""Planar scan projection and bounded, odom-aligned accumulation for Canvas."""

from collections import OrderedDict
import math
import struct


def scan_points(scan, max_range):
    start, step = scan.angle_min, scan.angle_increment
    if not math.isfinite(start) or not math.isfinite(step) or step == 0:
        return
    lower, upper = scan.range_min, min(scan.range_max, max_range)
    if not math.isfinite(lower) or not math.isfinite(upper):
        return
    for index, distance in enumerate(scan.ranges):
        if math.isfinite(distance) and max(0.0, lower) <= distance <= upper:
            angle = start + index * step
            if math.isfinite(angle):
                yield distance * math.cos(angle), distance * math.sin(angle), 0.0


def point_packet(points):
    """Canvas sensor/pointcloud: little-endian stride, count, XYZI float32."""
    return struct.pack("<II", 16, len(points)) + b"".join(
        struct.pack("<ffff", x, y, z, 0.0) for x, y, z in points
    )


class ScanCloud:
    def __init__(self, *, voxel_m=0.05, history_s=20.0, max_points=20000, max_range_m=12.0):
        if not (0 < voxel_m <= 1 and 0 < history_s <= 120 and 1 <= max_points <= 40000
                and 0 < max_range_m <= 100):
            raise ValueError("invalid laser_scan cloud limits")
        self.voxel_m = voxel_m
        self.history_s = history_s
        self.max_points = max_points
        self.max_range_m = max_range_m
        self.clear()

    def clear(self):
        self._cells = OrderedDict()
        self._last_pose = None
        self._last_stamp = None

    def update(self, scan, received, stamp_ns, transform):
        points = list(scan_points(scan, self.max_range_m))
        if transform is None:
            # Scans without a transform are shown alone, never inserted into the odom cloud.
            stride = max(1, math.ceil(len(points) / self.max_points))
            return points[::stride]
        t, q = transform.translation, transform.rotation
        length = math.hypot(q.x, q.y, q.z, q.w)
        if not length or not all(math.isfinite(v) for v in (length, t.x, t.y, t.z)):
            raise ValueError("invalid laser transform")
        x, y, z, w = (v / length for v in (q.x, q.y, q.z, q.w))
        # First two columns of the full 3D rotation: input points lie in the scan's XY plane.
        xx, xy, xz = 1 - 2 * (y*y + z*z), 2 * (x*y + w*z), 2 * (x*z - w*y)
        yx, yy, yz = 2 * (x*y - w*z), 1 - 2 * (x*x + z*z), 2 * (y*z + w*x)
        pose = (t.x, t.y, t.z)
        if self._last_stamp is not None and (
            stamp_ns < self._last_stamp or math.dist(self._last_pose, pose) > 1.0
        ):
            self.clear()
        self._last_stamp, self._last_pose = stamp_ns, pose
        while self._cells and next(iter(self._cells.values()))[0] < received - self.history_s:
            self._cells.popitem(last=False)
        for px, py, _ in points:
            point = (xx*px + yx*py + t.x, xy*px + yy*py + t.y, xz*px + yz*py + t.z)
            key = tuple(math.floor(value / self.voxel_m) for value in point)
            self._cells[key] = (received, point)
            self._cells.move_to_end(key)
            if len(self._cells) > self.max_points:
                self._cells.popitem(last=False)
        return [point for _, point in self._cells.values()]
