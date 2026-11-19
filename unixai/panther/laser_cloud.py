"""Project one planar scan into the Canvas point-cloud wire format."""

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


def current_points(scan, max_range_m, max_points):
    points = list(scan_points(scan, max_range_m))
    return points[::max(1, math.ceil(len(points) / max_points))]
