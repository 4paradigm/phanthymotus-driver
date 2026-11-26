"""Read chassis-frame XYZ point clouds in the vendor SDK's Python runtime."""

import math
import os
import struct
import sys
import threading
from array import array

SOURCES = {-4: "left_front", -8: "right_rear"}
MAX_POINTS = 150_000
XYZ = struct.Struct("<fff")
HEADER = struct.Struct("<4siQI")
MAGIC = b"HLC1"
MAX_STAMP_NS = (1 << 31) * 1_000_000_000 - 1


def pack_points(points):
    """Return finite XYZ as little-endian float32 with no per-point metadata."""
    if len(points) > MAX_POINTS:
        return array("f")
    data = array("f")
    for point in points:
        x, y, z = point.x, point.y, point.z
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            data.extend((x, y, z))
    if sys.byteorder != "little":
        data.byteswap()
    return data


def main(host, port):
    # Native library diagnostics must not enter the binary frame stream.
    output = os.fdopen(os.dup(sys.stdout.fileno()), "wb")
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    import point_cloud
    import py_client_common as rpc

    config = rpc.ClientConfig()
    config.ip = host
    config.port = int(port)
    config.connect_timeout_ms = config.call_timeout_ms = 3000
    if not rpc.init_client(config) or not point_cloud.has_point_cloud_dev():
        return 1

    lock = threading.Lock()

    def receive(frame):
        location, stamp = frame.location, frame.timestamp_ns
        if location not in SOURCES or not 0 < stamp <= MAX_STAMP_NS:
            return
        data = pack_points(frame.points)
        if data:
            with lock:
                output.write(HEADER.pack(MAGIC, location, stamp, len(data) // 3))
                output.write(data)
                output.flush()

    point_cloud.point_cloud_pub(receive)
    threading.Event().wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(*sys.argv[1:]))
