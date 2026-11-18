"""Payload shaping for the Panther cards: ROS-message-shaped objects in, JSON-safe data out.

Nothing in here imports ROS. Every function takes an object with the same attributes as
the ROS message (a `SimpleNamespace` in the tests) so the rules below can be checked on a
laptop, which is the only place a bug can be found cheaply.

Three rules recur and each one is a reported-but-wrong failure rather than a crash:

* **NaN is `null`.** Panther's `/joint_states` carries NaN velocity and effort, and its
  battery carries NaN charge and capacity. `json.dumps` writes those as the bare token
  `NaN`, which is not JSON: the dashboard's `JSON.parse` throws and the card goes blank.
* **A level is an integer.** `diagnostic_msgs/DiagnosticStatus.level` is a ROS `byte`,
  which rclpy hands over as `b'\\x00'`, not as `0`.
* **Unknown stays unknown.** An absent measurement is `None`, never `0.0` -- the same rule
  `common/odom.py` states for odometry.
"""
from __future__ import annotations

import math
import zlib

DEPTH_FORMAT = "16UC1; compressedDepth zlib"
DEPTH_OUTPUT_SIZE = (640, 480)

BATTERY_STATUS = {0: "unknown", 1: "charging", 2: "discharging", 3: "not_charging", 4: "full"}
BATTERY_HEALTH = {0: "unknown", 1: "good", 2: "overheat", 3: "dead", 4: "overvoltage",
                  5: "unspecified_failure", 6: "cold", 7: "watchdog_timer_expire",
                  8: "safety_timer_expire"}
BATTERY_TECHNOLOGY = {0: "unknown", 1: "nimh", 2: "lion", 3: "lipo", 4: "life",
                      5: "nicd", 6: "limn"}
DIAGNOSTIC_LEVELS = {0: "ok", 1: "warn", 2: "error", 3: "stale"}


def num(value, digits: int | None = None):
    """A finite float, or None. Bools, strings, NaN and +-inf are all None."""
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return round(number, digits) if digits is not None else number


def vec(values, digits: int | None = None) -> list:
    return [num(v, digits) for v in values]


def stamp_ms(header) -> int | None:
    """Wall-clock milliseconds from a ROS header, or None when it is unset (0 s)."""
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", 0) or 0
    nanosec = getattr(stamp, "nanosec", 0) or 0
    if sec <= 0:
        return None
    return int(sec) * 1000 + int(nanosec) // 1_000_000


def byte_value(value) -> int:
    """A ROS `byte` as an int. rclpy returns it as `bytes` of length one."""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    if isinstance(value, str):
        return ord(value[0]) if value else 0
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


# ── joints ───────────────────────────────────────────────────────────────────

def joint_state_payload(msg, known: set | frozenset | None = None) -> dict:
    """`sensor/skeleton` payload. The renderer matches joints to the URDF by exact name.

    A joint whose position is not a finite number is dropped rather than sent as `null`:
    the renderer cannot place it, and an omitted joint keeps its last pose where a `null`
    could throw.
    """
    names = list(getattr(msg, "name", []))
    positions = list(getattr(msg, "position", []))
    velocities = list(getattr(msg, "velocity", []))
    efforts = list(getattr(msg, "effort", []))
    joints, dropped = [], 0
    for index, name in enumerate(names):
        if known is not None and name not in known:
            continue
        q = num(positions[index], 5) if index < len(positions) else None
        if q is None:
            dropped += 1
            continue
        joints.append({
            "idx": index, "name": name, "q": q,
            "dq": num(velocities[index], 4) if index < len(velocities) else None,
            "tau": num(efforts[index], 3) if index < len(efforts) else None,
        })
    return {"joints": joints, "count": len(joints), "dropped": dropped,
            "stamp_ms": stamp_ms(getattr(msg, "header", None))}


def single_joint(msg) -> dict:
    """One-joint `JointState` (a gripper or the lift) as a flat record."""
    names = list(getattr(msg, "name", []))
    position = list(getattr(msg, "position", []))
    velocity = list(getattr(msg, "velocity", []))
    effort = list(getattr(msg, "effort", []))
    return {
        "name": names[0] if names else None,
        "position": num(position[0], 5) if position else None,
        "velocity": num(velocity[0], 5) if velocity else None,
        "effort": num(effort[0], 4) if effort else None,
        "stamp_ms": stamp_ms(getattr(msg, "header", None)),
    }


# ── battery ──────────────────────────────────────────────────────────────────

def battery_payload(msg) -> dict:
    """`percentage` in `sensor_msgs/BatteryState` is a 0..1 ratio, not a percentage."""
    ratio = num(getattr(msg, "percentage", None))
    percent = round(ratio * 100.0, 1) if ratio is not None and 0.0 <= ratio <= 1.0 else None
    status = int(getattr(msg, "power_supply_status", 0) or 0)
    health = int(getattr(msg, "power_supply_health", 0) or 0)
    technology = int(getattr(msg, "power_supply_technology", 0) or 0)
    return {
        "percent": percent,
        "voltage_v": num(getattr(msg, "voltage", None), 3),
        "current_a": num(getattr(msg, "current", None), 3),
        "temperature_c": num(getattr(msg, "temperature", None), 1),
        "charge_ah": num(getattr(msg, "charge", None), 3),
        "capacity_ah": num(getattr(msg, "capacity", None), 3),
        "design_capacity_ah": num(getattr(msg, "design_capacity", None), 3),
        "present": bool(getattr(msg, "present", False)),
        "status": BATTERY_STATUS.get(status, f"code_{status}"),
        "health": BATTERY_HEALTH.get(health, f"code_{health}"),
        "technology": BATTERY_TECHNOLOGY.get(technology, f"code_{technology}"),
        "raw_percentage": ratio,
        "stamp_ms": stamp_ms(getattr(msg, "header", None)),
    }


# ── imu ──────────────────────────────────────────────────────────────────────

def _covariance_unknown(covariance) -> bool:
    """ROS convention: covariance[0] == -1 means this field is not provided."""
    try:
        return len(covariance) > 0 and float(covariance[0]) == -1.0
    except (TypeError, ValueError):
        return False


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> list:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [roll, pitch, yaw]


def imu_payload(msg) -> dict:
    """Fields the sensor does not provide are `null`, following the covariance[0] == -1 rule."""
    o = getattr(msg, "orientation", None)
    quaternion = vec([o.x, o.y, o.z, o.w]) if o is not None else [None] * 4
    norm = (math.sqrt(sum(v * v for v in quaternion))
            if all(v is not None for v in quaternion) else 0.0)
    orientation_known = (norm > 0.5
                         and not _covariance_unknown(getattr(msg, "orientation_covariance", ())))
    w = getattr(msg, "angular_velocity", None)
    a = getattr(msg, "linear_acceleration", None)
    gyro_known = w is not None and not _covariance_unknown(
        getattr(msg, "angular_velocity_covariance", ()))
    accel_known = a is not None and not _covariance_unknown(
        getattr(msg, "linear_acceleration_covariance", ()))
    euler = (vec(euler_from_quaternion(*[v / norm for v in quaternion]), 4)
             if orientation_known else None)
    return {
        "orientation_xyzw": [round(v / norm, 5) for v in quaternion] if orientation_known else None,
        "euler_rpy_rad": euler,
        "angular_velocity_rad_s": vec([w.x, w.y, w.z], 5) if gyro_known else None,
        "linear_acceleration_m_s2": vec([a.x, a.y, a.z], 4) if accel_known else None,
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "stamp_ms": stamp_ms(getattr(msg, "header", None)),
    }


# ── odometry ─────────────────────────────────────────────────────────────────

def odom_reading(msg) -> tuple:
    """`(twist, meta)` for `common.odom`: `[vx, vy, vz, wx, wy, wz]` with `None` for unmeasured axes.

    Panther is a planar base, so the controller reports vx, vy and the yaw rate. vz, wx and
    wy are not measurements -- they are the zero a twist message defaults to -- and are
    reported as `None` instead.
    """
    twist = getattr(getattr(msg, "twist", None), "twist", None)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    reading = [
        num(getattr(linear, "x", None)), num(getattr(linear, "y", None)), None,
        None, None, num(getattr(angular, "z", None)),
    ]
    meta = {
        "vendor_ms": stamp_ms(getattr(msg, "header", None)),
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "child_frame_id": str(getattr(msg, "child_frame_id", "")),
    }
    return reading, meta


# ── diagnostics ──────────────────────────────────────────────────────────────

def diagnostic_entries(msg) -> list:
    """Each status of a `DiagnosticArray`, as plain data. The array's order changes per message."""
    entries = []
    for status in getattr(msg, "status", []):
        level = byte_value(getattr(status, "level", 0))
        entries.append({
            "name": str(getattr(status, "name", "")),
            "level": level,
            "level_name": DIAGNOSTIC_LEVELS.get(level, f"code_{level}"),
            "message": str(getattr(status, "message", "")),
            "hardware_id": str(getattr(status, "hardware_id", "")),
            "values": {str(kv.key): str(kv.value) for kv in getattr(status, "values", [])},
        })
    return entries


_ESTOP_WORDS = ("emergency stop", "emergency_stop", "estop", "e-stop", "e_stop")


def is_estop_entry(entry: dict) -> bool:
    name = str(entry.get("name", "")).lower()
    return any(word in name for word in _ESTOP_WORDS) or "stop_button_status" in entry.get("values", {})


def health_payload(entries: dict, now_ms: int, stale_after_ms: int) -> dict:
    """`entries` maps status name to `{"entry": ..., "received_ms": ...}`."""
    statuses, worst = [], 0
    for name in sorted(entries):
        record = entries[name]
        age = max(0, now_ms - record["received_ms"])
        item = dict(record["entry"])
        item["age_ms"] = age
        item["stale"] = age > stale_after_ms
        statuses.append(item)
        worst = max(worst, item["level"])
    return {"worst_level": worst, "worst_level_name": DIAGNOSTIC_LEVELS.get(worst, f"code_{worst}"),
            "count": len(statuses), "statuses": statuses, "stamp_ms": now_ms}


def estop_payload(entries: dict, now_ms: int, stale_after_ms: int) -> dict:
    """The emergency-stop diagnostic, verbatim, with how old it is.

    The vendor strings (`stop_button_status`, `message`) are passed through untouched. The
    only value seen on a robot so far is the "normal" one, so no other string is mapped to
    "pressed" here -- guessing that mapping is how a card ends up reporting safe on a robot
    that is not. A reading older than `stale_after_ms` is flagged `stale`; the caller
    republishes on a timer so that flag appears even when `/diagnostics` stops altogether.
    """
    matches = [(name, rec) for name, rec in entries.items() if is_estop_entry(rec["entry"])]
    if not matches:
        return {"found": False, "stale": True, "age_ms": None,
                "message": "no emergency-stop diagnostic has been received",
                "stamp_ms": now_ms}
    name, record = sorted(matches)[0]
    entry = record["entry"]
    age = max(0, now_ms - record["received_ms"])
    return {
        "found": True, "name": entry["name"], "level": entry["level"],
        "level_name": entry["level_name"], "message": entry["message"],
        "stop_button_status": entry["values"].get("stop_button_status"),
        "values": entry["values"], "age_ms": age, "stale": age > stale_after_ms,
        "stamp_ms": now_ms,
    }


# ── laser scan ───────────────────────────────────────────────────────────────

def laser_payload(msg, max_points: int = 360) -> dict:
    """Down-sample a scan by keeping the **nearest** valid return in each bin.

    Taking every n-th beam can step over a thin obstacle; the minimum of the bin cannot.
    Invalid returns (NaN, +-inf, outside `[range_min, range_max]`) become `None`.
    """
    ranges = list(getattr(msg, "ranges", []))
    total = len(ranges)
    range_min = num(getattr(msg, "range_min", None)) or 0.0
    range_max = num(getattr(msg, "range_max", None))
    range_max = range_max if range_max is not None else float("inf")
    step = max(1, math.ceil(total / max(1, int(max_points)))) if total else 1
    binned, valid = [], 0
    for start in range(0, total, step):
        best = None
        for raw in ranges[start:start + step]:
            value = num(raw)
            if value is None or value < range_min or value > range_max:
                continue
            if best is None or value < best:
                best = value
        if best is not None:
            valid += 1
        binned.append(None if best is None else round(best, 3))
    increment = num(getattr(msg, "angle_increment", None))
    nearest = min((v for v in binned if v is not None), default=None)
    return {
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "angle_min": num(getattr(msg, "angle_min", None), 5),
        "angle_increment": round(increment * step, 6) if increment is not None else None,
        "range_min": num(getattr(msg, "range_min", None), 3),
        "range_max": num(getattr(msg, "range_max", None), 3),
        "beams": total, "bins": len(binned), "beams_per_bin": step,
        "valid_ratio": round(valid / len(binned), 3) if binned else 0.0,
        "nearest_m": nearest,
        "ranges": binned,
        "stamp_ms": stamp_ms(getattr(msg, "header", None)),
    }


def laser_topdown_jpeg(scan: dict, size: int = 400, max_range_m: float = 6.0,
                       quality: int = 70) -> bytes | None:
    """Plan view of a `laser_payload`: robot at the centre, forward up. None when OpenCV is absent.

    The dashboard's lidar renderer does not read topic frames, so a scan sent only as numbers
    cannot be looked at. This is that picture.
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None
    image = np.full((size, size, 3), (24, 24, 24), dtype=np.uint8)
    centre = size // 2
    scale = (size // 2 - 12) / max_range_m
    for metres in range(1, int(max_range_m) + 1):
        cv2.circle(image, (centre, centre), int(metres * scale), (60, 60, 60), 1)
        cv2.putText(image, f"{metres}m", (centre + 3, centre - int(metres * scale) + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1)
    angle, increment = scan.get("angle_min"), scan.get("angle_increment")
    if angle is not None and increment is not None:
        for value in scan.get("ranges", []):
            if value is not None and value <= max_range_m:
                px = int(centre - value * math.sin(angle) * scale)
                py = int(centre - value * math.cos(angle) * scale)
                if 0 <= px < size and 0 <= py < size:
                    near = min(1.0, value / max_range_m)
                    cv2.circle(image, (px, py), 1, (int(255 * near), 200, int(255 * (1 - near))), -1)
            angle += increment
    cv2.arrowedLine(image, (centre, centre), (centre, centre - 24), (0, 255, 0), 2, tipLength=0.35)
    cv2.circle(image, (centre, centre), 4, (0, 255, 0), -1)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return bytes(encoded) if ok else None


class StaticTf:
    """The robot's fixed frame tree from `/tf_static`, enough to place a lidar in `base_link`.

    The vendor URDF has one lidar frame (`base_laser_link`) and none for the individual
    lidars, so where the front and rear lidars sit can only be read from the robot itself.
    Full 3-D composition (rotation matrix and translation) is used rather than a planar
    shortcut so that a lidar mounted tilted or upside down is still placed correctly; only the
    resulting x and y are used, because a scan is a plane.
    """

    def __init__(self):
        self._edges: dict = {}          # child -> (parent, rotation 3x3, translation 3)

    @staticmethod
    def _matrix(q) -> list | None:
        x, y, z, w = (num(q.x), num(q.y), num(q.z), num(q.w))
        if None in (x, y, z, w):
            return None
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-6:
            return None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]

    def add_message(self, msg) -> int:
        """Record every usable transform of a `tf2_msgs/TFMessage`; returns how many were kept."""
        kept = 0
        for item in getattr(msg, "transforms", []):
            child = str(item.child_frame_id).lstrip("/")
            parent = str(item.header.frame_id).lstrip("/")
            rotation = self._matrix(item.transform.rotation)
            t = item.transform.translation
            translation = [num(t.x), num(t.y), num(t.z)]
            if not child or not parent or rotation is None or None in translation:
                continue
            self._edges[child] = (parent, rotation, translation)
            kept += 1
        return kept

    def lookup(self, frame: str, target: str):
        """`(rotation, translation)` taking a point in `frame` to `target`, or None if unconnected."""
        frame, target = str(frame).lstrip("/"), str(target).lstrip("/")
        rotation = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
        translation = [0.0, 0.0, 0.0]
        current = frame
        for _ in range(32):
            if current == target:
                return rotation, translation
            edge = self._edges.get(current)
            if edge is None:
                return None
            parent, r, t = edge
            translation = [sum(r[i][k] * translation[k] for k in range(3)) + t[i] for i in range(3)]
            rotation = [[sum(r[i][k] * rotation[k][j] for k in range(3)) for j in range(3)]
                        for i in range(3)]
            current = parent
        return None

    def __len__(self) -> int:
        return len(self._edges)


def merge_scans(parts: list, target_frame: str, max_points: int = 360) -> dict:
    """One polar scan around `target_frame` from several lidars, keeping the nearest return per bin.

    `parts` is `[(msg, transform)]`, where `transform` is `StaticTf.lookup`'s result or None for
    a scan already in `target_frame`. Each valid beam is turned into a point, moved into the
    target frame, and put back into one of `max_points` bins round the robot; where two lidars
    see the same direction the nearer return wins, as in the single-lidar pooling. Ranges are
    therefore measured from the target frame's origin, not from either lidar.
    """
    bins = max(8, int(max_points))
    step = 2.0 * math.pi / bins
    merged = [None] * bins
    beams, range_mins, range_maxs = 0, [], []
    for msg, transform in parts:
        ranges = list(getattr(msg, "ranges", []))
        beams += len(ranges)
        low = num(getattr(msg, "range_min", None)) or 0.0
        high = num(getattr(msg, "range_max", None))
        high = high if high is not None else float("inf")
        range_mins.append(low)
        range_maxs.append(high)
        angle = num(getattr(msg, "angle_min", None))
        increment = num(getattr(msg, "angle_increment", None))
        if angle is None or increment is None:
            continue
        for index, raw in enumerate(ranges):
            value = num(raw)
            if value is None or value < low or value > high:
                continue
            theta = angle + index * increment
            x, y = value * math.cos(theta), value * math.sin(theta)
            if transform is not None:
                (r, t) = transform
                x, y = r[0][0] * x + r[0][1] * y + t[0], r[1][0] * x + r[1][1] * y + t[1]
            distance = math.hypot(x, y)
            slot = int(round((math.atan2(y, x) + math.pi) / step)) % bins
            if merged[slot] is None or distance < merged[slot]:
                merged[slot] = distance
    out = [None if v is None else round(v, 3) for v in merged]
    valid = sum(1 for v in out if v is not None)
    return {
        "frame_id": target_frame,
        "angle_min": round(-math.pi, 5), "angle_increment": round(step, 6),
        "range_min": round(min(range_mins), 3) if range_mins else None,
        "range_max": round(max(range_maxs), 3) if range_maxs and math.isfinite(max(range_maxs)) else None,
        "beams": beams, "bins": bins, "beams_per_bin": None,
        "valid_ratio": round(valid / bins, 3), "nearest_m": min((v for v in out if v is not None), default=None),
        "ranges": out, "stamp_ms": None,
    }


# ── images ───────────────────────────────────────────────────────────────────

class UnsupportedImage(ValueError):
    """An image encoding or layout this driver does not convert. Never silently dropped."""


_COLOR_ENCODINGS = {"rgb8": 3, "bgr8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}
_DEPTH_ENCODINGS = {"16UC1": "<u2", "mono16": "<u2", "32FC1": "<f4"}


def _pixels(msg, channels: int, dtype: str):
    import numpy as np

    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if bool(getattr(msg, "is_bigendian", 0)):
        dtype = dtype.replace("<", ">")
    itemsize = np.dtype(dtype).itemsize
    row = width * channels * itemsize
    if width <= 0 or height <= 0:
        raise UnsupportedImage(f"empty image {width}x{height}")
    if step < row:
        raise UnsupportedImage(f"row step {step} is smaller than a row of {row} bytes")
    buffer = bytes(msg.data)
    if len(buffer) < step * height:
        raise UnsupportedImage(f"{len(buffer)} bytes for a {step}x{height} image")
    rows = np.frombuffer(buffer, dtype=np.uint8, count=step * height).reshape(height, step)[:, :row]
    pixels = np.ascontiguousarray(rows).view(dtype)
    return pixels.reshape(height, width, channels) if channels > 1 else pixels.reshape(height, width)


def encode_color_jpeg(msg, quality: int = 80, max_width: int = 640) -> tuple:
    """`(jpeg_bytes, (width, height))`. Wider images are scaled down, keeping the aspect ratio."""
    encoding = str(getattr(msg, "encoding", ""))
    channels = _COLOR_ENCODINGS.get(encoding)
    if channels is None:
        raise UnsupportedImage(f"colour encoding {encoding!r} (supported: {', '.join(_COLOR_ENCODINGS)})")
    import cv2
    import numpy as np

    pixels = _pixels(msg, channels, "<u1")
    if encoding == "rgb8":
        pixels = pixels[:, :, ::-1]
    elif encoding == "rgba8":
        pixels = pixels[:, :, [2, 1, 0]]
    elif encoding == "bgra8":
        pixels = pixels[:, :, :3]
    # Channel reversal leaves negative strides, which OpenCV rejects.
    pixels = np.ascontiguousarray(pixels)
    height, width = pixels.shape[:2]
    if width > max_width:
        new_height = max(1, round(height * max_width / width))
        pixels = cv2.resize(pixels, (max_width, new_height), interpolation=cv2.INTER_AREA)
        width, height = max_width, new_height
    ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return bytes(encoded), (width, height)


def encode_depth_zlib(msg, output_size: tuple = DEPTH_OUTPUT_SIZE) -> tuple:
    """`(zlib_bytes, (width, height), (source_width, source_height))` as `16UC1; compressedDepth zlib`.

    The dashboard's depth contract is a fixed 640x480 array of uint16 millimetres, so any
    other size is resampled (nearest neighbour -- a depth value must never be averaged with
    its neighbour across an edge). `32FC1` is metres and is converted; NaN, infinity and
    negative values become 0, the "no reading" value.
    """
    import numpy as np

    encoding = str(getattr(msg, "encoding", ""))
    dtype = _DEPTH_ENCODINGS.get(encoding)
    if dtype is None:
        raise UnsupportedImage(f"depth encoding {encoding!r} (supported: {', '.join(_DEPTH_ENCODINGS)})")
    depth = _pixels(msg, 1, dtype)
    source = (depth.shape[1], depth.shape[0])
    if dtype.endswith("f4"):
        metres = depth.astype(np.float64)
        metres[~np.isfinite(metres) | (metres < 0)] = 0.0
        depth = np.clip(np.rint(metres * 1000.0), 0, 65535).astype("<u2")
    else:
        depth = depth.astype("<u2", copy=False)
    out_w, out_h = output_size
    if source != (out_w, out_h):
        rows = (np.arange(out_h) * source[1]) // out_h
        cols = (np.arange(out_w) * source[0]) // out_w
        depth = depth[rows][:, cols]
    return zlib.compress(np.ascontiguousarray(depth).tobytes(), 1), (out_w, out_h), source
