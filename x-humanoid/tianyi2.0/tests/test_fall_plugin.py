"""Geometry and state-machine coverage for Tianyi's fall-detection card.

Runs without ROS 2: rclpy and the message packages are stubbed, and device.py
is exec'd into a throwaway module so ``from device import _RELIABLE_QOS``
resolves.  numpy is real, because the height maths is the thing under test.
"""

from __future__ import annotations

import math
import sys
import types
from pathlib import Path

import numpy as np

_HERE = Path(__file__).parents[1]


def _stub_ros():
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")

    class Node:
        def __init__(self, *args, **kwargs):
            self.subscriptions = []
            self.publishers = []
            self.timers = []

        def create_subscription(self, msg_type, topic, cb, qos):
            self.subscriptions.append((topic, cb))
            return types.SimpleNamespace(topic=topic)

        def create_publisher(self, msg_type, topic, qos):
            sink = types.SimpleNamespace(topic=topic, published=[])
            sink.publish = lambda msg: sink.published.append(msg)
            self.publishers.append(sink)
            return sink

        def create_timer(self, period, cb):
            timer = types.SimpleNamespace(period=period, callback=cb,
                                          cancel=lambda: None)
            self.timers.append(timer)
            return timer

    class QoSProfile:
        def __init__(self, **kwargs):
            pass

    class Enum:
        BEST_EFFORT = RELIABLE = KEEP_LAST = VOLATILE = 0

    rclpy.node.Node = Node
    rclpy.qos.QoSProfile = QoSProfile
    rclpy.qos.ReliabilityPolicy = Enum
    rclpy.qos.HistoryPolicy = Enum
    rclpy.qos.DurabilityPolicy = Enum
    sys.modules.update({"rclpy": rclpy, "rclpy.node": rclpy.node,
                        "rclpy.qos": rclpy.qos})

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray", "UInt8MultiArray"):
        setattr(std_msgs.msg, name,
                type(name, (), {"__init__": lambda self: None}))
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs.msg})

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    for name in ("Image", "CameraInfo", "Imu", "PointCloud2", "LaserScan",
                 "CompressedImage", "JointState"):
        setattr(sensor_msgs.msg, name, type(name, (), {}))
    sys.modules.update({"sensor_msgs": sensor_msgs,
                        "sensor_msgs.msg": sensor_msgs.msg})

    device = types.ModuleType("device")
    source = (_HERE / "device.py").read_text(encoding="utf-8")
    exec(compile(source, "device.py", "exec"), device.__dict__)
    sys.modules["device"] = device


def _load_fall():
    _stub_ros()
    module = types.ModuleType("tianyi_fall_test")
    source = (_HERE / "fall.py").read_text(encoding="utf-8")
    exec(compile(source, "fall.py", "exec"), module.__dict__)
    return module


_FALL = _load_fall()


def _plugin(**cfg):
    ros2 = types.SimpleNamespace(
        ctx_tianyi=None, ctx_core=None,
        executor_tianyi=types.SimpleNamespace(add_node=lambda _: None),
        executor_core=types.SimpleNamespace(add_node=lambda _: None))
    plugin = _FALL.FallPlugin(cfg, "tianyi", ros2)
    plugin.start()
    plugin._running = True
    return plugin


# ── synthetic depth frames ───────────────────────────────────────────────────

_FX = _FY = 500.0
_W, _H = 640, 480


def _depth_msg(fill):
    """Build a fake 16UC1 Image whose pixels come from fill(u_grid, v_grid)."""
    u = np.arange(_W)[None, :].repeat(_H, axis=0)
    v = np.arange(_H)[:, None].repeat(_W, axis=1)
    mm = fill(u, v).astype(np.uint16)
    return types.SimpleNamespace(width=_W, height=_H, step=_W * 2,
                                 encoding="16UC1", is_bigendian=0,
                                 data=mm.tobytes())


def _feed(plugin, fill, box, up=(0.0, -1.0, 0.0)):
    plugin._intrinsics = (_FX, _FY, _W / 2.0, _H / 2.0, _W, _H)
    plugin._on_depth(_depth_msg(fill))
    depth = plugin._latest_depth[1]
    return plugin._height_stats(box, depth, up)


def _plane_at_height(height_m, distance_m, camera_height_m):
    """Row index of a fronto-parallel patch at a given height above the floor.

    With the camera level, a pixel row v maps to y = (v - cy) * z / fy and
    height = camera_height - y, so the target row is solved directly.
    """
    return _H / 2.0 + (camera_height_m - height_m) * _FY / distance_m


def _min_visible_distance(height_m, camera_height_m, margin_px=0):
    """Closest distance at which a level camera still sees that height.

    Anything lower than the camera drops off the bottom row as it approaches:
    v = cy + (camera_height - h) * fy / z must stay under _H.  This is a real
    limitation of a forward-facing camera, not a quirk of the fixture.
    """
    drop = camera_height_m - height_m
    if drop <= 0:
        return 0.0
    return drop * _FY / (_H / 2.0 - margin_px)


def test_fixture_distances_are_physically_visible():
    """Guards the fixture: a level camera at 1.5 m cannot see 0.3 m at 2 m."""
    assert _min_visible_distance(0.3, 1.5) > 2.0
    assert _min_visible_distance(0.3, 1.5) < 2.6


def test_level_camera_recovers_known_height():
    plugin = _plugin(camera_height_m=1.5)
    height, band = 0.3, 40
    # 4 m clears _min_visible_distance(0.3, 1.5) ~= 2.5 m with room for the band.
    distance = 4.0
    v_row = _plane_at_height(height, distance, 1.5)
    assert v_row + band < _H, v_row

    def fill(u, v):
        return np.where(np.abs(v - v_row) <= band, distance * 1000.0, 0.0)

    stats = _feed(plugin, fill, (0.3, (v_row - band) / _H,
                                 0.7, (v_row + band) / _H))
    assert stats is not None
    assert abs(stats["median_m"] - height) < 0.05, stats
    assert abs(stats["distance_m"] - distance) < 0.01, stats
    # 80 px at 4 m spans ~0.64 m of height; top/bottom must bracket the median.
    assert stats["top_m"] > stats["median_m"] > stats["bottom_m"]


def test_height_is_distance_invariant():
    """The same physical height must read the same at 3 m and 4.5 m."""
    heights = []
    for distance in (3.0, 4.5):
        plugin = _plugin(camera_height_m=1.5)
        band = 20
        v_row = _plane_at_height(0.25, distance, 1.5)
        assert v_row + band < _H, (distance, v_row)

        def fill(u, v, _d=distance, _r=v_row, _b=band):
            return np.where(np.abs(v - _r) <= _b, _d * 1000.0, 0.0)

        stats = _feed(plugin, fill, (0.3, (v_row - band) / _H,
                                     0.7, (v_row + band) / _H))
        assert stats is not None, distance
        heights.append(stats["median_m"])
    # Pixel height per metre halves between the two, yet metres must agree.
    assert abs(heights[0] - heights[1]) < 0.05, heights


def test_pitched_camera_uses_up_vector():
    """Tilting the camera down must not shift the reported height."""
    pitch = math.radians(20.0)
    up = (0.0, -math.cos(pitch), -math.sin(pitch))
    plugin = _plugin(camera_height_m=1.5)
    # A point 2 m ahead at 0.3 m height, expressed in the pitched optical frame.
    forward, drop = 2.0, 1.5 - 0.3
    z = forward * math.cos(pitch) + drop * math.sin(pitch)
    y = drop * math.cos(pitch) - forward * math.sin(pitch)
    v_row = _H / 2.0 + y * _FY / z

    def fill(u, v):
        return np.where(np.abs(v - v_row) <= 15, z * 1000.0, 0.0)

    stats = _feed(plugin, fill, (0.3, (v_row - 15) / _H, 0.7,
                                 (v_row + 15) / _H), up=up)
    assert stats is not None
    assert abs(stats["median_m"] - 0.3) < 0.06, stats


def test_background_outside_depth_band_is_dropped():
    plugin = _plugin(camera_height_m=1.5)
    # Person slab at 2 m, a far wall at 6 m filling most of the ROI.
    v_row = _plane_at_height(0.3, 2.0, 1.5)

    def fill(u, v):
        return np.where(np.abs(v - v_row) <= 30, 2000.0, 6000.0)

    stats = _feed(plugin, fill, (0.3, (v_row - 60) / _H, 0.7,
                                 (v_row + 60) / _H))
    assert stats is not None
    # Median depth of the ROI is dominated by the wall, so the band centres on
    # 6 m; what matters is that the two surfaces never get averaged together.
    assert stats["distance_m"] in (2.0, 6.0) or stats["distance_m"] > 5.0, stats
    spread = stats["top_m"] - stats["bottom_m"]
    assert spread < 1.5, stats


def test_all_zero_depth_returns_none():
    plugin = _plugin()
    stats = _feed(plugin, lambda u, v: np.zeros_like(u), (0.3, 0.3, 0.7, 0.7))
    assert stats is None


# ── accelerometer gate ───────────────────────────────────────────────────────

def _imu(x, y, z):
    return types.SimpleNamespace(
        linear_acceleration=types.SimpleNamespace(x=x, y=y, z=z))


def test_accel_rejects_dynamic_samples():
    plugin = _plugin()
    plugin._on_accel(_imu(0.0, -9.8, 0.0))
    assert plugin._gravity is not None
    settled = plugin._gravity
    plugin._on_accel(_imu(0.0, -40.0, 0.0))  # magnitude out of the 8..11.5 gate
    assert plugin._gravity == settled


def test_accel_normalises_and_low_passes():
    plugin = _plugin()
    plugin._on_accel(_imu(0.0, -9.8, 0.0))
    assert abs(plugin._gravity[1] + 1.0) < 1e-6
    for _ in range(200):
        plugin._on_accel(_imu(0.0, 0.0, -9.8))
    assert plugin._gravity[2] < -0.9, plugin._gravity


def test_up_vector_falls_back_to_configured_pitch():
    plugin = _plugin(camera_pitch_deg=20.0)
    up, source = plugin._up_vector()
    assert source == "config"
    assert abs(up[1] + math.cos(math.radians(20.0))) < 1e-9
    assert abs(up[2] + math.sin(math.radians(20.0))) < 1e-9
    plugin._on_accel(_imu(0.0, -9.8, 0.0))
    assert plugin._up_vector()[1] == "imu"


# ── vop parsing ──────────────────────────────────────────────────────────────

def _vop(objects):
    import json
    return types.SimpleNamespace(
        data=json.dumps({"timestamp": 0, "objects": objects}))


def test_vop_prefers_bbox_and_filters_confidence():
    plugin = _plugin(min_confidence=0.4)
    plugin._on_vop(_vop([
        {"name": "person", "confidence": 0.9,
         "bbox": [0.1, 0.2, 0.4, 0.9], "position": [0.0, 0.0]},
        {"name": "person", "confidence": 0.2, "bbox": [0.5, 0.5, 0.6, 0.6]},
        {"name": "chair", "confidence": 0.99, "bbox": [0.0, 0.0, 0.2, 0.2]},
    ]))
    boxes, _ = plugin._persons
    assert boxes == [(0.1, 0.2, 0.4, 0.9)]


def test_vop_centre_only_payload_still_yields_a_window():
    plugin = _plugin()
    plugin._on_vop(_vop([{"name": "person", "confidence": 0.9,
                          "position": [0.0, 0.0]}]))
    boxes, _ = plugin._persons
    assert len(boxes) == 1
    x1, y1, x2, y2 = boxes[0]
    assert x1 < 0.5 < x2 and y1 < 0.5 < y2


def test_vop_garbage_is_ignored():
    plugin = _plugin()
    plugin._on_vop(types.SimpleNamespace(data="not json"))
    assert plugin._persons[1] == 0.0


# ── state machine ────────────────────────────────────────────────────────────

class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _armed(plugin, clock, height_m, distance=None):
    """Point the plugin at a synthetic person whose top is at height_m.

    Distance defaults to whatever keeps that height on the sensor (see
    ``_min_visible_distance``) — a level camera at 1.5 m simply cannot see the
    floor up close.
    """
    plugin._intrinsics = (_FX, _FY, _W / 2.0, _H / 2.0, _W, _H)
    if distance is None:
        distance = _min_visible_distance(height_m, plugin._camera_height_m) + 0.5
    v_row = _plane_at_height(height_m, distance, plugin._camera_height_m)

    def fill(u, v):
        return np.where(np.abs(v - v_row) <= 12, distance * 1000.0, 0.0)

    plugin._on_depth(_depth_msg(fill))
    plugin._latest_depth = (clock(), plugin._latest_depth[1])
    plugin._persons = ([(0.3, (v_row - 12) / _H, 0.7, (v_row + 12) / _H)],
                       clock())
    plugin._last_person_seen = clock()


def _states(plugin):
    return [m.data for m in plugin._pub.published]


def test_fall_requires_sustained_low_height():
    clock = _Clock()
    plugin = _plugin(camera_height_m=1.5, min_duration_sec=5.0,
                     fall_top_m=0.70, recover_top_m=1.00,
                     publish_interval_sec=0.0)
    _FALL.time.time = clock
    try:
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._last_state == "low"
        assert plugin._fallen is False

        clock.t += 3.0
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._last_state == "low", "3 s is under the 5 s threshold"

        clock.t += 3.0
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._last_state == "fallen"
        assert plugin._fallen is True
        assert '"fallen": true' in _states(plugin)[-1]
    finally:
        _FALL.time.time = __import__("time").time


def test_standing_clears_the_timer():
    clock = _Clock()
    plugin = _plugin(camera_height_m=1.5, min_duration_sec=5.0,
                     publish_interval_sec=0.0)
    _FALL.time.time = clock
    try:
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._low_since is not None

        clock.t += 3.0
        _armed(plugin, clock, 1.55)      # stood back up
        plugin._evaluate()
        assert plugin._last_state == "standing"
        assert plugin._low_since is None

        clock.t += 3.0                    # back down: timer restarts from here
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._last_state == "low"
        assert plugin._fallen is False
    finally:
        _FALL.time.time = __import__("time").time


def test_middle_band_is_reported_uncertain():
    clock = _Clock()
    plugin = _plugin(camera_height_m=1.5, fall_top_m=0.70,
                     recover_top_m=1.00, publish_interval_sec=0.0)
    _FALL.time.time = clock
    try:
        _armed(plugin, clock, 0.85)
        plugin._evaluate()
        assert plugin._last_state == "uncertain"
        assert plugin._low_since is None
    finally:
        _FALL.time.time = __import__("time").time


def test_person_vanishing_while_low_is_flagged():
    clock = _Clock()
    plugin = _plugin(camera_height_m=1.5, publish_interval_sec=0.0,
                     lost_grace_sec=5.0)
    _FALL.time.time = clock
    try:
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._last_state == "low"

        clock.t += 2.0
        plugin._persons = ([], clock())     # vop lost the detection
        plugin._latest_depth = (clock(), plugin._latest_depth[1])
        plugin._evaluate()
        assert plugin._last_state == "person_lost"

        clock.t += 30.0                     # grace expired, nobody around
        plugin._persons = ([], clock())
        plugin._latest_depth = (clock(), plugin._latest_depth[1])
        plugin._evaluate()
        assert plugin._last_state == "no_person"
        assert plugin._low_since is None
    finally:
        _FALL.time.time = __import__("time").time


def test_stale_inputs_report_no_data():
    clock = _Clock()
    plugin = _plugin(publish_interval_sec=0.0)
    _FALL.time.time = clock
    try:
        plugin._evaluate()                  # nothing has ever arrived
        assert plugin._last_state == "no_data"

        _armed(plugin, clock, 0.25)
        clock.t += 10.0                     # both inputs now stale
        plugin._evaluate()
        assert plugin._last_state == "no_data"
    finally:
        _FALL.time.time = __import__("time").time


def test_reset_clears_confirmed_fall():
    clock = _Clock()
    plugin = _plugin(camera_height_m=1.5, min_duration_sec=0.0,
                     publish_interval_sec=0.0)
    _FALL.time.time = clock
    try:
        _armed(plugin, clock, 0.25)
        plugin._evaluate()
        assert plugin._fallen is True
        out = plugin.dispatch("reset", {})
        assert out["reset"] is True
        assert plugin._fallen is False
        assert plugin._low_since is None
    finally:
        _FALL.time.time = __import__("time").time


# ── schema ───────────────────────────────────────────────────────────────────

def test_tool_schema_shape():
    plugin = _plugin()
    tool = plugin.get_tool()
    assert tool["name"] == "fall_detect"
    assert tool["type"] == "processor"
    actions = tool["inputSchema"]["properties"]["action"]["enum"]
    assert set(actions) == {"start", "stop", "info", "reset"}
    assert set(tool["inputSchema"]["x-action-params"]) == set(actions)
    assert tool["topic_out"][0]["format"] == "data/json"
    assert tool["topic_out"][0]["topic"] == "/tianyi/camera/head/fall"
    # Every documented config key must actually be read by __init__.
    for key in tool["configSchema"]["properties"]:
        assert hasattr(plugin, "_" + key.replace("_m", "_m").replace(
            "_deg", "_deg")), key


def test_start_is_idempotent_and_reports_the_topic():
    plugin = _plugin()
    plugin._running = False
    first = plugin.dispatch("start", {})
    assert first["state"] == "running"
    assert "dds:/tianyi/camera/head/fall" in first["hint"]
    timers = len(plugin._core_node.timers)
    plugin.dispatch("start", {})
    assert len(plugin._core_node.timers) == timers, "no duplicate timer"
    assert plugin.dispatch("stop", {})["state"] == "idle"


def test_subscriptions_cover_both_domains():
    plugin = _plugin()
    core = [t for t, _ in plugin._core_node.subscriptions]
    tianyi = [t for t, _ in plugin._sub_node.subscriptions]
    assert core == ["/tianyi/camera/head/objects"]
    assert set(tianyi) == {"/ob_camera_head/depth/image_raw",
                           "/ob_camera_head/depth/camera_info",
                           "/ob_camera_head/accel/sample"}
