"""Hardware-free tests for Adam's Livox MID-360 card."""

from __future__ import annotations

import json
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml

import device
from lidar import (
    LivoxDriverRuntime,
    LivoxLidarPlugin,
    _rotation_matrix,
    build_livox_config,
    encode_pointcloud,
)


class _Field:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


def _cloud(points, *, bigendian=False, row_padding=0):
    endian = ">" if bigendian else "<"
    point_step = 20
    rows = []
    for point in points:
        record = bytearray(point_step)
        struct.pack_into(endian + "fff", record, 4, *point)
        rows.append(bytes(record))
    data = b"".join(rows) + bytes(row_padding)
    return types.SimpleNamespace(
        width=len(points), height=1, point_step=point_step,
        row_step=len(points) * point_step + row_padding,
        is_bigendian=bigendian, data=data,
        fields=[_Field("intensity", 0), _Field("x", 4),
                _Field("y", 8), _Field("z", 12)],
    )


class PointCloudEncodingTests(unittest.TestCase):
    def encode(self, message, max_points=10000, rotation=None, translation=None):
        return encode_pointcloud(
            message, max_points=max_points, max_distance_m=None,
            rotation=np.eye(3, dtype=np.float32) if rotation is None else rotation,
            translation=np.zeros(3, dtype=np.float32) if translation is None else translation)

    def test_offsets_padding_and_renderer_axis_map(self):
        packet = self.encode(_cloud([(1.0, 2.0, 3.0)], row_padding=7))
        self.assertEqual((12, 1), struct.unpack_from("<II", packet))
        self.assertEqual((-3.0, 1.0, -2.0), struct.unpack_from("<fff", packet, 8))

    def test_big_endian_and_invalid_points(self):
        packet = self.encode(_cloud([
            (1.0, 2.0, 3.0),
            (float("nan"), 0.0, 0.0),
            (4.0, 5.0, 6.0),
        ], bigendian=True))
        self.assertEqual((12, 2), struct.unpack_from("<II", packet))
        self.assertEqual((-3.0, 1.0, -2.0), struct.unpack_from("<fff", packet, 8))
        self.assertEqual((-6.0, 4.0, -5.0), struct.unpack_from("<fff", packet, 20))

    def test_uniform_sampling_keeps_extent(self):
        packet = self.encode(
            _cloud([(float(i), 0.0, 0.0) for i in range(11)]), max_points=3)
        self.assertEqual((12, 3), struct.unpack_from("<II", packet))
        xs = [struct.unpack_from("<fff", packet, 8 + index * 12)[1]
              for index in range(3)]
        self.assertEqual([0.0, 5.0, 10.0], xs)

    def test_mount_transform_before_packing(self):
        packet = self.encode(
            _cloud([(1.0, 0.0, 0.0)]),
            rotation=_rotation_matrix(0, 0, 90),
            translation=np.asarray([1.0, 2.0, 3.0], dtype=np.float32))
        np.testing.assert_allclose(
            struct.unpack_from("<fff", packet, 8), (-3.0, 1.0, -3.0), atol=1e-6)

    def test_rejects_truncated_cloud(self):
        message = _cloud([(1.0, 2.0, 3.0)])
        message.data = message.data[:-1]
        with self.assertRaisesRegex(ValueError, "shorter"):
            self.encode(message)


class RuntimeConfigurationTests(unittest.TestCase):
    def test_network_config_matches_observed_link(self):
        config = build_livox_config({
            "host_ip": "192.168.1.50", "lidar_ip": "192.168.1.173"})
        self.assertFalse(config["lidar_log_enable"])
        host = config["MID360"]["host_net_info"]
        self.assertEqual("192.168.1.50", host["cmd_data_ip"])
        self.assertEqual("192.168.1.50", host["point_data_ip"])
        self.assertEqual("192.168.1.50", host["imu_data_ip"])
        self.assertEqual(56301, host["point_data_port"])
        self.assertEqual(56401, host["imu_data_port"])
        self.assertEqual("192.168.1.173", config["lidar_configs"][0]["ip"])

    def test_runtime_writes_config_and_requests_pointcloud2(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mid360.json"
            runtime = LivoxDriverRuntime({
                "driver_config_path": str(path),
                "host_ip": "192.168.1.50", "lidar_ip": "192.168.1.173",
                "publish_freq_hz": 10,
            })
            runtime._write_config()
            self.assertEqual("192.168.1.173", json.loads(path.read_text())[
                "lidar_configs"][0]["ip"])
            command = runtime.command()
            self.assertIn("xfer_format:=0", command)
            self.assertIn("publish_freq:=10.0", command)
            self.assertIn(f"user_config_path:={path}", command)


class _FakeNode:
    output_topic = "/adam/lidar/cloud"
    source_topic = "/livox/lidar"

    def __init__(self, config, namespace):
        self.active = True

    def set_active(self, active):
        self.active = active

    def status(self):
        return {"received": 3, "published": 2}

    def close(self):
        pass


class _FakeRuntime:
    def __init__(self, config):
        self.starts = 0
        self.stops = 0

    def start(self):
        self.starts += 1

    def stop(self):
        self.stops += 1

    def status(self):
        return {"driver_state": "running"}


class _FakeExecutor:
    def add_node(self, node):
        self.node = node

    def remove_node(self, node):
        pass


class PluginContractTests(unittest.TestCase):
    def plugin(self):
        return LivoxLidarPlugin({}, "adam", _FakeExecutor(),
                                node_factory=_FakeNode, runtime_factory=_FakeRuntime)

    def test_card_contract_and_warm_stop(self):
        plugin = self.plugin()
        tool = plugin.get_tool()
        self.assertEqual("lidar_cloud", tool["name"])
        self.assertEqual("sensor/pointcloud", tool["topic_out"][0]["format"])
        plugin.start()
        stopped = plugin.dispatch("stop", {})
        self.assertEqual("idle", stopped["state"])
        self.assertFalse(plugin._node.active)
        self.assertEqual(0, plugin._runtime.stops)
        plugin.stop()
        self.assertEqual(1, plugin._runtime.stops)

    def test_repository_configuration_and_packaging_are_in_sync(self):
        base = Path(__file__).resolve().parent
        config = yaml.safe_load((base / "config.yaml").read_text())
        manifest = yaml.safe_load((base / "driver.yaml").read_text())
        self.assertTrue(config["plugins"]["lidar"]["enabled"])
        self.assertIn("lidar_cloud", {card["name"] for card in manifest["cards"]})
        dockerfile = (base / "Dockerfile").read_text()
        self.assertIn("COPY lidar.py", dockerfile)
        self.assertIn("source /livox_ws/install/setup.bash", dockerfile)
        self.assertIn("c0796f04c143143899c87a773d9f6b7136453c0b", dockerfile)
        self.assertIn("21445540f0d100dc86a7e6df312dd70bbdb4afdf", dockerfile)

    def test_adam_bundle_registers_the_lidar_card(self):
        class BundleLidar:
            def __init__(self, config, namespace, executor):
                self.topic = f"/{namespace}/lidar/cloud"

            def get_tool(self):
                return {"name": "lidar_cloud", "type": "sensor",
                        "topic_out": [{"topic": self.topic,
                                       "format": "sensor/pointcloud"}]}

            def start(self):
                pass

            def stop(self):
                pass

        disabled = {name: {"enabled": False} for name in (
            "state", "estop", "loco", "motion", "tracking_motion", "camera",
            "vision_capture", "hand", "hand_gesture", "hand_state", "arm",
            "waist", "head", "model",
        )}
        fake_module = types.SimpleNamespace(LivoxLidarPlugin=BundleLidar)
        with mock.patch.object(device, "HAS_ROS2", True), mock.patch.dict(
                sys.modules, {"lidar": fake_module}):
            bundle = device.AdamDeviceBundle(
                {"plugins": {**disabled, "lidar": {"enabled": True}}},
                "adam", _FakeExecutor(), None, ros2_enabled=True)
        self.assertIn("lidar_cloud", bundle._tool_map)


if __name__ == "__main__":
    unittest.main()
