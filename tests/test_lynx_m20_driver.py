from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics/lynx_m20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


protocol = load("lynx_m20_basic_server", DRIVER / "basic_server.py")
m20 = load("lynx_m20_contract", DRIVER / "device.py")


class FakeNative:
    def request(self, message_type, command, items=None):
        return {"type": message_type, "command": command, "items": items or {}}
    def send_velocity(self, command, items):
        return {"command": command, "items": items}


class FakeNodes:
    def __init__(self): self.native = FakeNative()
    def publish_motion_state(self, state): return {"motion_state": state}
    def publish_gait(self, gait): return {"gait": gait}
    def set_ros_velocity(self, x, y, yaw): return {"velocity": [x, y, yaw]}
    def set_native_velocity(self, command, values): return self.native.send_velocity(command, values)
    def stop_velocity(self): pass
    def motion_summary(self): return {"state": 17}


class LynxM20ContractTests(unittest.TestCase):
    def test_basic_server_json_frame_round_trip(self):
        frame = protocol.encode_frame(7, 2, 23, {"GaitParam": 0x3002})
        message_id, payload = protocol.decode_frame(frame)
        self.assertEqual(7, message_id)
        body = payload["PatrolDevice"]
        self.assertEqual((2, 23), (body["Type"], body["Command"]))
        self.assertEqual(0x3002, body["Items"]["GaitParam"])
        self.assertEqual(protocol.SYNC, frame[:4])

    def test_tcp_stream_decoder_handles_fragmented_and_concatenated_frames(self):
        first = protocol.encode_frame(1, 100, 100)
        second = protocol.encode_frame(2, 1002, 6)
        decoder = protocol.StreamDecoder()
        self.assertEqual([], decoder.feed(first[:9]))
        decoded = decoder.feed(first[9:] + second)
        self.assertEqual([1, 2], [item[0] for item in decoded])

    def test_official_motion_commands_use_documented_values(self):
        plugin = m20.M20MotionPlugin(FakeNodes())
        self.assertEqual(1, plugin.dispatch("stand", {"transport": "native"})["items"]["MotionParam"])
        self.assertEqual(4, plugin.dispatch("lie", {"transport": "native"})["items"]["MotionParam"])
        gait = plugin.dispatch("gait", {"transport": "native", "gait": "agile_flat"})
        self.assertEqual((2, 23, 0x3002), (gait["type"], gait["command"], gait["items"]["GaitParam"]))
        axis = plugin.dispatch("axis", {"x": 0.5, "y": 0, "yaw": -0.2})
        self.assertEqual(21, axis["command"])
        with self.assertRaises(ValueError): plugin.dispatch("axis", {"x": 1.1, "y": 0, "yaw": 0})

    def test_device_commands_use_documented_native_protocol(self):
        plugin = m20.M20DevicePlugin(FakeNodes())
        lights = plugin.dispatch("lights", {"front": True, "rear": False})
        self.assertEqual((1101, 2, {"Front": 1, "Back": 0}), (lights["type"], lights["command"], lights["items"]))
        mode = plugin.dispatch("mode", {"mode": "assist"})
        self.assertEqual({"Mode": 2}, mode["items"])
        sleep = plugin.dispatch("auto_sleep", {"enabled": True, "minutes": 10})
        self.assertEqual({"Auto": True, "Sleep": False, "Time": 10}, sleep["items"])

    def test_standard_and_pro_boundaries_are_explicit(self):
        config = (DRIVER / "config.yaml").read_text()
        source = (DRIVER / "device.py").read_text()
        readme = (DRIVER / "README.md").read_text()
        self.assertIn('model_variant: "standard"', config)
        self.assertIn("if nodes.is_pro", source)
        self.assertIn("仅 M20 Pro", readme)
        self.assertIn("未提供舞蹈", readme)

    def test_ros2_uses_official_fastdds_topics_and_pinned_drdds(self):
        config = (DRIVER / "config.yaml").read_text()
        dockerfile = (DRIVER / "Dockerfile").read_text()
        for topic in ("/MOTION_STATE", "/GAIT", "/NAV_CMD", "/MOTION_INFO", "/IMU", "/LIDAR/POINTS", "/HES_STATUS", "/CHARGE", "/JOINTS_DATA"):
            self.assertIn(topic, config)
        self.assertIn("rmw_fastrtps_cpp", dockerfile)
        self.assertIn("a0d1a29eec5c4db5a9107595bb51e3be8122b86c", dockerfile)

    def test_no_stale_s10_identity_remains(self):
        old_model = "s" + "10"
        files = list(DRIVER.glob("*")) + [ROOT / "README.md", ROOT / "README_zh.md"]
        stale = [str(path) for path in files if path.is_file() and old_model in path.read_text().lower()]
        self.assertEqual([], stale)


if __name__ == "__main__":
    unittest.main()
