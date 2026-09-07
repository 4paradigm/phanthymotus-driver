import time
import unittest

import estop


class EstopPayloadTests(unittest.TestCase):
    def test_explicit_estop_is_detected(self):
        now = int(time.time() * 1000)
        data = estop.build({"fsm_state": "E_STOP"}, now)
        self.assertTrue(data["emergency_stop"])
        self.assertTrue(data["fsm_estop_detected"])
        self.assertTrue(data["detection_supported"])

    def test_stop_mode_is_not_physical_estop(self):
        now = int(time.time() * 1000)
        data = estop.build({"fsm_state": "Stop"}, now)
        self.assertFalse(data["emergency_stop"])
        self.assertFalse(data["fsm_estop_detected"])

    def test_stale_estop_remains_reported_but_not_current(self):
        old = int(time.time() * 1000) - 6000
        data = estop.build({"fsm_state": "emergency-stop"}, old)
        self.assertIsNone(data["emergency_stop"])
        self.assertFalse(data["fsm_estop_detected"])
        self.assertTrue(data["fsm_estop_reported"])

    def test_legacy_numeric_mode_is_unknown_not_safe(self):
        now = int(time.time() * 1000)
        data = estop.build({"mode": 4}, now)
        self.assertIsNone(data["emergency_stop"])
        self.assertFalse(data["detection_supported"])
        self.assertIn("无法可靠判断", data["message"])

    def test_rpc_error_is_unavailable(self):
        now = int(time.time() * 1000)
        data = estop.build({"error": "offline"}, now)
        self.assertFalse(data["available"])
        self.assertIsNone(data["emergency_stop"])

    def test_plugin_info_refreshes_grpc_state(self):
        class FakeGrpc:
            def get_robot_state(self):
                return {"fsm_state": "ESTOP"}

        plugin = estop.Plugin({}, "adam", None, FakeGrpc())
        result = plugin.dispatch("info", {})
        self.assertTrue(result["data"]["emergency_stop"])
        self.assertEqual(
            result["topic_out"],
            [{"topic": "/adam/state/estop", "format": "data/json"}],
        )


if __name__ == "__main__":
    unittest.main()
