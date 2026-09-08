import time
import unittest

import estop


class EstopPayloadTests(unittest.TestCase):
    def test_power_enabled_is_not_estop(self):
        now = int(time.time() * 1000)
        data = estop.build(
            {"actuator_status": True, "rcu_power_enabled": True, "fsm_state": "STOP"},
            now,
        )
        self.assertFalse(data["emergency_stop"])
        self.assertTrue(data["detection_supported"])

    def test_power_disabled_is_estop(self):
        now = int(time.time() * 1000)
        data = estop.build(
            {"actuator_status": False, "rcu_power_enabled": False, "fsm_state": "STOP"},
            now,
        )
        self.assertTrue(data["emergency_stop"])
        self.assertEqual(data["fsm_state"], "STOP")

    def test_either_physical_signal_fails_safe(self):
        now = int(time.time() * 1000)
        data = estop.build(
            {"actuator_status": True, "rcu_power_enabled": False}, now
        )
        self.assertTrue(data["emergency_stop"])
        self.assertIn("不一致", data["message"])

    def test_stale_state_is_unknown(self):
        old = int(time.time() * 1000) - 6000
        data = estop.build({"actuator_status": False}, old)
        self.assertIsNone(data["emergency_stop"])
        self.assertIn("过期", data["message"])

    def test_missing_physical_signals_is_unavailable(self):
        now = int(time.time() * 1000)
        data = estop.build({"fsm_state": "STOP", "error": "offline"}, now)
        self.assertFalse(data["available"])
        self.assertFalse(data["detection_supported"])
        self.assertIsNone(data["emergency_stop"])

    def test_plugin_info_refreshes_pac_state(self):
        class FakeGrpc:
            def get_robot_state(self):
                return {"fsm_state": "STOP"}

        plugin = estop.Plugin(
            {},
            "adam",
            None,
            FakeGrpc(),
            status_reader=lambda: {
                "actuator_status": False,
                "rcu_power_enabled": False,
                "fsm_state": "STOP",
            },
        )
        result = plugin.dispatch("info", {})
        self.assertTrue(result["data"]["emergency_stop"])
        self.assertEqual(
            result["topic_out"],
            [{"topic": "/adam/state/estop", "format": "data/json"}],
        )


if __name__ == "__main__":
    unittest.main()
