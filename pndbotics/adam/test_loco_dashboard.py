"""Unit tests for Adam's bounded Dashboard locomotion card."""

from __future__ import annotations

import unittest

from device import LocoPlugin


class _Grpc:
    def __init__(self):
        self.speed_commands = []

    def set_speed(self, vx, vy, vyaw):
        self.speed_commands.append((vx, vy, vyaw))
        return {"code": 0, "message": "ok"}


class LocoDashboardTests(unittest.TestCase):
    def setUp(self):
        self.grpc = _Grpc()
        self.plugin = LocoPlugin({
            "max_vx_mps": 0.25,
            "max_vy_mps": 0.15,
            "max_vyaw_radps": 0.5,
            "command_timeout_s": 60,
        }, "adam", None, self.grpc)

    def tearDown(self):
        self.plugin.stop()

    def test_move_is_clamped_and_advertises_limits(self):
        result = self.plugin.dispatch("move", {"vx": 1, "vy": -1, "vyaw": 2})

        self.assertEqual(self.grpc.speed_commands[-1], (0.25, -0.15, 0.5))
        self.assertEqual(result["command"], {"vx": 0.25, "vy": -0.15, "vyaw": 0.5})
        self.assertEqual(result["watchdog_timeout_s"], 60.0)
        self.assertEqual(self.plugin.dispatch("info", {})["limits"]["vx_mps"], 0.25)

    def test_invalid_speed_is_rejected_without_a_grpc_call(self):
        result = self.plugin.dispatch("move", {"vx": float("nan")})

        self.assertEqual(result, {"error": "vx must be a finite number"})
        self.assertEqual(self.grpc.speed_commands, [])

    def test_watchdog_and_stop_send_zero_speed(self):
        self.plugin.dispatch("move", {"vx": 0.1, "vy": 0.0, "vyaw": 0.0})
        self.plugin._stop_after_timeout(self.plugin._motion_sequence)
        self.assertEqual(self.grpc.speed_commands[-1], (0.0, 0.0, 0.0))

        self.plugin.dispatch("move", {"vx": 0.1})
        result = self.plugin.dispatch("stop", {})
        self.assertEqual(result["command"], "zero_speed")
        self.assertEqual(self.grpc.speed_commands[-1], (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
