import json
from pathlib import Path
import queue
import sys
import threading
import time
import types
import unittest


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {})
    qos = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    policy = type(
        "Policy",
        (),
        {
            "RELIABLE": "reliable",
            "BEST_EFFORT": "best_effort",
            "KEEP_LAST": "keep_last",
            "VOLATILE": "volatile",
        },
    )
    qos.QoSProfile = QoSProfile
    qos.ReliabilityPolicy = policy
    qos.HistoryPolicy = policy
    qos.DurabilityPolicy = policy
    rclpy.node = node
    rclpy.qos = qos
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", node)
    sys.modules.setdefault("rclpy.qos", qos)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    std_msgs.msg = std_msgs_msg
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)


_install_ros_stubs()

from proposal_controller import Go2VelocityProposalController  # noqa: E402
from velocity_proposal import ProposalLimits, VelocityProposalGate  # noqa: E402


class FakeRpc:
    def __init__(self):
        self.moves = []
        self.stops = 0
        self.on_stop = None
        self.move_delay = 0.0

    def Move(self, vx, vy, vyaw):
        time.sleep(self.move_delay)
        self.moves.append((vx, vy, vyaw))
        return 0

    def StopMove(self):
        self.stops += 1
        if self.on_stop:
            self.on_stop()
        return 0


def proposal(nav_id="nav-1", sequence=1, **changes):
    payload = {
        "schema": "phanthy.navigation.velocity_proposal.v1",
        "nav_id": nav_id,
        "sequence": sequence,
        "issued_at_unix_ms": time.time() * 1000.0,
        "ttl_ms": 250,
        "frame": "base_link",
        "shadow_only": True,
        "physical_execution": False,
        "nav_status": "navigating",
        "velocity": {"x": 0.2, "y": 0.0, "yaw": 0.4},
    }
    payload.update(changes)
    return payload


class ProposalControllerTest(unittest.TestCase):
    def make_controller(self):
        controller = Go2VelocityProposalController.__new__(
            Go2VelocityProposalController
        )
        controller._rpc = FakeRpc()
        controller._topic = "/ubuntu/navigation/nav2/velocity_proposal"
        controller._proposal_sub = None
        controller._limits = ProposalLimits()
        controller._gate = VelocityProposalGate(controller._limits)
        controller._gate.bind("/ubuntu/navigation/nav2/velocity_proposal")
        controller._lock = threading.RLock()
        controller._rpc_lock = threading.Lock()
        controller._state_condition = threading.Condition()
        controller._last_state_monotonic = 0.0
        controller._last_velocity = (float("inf"),) * 3
        controller._stop_timeout = 0.05
        controller._linear_epsilon = 0.04
        controller._yaw_epsilon = 0.08
        controller._commands = queue.Queue(maxsize=1)
        controller._stop_transition = threading.Event()
        controller._counters = {
            "received": 0,
            "accepted": 0,
            "applied": 0,
            "rejected": 0,
            "coalesced": 0,
            "rejections_by_reason": {},
            "watchdog_faults_by_reason": {},
        }
        controller._last_rpc = {}
        return controller

    def test_callback_keeps_only_latest_valid_proposal(self):
        controller = self.make_controller()
        controller._on_proposal(
            types.SimpleNamespace(data=json.dumps(proposal(sequence=1)))
        )
        controller._on_proposal(
            types.SimpleNamespace(data=json.dumps(proposal(sequence=2)))
        )

        queued = controller._commands.get_nowait()
        self.assertEqual(queued["kind"], "move")
        self.assertEqual(queued["sequence"], 2)
        self.assertEqual(controller._counters["coalesced"], 1)

    def test_invalid_active_proposal_replaces_motion_with_stop(self):
        controller = self.make_controller()
        controller._on_proposal(
            types.SimpleNamespace(data=json.dumps(proposal(sequence=1)))
        )
        invalid = proposal(sequence=2)
        invalid["velocity"]["yaw"] = 3.0
        controller._on_proposal(types.SimpleNamespace(data=json.dumps(invalid)))

        queued = controller._commands.get_nowait()
        self.assertEqual(queued, {"kind": "stop", "reason": "velocity_yaw_limit"})
        self.assertFalse(controller._gate.armed)

    def test_rpc_completion_after_ttl_requests_a_stop(self):
        controller = self.make_controller()
        controller._on_proposal(
            types.SimpleNamespace(data=json.dumps(proposal(sequence=1)))
        )
        command = controller._commands.get_nowait()
        command["deadline"] = time.monotonic() + 0.001
        controller._rpc.move_delay = 0.005

        controller._apply_move(command)

        self.assertEqual(controller._counters["applied"], 0)
        self.assertEqual(controller._counters["rejections_by_reason"], {"proposal_ttl_expired": 1})
        self.assertEqual(
            controller._commands.get_nowait(),
            {"kind": "stop", "reason": "proposal_ttl_expired"},
        )

    def test_stop_requires_a_fresh_zero_loco_state(self):
        controller = self.make_controller()

        def publish_zero():
            with controller._state_condition:
                controller._last_velocity = (0.01, 0.01, 0.02)
                controller._last_state_monotonic = time.monotonic()
                controller._state_condition.notify_all()

        controller._rpc.on_stop = publish_zero
        ret, confirmed, error = controller._stop_and_confirm()

        self.assertEqual(ret, 0)
        self.assertTrue(confirmed)
        self.assertIsNone(error)
        self.assertEqual(controller._rpc.stops, 1)

    def test_repeated_connect_rejects_a_conflicting_explicit_lease(self):
        controller = self.make_controller()
        controller._gate.bind(controller._topic, "nav-1")
        controller._proposal_sub = object()

        result = controller.connect(controller._topic, "nav-2")

        self.assertEqual(result["error"], "proposal_lease_already_connected")
        self.assertEqual(result["expected_nav_id"], "nav-1")

    def test_subscription_creation_failure_leaves_gate_unbound(self):
        controller = self.make_controller()
        controller._gate.unbind("test_reset")
        controller._stop_and_confirm = lambda: (0, True, None)
        controller.create_subscription = lambda *_args: (_ for _ in ()).throw(
            RuntimeError("subscribe failed")
        )

        result = controller.connect(controller._topic)

        self.assertEqual(result["error"], "subscribe failed")
        self.assertFalse(result["connected"])
        self.assertEqual(result["last_reason"], "proposal_bind_failed")

    def test_manual_override_fails_closed_without_zero_confirmation(self):
        controller = self.make_controller()
        controller._stop_and_confirm = lambda: (0, False, None)

        self.assertFalse(controller.manual_override())
        self.assertFalse(controller._gate.armed)
        self.assertEqual(controller._gate.last_reason, "manual_override")

        source = (Path(__file__).resolve().parents[1] / "device.py").read_text()
        self.assertIn('"error": "manual_override_stop_unconfirmed"', source)

    def test_spawned_rpc_worker_installs_atomic_logging_first(self):
        source = (Path(__file__).resolve().parents[1] / "rpc_proxy.py").read_text()
        worker = source[source.index("def _rpc_worker") : source.index("class RpcProxy")]
        self.assertLess(
            worker.index("logsafe.install(check_fd=False)"),
            worker.index("ChannelFactoryInitialize"),
        )


if __name__ == "__main__":
    unittest.main()
