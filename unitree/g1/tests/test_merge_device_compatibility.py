"""Offline merge regression: stop must not wait for a Move RPC reply."""
import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest import mock

from sensor_output import OutputGate


def loco_class():
    path = Path(__file__).resolve().parents[1] / "device.py"
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LocoPlugin")
    scope = {"threading": threading, "math": math, "time": time,
             "_SMS": SimpleNamespace(LOCO_STATES=set(), LIMP_STATES=set(),
                                     BALANCED_SQUAT=706, LIE_TO_STAND=702),
             "_loco_acp_notify": lambda *args, **kwargs: None,
             "resolve_input_topic": lambda args, default: default,
             "resolve_optional_expected_nav_id": lambda args: None,
             "DEFAULT_VELOCITY_PROPOSAL_TOPIC": "/proposal"}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), scope)
    return scope["LocoPlugin"]


def test_direct_timed_move_allows_concurrent_stop_and_records_cancellation():
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()

    class Client:
        def BeginMove(self, *args):
            def wait():
                entered.set()
                assert release.wait(2)
                return 0
            return wait

        def BeginStopMove(self):
            stopped.set()
            return lambda: 0

        def StopMove(self):
            stopped.set()
            return 0

    card = loco_class()({}, "test", None, Client())
    card._velocity_proposal_port = lambda: {}
    result = []
    move = threading.Thread(target=lambda: result.append(card.dispatch("move", {"duration": 30})))
    move.start()
    try:
        assert entered.wait(1)
        reply = card.dispatch("stop_move", {})
        assert stopped.is_set()
        assert reply["ret"] == 0
    finally:
        release.set()
        move.join(2)
    assert not move.is_alive()
    assert card._fallback_motion.get_result(result[0]["action_id"])["status"] == "cancelled"


def test_rejected_manual_move_does_not_stop_active_proposal():
    calls = []
    client = SimpleNamespace(StopMove=lambda: calls.append("stop"))
    smart = SimpleNamespace(move=lambda *args, **kwargs: {
        "error": "proposal active", "rejected": True})
    card = loco_class()({}, "test", None, client, smart_motion=smart)
    result = card.dispatch("move", {"duration": 1})
    assert result["rejected"] is True
    assert calls == []


def test_rejected_proposal_does_not_stop_active_timed_move():
    calls = []
    client = SimpleNamespace(StopMove=lambda: calls.append("stop"))
    smart = SimpleNamespace(bind_velocity_proposal=lambda *args: {
        "error": "timed_motion_not_idle", "rejected": True, "stop_confirmed": False})
    card = loco_class()({}, "test", None, client, smart_motion=smart)
    card._velocity_proposal_port = lambda: {}
    result = card.dispatch("start", {})
    assert result["rejected"] is True
    assert calls == []


def test_shutdown_disconnects_proposals_and_stops_timed_motion():
    card = loco_class()({}, "test", None, SimpleNamespace(StopMove=lambda: 0))
    calls = []
    card._disconnect_velocity_proposal = lambda reason: calls.append(("disconnect", reason))
    card._smart_motion = SimpleNamespace(stop=lambda reason: calls.append(("stop", reason)))
    card.stop()
    assert calls == [("disconnect", "driver_stop"), ("stop", "shutdown")]


def test_lidar_global_and_repeated_stop_gate_cloud_but_keep_imu():
    path = Path(__file__).resolve().parents[1] / "device.py"
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LidarPlugin")
    scope = {"threading": threading}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), scope)
    plugin = scope["LidarPlugin"].__new__(scope["LidarPlugin"])
    cloud, imu, legacy = OutputGate(), OutputGate(), OutputGate()
    stale_cloud, stale_legacy = cloud.token(), legacy.token()

    from test_navigation_sensor_card import NavigationSensorCardContractTest
    module = NavigationSensorCardContractTest.load_bridge_module()
    nav = module.NavigationSensorPlugin.__new__(module.NavigationSensorPlugin)
    nav._lifecycle_lock = threading.RLock()
    nav._gates = {"cloud": cloud, "imu": imu}
    nav._requested_outputs = {"cloud": True, "imu": True}
    nav._proc = mock.Mock(pid=123)
    nav._proc.is_alive.return_value = True
    nav._status_node = SimpleNamespace(cloud_topic="/navigation/lidar", lidar_frame="lidar")

    def close():
        assert cloud.token() is None
        assert legacy.token() is None

    legacy_node = SimpleNamespace(output_gate=legacy, close=mock.Mock(side_effect=close),
                                  destroy_node=mock.Mock())
    plugin._node = legacy_node
    plugin._navigation = nav
    plugin._lifecycle_lock = threading.RLock()
    plugin._executor = mock.Mock()
    plugin._cloud_topic = "/lidar/cloud"
    plugin.stop()
    plugin.stop()
    assert plugin._node is None
    legacy_node.close.assert_called_once()
    assert imu.token() is not None
    assert nav._requested_outputs == {"cloud": False, "imu": True}
    nav._proc.terminate.assert_not_called()
    publisher = mock.Mock()
    cloud.publish(stale_cloud, publisher, "stale")
    legacy.publish(stale_legacy, publisher, "stale")
    publisher.publish.assert_not_called()

    # Even an independently re-enabled gate must be closed when the legacy
    # node has already disappeared; this used to return idle prematurely.
    cloud.set_enabled(True)
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert cloud.token() is None
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert imu.token() is not None
