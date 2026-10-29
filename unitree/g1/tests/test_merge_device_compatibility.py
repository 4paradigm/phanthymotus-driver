"""Offline merge regression: stop must not wait for a Move RPC reply."""
import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest import mock

import pytest

from sensor_output import OutputGate


def loco_class():
    path = Path(__file__).resolve().parents[1] / "device.py"
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LocoPlugin")
    scope = {"threading": threading, "math": math, "time": time,
             "_SMS": SimpleNamespace(LOCO_STATES=set(), LIMP_STATES=set(),
                                     BALANCED_SQUAT=706, LIE_TO_STAND=702),
             "_loco_acp_notify": lambda *args, **kwargs: None}
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
    nav._imu_requests = {"lidar": True, "imu": True}
    nav._proc = mock.Mock(pid=123)
    nav._proc.is_alive.return_value = True
    nav._status_node = SimpleNamespace(cloud_topic="/navigation/lidar", lidar_frame="lidar",
                                       imu_topic="/navigation/imu", imu_frame="lidar")

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

    # Bundle-level restart recreates the legacy node and reclaims both streams.
    scope["_LidarNode"] = lambda topic: SimpleNamespace(output_gate=OutputGate())
    plugin.start()
    assert nav._imu_requests == {"lidar": True, "imu": True}
    assert cloud.token() is not None
    assert imu.token() is not None


def make_fault_test_lidar():
    path = Path(__file__).resolve().parents[1] / "device.py"
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LidarPlugin")
    legacy = SimpleNamespace(output_gate=OutputGate(), close=mock.Mock(), destroy_node=mock.Mock())
    scope = {"threading": threading, "_LidarNode": mock.Mock(return_value=legacy)}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), scope)
    nav = mock.Mock()
    executor = mock.Mock()
    plugin = scope["LidarPlugin"]({}, "ubuntu", executor, nav)
    return plugin, nav, executor, legacy, scope


@pytest.mark.parametrize("failure_stream", ["cloud", "imu"])
def test_lidar_start_retries_navigation_after_partial_failure(failure_stream):
    plugin, nav, executor, legacy, scope = make_fault_test_lidar()
    plugin.stop()
    nav.reset_mock()
    executor.reset_mock()
    failed = False
    def fail_once(stream, enabled, owner=None):
        nonlocal failed
        if enabled and stream == failure_stream and not failed:
            failed = True
            raise TimeoutError("injected")
    nav.set_output.side_effect = fail_once
    with pytest.raises(TimeoutError, match="injected"):
        plugin.start()
    assert plugin._node is legacy
    assert plugin._navigation_restart_pending
    plugin.start()
    assert not plugin._navigation_restart_pending
    assert nav.set_output.call_args_list[-2:] == [mock.call("cloud", True), mock.call("imu", True, owner="lidar")]
    assert executor.add_node.call_count == 1
    count = nav.set_output.call_count
    plugin.start()
    assert nav.set_output.call_count == count


@pytest.mark.parametrize("failure_stream", ["cloud", "imu"])
def test_lidar_stop_attempts_sibling_and_legacy_on_navigation_failure(failure_stream):
    plugin, nav, executor, legacy, _ = make_fault_test_lidar()
    def fail_one(stream, enabled, owner=None):
        if stream == failure_stream:
            raise TimeoutError("injected")
    nav.set_output.side_effect = fail_one
    with pytest.raises(TimeoutError, match="injected"):
        plugin.stop()
    assert nav.set_output.call_args_list == [mock.call("cloud", False, owner=None),
                                             mock.call("imu", False, owner="lidar")]
    assert legacy.output_gate.token() is None
    legacy.close.assert_called_once()
    legacy.destroy_node.assert_called_once()
    assert plugin._node is None
    nav.set_output.side_effect = None
    plugin.stop()  # retry releases whichever request failed
    assert nav.set_output.call_count == 4


@pytest.mark.parametrize("failure_stream", ["cloud", "imu"])
@pytest.mark.parametrize("node_exists", [True, False])
def test_lidar_public_stop_releases_sibling_and_reports_first_failure(failure_stream, node_exists):
    plugin, nav, _, legacy, _ = make_fault_test_lidar()
    if not node_exists:
        plugin._node = None
    def fail_one(stream, enabled, owner=None):
        if stream == failure_stream:
            raise RuntimeError(f"{stream} failed")
    nav.set_output.side_effect = fail_one
    nav.output_status.side_effect = AssertionError("must not mask stop failure")
    nav.cloud_descriptor.side_effect = AssertionError("must not mask stop failure")
    result = plugin.dispatch("stop", {})
    assert result == {"state": "error", "ready": False, "error": f"{failure_stream} failed"}
    assert nav.set_output.call_args_list == [mock.call("cloud", False, owner=None),
                                             mock.call("imu", False, owner="lidar")]
    if node_exists:
        assert legacy.output_gate.token() is None
    nav.output_status.assert_not_called()
    nav.cloud_descriptor.assert_not_called()


def test_lidar_public_stop_reports_first_failure_if_multiple_operations_fail():
    plugin, nav, _, legacy, _ = make_fault_test_lidar()
    legacy.output_gate = mock.Mock()
    legacy.output_gate.set_enabled.side_effect = TimeoutError("legacy first")
    nav.set_output.side_effect = RuntimeError("navigation later")
    nav.output_status.side_effect = AssertionError("status must not be queried")
    result = plugin.dispatch("stop", {})
    assert result == {"state": "error", "ready": False, "error": "legacy first"}
    assert nav.set_output.call_count == 2
    nav.output_status.assert_not_called()


def test_lidar_stop_keeps_node_when_legacy_gate_fails():
    plugin, nav, executor, legacy, _ = make_fault_test_lidar()
    legacy.output_gate = mock.Mock()
    legacy.output_gate.set_enabled.side_effect = TimeoutError("gate busy")
    with pytest.raises(TimeoutError, match="gate busy"):
        plugin.stop()
    assert nav.set_output.call_count == 2
    assert plugin._node is legacy
    legacy.close.assert_not_called()
    legacy.destroy_node.assert_not_called()
    executor.remove_node.assert_not_called()
    legacy.output_gate.set_enabled.side_effect = None
    plugin.stop()
    assert plugin._node is None
    legacy.close.assert_called_once()


def test_real_bundle_registration_order_stop_all_then_restart_keeps_worker_stopped():
    from test_navigation_sensor_card import NavigationSensorCardContractTest
    module = NavigationSensorCardContractTest.load_bridge_module()
    root = Path(__file__).resolve().parents[1]
    bundle_tree = ast.parse((root / "main.py").read_text())
    bundle_class = next(n for n in bundle_tree.body if isinstance(n, ast.ClassDef) and n.name == "G1DeviceBundle")
    methods = [n for n in bundle_class.body if isinstance(n, ast.FunctionDef) and n.name in ("start_all", "stop_all")]
    bundle_scope = {}
    test_tree = ast.Module(body=[ast.ClassDef(name="Bundle", bases=[], keywords=[], body=methods, decorator_list=[])],
                           type_ignores=[])
    exec(compile(ast.fix_missing_locations(test_tree), str(root / "main.py"), "exec"), bundle_scope)
    lidar_tree = ast.parse((root / "device.py").read_text())
    lidar_class = next(n for n in lidar_tree.body if isinstance(n, ast.ClassDef) and n.name == "LidarPlugin")
    def lidar_node(topic):
        return SimpleNamespace(output_gate=OutputGate(), close=mock.Mock(), destroy_node=mock.Mock())
    scope = {"threading": threading, "_LidarNode": lidar_node}
    exec(compile(ast.Module(body=[lidar_class], type_ignores=[]), str(root / "device.py"), "exec"), scope)

    def monitor(config, namespace):
        return SimpleNamespace(cloud_topic="/ubuntu/navigation/lidar", imu_topic="/ubuntu/navigation/imu",
                               lidar_frame="livox_frame", imu_frame="livox_frame", reset=mock.Mock(),
                               destroy_node=mock.Mock(), status=lambda running: {"blockers": []})
    processes = []
    def process(**kwargs):
        proc = mock.Mock(pid=100 + len(processes))
        proc.is_alive.return_value = True
        proc.terminate.side_effect = lambda: setattr(proc.is_alive, "return_value", False)
        processes.append(proc)
        return proc

    with mock.patch.object(module, "_NavigationSensorMonitorNode", side_effect=monitor), \
         mock.patch.object(module.multiprocessing.get_context("spawn"), "Process", side_effect=process):
        executor = mock.Mock()
        nav = module.NavigationSensorPlugin({}, "ubuntu", executor, lidar_enabled=True)
        lidar = scope["LidarPlugin"]({}, "ubuntu", executor, nav)
        bundle = bundle_scope["Bundle"]()
        bundle._low_state = None
        bundle._plugins = [nav, lidar]  # main.py registers navigation before lidar
        bundle.start_all()
        assert len(processes) == 1 and nav._worker_running()
        bundle.stop_all()
        assert len(processes) == 1 and nav._proc is None
        processes[0].terminate.assert_called_once()
        assert nav._imu_requests == {"lidar": False, "imu": True}
        bundle.start_all()
        assert len(processes) == 2 and nav._worker_running()
        assert nav._imu_requests == {"lidar": True, "imu": True}
        assert nav._gates["cloud"].token() is not None
        assert nav._gates["imu"].token() is not None
        assert nav.dispatch("stop", {})["state"] == "idle"
        assert nav._gates["imu"].token() is not None
        assert lidar.dispatch("stop", {})["state"] == "idle"
        assert nav._proc is None
        assert nav.dispatch("start", {})["enabled"]
        assert len(processes) == 3 and nav._worker_running()
        bundle.stop_all()
        assert len(processes) == 3 and nav._proc is None
