"""Host-only best-effort shutdown, including the real Bumi safety stop chain."""
import sys
import types

import pytest

from test_bumi_bundle import bundle_module  # shared ROS/SDK-free main import
from test_bumi_traffic_police_gesture import call, robot, wait_until
from test_traffic_light import load_bumi


class StopPlugin:
    def __init__(self, name, events, *, error=None, result=None):
        self.name, self.events, self.error, self.result = name, events, error, result

    def start(self):
        self.events.append((self.name, "start"))

    def stop(self):
        self.events.append((self.name, "stop"))
        if self.error is not None:
            raise self.error
        return self.result


def bundle_for(module, plugins):
    # Real lifecycle methods; only registration/hardware construction is skipped.
    bundle = object.__new__(module.BumiDeviceBundle)
    bundle._plugins = plugins
    return bundle


@pytest.mark.parametrize("failures", [(), (0,), (1,), (2,), (0, 2), (0, 1, 2)])
def test_every_plugin_is_attempted_in_reverse_order_and_all_exceptions_are_reported(
        bundle_module, capsys, failures):
    events = []
    errors = [ValueError("failure_A"), KeyError("failure_B"), OSError("failure_C")]
    plugins = [StopPlugin(name, events, error=errors[i] if i in failures else None)
               for i, name in enumerate("ABC")]
    bundle = bundle_for(bundle_module, plugins)
    if failures:
        with pytest.raises(RuntimeError) as caught:
            bundle.stop_all()
        assert caught.value.__cause__ is errors[max(failures)]
        message = str(caught.value)
        assert f"{len(failures)}/3 plugins failed to stop" in message
        for i in failures:
            assert f"Plugin {i} (StopPlugin) stop() FAILED" in message
            assert type(errors[i]).__name__ in message and f"failure_{'ABC'[i]}" in message
        output = capsys.readouterr().out
        assert output.count("stop() FAILED") == len(failures)
        assert "All plugins stopped" not in output
    else:
        assert bundle.stop_all() is None
        assert "All plugins stopped" in capsys.readouterr().out
    assert events == [(name, "stop") for name in "CBA"]


@pytest.mark.parametrize("result", [False,
    {"state": "error", "error": "cleanup refused", "config": {"secret": "DO_NOT_LOG"}},
    {"state": "error", "cleanup_pending": ["one"]},
    {"ok": False, "message": "stop rejected"}])
def test_explicit_failure_results_are_observable_and_do_not_skip_other_plugins(bundle_module, capsys, result):
    events = []
    bundle = bundle_for(bundle_module, [StopPlugin("safety", events), StopPlugin("bad", events, result=result)])
    with pytest.raises(RuntimeError, match="1/2 plugins failed to stop") as caught:
        bundle.stop_all()
    assert events == [("bad", "stop"), ("safety", "stop")]
    assert "StopPlugin" in str(caught.value)
    output = capsys.readouterr().out
    assert "stop() FAILED" in output and "DO_NOT_LOG" not in output
    assert "DO_NOT_LOG" not in str(caught.value)


@pytest.mark.parametrize("result", [None, True, {"state": "idle"}, {"ok": True, "state": "idle"}])
def test_successful_return_conventions_and_repeated_stop_do_not_start_anything(bundle_module, result):
    events = []
    bundle = bundle_for(bundle_module, [StopPlugin("A", events, result=result)])
    assert bundle.stop_all() is None
    assert bundle.stop_all() is None
    assert events == [("A", "stop"), ("A", "stop")]


def test_empty_bundle_shutdown_keeps_its_normal_none_return(bundle_module):
    assert bundle_for(bundle_module, []).stop_all() is None


@pytest.mark.parametrize("interrupt", [KeyboardInterrupt, SystemExit])
def test_process_interrupts_are_not_swallowed_or_aggregated(bundle_module, interrupt):
    events = []
    bundle = bundle_for(bundle_module, [StopPlugin("later", events),
        StopPlugin("interrupt", events, error=interrupt("leave"))])
    with pytest.raises(interrupt):
        bundle.stop_all()
    assert events == [("interrupt", "stop")]


@pytest.mark.parametrize("gesture_state", ["running", "awaiting_verification", "locked"])
def test_real_camera_stop_failure_still_reaches_loco_and_original_gesture_shutdown(
        robot, monkeypatch, gesture_state, capsys):
    main = load_bumi(monkeypatch)
    plugin, gesture = robot.plugin, robot.plugin._traffic_gesture
    started = call(plugin, "go", execute=True)
    assert started["state"] == "running"
    if gesture_state == "awaiting_verification":
        robot.ctrl.mode = 2
        wait_until(lambda: gesture.info()["gesture_state"] == "awaiting_verification")
    elif gesture_state == "locked":
        monkeypatch.setattr(plugin._observation, "_settled", lambda: (False, "stop unconfirmed"))
        assert call(plugin, "cancel")["state"] == "locked"

    order, exits = [], []
    original_stop, original_shutdown = plugin.stop, gesture.shutdown

    def loco_stop():
        order.append(("loco", "stop"))
        return original_stop()

    def gesture_shutdown():
        order.append(("gesture", "shutdown"))
        result = original_shutdown()
        exits.append(result)
        return result

    monkeypatch.setattr(plugin, "stop", loco_stop)
    monkeypatch.setattr(gesture, "shutdown", gesture_shutdown)

    class FailedCameraProcess:
        def terminate(self):
            order.append(("camera", "stop"))
            raise OSError("camera terminate failed")

    # The actual CameraPlugin.stop, with only its subprocess handle replaced.
    camera = object.__new__(robot.module.CameraPlugin)
    camera._proc = FailedCameraProcess()
    servo = StopPlugin("servo", order)
    other = StopPlugin("other", order)
    bundle = bundle_for(main, [other, plugin, servo, camera])
    worker = gesture.worker
    for index in range(2):
        with pytest.raises(RuntimeError, match="CameraPlugin.*camera terminate failed"):
            bundle.stop_all()
        assert order[index*5:(index+1)*5] == [("camera", "stop"), ("servo", "stop"),
            ("loco", "stop"), ("gesture", "shutdown"), ("other", "stop")]
        info = gesture.info()
        assert info["closed"] and not info["started"]
        assert not info["physical_completion_confirmed"]
        assert not worker.is_alive()
        if gesture_state == "locked":
            assert exits[-1]["state"] == "locked" and not exits[-1]["playback_exit_confirmed"]
            assert info["state"] == "error" and info["gesture_state"] == "locked" and info["busy"]
            assert plugin._body_action == "traffic_police_gesture.go"
        else:
            assert info["state"] == info["gesture_state"] == "idle" and not info["busy"]
            assert plugin._body_action is None
    assert len(exits) == 2
    assert [event for event in robot.events if event[0] == "PLAYTEACH"] == [("PLAYTEACH", 202)]
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1 and robot.callbacks[0][0] == started["action_id"]
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][1] == ("error" if gesture_state == "locked" else "cancelled")
    assert all(not result.get("physical_completion_confirmed") for result in exits)
    assert "CameraPlugin) stop() FAILED" in capsys.readouterr().out


@pytest.mark.parametrize("path", ["normal", "SIGTERM", "SIGINT"])
@pytest.mark.parametrize("plugin_fails", [False, True])
def test_real_main_finally_and_signal_handlers_close_outer_resources_after_plugin_errors(
        bundle_module, monkeypatch, path, plugin_fails):
    module = bundle_module
    events, handlers = [], {}
    camera_error = OSError("camera cleanup failed") if plugin_fails else None
    bundle = bundle_for(module, [StopPlugin("loco", events), StopPlugin("camera", events, error=camera_error)])

    class Executor:
        def shutdown(self):
            events.append(("executor", "shutdown"))

    class Server:
        def __init__(self, *_args):
            pass

        def serve_forever(self):
            events.append(("server", "serve"))
            if path != "normal":
                handlers[path](path, None)

        def shutdown(self):
            events.append(("server", "shutdown"))

    class Thread:
        def __init__(self, target, **_kwargs):
            self.target = target

        def start(self):
            self.target()  # no OS thread or signal is created

    def forbidden(*_args, **_kwargs):
        pytest.fail("Host shutdown tests must not run subprocesses or registration")

    # Run the real main() with every hardware/network/process boundary replaced.
    monkeypatch.setitem(sys.modules, "highcontrol_py", None)
    monkeypatch.setattr(module, "sys", types.SimpleNamespace(path=[], executable="unused"))
    monkeypatch.setattr(module, "subprocess", types.SimpleNamespace(run=forbidden))
    monkeypatch.setattr(module, "_load_config", lambda: {"ros_namespace": "shutdown_test"})
    monkeypatch.setattr(module, "_disable_vendor_video_capture_service", lambda cfg: None)
    monkeypatch.setattr(module, "_start_registration", lambda *_args: None)
    monkeypatch.setattr(module, "BumiDeviceBundle", lambda *_args: bundle)
    monkeypatch.setattr(module, "ThreadingHTTPServer", Server)
    monkeypatch.setattr(module, "threading", types.SimpleNamespace(Thread=Thread))
    monkeypatch.setattr(module, "signal", types.SimpleNamespace(SIGTERM="SIGTERM", SIGINT="SIGINT",
        signal=lambda signum, handler: handlers.setdefault(signum, handler)))
    monkeypatch.setattr(module, "rclpy", types.SimpleNamespace(init=lambda: None, ok=lambda: False,
        executors=types.SimpleNamespace(MultiThreadedExecutor=Executor),
        shutdown=lambda: events.append(("rclpy", "shutdown"))))
    if plugin_fails:
        with pytest.raises(RuntimeError, match="StopPlugin.*camera cleanup failed"):
            module.main()
    else:
        assert module.main() is None
    count = 1 if path == "normal" else 2  # preserve signal + finally semantics
    stops = [event for event in events if event[1] == "stop"]
    assert stops == [("camera", "stop"), ("loco", "stop")]*count
    assert events[-2:] == [("executor", "shutdown"), ("rclpy", "shutdown")]
    assert (("server", "shutdown") in events) is (path != "normal")
    assert [event for event in events if event[1] == "start"] == [("loco", "start"), ("camera", "start")]


def test_real_main_finally_attempts_rclpy_shutdown_even_if_executor_cleanup_raises(bundle_module, monkeypatch):
    # Reuse the full main boundary setup above, then make executor shutdown fail.
    events = []
    module = bundle_module
    bundle = bundle_for(module, [StopPlugin("safety", events)])
    monkeypatch.setitem(sys.modules, "highcontrol_py", None)
    monkeypatch.setattr(module, "sys", types.SimpleNamespace(path=[]))
    monkeypatch.setattr(module, "_load_config", lambda: {"ros_namespace": "shutdown_test"})
    monkeypatch.setattr(module, "_disable_vendor_video_capture_service", lambda cfg: None)
    monkeypatch.setattr(module, "_start_registration", lambda *_args: None)
    monkeypatch.setattr(module, "BumiDeviceBundle", lambda *_args: bundle)
    monkeypatch.setattr(module, "threading", types.SimpleNamespace(Thread=lambda **kw: types.SimpleNamespace(start=lambda: None)))
    monkeypatch.setattr(module, "signal", types.SimpleNamespace(SIGTERM=15, SIGINT=2, signal=lambda *_args: None))
    monkeypatch.setattr(module, "ThreadingHTTPServer", lambda *_args: types.SimpleNamespace(serve_forever=lambda: None))
    def executor_shutdown():
        events.append(("executor", "shutdown"))
        raise RuntimeError("executor cleanup failed")
    monkeypatch.setattr(module, "rclpy", types.SimpleNamespace(init=lambda: None,
        executors=types.SimpleNamespace(MultiThreadedExecutor=lambda: types.SimpleNamespace(shutdown=executor_shutdown)),
        shutdown=lambda: events.append(("rclpy", "shutdown"))))
    with pytest.raises(RuntimeError, match="executor cleanup failed"):
        module.main()
    assert events == [("safety", "start"), ("safety", "stop"), ("executor", "shutdown"), ("rclpy", "shutdown")]
