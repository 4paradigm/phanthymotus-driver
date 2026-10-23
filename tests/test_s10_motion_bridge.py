"""Independent motion feedback leases and startup cleanup, without a robot."""
from __future__ import annotations

import importlib.util
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "deep_robotics/s10_pro"))


@pytest.fixture
def runtime():
    spec = importlib.util.spec_from_file_location("s10_motion_bridge_runtime", ROOT / "deep_robotics/s10_pro/main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("isolated reader condition timed out")


class NoRobotBridge:
    def __init__(self):
        self.active = {}

    def acquire(self, owner, sources, options):
        self.active[owner] = (set(sources), dict(options))

    def release(self, owner):
        self.active.pop(owner, None)

    def snapshot(self, source):
        return {}

    def status(self):
        return {"reader_running": False}


def test_motion_observation_uses_its_own_bridge_without_changing_state_card(runtime):
    sensors, motion = NoRobotBridge(), NoRobotBridge()
    plugins = runtime.build_runtime_plugins({}, "isolated", sensors, lambda *args: None, motion_bridge=motion)
    cards = {plugin.name: plugin for plugin in plugins}
    try:
        assert not sensors.active and not motion.active
        cards["state"].start()
        cards["motion"].start()
        assert sensors.active["state"][0] == {"battery", "motion_info", "hard_estop", "exception", "fault_event", "fault_aggregator"}
        assert set(sensors.active) == {"state"}
        assert motion.active == {"motion_observer": ({"motion_info"}, {"refresh_hz": 10., "stale_after_s": .5})}
        assert cards["motion"].dispatch("stand", {})["accepted"] is False
        assert cards["motion"].status()["execution_authorized"] is False
        cards["motion"].stop()
        assert not motion.active and set(sensors.active) == {"state"}
        cards["motion"].start()
        cards["state"].stop()
        assert not sensors.active and set(motion.active) == {"motion_observer"}
    finally:
        for plugin in plugins:
            plugin.stop()
            if hasattr(plugin, "close"):
                plugin.close()


def test_sensor_ipc_block_does_not_stall_motion_and_each_reader_is_reaped(runtime, tmp_path):
    # Real child pipes deliberately block the sensor event consumer. This checks
    # process/IPC isolation, rather than simulating physical feedback or ROS.
    from bridge import ReaderBridge
    children = []
    child_code = '''import json,sys,threading,time
active=[]
def control():
 global active
 for line in sys.stdin:active=json.loads(line)['active']
threading.Thread(target=control,daemon=True).start()
while True:
 for key in active:
  event={'key':key,'data':{'data':{'motion_state':{'state':0},'gait_state':{'gait':0x3002},'vel_x':0.,'vel_y':0.,'vel_yaw':0.}},'received_monotonic':time.monotonic()}
  print(json.dumps(event),flush=True)
 time.sleep(.025)
'''

    def factory(command, **kwargs):
        child = subprocess.Popen([sys.executable, "-u", "-c", child_code], **kwargs)
        children.append(child)
        return child

    sensors = ReaderBridge({}, tmp_path / "unused.yaml", process_factory=factory)
    motion = ReaderBridge({}, tmp_path / "unused.yaml", process_factory=factory)
    entered, release = threading.Event(), threading.Event()
    original_accept = sensors._accept_event

    def blocked_sensor(event):
        entered.set()
        assert release.wait(3), "isolated test did not release the sensor pipe"
        return original_accept(event)

    sensors._accept_event = blocked_sensor
    plugins = runtime.build_runtime_plugins({}, "isolated", sensors, lambda *args: None, motion_bridge=motion)
    cards = {plugin.name: plugin for plugin in plugins}
    try:
        cards["state"].start()
        cards["motion"].start()
        assert entered.wait(2)
        wait_for(lambda: cards["motion"]._backend.feedback.observation()["fresh"])
        assert sensors.status()["reader_pid"] != motion.status()["reader_pid"]
        sequence = motion.snapshot("motion_info")["sequence"]
        ages = []
        until = time.monotonic() + .7
        while time.monotonic() < until:
            observed = cards["motion"]._backend.feedback.observation()
            assert observed["fresh"] is True
            ages.append(observed["age_s"])
            time.sleep(.02)
        assert max(ages) < .5
        assert motion.snapshot("motion_info")["sequence"] > sequence + 5
        assert sensors.snapshot("motion_info")["data"] is None
        release.set()
        wait_for(lambda: sensors.snapshot("motion_info")["fresh"])
        motion_pid = motion.status()["reader_pid"]
        cards["motion"].stop()
        wait_for(lambda: not motion.status()["reader_running"])
        assert sensors.status()["reader_running"]
        assert sensors.snapshot("motion_info")["fresh"]
        assert not motion.status()["active_sources"]
        cards["motion"].start()
        wait_for(lambda: motion.status()["reader_running"] and motion.status()["reader_pid"] != motion_pid)
        wait_for(lambda: cards["motion"]._backend.feedback.observation()["fresh"])
        cards["state"].stop()
        wait_for(lambda: not sensors.status()["reader_running"])
        assert motion.status()["reader_running"]
        assert cards["motion"]._backend.feedback.observation()["fresh"]
        cards["motion"].stop()
        wait_for(lambda: all(child.poll() is not None for child in children))
    finally:
        release.set()
        for plugin in plugins:
            plugin.stop()
            if hasattr(plugin, "close"):
                plugin.close()
        sensors.close()
        motion.close()
    assert all(child.poll() is not None for child in children)
    assert not sensors._thread.is_alive() and not motion._thread.is_alive()


@pytest.mark.parametrize("failure", [None, "bus", "sensor_reader", "motion_reader", "tasks",
                                    "notifier", "notifier_start", "plugins", "server", "serve"])
def test_main_closes_all_created_resources_on_startup_failure(runtime, monkeypatch, tmp_path, failure):
    class StartupError(RuntimeError):
        pass

    created, closed, events = [], [], []

    class Resource:
        def __init__(self, label):
            self.label = label
            if failure == label:
                raise StartupError(label)
            created.append(label)

        def close(self):
            closed.append(self.label)
            events.append("close_" + self.label)

    def bus(domain):
        value = Resource("bus")
        value.publish = lambda *args: None
        return value

    def reader(config, path):
        return Resource("motion_reader" if "sensor_reader" in created else "sensor_reader")

    def notifier(path):
        value = Resource("notifier")

        def start():
            if failure == "notifier_start":
                raise StartupError("notifier_start")

        value.start = start
        return value

    def plugins(config, namespace, bridge, publish, **kwargs):
        if failure == "plugins":
            raise StartupError("plugins")
        assert bridge.label == "sensor_reader" and kwargs["motion_bridge"].label == "motion_reader"
        assert bridge is not kwargs["motion_bridge"]
        value = Resource("plugin")
        value.stop = lambda: events.append("stop_plugin")
        return [value]

    class Bundle:
        def __init__(self, plugins):
            self.plugins = plugins

        def stop_all(self):
            for plugin in self.plugins:
                plugin.stop()

    def server(*args):
        value = Resource("server")
        value.server_close = value.close

        def serve():
            if failure == "serve":
                raise StartupError("serve")

        value.serve_forever = serve
        return value

    profile = tmp_path / "profile.xml"
    profile.write_text("unused: validation is isolated in its own tests")
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", str(profile))
    monkeypatch.setenv("S10_STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setattr(runtime, "validate_platform_profile", lambda path: None)
    monkeypatch.setattr(runtime, "load_config", lambda path: {"mcp_port": 15745, "register": False})
    monkeypatch.setattr(runtime, "resolve_namespace", lambda config: "isolated")
    monkeypatch.setattr(runtime, "PlatformBus", bus)
    monkeypatch.setattr(runtime, "ReaderBridge", reader)
    monkeypatch.setattr(runtime, "MotionTasks", lambda path: Resource("tasks"))
    monkeypatch.setattr(runtime, "ACPNotifier", notifier)
    monkeypatch.setattr(runtime, "build_runtime_plugins", plugins)
    monkeypatch.setattr(runtime, "DriverBundle", Bundle)
    monkeypatch.setattr(runtime, "ThreadingHTTPServer", server)
    monkeypatch.setattr(runtime, "make_handler", lambda *args: object())
    monkeypatch.setattr(runtime.signal, "signal", lambda *args: None)
    if failure is None:
        runtime.main()
    else:
        with pytest.raises(StartupError, match=failure):
            runtime.main()
    assert closed == list(reversed(created))
    if "plugin" in created:
        assert events.index("stop_plugin") < events.index("close_plugin")
