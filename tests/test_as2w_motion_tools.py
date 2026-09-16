"""No-hardware contracts for AS2W trajectory and recording cards."""

import json
from pathlib import Path
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "unitree/as2w"))
import motion_tools


class FakeProxy:
    def __init__(self):
        self.moves = []
        self.stops = 0
        self.move_ret = 0

    def Move(self, *velocity):
        self.moves.append(velocity)
        return self.move_ret

    def StopMove(self):
        self.stops += 1
        return 0


class CapturingExecutor:
    def __init__(self):
        self.owner = None
        self.worker = None
        self.stops = []

    def start(self, owner, worker):
        self.owner = owner
        self.worker = worker
        return {"state": "running", "action_id": "test-action", "owner": owner}

    def stop(self, owner=None):
        self.stops.append(owner)
        return {"state": "idle", "stopped_owner": owner}

    def status(self, owner=None):
        return {"state": "running" if owner == self.owner else "idle",
                "active_owner": self.owner, "action_id": "test-action",
                "elapsed": 0.1, "last_result": None}


def wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_motion_executor_cancels_worker_and_stops_robot():
    proxy = FakeProxy()
    executor = motion_tools.MotionExecutor(proxy)
    entered = threading.Event()

    def worker(stop_event):
        entered.set()
        stop_event.wait(2)
        return {"state": "cancelled"}

    started = executor.start("trajectory_motion", worker)
    assert started["state"] == "running"
    assert entered.wait(0.5)
    stopped = executor.stop("trajectory_motion")

    assert stopped["state"] == "idle"
    assert proxy.stops >= 2
    assert executor.status()["state"] == "idle"


def test_trajectory_contract_and_circle_math():
    proxy = FakeProxy()
    executor = CapturingExecutor()
    loco_stops = []
    plugin = motion_tools.TrajectoryMotionPlugin({}, proxy, executor,
                                                 lambda: loco_stops.append(True))

    schema = plugin.get_tool()["inputSchema"]
    assert schema["x-is-dangerous"] is True
    assert set(schema["x-completion"]["actions"]) == {"circle", "figure_eight", "slalom"}

    result = plugin.dispatch("circle", {
        "radius": 1.0, "speed": 0.5, "loops": 1, "direction": "right",
    })
    assert result["state"] == "running"
    assert loco_stops == [True]
    assert executor.owner == "trajectory_motion"
    command = executor.worker

    # The worker itself is tested through its captured velocity generator by
    # replacing wall-clock time with a short deterministic sequence.
    original_monotonic = motion_tools.time.monotonic
    times = iter([0.0, 0.0, 100.0, 100.0])
    motion_tools.time.monotonic = lambda: next(times)
    try:
        completed = command(threading.Event())
    finally:
        motion_tools.time.monotonic = original_monotonic
    assert completed["state"] == "completed"
    assert proxy.moves == [(0.5, 0.0, -0.5)]


def test_trajectory_rejects_unbounded_duration():
    plugin = motion_tools.TrajectoryMotionPlugin(
        {}, FakeProxy(), CapturingExecutor(), lambda: None,
    )
    result = plugin.dispatch("figure_eight", {
        "radius": 3.0, "speed": 0.05, "loops": 3,
    })
    assert result["code"] == "INVALID_ARGUMENT"
    assert "60 seconds" in result["error"]


def test_record_drive_save_list_play_and_delete(tmp_path):
    proxy = FakeProxy()
    executor = CapturingExecutor()
    plugin = motion_tools.MotionRecorderPlugin(
        {"recordings_dir": str(tmp_path)}, proxy, executor, lambda: None,
    )
    plugin.start()

    started = plugin.dispatch("record_start", {"label": "demo"})
    assert started["state"] == "recording"
    assert plugin.dispatch("drive", {"vx": 0.2, "vy": -0.1, "vyaw": 0.3})["ret"] == 0
    saved = plugin.dispatch("record_stop", {})

    assert saved["state"] == "saved"
    assert saved["frames"] == 1
    path = tmp_path / f"{saved['name']}.json"
    payload = json.loads(path.read_text())
    assert payload["frames"][0]["vx"] == 0.2
    assert plugin.dispatch("list", {})["recordings"][0]["name"] == saved["name"]

    playing = plugin.dispatch("play", {"name": saved["name"], "speed_scale": 1})
    assert playing["state"] == "running"
    assert executor.owner == "motion_recorder"
    playback_result = executor.worker(threading.Event())
    assert playback_result == {"state": "completed", "name": saved["name"], "frames_sent": 1}

    deleted = plugin.dispatch("delete", {"name": saved["name"]})
    assert deleted == {"state": "deleted", "name": saved["name"]}
    assert not path.exists()


def test_recorder_requires_session_and_rejects_path_names(tmp_path):
    plugin = motion_tools.MotionRecorderPlugin(
        {"recordings_dir": str(tmp_path)}, FakeProxy(), CapturingExecutor(), lambda: None,
    )
    plugin.start()

    assert plugin.dispatch("drive", {"vx": 0.1})["code"] == "NOT_RECORDING"
    assert plugin.dispatch("play", {"name": "../secret"})["code"] == "INVALID_RECORDING"
    assert plugin.dispatch("delete", {"name": "../secret"})["code"] == "INVALID_RECORDING"


def test_empty_recording_is_not_persisted(tmp_path):
    plugin = motion_tools.MotionRecorderPlugin(
        {"recordings_dir": str(tmp_path)}, FakeProxy(), CapturingExecutor(), lambda: None,
    )
    plugin.start()
    plugin.dispatch("record_start", {"label": "empty"})

    result = plugin.dispatch("record_stop", {})

    assert result["saved"] is False
    assert list(tmp_path.iterdir()) == []


def test_manifest_registers_selected_three_cards():
    manifest = (ROOT / "unitree/as2w/driver.yaml").read_text()
    assert "- { name: remote_controller, type: sensor }" in manifest
    assert "- { name: trajectory_motion, type: actuator }" in manifest
    assert "- { name: motion_recorder, type: actuator }" in manifest
