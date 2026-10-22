"""Q5 squat card tests without a robot or ROS installation."""

from __future__ import annotations

import multiprocessing as mp
import os
import sys
import threading
import time
import types
from pathlib import Path

import yaml

from common import logsafe
from robotera.q5_bundle import squat_counter


def test_q5_bundle_registers_squat_processor():
    bundle = Path(__file__).parent
    config = yaml.safe_load((bundle / "config.yaml").read_text())
    catalog = yaml.safe_load((bundle / "driver.yaml").read_text())
    assert config["plugins"]["squat_counter"]["enabled"] is True
    assert config["plugins"]["squat_counter"]["require_explicit_workout_start"] is True
    assert config["plugins"]["squat_counter"]["coach_mode"] is True
    assert config["plugins"]["squat_counter"]["local_milestone_voice"] is True
    assert config["plugins"]["squat_counter"]["default_sets"] == 2
    assert config["plugins"]["squat_counter"]["default_reps_per_set"] == 8
    assert {"name": "squat_counter", "type": "processor"} in catalog["cards"]
    assert "COPY squat_counter.py /work/squat_counter.py" in (bundle / "Dockerfile").read_text()


def test_proxy_contract_and_action_forwarding():
    card = squat_counter.Plugin({}, "q5_test", None, None)
    tool = card.get_tool()
    assert tool["type"] == "processor"
    assert {"begin", "next_set", "finish"} <= set(tool["inputSchema"]["properties"]["action"]["enum"])
    assert tool["inputSchema"]["properties"]["exercise"]["enum"] == [
        "squat", "dumbbell_curl", "lateral_raise"]
    assert "exercise" in tool["inputSchema"]["x-action-params"]["begin"]["params"]
    assert tool["topic_in"][0]["format"] == "sensor/pose2d"
    assert [port["topic"] for port in tool["topic_out"]] == [
        "/q5_test/fitness/squat/events",
        "/q5_test/fitness/squat/status",
        "/q5_test/fitness/squat/debug",
    ]

    requests = []
    card._proc = types.SimpleNamespace(is_alive=lambda: True)
    card._conn = types.SimpleNamespace(
        send=requests.append, poll=lambda timeout: True,
        recv=lambda: {"count_total": 2, "session_id": "active"},
    )
    assert card.dispatch("reset", {})["count_total"] == 2
    assert card.dispatch("begin", {"exercise": "dumbbell_curl"})["count_total"] == 2
    assert card.dispatch("next_set", {})["count_total"] == 2
    assert card.dispatch("finish", {})["count_total"] == 2
    assert requests == [("reset", {}), ("begin", {"exercise": "dumbbell_curl"}),
                        ("next_set", {}), ("finish", {})]


def test_coach_tool_exposes_default_plan_to_decision_core():
    card = squat_counter.Plugin({"coach_mode": True, "local_milestone_voice": True},
                                "q5_test", None, None)
    begin = card.get_tool()["inputSchema"]["x-action-params"]["begin"]
    assert "2 sets of 8 reps" in begin["description"]
    assert "90 seconds" in begin["description"]
    assert card.get_tool()["topic_out"][3]["topic"] == "/q5_test/fitness/squat/voice"


def test_dead_processor_is_reported_instead_of_returning_stale_count():
    card = squat_counter.Plugin({}, "q5_test", None, None)
    card._proc = types.SimpleNamespace(is_alive=lambda: False)
    result = card.dispatch("status", {})
    assert result["state"] == "error"


def test_worker_installs_logsafe_before_fastdds_configuration(monkeypatch):
    calls = []
    monkeypatch.setattr(logsafe, "install", lambda **kwargs: calls.append(("logsafe", kwargs)))
    monkeypatch.setenv("ROS_DOMAIN_ID", "211")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", "test-profile")

    def configure():
        calls.append(("fastdds", None))
        raise RuntimeError("stop before ROS startup")

    fake_transport = types.ModuleType("fastdds_transport")
    fake_transport.configure_fastdds_transport = configure
    monkeypatch.setitem(sys.modules, "fastdds_transport", fake_transport)

    class FakeConnection:
        def __init__(self):
            self.reply = None
            self.closed = False

        def send(self, reply):
            self.reply = reply

        def close(self):
            self.closed = True

    conn = FakeConnection()
    squat_counter._run_worker(conn, {}, "q5_test")

    assert calls == [("logsafe", {"check_fd": False}), ("fastdds", None)]
    assert conn.reply == {"ready": False, "error": "stop before ROS startup"}
    assert conn.closed


def test_worker_uses_pose_domain_and_handles_actions(monkeypatch):
    seen = {}
    monkeypatch.setattr(logsafe, "install", lambda **kwargs: seen.update(logsafe=kwargs))

    class FakeExecutor:
        def spin_once(self, timeout_sec):
            time.sleep(0.001)

        def shutdown(self):
            seen["executor_stopped"] = True

    class FakeCard:
        def __init__(self, config, namespace, executor, *, node_name):
            seen.update(config=config, namespace=namespace, node_name=node_name,
                        domain=os.environ["ROS_DOMAIN_ID"],
                        rmw=os.environ["RMW_IMPLEMENTATION"])

        def dispatch(self, action, args):
            seen["request"] = (action, args)
            return {"count_total": 0, "session_id": "fresh"}

        def stop(self):
            seen["card_stopped"] = True

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda: None
    fake_rclpy.ok = lambda: True
    fake_rclpy.shutdown = lambda: seen.update(ros_stopped=True)
    fake_executors = types.ModuleType("rclpy.executors")
    fake_executors.SingleThreadedExecutor = FakeExecutor
    fake_rclpy.executors = fake_executors
    fake_transport = types.ModuleType("fastdds_transport")
    def configure():
        assert seen["logsafe"] == {"check_fd": False}
        seen["profile"] = True

    fake_transport.configure_fastdds_transport = configure
    fake_card_module = types.ModuleType("common.squat_counter_card")
    fake_card_module.SquatCounterPlugin = FakeCard
    for name, module in (("rclpy", fake_rclpy), ("rclpy.executors", fake_executors),
                         ("fastdds_transport", fake_transport),
                         ("common.squat_counter_card", fake_card_module)):
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "211")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", "test-profile")
    parent, child = mp.Pipe()
    worker = threading.Thread(target=squat_counter._run_worker,
                              args=(child, {"debug_interval_s": 0.5}, "q5_test"))
    worker.start()
    try:
        assert parent.poll(2.0) and parent.recv() == {"ready": True}
        parent.send(("reset", {}))
        assert parent.poll(2.0) and parent.recv()["session_id"] == "fresh"
        assert seen["request"] == ("reset", {})
        assert seen["domain"] == "42"
        assert seen["rmw"] == "rmw_fastrtps_cpp"
        assert seen["node_name"] == "q5_squat_counter"
        assert seen["profile"] is True
    finally:
        parent.send(None)
        worker.join(timeout=2.0)
        parent.close()
        child.close()
    assert not worker.is_alive()
    assert seen["card_stopped"] and seen["executor_stopped"] and seen["ros_stopped"]
