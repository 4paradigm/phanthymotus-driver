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

from robotera.q5_bundle import squat_counter


def test_q5_bundle_registers_squat_processor():
    bundle = Path(__file__).parent
    config = yaml.safe_load((bundle / "config.yaml").read_text())
    catalog = yaml.safe_load((bundle / "driver.yaml").read_text())
    assert config["plugins"]["squat_counter"]["enabled"] is True
    assert {"name": "squat_counter", "type": "processor"} in catalog["cards"]
    assert "COPY squat_counter.py /work/squat_counter.py" in (bundle / "Dockerfile").read_text()


def test_proxy_contract_and_action_forwarding():
    card = squat_counter.Plugin({}, "q5_test", None, None)
    tool = card.get_tool()
    assert tool["type"] == "processor"
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
    assert requests == [("reset", {})]


def test_dead_processor_is_reported_instead_of_returning_stale_count():
    card = squat_counter.Plugin({}, "q5_test", None, None)
    card._proc = types.SimpleNamespace(is_alive=lambda: False)
    result = card.dispatch("status", {})
    assert result["state"] == "error"


def test_worker_uses_pose_domain_and_handles_actions(monkeypatch):
    seen = {}

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
    fake_transport.configure_fastdds_transport = lambda: seen.update(profile=True)
    fake_card_module = types.ModuleType("common.squat_counter_card")
    fake_card_module.SquatCounterPlugin = FakeCard
    for name, module in (("rclpy", fake_rclpy), ("rclpy.executors", fake_executors),
                         ("fastdds_transport", fake_transport),
                         ("common.squat_counter_card", fake_card_module)):
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "211")
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
