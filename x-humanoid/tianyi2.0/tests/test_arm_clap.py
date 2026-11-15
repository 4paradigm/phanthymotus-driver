"""Clap sequence contract without ROS or hardware."""
import threading
import time

from test_camera_lifecycle import device, _Ros2


def make_plugin(calibrated=True):
    cfg = {"clap": {"calibrated": calibrated, "prepare": [-10, 10, 0, -20, 0, 0, 0],
                    "open": [-30, 20, 0, -50, 0, 0, 0],
                    "close": [-30, 15, 0, -50, 0, 0, 0]}}
    plugin = device.ArmGesturePlugin(cfg, "", _Ros2())
    plugin._publisher = object()
    plugin._arm_status_time = time.monotonic()
    plugin._arm_status = {i: {"pos": 0., "error": 0} for i in plugin._motor_ids("both")}
    return plugin


def test_schema_and_calibration_gate():
    plugin = make_plugin(False)
    assert "clap" in plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    assert plugin.dispatch("clap", {})["code"] == "clap_not_calibrated"
    assert plugin.dispatch("clap", {"side": "left"})["code"] == "clap_requires_both_arms"
    assert plugin.dispatch("clap", {"cycles": 2})["code"] == "clap_requires_three_cycles"


def test_three_cycles_complete_only_after_all_frames(monkeypatch):
    plugin = make_plugin()
    published, notices = [], []
    monkeypatch.setattr(device, "_acp_notify", lambda *args: notices.append(args))
    monkeypatch.setattr(plugin, "_publish_pose", lambda side, pose, speed: published.append((side, pose, speed)) or {})
    monkeypatch.setattr(plugin, "_wait_for_clap_target", lambda *args: None)
    result = plugin.dispatch("clap", {})
    plugin._sequence._thread and plugin._sequence._thread.join(1)
    cfg = plugin._clap_config
    assert [p[1] for p in published] == [cfg["prepare"], cfg["open"]] + [p for _ in range(3) for p in (cfg["close"], cfg["open"])] + [cfg["prepare"], plugin._NEUTRAL]
    assert all(side == "both" and speed == .3 for side, _, speed in published)
    assert result["cycles"] == 3
    assert notices[0][1] == "completed"


def test_frame_failure_does_not_report_completion(monkeypatch):
    plugin = make_plugin()
    notices, published = [], []
    monkeypatch.setattr(device, "_acp_notify", lambda *args: notices.append(args))
    monkeypatch.setattr(plugin, "_publish_pose", lambda *args: published.append(args) or {})
    monkeypatch.setattr(plugin, "_wait_for_clap_target", lambda *args: {"error": "timeout"})
    plugin.dispatch("clap", {})
    plugin._sequence._thread and plugin._sequence._thread.join(1)
    assert len(published) == 1
    assert notices[0][1] == "error"


def test_target_wait_needs_fresh_feedback_and_cancel():
    plugin = make_plugin()
    cancelled = threading.Event()
    cancelled.set()
    assert plugin._wait_for_clap_target(plugin._NEUTRAL, 0, plugin._feedback_snapshot("both")[1], cancelled, .3)["code"] == "clap_cancelled"
