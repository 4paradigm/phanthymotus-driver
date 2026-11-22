import math
from pathlib import Path
import sys

BUMI = Path(__file__).resolve().parents[1] / "noetix" / "bumi"
sys.path.insert(0, str(BUMI))
from tf_integrity import Integrity
from tf_integrity_monitor import TfIntegrityMonitor

NS = 10**9


def send(view, parent="base", child="joint", stamp=NS, now=None, arrival=1,
         static=False, source="A", xyz=(0, 0, 0), q=(0, 0, 0, 1)):
    view.observe(parent, child, stamp, xyz, q, static, source,
                 stamp if now is None else now, arrival)


def test_normal_tree_and_missing_chain():
    view = Integrity({"required_frames": ["base"], "required_chains": [["base", "tool"]]})
    send(view, parent="other", child="tool", static=True)
    report = view.snapshot(NS, 1)
    assert report["missing_frames"] == ["base"]
    assert report["disconnected_frames"] == [["base", "tool"]]
    send(view, child="other")
    assert view.snapshot(NS, 1)["state"] == "healthy"


def test_loop_and_conflicting_declarations():
    view = Integrity({"required_frames": []})
    send(view, static=True)
    send(view, parent="joint", child="base", static=True)
    send(view, parent="other", source="B", static=True)
    send(view, source="C", static=True, xyz=(.1, 0, 0))
    report = view.snapshot(NS, 1)
    assert report["state"] == "inconsistent"
    assert report["cyclic_frames"] == ["base", "joint"]
    assert report["conflicting_frames"] == ["joint"]


def test_static_durability_and_dynamic_expiry():
    view = Integrity({"required_frames": ["base"]})
    send(view, static=True, stamp=0)
    assert view.snapshot(200 * NS, 200)["state"] == "healthy"
    send(view, child="live", stamp=200 * NS, arrival=200)
    report = view.snapshot(203 * NS, 203)
    assert report["state"] == "stale"
    assert report["stale_frames"] == ["live"]


def test_ros_time_pause_seek_and_bad_stamp():
    view = Integrity({"required_frames": ["base"], "use_sim_time": True})
    send(view, child="fixed", static=True, stamp=0)
    send(view, stamp=30 * NS)
    assert view.snapshot(30 * NS, 300)["state"] == "healthy"
    send(view, stamp=29 * NS, now=30 * NS, arrival=300)
    send(view, stamp=31 * NS, now=30 * NS, arrival=300)
    assert set(view.snapshot(30 * NS, 300)["issues"]) == {"future_stamp", "stamp_regression"}
    report = view.snapshot(2 * NS, 301)
    assert report["clock_resets"] == 1
    assert report["checked_frames"] == ["base", "fixed"]
    send(view, stamp=2 * NS, arrival=301)
    assert view.snapshot(2 * NS, 301)["state"] == "healthy"


def yaw(theta):
    return (0, 0, math.sin(theta / 2), math.cos(theta / 2))


def test_pi_boundary_sign_and_bad_quaternion():
    view = Integrity({"required_frames": ["base"]})
    send(view, q=yaw(math.pi - .01))
    send(view, stamp=NS + 10**7, q=yaw(-math.pi + .01))
    angle = view.snapshot(NS + 10**7, 1)["motion"][0]["rotation_rad"]
    assert abs(angle - .02) < 1e-8
    send(view, stamp=NS + 2 * 10**7, q=tuple(-v for v in yaw(-math.pi + .01)))
    send(view, stamp=NS + 3 * 10**7, q=(0, 0, 0, 0))
    send(view, stamp=NS + 4 * 10**7, xyz=(math.nan, 0, 0))
    report = view.snapshot(NS + 4 * 10**7, 1)
    assert report["rotation_jump_count"] == 0
    assert report["motion"][0]["consecutive_normal"] == 2
    assert set(report["issues"]) == {"invalid_quaternion", "invalid_transform"}


def test_stationary_and_fast_motion_then_jump():
    view = Integrity({"required_frames": ["base"]})
    for index in range(1000):
        stamp = NS + index * 10**7
        xyz = (0, 0, 0) if index < 500 else ((index - 500) * .05, 0, 0)
        send(view, stamp=stamp, arrival=stamp / NS, xyz=xyz, q=yaw(max(0, index - 500) * .15))
        assert view.snapshot(stamp, stamp / NS)["state"] == "healthy"
    send(view, stamp=stamp + 10**7, arrival=stamp / NS, xyz=(40, 0, 0), q=yaw(0))
    report = view.snapshot(stamp + 10**7, stamp / NS)
    assert report["translation_jump_count"] == report["rotation_jump_count"] == 1
    assert report["motion"][0]["consecutive_normal"] == 0


def test_skew_and_missing_visual_evidence():
    view = Integrity({"required_frames": ["base"], "required_chains": [["base", "tool"]], "visual_check": True})
    send(view)
    send(view, parent="joint", child="tool", stamp=2*NS)
    report = view.snapshot(2*NS, 1)
    assert report["skewed_chains"] == [["base", "tool"]]
    send(view, stamp=2*NS)
    report = view.snapshot(2*NS, 1)
    assert report["state"] == "insufficient_evidence"
    assert report["visual_residual"] is None


def test_sensor_manifest_and_monitor_port():
    card = TfIntegrityMonitor({}, "bumi", None)
    assert card.get_tool()["type"] == "sensor"
    assert card.get_tool()["topic_out"] == [{"topic": "/bumi/tf_integrity_monitor", "format": "data/json"}]
    assert "tf_integrity_monitor, type: sensor" in (BUMI / "driver.yaml").read_text(encoding="utf-8")
    assert card.start()
    assert card.dispatch("info", {})["state"] == "idle"
    assert card.dispatch("stop", {})["state"] == "idle"


def test_bad_names_and_simultaneous_dynamic_disagreement():
    view = Integrity({"required_frames": ["base"]})
    send(view, parent="/base")
    send(view, child="joint//axis")
    send(view)
    send(view, source="B", xyz=(.05, 0, 0))
    report = view.snapshot(NS, 1)
    assert report["issues"]["invalid_frame"] == ["joint", "joint//axis"]
    assert report["conflicting_frames"] == ["joint"]
    assert report["translation_jump_count"] == 0


def test_inactive_publisher_does_not_create_permanent_parent_conflict():
    view = Integrity({"required_frames": ["base"], "required_chains": [["base", "joint"]]})
    send(view, parent="other", stamp=NS, arrival=1)
    send(view, source="replacement", stamp=4*NS, arrival=4)
    report = view.snapshot(4*NS, 4)
    assert report["state"] == "healthy"
    assert not report["stale_frames"] and not report["conflicting_frames"]
