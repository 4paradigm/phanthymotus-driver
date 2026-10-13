"""Preserve the deployed status split without opening a second CAN connection."""
import time
from unittest.mock import Mock

import pytest

from agilex.piper.device import build_plugins
from agilex.piper.monitor import monitor_frame
from common.vendor_runtime import DriverBundle


def test_status_lifecycle_and_model_do_not_connect(tmp_path):
    plugins = build_plugins({"data_dir": str(tmp_path)}, "piper", None)
    runtime, status, camera = plugins
    runtime.factory = Mock(side_effect=AssertionError("unexpected CAN connection"))
    bundle = DriverBundle(plugins)
    assert {t["name"] for p in plugins for t in p.get_tools()} == {
        "arm_control", "piper_photo", "piper_status", "model", "piper_camera_live"}
    for action in ("start", "info", "stop"):
        bundle.dispatch("piper_status", {"action": action})
    assert "joint6" in bundle.dispatch("model", {})["urdf"]
    assert bundle.dispatch("arm_control", {"action": "select_action"})["executed"] is False
    with pytest.raises(ValueError):
        bundle.dispatch("arm_control", {"action": "read"})
    with pytest.raises(ValueError):
        bundle.dispatch("piper_status", {"action": "read", "execute": True})
    runtime.factory.assert_not_called()
    runtime.stop()


def test_stale_feedback_never_creates_a_skeleton():
    now = time.time()
    snapshot = dict(connected=True, fresh=True, joints_deg=[0.] * 6,
                    feedback_time=now, motion_enabled=False)
    assert monitor_frame(snapshot, now=now, device_name="test")[0] is not None
    assert monitor_frame(snapshot, now=now + 1, device_name="test")[0] is None
    assert monitor_frame(snapshot, now=now - 1, device_name="test")[0] is None


def test_camera_gate_and_exclusive_owner(tmp_path):
    from agilex.piper.camera_live import PiperCameraLive
    from agilex.piper.camera_owner import CameraOwner
    runtime = Mock(config={"camera": {"live": {"enabled": True}}},
                   topic="/piper/rgb", camera_enabled=False)
    factory = Mock(side_effect=AssertionError("unexpected camera capture"))
    camera = PiperCameraLive(runtime, session_factory=factory)
    assert camera.start()["state"] == "disabled"
    camera.stop()
    factory.assert_not_called()
    with CameraOwner(tmp_path, "/dev/piper-camera-1"):
        with pytest.raises(RuntimeError):
            CameraOwner(tmp_path, "/dev/piper-camera-1")
    with CameraOwner(tmp_path, "/dev/piper-camera-1"):
        pass


def test_diagnose_uses_only_passive_info(tmp_path, monkeypatch):
    import json
    from agilex.piper.deploy import diagnose
    seen = []
    names = ["arm_control", "piper_photo", "piper_status", "model", "piper_camera_live"]
    def rpc(endpoint, method, params=None):
        seen.append((method, params))
        if method == "tools/list":
            return {"tools": [{"name": name} for name in names]}
        if method == "initialize":
            return {}
        assert params == {"name": "arm_control", "arguments": {"action": "info"}}
        return {"content": [{"text": json.dumps({"snapshot": {"connected": False},
                                                  "motion_enabled": False})}]}
    output = tmp_path / "report.json"
    monkeypatch.setattr(diagnose, "rpc", rpc)
    monkeypatch.setattr(diagnose, "host_check", lambda args: {})
    monkeypatch.setattr("sys.argv", ["diagnose", "--output", str(output)])
    with pytest.raises(SystemExit) as result:
        diagnose.main()
    assert result.value.code == 0
    report = json.loads(output.read_text())
    assert report["validation_scope"] == "software_only"
    assert report["control_commands_requested"] == 0
