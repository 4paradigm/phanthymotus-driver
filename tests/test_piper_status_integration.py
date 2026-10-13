"""Preserve the deployed status split without opening a second CAN connection."""
import time
from unittest.mock import Mock

import pytest

from agilex.piper.device import build_plugins
from agilex.piper.monitor import monitor_frame
from common.vendor_runtime import DriverBundle


def test_status_lifecycle_and_model_do_not_connect(tmp_path):
    plugins = build_plugins({"data_dir": str(tmp_path)}, "piper", None)
    runtime, status = plugins
    runtime.factory = Mock(side_effect=AssertionError("unexpected CAN connection"))
    bundle = DriverBundle(plugins)
    assert {t["name"] for p in plugins for t in p.get_tools()} == {
        "arm_control", "piper_photo", "piper_status", "model"}
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
