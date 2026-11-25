"""Hardware-free tests for the virtual cards exposed by the physical L20 registration."""
import json
from io import BytesIO
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand/l20"
sys.path[:0] = [str(ROOT), str(DRIVER)]

from common.vendor_runtime import DriverBundle
from hand_device import build_plugins
from mujoco_card.playback import Playback
from mujoco_card.preview_cards import PreviewCards
from mujoco_card.trajectories import VERSION


def test_primary_registration_lists_and_routes_virtual_cards_without_hardware():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    bundle = DriverBundle(build_plugins(config, "test_l20", None))
    tools = {tool["name"] for tool in bundle.get_all_tools()}
    assert {"l20_mujoco", "hand_gesture_sim", "hand_grasp_sim", "hand_gesture", "hand_grasp"} <= tools
    reply = {"result": {"content": [{"type": "text", "text": '{"simulation_only":true}'}]}}
    for public, target in PreviewCards.routes.items():
        with patch("mujoco_card.preview_cards.http.client.HTTPConnection") as connection:
            conn = connection.return_value
            conn.getresponse.return_value = BytesIO(json.dumps(reply).encode())
            assert bundle.dispatch(public, {"action": "info", "instance_id": "my-card"}) == {"simulation_only": True}
            connection.assert_called_once_with("127.0.0.1", config["plugins"]["l20_mujoco"]["worker_port"], timeout=8)
            body = json.loads(conn.request.call_args.args[2])
            assert body["params"] == {"name": target, "arguments": {"action": "info", "instance_id": "my-card"}}


def test_unavailable_preview_fails_without_falling_back_to_physical_grasp():
    settings = yaml.safe_load((DRIVER / "config.yaml").read_text())
    bundle = DriverBundle(build_plugins(settings, "test_l20", None))
    with patch("mujoco_card.preview_cards.http.client.HTTPConnection") as connection:
        connection.return_value.request.side_effect = ConnectionRefusedError
        with pytest.raises(RuntimeError, match="虚拟手服务暂不可用"):
            bundle.dispatch("hand_grasp_sim", {"action": "prepare", "instance_id": "only-virtual"})


def test_playback_binding_busy_and_rejection():
    pose = [120] * 20
    bus = Mock()
    model = Mock()
    rt = Playback({"right": model}, SimpleNamespace(open={"right": pose}), bus)
    topic = "/sim/test"
    try:
        rt.start("first", "right", [topic])
        bus.bind.assert_called_once()
        with pytest.raises(ValueError, match="同侧"):
            rt.start("second", "right", [topic])
        packet = dict(schema=VERSION, id="accepted", action="gesture", side="right",
                      frame_ms=1000, frames=[pose, pose, pose])
        for invalid in ({**packet, "side": "left"}, {**packet, "frames": [[1]]}):
            rt.receive("first", topic, invalid)
            state = rt.read("first", "right")
            assert state["rejected"] and not state["busy"]
        rt.receive("first", topic, packet)
        rt.receive("first", topic, {**packet, "id": "competing"})
        assert rt.read("first", "right")["active_id"] == "accepted"
        assert "忙" in rt.read("first", "right")["rejected"]
        rt.stop("first")
        bus.unbind.assert_called_once_with("first")
        assert not rt.read("first", "right")["enabled"]
        rt.start("second", "right", [topic])
    finally:
        rt.shutdown()
