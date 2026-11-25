"""Regressions for the upstream camera, VOP and one-shot actuator contracts."""
import json
import sys
import threading
import time
import zlib
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera import PantherCameraRuntime
from common.vendor_runtime import DriverBundle
from device import PantherDevice
from plugins import build_plugins
from tabletop_pick import TabletopPick, VopSnapshot, validate_path


def setup_config():
    return {
        "enabled": True, "allow_motion": True, "side": "right",
        "color_optical_frame": "rgb_optical",
        "base_from_color": [1, 0, 0, 0.1, 0, -1, 0, 0.2, 0, 0, -1, 1.5, 0, 0, 0, 1],
        "workspace_m": [-1, -1, 0.2, 1, 1, 1], "tool_orientation_xyzw": [1, 0, 0, 0],
        "head_rad": [0, 0], "lift_position": 0.6, "clearance_m": 0.08,
        "moveit_group": "right_arm", "tool_link": "gripper_tcp",
        "objects": {"bottle": {"surface_to_tcp_z_m": -0.02, "gripper_open": 0., "gripper_close": 1.}},
    }


def camera_observation():
    camera = PantherCameraRuntime.__new__(PantherCameraRuntime)
    camera._lock = threading.RLock()
    camera._active = {"rgb", "depth"}
    camera._geometry = {"K": np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])}
    color = {"received": time.monotonic(), "stamp_ns": 1_000_000_000, "frame_id": "rgb_optical",
             "geometry": camera._geometry, "data": b"first-jpeg"}
    depth = np.full((480, 640), 1000, dtype="<u2")
    depth[250, 370] = 0
    registered = {**color, "stamp_ns": 1_010_000_000, "data": zlib.compress(depth.tobytes())}
    later_color = {**color, "stamp_ns": 1_500_000_000, "data": b"unsynchronized-jpeg"}
    camera._pick_frames = {"rgb": deque([color, later_color]), "depth": deque([registered])}
    return camera


def test_preview_picks_matching_rectified_jpeg_and_uses_its_depth():
    camera = camera_observation()
    detector = Mock()
    detector.recognize.return_value = {"image_size": [640, 480], "objects": [
        {"name": "bottle", "bbox": [350, 230, 390, 270], "confidence": 0.9,
         "color": {"color_name": "bright red"}},
    ]}
    device = NS(_camera=camera)
    plugin = TabletopPick(setup_config(), device, vop=detector, motion=Mock())
    bundle = DriverBundle([plugin])
    bundle.start_all()
    assert bundle.dispatch("tabletop_pick", {"action": "start"})["state"] == "ready"
    result = bundle.dispatch("tabletop_pick", {"action": "preview", "object_name": "bottle", "color": "red"})
    detector.recognize.assert_called_once_with(b"first-jpeg", 0.5)
    assert result["point_m"] == pytest.approx([0.2, 0.18, 0.48])
    assert result["above_m"] == pytest.approx([0.2, 0.18, 0.56])

    camera._geometry = {"K": camera._geometry["K"].copy()}
    result = bundle.dispatch("tabletop_pick", {"action": "preview", "object_name": "bottle"})
    assert result["state"] == "error"
    assert "no fresh registered RGB-D" in result["message"]


def test_vop_client_uses_existing_perception_upload_and_photo_action(monkeypatch):
    detector = VopSnapshot()
    path = "/models/uploads/panther-tabletop-pick/example.jpg"
    jpeg = b"original jpeg"
    calls = []

    def post(endpoint, body, headers):
        calls.append(endpoint)
        if endpoint == "/file/upload":
            assert b"\r\n\r\n" + jpeg + b"\r\n" in body
            assert headers["Content-Type"].startswith("multipart/form-data")
            return {"ok": True, "path": path}
        assert json.loads(body)["params"] == {"name": "vop", "arguments": {
            "action": "recognize_by_photo", "image_path": path, "confidence": 0.5}}
        return {"result": {"content": [{"text": json.dumps({"ok": True, "source": path})}]}}

    monkeypatch.setattr(detector, "_post", post)
    assert detector.recognize(jpeg, 0.5)["source"] == path
    assert calls == ["/file/upload", "/mcp"]


def test_pick_routes_one_sequence_and_requires_enabled_motion(monkeypatch):
    config = setup_config()
    motion = Mock()
    done = threading.Event()
    completion = []

    def notify(*args):
        completion.append(args)
        done.set()

    plugin = TabletopPick(config, NS(), motion=motion, notify=notify)
    monkeypatch.setattr(plugin, "_preview", lambda args: {
        "object_name": "bottle", "point_m": [0.2, 0.18, 0.48], "above_m": [0.2, 0.18, 0.56],
        "gripper_open": 0., "gripper_close": 1.,
    })
    assert plugin.dispatch("start", {})["state"] == "ready"
    assert plugin.dispatch("pick", {"object_name": "bottle"})["state"] == "error"
    config["allow_motion"] = False
    args = {"object_name": "bottle", "confirm_motion": True, "action_id": "test-pick"}
    assert plugin.dispatch("pick", args)["state"] == "error"
    motion.reserve_pick.assert_not_called()
    config["allow_motion"] = True

    def run_step(side, point, orientation, guard, before_send):
        guard()
        if before_send is not None:
            before_send()

    motion.pick_move.side_effect = run_step
    assert plugin.dispatch("pick", args)["action_id"] == "test-pick"
    assert done.wait(2)
    action_id, status, result, card = completion[0]
    assert (action_id, status, card) == ("test-pick", "completed", "tabletop_pick")
    assert result["grasp_checked"] is False
    assert [call.args[1] for call in motion.pick_move.call_args_list] == [
        [0.2, 0.18, 0.56], [0.2, 0.18, 0.48], [0.2, 0.18, 0.56],
    ]
    assert [call.kwargs["opening"] for call in motion.pick_gripper.call_args_list] == [True, False]
    motion.release_pick.assert_called_once()


def test_pick_excludes_existing_base_and_arm_owners():
    device = PantherDevice.__new__(PantherDevice)
    device._base_gate = device._arm_lock = device._head_lock = threading.RLock()
    device._tabletop_owner = device._base_servo = device._controlled_spatial = None
    device._drive_stop_request = device._drive_action_id = device._head_active = None
    device._closed = False
    owner = threading.Event()
    device._arm_active = {"action_id": "moving"}
    with pytest.raises(RuntimeError, match="stop arm"):
        device.claim_tabletop(owner)
    device._arm_active = None
    device.claim_tabletop(owner)
    for competitor in ("base_drive", "base_servo", "controlled_spatial"):
        with pytest.raises(RuntimeError, match="owns the chassis"):
            device.base_check_available(competitor)
    with pytest.raises(RuntimeError, match="owns the motion"):
        device._require_tabletop_free()
    device._require_tabletop_free(owner)
    device.release_tabletop(owner)
    device.base_check_available("base_drive")


def test_manifest_plugin_and_cartesian_path_limits():
    cfg = {"plugins": {"tabletop_pick": {"enabled": True, "allow_motion": False}}}
    bundle = DriverBundle(build_plugins(cfg, "test", NS()))
    assert bundle.get_all_tools()[0]["type"] == "actuator"
    samples = [NS(positions=[x], velocities=[velocity], accelerations=[], effort=[],
                  time_from_start=NS(sec=0, nanosec=ns))
               for x, velocity, ns in [(0., 0., 0), (0.1, 1., 100_000_000)]]
    trajectory = NS(joint_names=["joint"], points=samples)
    cap = {"speed_deg_s": 20, "max_target_delta_deg": 20, "max_duration_s": 20}
    duration = validate_path(trajectory, ("joint",), {"joint": (-1., 1.)}, {"joint": 0.}, cap)
    assert duration > 0.28
    assert samples[-1].velocities[0] < 0.35
    samples[-1].positions = [0.9]
    with pytest.raises(ValueError, match="delta"):
        validate_path(trajectory, ("joint",), {"joint": (-1., 1.)}, {"joint": 0.}, cap)
