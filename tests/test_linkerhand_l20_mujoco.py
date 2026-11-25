"""Hardware-free tests for the virtual cards exposed by the physical L20 registration."""
import hashlib
import json
from io import BytesIO
from pathlib import Path
import struct
import sys
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET
import zipfile

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand/l20"
sys.path[:0] = [str(ROOT), str(DRIVER)]

from common.vendor_runtime import DriverBundle
from hand_device import build_plugins
from mujoco_card.playback import Playback
from mujoco_card.preview_cards import PreviewCards
from mujoco_card.simulation import Simulation
from mujoco_card.trajectories import VERSION, initial_poses
from mujoco_card.worker import build_virtual_cards


def test_primary_registration_lists_and_routes_virtual_cards_without_hardware():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    bundle = DriverBundle(build_plugins(config, "test_l20", None))
    tools = {tool["name"] for tool in bundle.get_all_tools()}
    assert {"l20_mujoco", "hand_gesture_sim", "hand_grasp_sim", "hand_gesture", "hand_grasp"} <= tools
    reply = {"result": {"content": [{"type": "text", "text": '{"simulation_only":true}'}]}}
    for public, target in {"l20_mujoco": "l20_mujoco", "hand_grasp_sim": "hand_grasp", "hand_gesture_sim": "hand_gesture"}.items():
        with patch("mujoco_card.preview_cards.http.client.HTTPConnection") as connection:
            conn = connection.return_value
            conn.getresponse.return_value = BytesIO(json.dumps(reply).encode())
            assert bundle.dispatch(public, {"action": "info", "instance_id": "my-card"}) == {"simulation_only": True}
            connection.assert_called_once_with("127.0.0.1", config["plugins"]["l20_mujoco"]["worker_port"], timeout=8)
            body = json.loads(conn.request.call_args.args[2])
            assert body["params"] == {"name": target, "arguments": {"action": "info", "instance_id": "my-card"}}


@pytest.mark.parametrize("side", ["left", "right"])
def test_external_trajectory_plays_unmodified_without_motion_sources(side):
    config = {"plugins": {"l20_mujoco": {"worker_port": 15769}}}
    # A custom upstream only needs the wire format. No built-in source or feedback controller is installed.
    absent = {name: None for name in ("simulated_motion_device", "adaptive_grasp_device", "grasp_feedback", "gesture_device")}
    model = Mock()
    model.transforms.side_effect = list
    bus = Mock()
    runtime = Playback({side: model}, initial_poses(config), bus)
    runtime.shutdown()  # Advance the same playback clock deterministically below.
    with patch.dict(sys.modules, absent):
        registered = PreviewCards(config)
        assert [tool["name"] for tool in registered.get_tools()] == ["l20_mujoco"]
        cards = build_virtual_cards(config, runtime)
        assert set(cards) == {"l20_mujoco"}
        card = cards["l20_mujoco"]
        args = {"instance_id": "player"}
        card.dispatch("config", {**args, "bound_side": side})
        route = "/custom/joint_sequence"
        assert card.dispatch("start", {**args, "input_topics": [route]})["state"] == "running"
        receive = bus.bind.call_args.args[2]
        frames = [[(j * 11 + k) % 256 for j in range(20)] for k in (23, 181, 5, 137)]
        packet = dict(schema=VERSION, id="motion-42", side=side, action="calibration_sweep",
                      frame_ms=250, frames=frames)
        with patch("mujoco_card.playback.time.monotonic", return_value=10.0) as clock:
            receive(route, packet)
            for i, frame in enumerate(frames):
                clock.return_value = 10 + i * 0.25
                sample = card.dispatch("live", args)
                assert sample["pose"] == sample["poses"] == frame
                assert sample["action"] == "calibration_sweep"
                if i < len(frames) - 1:
                    clock.return_value += 0.125
                    assert card.dispatch("live", args)["pose"] == frame
            assert sample["last_id"] == packet["id"]
            assert card.dispatch("inspect", {**args, "run_id": packet["id"]})["frames"] == frames
        assert card.dispatch("stop", args)["state"] == "idle"


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
    rt = Playback({"right": model}, {"right": pose}, bus)
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


def test_simulation_lifecycle_reports_bound_instance_state():
    runtime = Playback({"right": Mock()}, {"right": [120] * 20}, Mock())
    card = Simulation(runtime)
    args = {"instance_id": "lifecycle-test", "input_topics": ["/sim/trajectory"]}
    try:
        for action, expected in (("info", "idle"), ("start", "running"), ("start", "running"),
                                 ("info", "running"), ("stop", "idle"), ("stop", "idle"), ("info", "idle")):
            result = card.dispatch(action, args)
            assert result["state"] == expected
            assert result["enabled"] == (expected == "running")
            assert [topic["topic"] for topic in result["topic_in"]] == (args["input_topics"] if result["enabled"] else [])
    finally:
        runtime.shutdown()


def test_preview_logs_first_and_hundredth_failure_and_resets_after_success(capsys):
    bus = Mock()
    initial = {"right": [120] * 20, "left": [120] * 20}
    error = ValueError("bad\nframe\x1b[31m" + "x" * 400)
    broken = Mock()
    broken.jpeg.side_effect = [error] * 101 + [b"ok", error, error]
    runtime = Playback({"left": Mock(), "right": broken}, initial, bus)
    runtime.shutdown()  # Drive the real render loop synchronously, one scheduled frame at a time.
    for side in ("right", "left"):
        runtime.start(side, side, [])
    ticks = 104
    def tick(_interval):
        nonlocal ticks
        if not ticks:
            return True
        runtime.slots["right"].pose[0] = ticks
        ticks -= 1
        return False

    with patch.object(runtime.closed, "wait", side_effect=tick):
        runtime._stream()
    lines = capsys.readouterr().out.splitlines()
    assert [line.split(":", 1)[0] for line in lines] == [
        "[sim preview] right failures=1", "[sim preview] right failures=100", "[sim preview] right failures=1"]
    assert all(r"\n" in line and r"\x1b" in line and len(line) < 225 for line in lines)
    assert broken.jpeg.call_count == 104


@pytest.fixture
def g20_fixture_archive(tmp_path):
    """Use the shipped calibration and URDFs with tiny binary STL fixtures."""
    vertices = [(0, 0, 0), (0.02, 0, 0), (0, 0.02, 0), (0, 0, 0.02)]
    faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
    stl = bytearray(80) + struct.pack("<I", len(faces))
    for a, b, c in faces:
        stl.extend(struct.pack("<12fH", 0, 0, 0, *vertices[a], *vertices[b], *vertices[c], 0))
    archive = tmp_path / "vendor.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as package:
        for side in ("left", "right"):
            prefix = f"L20/{side.upper()}/"
            urdf = DRIVER / "resource" / f"linkerhand_g20_{side}.urdf"
            calibration = DRIVER / "resource" / f"g20_{side}_G20.json"
            for file in (urdf, calibration):
                package.write(file, prefix + file.name)
            for name in {mesh.attrib["filename"] for mesh in ET.parse(urdf).iter("mesh")}:
                package.writestr(prefix + name, stl)
    return archive


def test_both_hand_models_load_from_pinned_archive_layout(g20_fixture_archive, monkeypatch):
    pytest.importorskip("mujoco")
    from mujoco_card import hand_model

    archive = g20_fixture_archive
    # The integrity check applies to the fixture just as it does to the real ZIP.
    monkeypatch.setattr(hand_model, "ARCHIVE_SHA", hashlib.sha256(archive.read_bytes()).hexdigest())
    for side in ("left", "right"):
        model = hand_model.HandModel(archive, side)
        assert model.model.njnt == 21
        assert len(model.transforms([128] * 20)) == len(model.meshes) > 0
        assert sum("pad_faces" in mesh for mesh in model.meshes) == 5
    with archive.open("ab") as fp:
        fp.write(b"changed")
    with pytest.raises(ValueError, match="checksum"):
        hand_model.HandModel(archive, "left")
