"""USB role pairing: duplicate adapters, persistence and motion admission."""
import json
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linkerhand/l20_lite"))
import usb_pairing as usb

RECEIVER = "/dev/serial/by-path/platform-usb-0:2.4:1.0-port0"
HAND = "/dev/serial/by-path/platform-usb-0:2.1:1.0-port0"
OTHER = "/dev/serial/by-path/platform-usb-0:2.2:1.0-port0"


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv("L20_USB_PAIRING_FILE", str(tmp_path / "pairing.json"))
    monkeypatch.setenv("GLOVE_SERIAL_PORT", "/dev/old-glove")
    monkeypatch.setenv("L20_SERIAL_PORT", "/dev/old-hand")
    closed = []
    connection = SimpleNamespace(port="/dev/old-hand", _glove_port="/dev/old-glove", close=lambda: closed.append(True))
    hand = SimpleNamespace(_legacy=SimpleNamespace(_active=None, connection=connection),
        _controllers={}, _ownership_lock=threading.RLock(), _bus_lock=threading.RLock(),
        _teleop_active=False, config={"serial_port": "/dev/old-hand"})
    cleanup = threading.Event()
    cleanup.set()
    glove = SimpleNamespace(_armed=False, _arming=False, _pending_disarm=False,
        _cleanup_done=cleanup, _stop=lambda: None, glove_port="/dev/old-glove")
    devices, probes = [], []
    now = [0.0]
    def probe(port):
        assert port != RECEIVER, "receiver must never get a Modbus probe"
        probes.append(port)
        return ["left"]
    plugin = usb.UsbPairingPlugin(hand, glove, list_devices=lambda: devices[:],
                                 probe=probe, clock=lambda: now[0])
    return SimpleNamespace(plugin=plugin, hand=hand, glove=glove, devices=devices,
                           probes=probes, now=now, closed=closed)


def call(rig, action, session=None):
    return rig.plugin.dispatch(action, {} if session is None else {"session_id": session})


def review(rig):
    session = call(rig, "begin")["session_id"]
    rig.devices.append(RECEIVER)
    call(rig, "receiver", session)
    rig.devices.append(HAND)
    call(rig, "hands", session)
    return session


def test_pairing_without_paths_persists_restores_and_never_probes_receiver(rig, monkeypatch):
    session = review(rig)
    result = call(rig, "apply", session)
    assert result["state"] == "paired" and result["armed"] is False
    assert rig.probes == [HAND, HAND]
    assert rig.glove.glove_port == RECEIVER
    assert rig.hand.config["usb_paired_hands"] == {"left": HAND}
    monkeypatch.setenv("GLOVE_SERIAL_PORT", "/dev/wrong-after-restart")
    assert usb.restore_pairing()["hands"] == {"left": HAND}
    assert os.environ["GLOVE_SERIAL_PORT"] == RECEIVER
    assert os.environ["L20_SERIAL_PORT"] == HAND
    assert call(rig, "status")["missing"] == []
    rig.devices.remove(RECEIVER)
    assert call(rig, "status")["missing"] == [RECEIVER]


def test_two_identical_adapters_inserted_together_are_rejected(rig):
    session = call(rig, "begin")["session_id"]
    rig.devices.extend([RECEIVER, HAND])
    with pytest.raises(ValueError, match="只插入一个"):
        call(rig, "receiver", session)
    assert rig.probes == []
    assert not usb.pairing_file().exists()


def test_existing_unrelated_serial_device_is_never_probed(rig):
    rig.devices.append(OTHER)
    session = review(rig)
    call(rig, "apply", session)
    assert OTHER not in rig.probes


def test_receiver_unplugged_during_wizard_is_rejected_before_probe(rig):
    session = call(rig, "begin")["session_id"]
    rig.devices.append(RECEIVER)
    call(rig, "receiver", session)
    rig.devices[:] = [HAND]
    with pytest.raises(ValueError, match="接收器已断开"):
        call(rig, "hands", session)
    assert rig.probes == []


def test_duplicate_side_and_non_hand_devices_do_not_save(rig):
    session = call(rig, "begin")["session_id"]
    rig.devices.append(RECEIVER)
    call(rig, "receiver", session)
    rig.devices.extend([HAND, OTHER])
    with pytest.raises(ValueError, match="重复"):
        call(rig, "hands", session)
    rig.devices.remove(OTHER)
    rig.plugin.probe = lambda port: []
    with pytest.raises(ValueError, match="未识别"):
        call(rig, "hands", session)
    assert not usb.pairing_file().exists()


def test_apply_revalidates_device_and_preserves_previous_mapping(rig):
    session = review(rig)
    old = {"version": 1, "receiver": RECEIVER, "hands": {"left": OTHER}}
    usb.pairing_file().write_text(json.dumps(old))
    rig.plugin.probe = lambda port: ["right"]
    with pytest.raises(ValueError, match="识别发生变化"):
        call(rig, "apply", session)
    assert usb.load_pairing() == old
    assert rig.glove.glove_port == "/dev/old-glove"


@pytest.mark.parametrize("busy", ["teleop", "motion", "cleanup"])
def test_begin_refuses_live_control_or_pending_release(rig, busy):
    if busy == "teleop": rig.hand._teleop_active = True
    elif busy == "motion": rig.hand._legacy._active = {"action_id": "moving"}
    else: rig.glove._cleanup_done.clear()
    with pytest.raises(RuntimeError, match="退出遥操"):
        call(rig, "begin")
    assert not rig.closed


def test_session_blocks_motion_other_tabs_and_expires(rig):
    session = call(rig, "begin")["session_id"]
    with pytest.raises(RuntimeError, match="配对进行中"):
        usb.ensure_not_pairing(rig.hand)
    with pytest.raises(RuntimeError, match="已有配对"):
        call(rig, "begin")
    with pytest.raises(ValueError, match="不匹配"):
        call(rig, "cancel", "another-tab")
    rig.now[0] = 601
    usb.ensure_not_pairing(rig.hand)
    with pytest.raises(ValueError, match="过期"):
        call(rig, "apply", session)
    assert call(rig, "cancel", session)["state"] == "cancelled"


def test_cancel_keeps_previous_runtime_configuration(rig):
    session = review(rig)
    assert call(rig, "cancel", session)["state"] == "cancelled"
    assert rig.glove.glove_port == "/dev/old-glove"
    assert not usb.pairing_file().exists()


def test_first_boot_gate_does_not_allow_device_access_before_pairing(rig, monkeypatch):
    monkeypatch.setenv("L20_USB_PAIRING_REQUIRED", "1")
    with pytest.raises(RuntimeError, match="请先通过"):
        usb.ensure_not_pairing(rig.hand)
    session = review(rig)
    call(rig, "apply", session)
    usb.ensure_not_pairing(rig.hand)


@pytest.mark.parametrize("data", [
    {"version": 1, "receiver": RECEIVER, "hands": {"left": RECEIVER}},
    {"version": 1, "receiver": "/tmp/fake", "hands": {"left": HAND}},
    {"version": 1, "receiver": RECEIVER, "hands": {}},
    {"version": 1, "receiver": RECEIVER, "hands": {"unknown": HAND}},
])
def test_corrupt_saved_roles_are_rejected(data):
    with pytest.raises(ValueError):
        usb.validate_mapping(data)
