"""G1 teaching over the real recorder/executor with synthetic SDK and inputs."""
import copy
from pathlib import Path
import sys
import threading
import time
from unittest.mock import Mock

import pytest

from common.vendor_runtime import DriverBundle
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teaching import G1TeachingAdapter, teaching_plugin
from test_arm_stream import packet
from test_g1_trajectory import api_rig


def wait_for(predicate, timeout=8):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("condition did not become true")


@pytest.fixture
def g1_teaching(tmp_path):
    card, arm, sample, channel, receipts, done, ticker = api_rig(tmp_path)
    service = teaching_plugin(card)
    frame = copy.deepcopy(card._latest)
    feeding = threading.Event()
    def feed():
        seq = 100
        while not feeding.wait(.04):
            if card._operator and not card._closed.is_set():
                frame.update(sequence=seq, received_monotonic_ns=card.clock())
                card.receive(copy.deepcopy(frame))
                if service.adapter.trajectory.recorder and arm.session_id:
                    arm.accept_control(packet(arm, seq, target=.01))
                seq += 1
    source = threading.Thread(target=feed, daemon=True)
    source.start()
    service.start()
    yield service, card, arm, sample, channel, receipts
    feeding.set(); source.join(2)
    try:
        service.stop()
    finally:
        done.set(); ticker.join(2)


def call(service, action, **args):
    return DriverBundle([service]).dispatch("g1_teaching", {"action": action, **args})


def record(service):
    draft = call(service, "new")
    call(service, "record", draft_id=draft["id"])
    wait_for(lambda: (call(service, "info")["recording"] or {}).get("frames", 0) >= 3)
    result = call(service, "finish", draft_id=draft["id"])
    assert result["complete"] and result["stop"]["authority_released"]
    return call(service, "info")["draft"]


def confirm(service, action, **args):
    preview = call(service, action, execute=False, **args)
    return call(service, action, execute=True, preview_id=preview["preview_id"],
                workspace_clear=True, take_control=True, **args)


def save(service):
    draft = record(service)
    result = confirm(service, "trial", draft_id=draft["id"])
    wait_for(lambda: not call(service, "info")["busy"])
    saved = call(service, "save", draft_id=draft["id"], name="双臂演示", conditions="移动路径畅通，物体位置固定",
                 effect_confirmed=True)
    return draft, saved, result


def test_registration_and_checks_do_not_write_or_create_another_controller(g1_teaching):
    service, card, arm, _, channel, _ = g1_teaching
    before = list(channel.positions)
    tool = service.get_tools()[0]
    assert tool["name"] == "g1_teaching"
    assert tool["inputSchema"]["x-resource"] == ["arm_l", "arm_r"]
    assert service.adapter.trajectory is card.trajectory
    assert call(service, "check")["ready"]
    value = call(service, "info")
    assert value["busy"] and value["can_prepare"]
    assert value["presentation"]["joint_groups"][0]["count"] == 5
    assert not value["presentation"]["grippers"]
    assert channel.positions == before
    assert "html" in call(service, "ui")


def test_recording_trial_saved_skill_reload_run_and_original_acp(g1_teaching):
    service, card, arm, sample, channel, receipts = g1_teaching
    draft, saved, trial = save(service)
    assert receipts[-1][0] == trial["action_id"]
    assert receipts[-1][1] == "completed"
    assert len(receipts) == 1
    instruction = saved["skill"]["instruction"]
    assert "g1_teaching" in instruction and saved["asset_id"] in instruction
    assert "先低速移动到录制起点" in instruction
    assert "不得自动回起点" not in instruction
    assert saved["skill"]["requiredTools"] == ["g1_teaching"]
    recording = service.adapter.inspect(draft["recording"]["filename"])
    assert len(recording["preview"][0]["joints"]) == 10
    assert "widths" not in recording["preview"][0]
    # Reload the published asset into a fresh service: no active PICO is needed.
    reloaded = teaching_plugin(card)
    result = confirm(reloaded, "run", asset_id=saved["asset_id"])
    wait_for(lambda: call(reloaded, "info")["last_run"] is not None)
    assert call(reloaded, "info")["last_run"]["status"] == "completed"
    assert receipts[-1][0] == result["action_id"] and len(receipts) == 2
    assert not arm.session_id and channel.positions


def test_g1_start_difference_is_display_not_piper_position_gate(g1_teaching):
    service, _, _, sample, _, _ = g1_teaching
    draft = record(service)
    sample["dq"] = [.03]*10
    result = call(service, "inspect", draft_id=draft["id"])["start"]
    assert result["ready"] is False
    assert result["can_trial"] and result["mode"] == "move_to_start"
    assert "tolerance_label" not in result and "width_delta_mm" not in result


def test_g1_cannot_record_before_native_mapping_or_after_session_change(g1_teaching):
    service, card, _, _, _, _ = g1_teaching
    draft = call(service, "new")
    session = card._operator
    card._operator = None
    with pytest.raises(ValueError, match="live_mapped"):
        call(service, "record", draft_id=draft["id"])
    card._operator = session
    call(service, "record", draft_id=draft["id"])
    card._operator = "new-native-session"
    with pytest.raises(ValueError, match="start_teaching_teleop_first"):
        call(service, "record", draft_id=draft["id"])
    card._operator = session


def test_finish_waits_for_file_and_requires_stop_confirmation(g1_teaching, monkeypatch):
    service, card, _, _, _, _ = g1_teaching
    partial = {"complete": False, "stopping": True}
    monkeypatch.setattr(card.trajectory, "record_stop", lambda: partial)
    stop = Mock(return_value={"authority_released": True})
    monkeypatch.setattr(card, "stop", stop)
    assert service.adapter.finish() == partial
    stop.assert_not_called()
    monkeypatch.setattr(card.trajectory, "record_stop", lambda: {"complete": True, "filename": "g1-test.jsonl"})
    stop.return_value = {"state": "error", "error": "stop_unconfirmed"}
    with pytest.raises(ValueError, match="stop_unconfirmed"):
        service.adapter.finish()
    # Restore before fixture shutdown.
    monkeypatch.undo()


def test_preview_does_not_play_and_stop_invalidates_it(g1_teaching, monkeypatch):
    service, card, _, _, channel, _ = g1_teaching
    draft = record(service)
    before = len(channel.positions)
    preview = call(service, "trial", draft_id=draft["id"], execute=False)
    assert len(channel.positions) == before
    call(service, "stop")
    with pytest.raises(ValueError, match="teaching_preview_expired"):
        call(service, "trial", draft_id=draft["id"], execute=True,
             preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
    assert len(channel.positions) == before


def test_stop_during_inspection_cannot_launch_replay(g1_teaching, monkeypatch):
    service, card, _, _, channel, _ = g1_teaching
    draft = record(service)
    original = service.adapter.inspect
    def inspect(filename):
        value = original(filename)
        service.adapter.stop()
        return value
    monkeypatch.setattr(service.adapter, "inspect", inspect)
    before = len(channel.positions)
    with pytest.raises(ValueError, match="replay_cancelled"):
        service.adapter.play(draft["recording"]["filename"], {})
    assert len(channel.positions) == before


def test_changed_device_cannot_run_saved_skill(g1_teaching):
    service, card, _, _, _, _ = g1_teaching
    _, saved, _ = save(service)
    card.cfg["trajectory_device_id"] = "another-g1"
    with pytest.raises(ValueError, match="binding_mismatch"):
        call(service, "run", asset_id=saved["asset_id"])


def test_preview_fences_stop_at_native_handoff(g1_teaching, monkeypatch):
    service, card, _, _, channel, _ = g1_teaching
    draft = record(service)
    preview = call(service, "trial", draft_id=draft["id"], execute=False)
    original = card.trajectory.play
    def play(args):
        card.stop()
        return original(args)
    monkeypatch.setattr(card.trajectory, "play", play)
    before = len(channel.positions)
    with pytest.raises(ValueError, match="replay_cancelled"):
        call(service, "trial", draft_id=draft["id"], execute=True,
             preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
    assert len(channel.positions) == before


def test_native_play_checks_preview_digest_before_claim(g1_teaching):
    service, card, arm, _, channel, _ = g1_teaching
    draft = record(service)
    before = len(channel.positions)
    with pytest.raises(ValueError, match="recording_changed_after_preview"):
        card.trajectory.play({"filename": draft["recording"]["filename"], "expected_sha256": "changed"})
    assert len(channel.positions) == before and not arm.session_id


def test_existing_recording_can_be_loaded_without_vr(g1_teaching):
    service, card, _, _, _, _ = g1_teaching
    draft = record(service)
    card._latest = None
    assert not call(service, "check")["ready"]
    loaded = call(service, "new", filename=draft["recording"]["filename"])
    assert loaded["recording"]["sha256"] == draft["recording"]["sha256"]
    assert call(service, "inspect", draft_id=loaded["id"])["start"]["can_trial"]


def test_g1_dockerfile_covers_teaching_imports():
    from test_dockerfile_copies import test_every_copied_path_exists, test_every_imported_sibling_is_copied
    dockerfile = Path(__file__).resolve().parents[1] / "Dockerfile"
    test_every_copied_path_exists(dockerfile)
    test_every_imported_sibling_is_copied(dockerfile)


def test_cancelled_trial_cannot_publish(g1_teaching):
    service, card, _, _, _, _ = g1_teaching
    draft = record(service)
    confirm(service, "trial", draft_id=draft["id"])
    service.adapter.stop()
    wait_for(lambda: not call(service, "info")["busy"])
    assert service.draft["trial"]["status"] == "cancelled"
    with pytest.raises(ValueError, match="successful_trial_required"):
        call(service, "save", draft_id=draft["id"], name="test", conditions="test", effect_confirmed=True)
