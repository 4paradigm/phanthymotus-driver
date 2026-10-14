"""Real teaching lifecycle over the simulated SDK; no CAN or VR hardware opened."""
import copy
import json
import math
import time
from pathlib import Path
from unittest.mock import Mock

import pytest

from agilex.piper.device import build_plugins
from agilex.piper.teaching import PiperTeachingAdapter
from common.teaching import TeachingPlugin
from common.vendor_runtime import DriverBundle
from test_piper_dual import pair, ready, wait_for, feed, frame


@pytest.fixture
def teaching(pair, tmp_path, monkeypatch):
    monkeypatch.setattr(pair.arms["left"], "acp_complete", Mock())
    service = TeachingPlugin(PiperTeachingAdapter(pair), tmp_path / "skills", "piper_teaching")
    service.start()
    yield service
    service.stop()


def call(service, action, **args):
    return DriverBundle([service]).dispatch("piper_teaching", {"action": action, **args})


def confirm(service, action, **args):
    preview = call(service, action, **args)
    return call(service, action, **args, execute=True, preview_id=preview["preview_id"],
                workspace_clear=True, take_control=True)


def recorded(service, pair):
    ready(pair)
    draft = call(service, "new")
    confirm(service, "begin", draft_id=draft["id"])
    wait_for(lambda: pair.motion.reason == "waiting_for_input")
    pair.motion.receive(frame(1))
    wait_for(lambda: pair.motion.reason == "mapped_waiting_for_both_grips")
    call(service, "record", draft_id=draft["id"])
    feed(pair, .4)
    result = call(service, "finish", draft_id=draft["id"])
    assert result["complete"]
    wait_for(lambda: not call(service, "info")["busy"])
    return call(service, "info")["draft"]


def restore_simulated_start(service, pair, draft):
    # Change simulated feedback only. The production adapter deliberately has
    # no automatic return-to-start command.
    start = call(service, "inspect", draft_id=draft["id"])["start"]
    for i, arm in enumerate(pair.arms.values()):
        arm.driver.arm.raw = tuple(round(q * 1000) for q in start["target_joints_deg"][i*6:(i+1)*6])
        arm.driver.arm.width = start["target_width_mm"][i]


METADATA = {"name": "零件放入托盘", "description": "把固定位置的零件放到右侧托盘",
            "conditions": "双臂、夹爪、物体恢复示教起点", "phrase": "把零件放到托盘里"}


def saved(service, pair):
    draft = recorded(service, pair)
    restore_simulated_start(service, pair, draft)
    confirm(service, "trial", draft_id=draft["id"])
    wait_for(lambda: not call(service, "info")["busy"])
    result = call(service, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)
    return draft, result


def test_factory_and_discovery_are_passive(tmp_path):
    plugins = build_plugins({"data_dir": str(tmp_path),
                             "arms": {"left": {"can_interface": "can0"}, "right": {"can_interface": "can1"}}}, "piper", None)
    assert len(plugins) == 2
    definition = plugins[1].get_tools()[0]
    assert definition["inputSchema"]["x-workbench"]["protocol"] == "motus.workbench/1"
    assert "run" in definition["inputSchema"]["x-completion"]["actions"]
    assert all(arm.driver is None for arm in plugins[0].arms.values())
    assert len(plugins[0].get_tools()) == 12
    assert not tmp_path.joinpath("skills").exists()


def test_preflight_reports_independent_failures_without_moving(teaching, pair):
    ready(pair)
    pair.arms['left'].motion_enabled = False
    pair.bus.running = False
    result = call(teaching, 'check')
    checks = {item['id']: item for item in result['checks']}
    assert not result['ready']
    assert not checks['permission']['ok'] and not checks['transport']['ok']
    assert checks['feedback']['ok'] and checks['calibration']['ok']
    assert checks['permission']['help'] and checks['permission']['reason']
    assert all(arm.driver.arm.calls == [] for arm in pair.arms.values())
    assert pair.motion.mode == 'idle'
    assert not call(teaching, 'info')['can_record']
    pair.arms['left'].motion_enabled = True
    pair.bus.running = True
    with pair.arms['left'].gate:
        result = call(teaching, 'check')
        assert not next(c for c in result['checks'] if c['id'] == 'control')['ok']
    assert call(teaching, 'check')['ready']


def test_record_requires_fresh_input_and_completed_mapping(teaching, pair):
    ready(pair)
    draft = call(teaching, 'new')
    confirm(teaching, 'begin', draft_id=draft['id'])
    wait_for(lambda: pair.motion.reason == 'waiting_for_input')
    with pytest.raises(RuntimeError, match='live_mapped'):
        call(teaching, 'record', draft_id=draft['id'])
    pair.motion.receive(frame(1, True))
    wait_for(lambda: pair.motion.reason == 'release_both_grips_to_map')
    info = call(teaching, 'info')
    assert info['input_fresh'] and not info['mapped'] and not info['can_record']
    with pytest.raises(RuntimeError, match='live_mapped'):
        call(teaching, 'record', draft_id=draft['id'])
    pair.motion.receive(frame(2))
    wait_for(lambda: pair.motion.reason == 'mapped_waiting_for_both_grips')
    assert call(teaching, 'info')['can_record']
    # Simulate an expired sample without running a stale-motion test on hardware.
    with pair.motion.lock:
        pair.motion.latest['received_monotonic_ns'] = time.monotonic_ns() - 1_000_000_000
        assert not call(teaching, 'info')['can_record']
        with pytest.raises(RuntimeError, match='live_mapped'):
            call(teaching, 'record', draft_id=draft['id'])
    assert pair.motion.recorder is None


def test_record_trial_save_restart_and_run_share_motion_owner(teaching, pair):
    draft = recorded(teaching, pair)
    assert draft["recording"]["frames"] >= 3
    inspection = call(teaching, "inspect", draft_id=draft["id"])
    assert not inspection["start"]["ready"]
    with pytest.raises(ValueError, match="start_mismatch"):
        call(teaching, "trial", draft_id=draft["id"])
    with pytest.raises(ValueError, match="successful_trial_required"):
        call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)
    restore_simulated_start(teaching, pair, draft)
    assert call(teaching, "inspect", draft_id=draft["id"])["start"]["ready"]
    with pytest.raises(ValueError, match="confirmation|workspace_clear"):
        call(teaching, "trial", draft_id=draft["id"], execute=True)
    running = confirm(teaching, "trial", draft_id=draft["id"])
    assert running["state"] == "running"
    with pytest.raises(RuntimeError, match="busy"):
        call(teaching, "new")
    wait_for(lambda: not call(teaching, "info")["busy"])
    assert call(teaching, "info")["draft"]["trial"]["status"] == "completed"
    with pytest.raises(ValueError, match="user_effect_confirmation_required"):
        call(teaching, "save", draft_id=draft["id"], **METADATA)
    result = call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)
    assert result == call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)
    with pytest.raises(ValueError, match="already_saved"):
        call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **{**METADATA, "name": "改变名称"})
    asset = json.loads((teaching.directory / (result["asset_id"] + ".json")).read_text())
    assert asset["recording"]["sha256"] == draft["recording"]["sha256"]
    assert result["skill"]["requiredTools"] == ["piper_teaching"]
    assert result["asset_id"] in result["skill"]["instruction"]
    restarted = TeachingPlugin(teaching.adapter, teaching.directory, teaching.name)
    restore_simulated_start(teaching, pair, draft)
    preview = call(restarted, "run", asset_id=result["asset_id"])
    assert preview["executed"] is False
    assert pair.motion.mode == "idle"
    pair.arms["left"].acp_complete.reset_mock()
    running = call(restarted, "run", asset_id=result["asset_id"], execute=True,
                   preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
    wait_for(lambda: not call(restarted, "info")["busy"])
    assert call(restarted, "info")["last_run"]["action_id"] == running["action_id"]
    assert call(restarted, "info")["last_run"]["status"] == "completed"
    # A single original ACP callback also completes the teaching tool's pending ID.
    pair.arms["left"].acp_complete.assert_called_once()
    assert pair.arms["left"].acp_complete.call_args.args[0] == running["action_id"]


def test_cancelled_trial_cannot_be_saved(teaching, pair):
    draft = recorded(teaching, pair)
    restore_simulated_start(teaching, pair, draft)
    confirm(teaching, "trial", draft_id=draft["id"])
    assert call(teaching, "stop")["motor_torque_released"] is False
    wait_for(lambda: not call(teaching, "info")["busy"])
    assert call(teaching, "info")["draft"]["trial"]["status"] == "cancelled"
    with pytest.raises(ValueError, match="successful_trial_required"):
        call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)


def test_completion_observer_failure_stops_and_blocks_publication(teaching, pair, monkeypatch):
    draft = recorded(teaching, pair)
    restore_simulated_start(teaching, pair, draft)
    def failed_wait(action_id):
        raise RuntimeError("completion_transport_failed")
    monkeypatch.setattr(teaching.adapter, "wait", failed_wait)
    confirm(teaching, "trial", draft_id=draft["id"])
    wait_for(lambda: not call(teaching, "info")["busy"])
    assert call(teaching, "info")["draft"]["trial"]["status"] == "error"
    with pytest.raises(ValueError, match="successful_trial_required"):
        call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)


def test_asset_tampering_or_changed_binding_is_rejected(teaching, pair):
    draft, result = saved(teaching, pair)
    path = teaching.directory / (result["asset_id"] + ".json")
    asset = json.loads(path.read_text())
    for field, bad in (("sha256", "0"*64), ("binding", {})):
        broken = copy.deepcopy(asset); broken["recording"][field] = bad
        path.write_text(json.dumps(broken))
        with pytest.raises(ValueError, match="recording_or_binding_changed"):
            call(teaching, "run", asset_id=result["asset_id"])
    path.write_text(json.dumps(asset))
    record_path = pair.motion.directory / draft["recording"]["filename"]
    record_path.write_text(record_path.read_text().replace('"send_ok":[true,true]', '"send_ok":[false,true]'))
    with pytest.raises(ValueError):
        call(teaching, "run", asset_id=result["asset_id"])


@pytest.mark.parametrize("asset_id", ["../secrets", "/tmp/file", "", "g"*32, None])
def test_asset_path_is_not_caller_controlled(teaching, asset_id):
    with pytest.raises(ValueError):
        call(teaching, "run", asset_id=asset_id)


def test_asset_symlink_and_stale_draft_are_rejected(teaching, tmp_path):
    teaching.directory.mkdir()
    outside = tmp_path / "outside.json"; outside.write_text('{}')
    (teaching.directory / ("a"*32 + ".json")).symlink_to(outside)
    with pytest.raises(OSError):
        call(teaching, "run", asset_id="a"*32)
    first = call(teaching, "new")
    call(teaching, "new")
    with pytest.raises(ValueError, match="draft_changed"):
        call(teaching, "record", draft_id=first["id"])


def test_empty_recording_and_unverified_devices_do_not_create_skills(teaching, pair):
    assert not call(teaching, "check")["ready"]
    draft = call(teaching, "new")
    with pytest.raises(ValueError, match="complete_recording_required"):
        call(teaching, "save", draft_id=draft["id"], effect_confirmed=True, **METADATA)
    assert not teaching.directory.exists()


def test_workbench_is_shipped_in_driver_and_has_no_external_dependencies(teaching):
    ui = call(teaching, "ui")
    assert ui["protocol"] == "motus.workbench/1"
    assert '<script src=' not in ui["html"]
    assert 'motus.workbench/1' in ui["html"]
    # The repository-wide parametrization excludes hidden paths (including
    # .worktrees). Exercise its existing checks explicitly for this image.
    from test_dockerfile_copies import test_every_copied_path_exists, test_every_imported_sibling_is_copied
    root = Path(__file__).resolve().parents[1]
    dockerfile = root / 'agilex/piper/Dockerfile'
    test_every_copied_path_exists(dockerfile)
    test_every_imported_sibling_is_copied(dockerfile)
    assert 'COPY common/ /work/common/' in dockerfile.read_text()
    assert (root / 'common/teaching/workbench.html').is_file()
