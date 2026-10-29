"""Portable recordings lose source authority and require a local trial."""
import hashlib
import json
from pathlib import Path

import pytest

from skill_workbench.package import pack, unpack
from test_g1_teaching import g1_teaching, call, confirm, save, wait_for


def export(service, asset_id):
    result = call(service, "export", asset_id=asset_id)
    data = Path(result["path"]).read_bytes()
    assert hashlib.sha256(data).hexdigest() == result["sha256"]
    return data


def forget_local_assets(service):
    # A second same-model G1 has none of this device's verified receipts.
    for path in service.directory.glob("*.json"):
        path.unlink()


def upload(service, data):
    path = service.directory.parent / 'uploaded.motus-skill'
    path.write_bytes(data)
    return call(service, "import", package=str(path.resolve()))


def test_roundtrip_to_another_device_requires_fresh_trial(g1_teaching):
    service, card, _, _, channel, _ = g1_teaching
    original, saved, _ = save(service)
    before = len(channel.positions)
    data = export(service, saved["asset_id"])
    definition = call(service, "package_definition", asset_id=saved["asset_id"])["skill"]
    assert definition["attachment"]["asset_id"] == saved["asset_id"]
    assert ".preview" in definition["instruction"] and "必须一次传齐" in definition["instruction"]
    manifest, recording = unpack(data)
    assert manifest["compatibility"]["variant"] == "G1_23"
    assert manifest["compatibility"]["end_effectors"] == "fixed_hand"
    assert len(manifest["compatibility"]["joint_names"]) == 10
    assert b'"device_id"' not in recording
    lines = [json.loads(line) for line in recording.splitlines()]
    assert all(f["session_id"] == "portable" for f in lines[1:-1])
    forget_local_assets(service)
    result = upload(service, data)
    draft = result["draft"]
    assert result["requires_local_trial"] and not result["installed"]
    assert draft["id"] != saved["asset_id"] and draft["trial"] is None
    assert draft["recording"]["binding"] == card.trajectory.binding()
    assert draft["recording"]["filename"] != original["recording"]["filename"]
    assert len(channel.positions) == before
    with pytest.raises(ValueError, match="successful_trial_required"):
        call(service, "save", draft_id=draft["id"], name="导入动作", conditions="路径畅通", effect_confirmed=True)
    confirm(service, "trial", draft_id=draft["id"])
    wait_for(lambda: not call(service, "info")["busy"])
    local = call(service, "save", draft_id=draft["id"], name="导入动作", conditions="路径畅通", effect_confirmed=True)
    assert local["skill"]["attachment"]["asset_id"] == draft["id"]
    assert saved["asset_id"] not in local["skill"]["instruction"]


def test_mismatched_robot_and_pending_draft_rejected_without_motion(g1_teaching):
    service, _, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service)
    data = export(service, saved["asset_id"])
    manifest, recording = unpack(data)
    before = len(channel.positions)
    for key, value in (("variant", "G1_29"), ("end_effectors", "dexterous_hand"),
                       ("joint_names", list(reversed(manifest["compatibility"]["joint_names"]))),
                       ("model_sha256", "0"*64), ("geometry_sha256", "0"*64)):
        _, changed = pack(manifest["skill"], {**manifest["compatibility"], key: value}, recording,
                          manifest["recording"]["schema"])
        with pytest.raises(ValueError, match="incompatible_robot_" + key):
            upload(service, changed)
        assert service.draft["saved"]["id"] == saved["asset_id"]
    # An empty workbench draft is safe to replace; an unsaved recording is not.
    call(service, "new", filename=service.draft['recording']['filename'])
    with pytest.raises(ValueError, match="current_draft"):
        upload(service, data)
    assert len(channel.positions) == before


def test_same_device_reuses_only_saved_validation_and_pending_survives_restart(g1_teaching):
    from skill_workbench import TeachingPlugin
    service, card, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service)
    data = export(service, saved['asset_id'])
    before = len(channel.positions)
    result = upload(service, data)
    assert result['reused'] and result['asset_id'] == saved['asset_id']
    assert not result['requires_local_trial'] and not call(service, 'draft_list')['drafts']
    forget_local_assets(service)
    imported = upload(service, data)['draft']
    other = upload(service, data)['draft']
    assert other['id'] != imported['id']
    assert len(call(service, 'draft_list')['drafts']) == 2
    restarted = TeachingPlugin(service.adapter, service.directory, service.name)
    reopened = call(restarted, 'draft_open', draft_id=imported['id'])['draft']
    assert reopened['trial'] is None and reopened['imported']['metadata']['name']
    assert reopened['recording']['binding'] == card.trajectory.binding()
    with pytest.raises(ValueError, match='successful_trial_required'):
        call(restarted, 'save', draft_id=imported['id'], effect_confirmed=True,
             **reopened['imported']['metadata'])
    assert len(channel.positions) == before
