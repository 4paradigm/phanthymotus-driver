"""Portable recordings lose source authority and require a local trial."""
import base64
import hashlib
import io
import json
import zipfile

import pytest

from common.teaching.package import CHUNK, MAX_RECORDING, Transfers, pack, unpack
from test_g1_teaching import g1_teaching, call, confirm, save, wait_for


def export(service, asset_id):
    start = call(service, "package_export", asset_id=asset_id)
    data = bytearray()
    while len(data) < start["bytes"]:
        data.extend(base64.b64decode(call(service, "package_read", transfer_id=start["transfer_id"], offset=len(data))["data"]))
    assert hashlib.sha256(data).hexdigest() == start["sha256"]
    return bytes(data)


def upload(service, data):
    start = call(service, "package_begin", bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    for offset in range(0, len(data), CHUNK):
        call(service, "package_write", transfer_id=start["transfer_id"], offset=offset,
             data=base64.b64encode(data[offset:offset+CHUNK]).decode())
    return call(service, "package_finish", transfer_id=start["transfer_id"])


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
    assert original["recording"]["binding"]["device_id"].encode() not in recording
    lines = [json.loads(line) for line in recording.splitlines()]
    assert all(f["session_id"] == "portable" for f in lines[1:-1])
    card.cfg["trajectory_device_id"] = "another-compatible-g1"
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
    from common.teaching import TeachingPlugin
    service, card, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service)
    data = export(service, saved['asset_id'])
    before = len(channel.positions)
    result = upload(service, data)
    assert result['reused'] and result['asset_id'] == saved['asset_id']
    assert not result['requires_local_trial'] and not call(service, 'draft_list')['drafts']
    card.cfg['trajectory_device_id'] = 'fresh-target-device'
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


def archive(manifest, recording, extra=None):
    out = io.BytesIO()
    with zipfile.ZipFile(out, "w") as value:
        value.writestr("manifest.json", json.dumps(manifest))
        value.writestr("trajectory.jsonl", recording)
        if extra:
            value.writestr(extra, b"bad")
    return out.getvalue()


def test_package_rejects_checksum_paths_duplicate_files_and_limits():
    manifest, data = pack(dict(name="动作", description="描述", conditions="条件", phrase="动作"), {}, b"recording", "test/1")
    assert unpack(data)[1] == b"recording"
    for name in ("../outside", "trajectory.jsonl", "code.py"):
        with pytest.raises(ValueError, match="unexpected_package_files"):
            unpack(archive(manifest, b"recording", name))
    with pytest.raises(ValueError, match="checksum"):
        unpack(archive(manifest, b"modified!"))
    manifest["recording"]["bytes"] = MAX_RECORDING + 1
    with pytest.raises(ValueError, match="invalid_package_recording"):
        unpack(archive(manifest, b"recording"))


def test_transfer_rejects_reordered_chunks_and_incomplete_content():
    transfers = Transfers()
    start = transfers.begin(4, hashlib.sha256(b"test").hexdigest(), "upload")
    ident = start["transfer_id"]
    with pytest.raises(ValueError, match="offset"):
        transfers.write(ident, 1, "dGU=")
    transfers.write(ident, 0, "dGU=")
    with pytest.raises(ValueError, match="incomplete"):
        transfers.finish(ident)
    with pytest.raises(ValueError, match="offset"):
        transfers.write(ident, 0, "c3Q=")
    transfers.items[ident]["expires"] = 0
    with pytest.raises(ValueError, match="expired"):
        transfers.write(ident, 2, "c3Q=")
