"""File/manifest contracts only: no robot SDK, plant or motion simulation."""
import io
import json
import os
from types import SimpleNamespace
import zipfile

import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'unitree/g1'))
from skill_workbench.package import MAX_ARCHIVE, MAX_RECORDING, pack, read_package, unpack
from skill_workbench.service import TeachingPlugin


def package():
    return pack(dict(name='动作', description='描述', conditions='条件', phrase='动作'),
                {'model': 'G1'}, b'recording', 'test/1')


def archive(manifest, recording=b'recording', extra=None):
    out = io.BytesIO()
    with zipfile.ZipFile(out, 'w') as value:
        value.writestr('manifest.json', json.dumps(manifest))
        value.writestr('trajectory.jsonl', recording)
        if extra is not None:
            value.writestr(extra, b'bad')
    return out.getvalue()


def test_additive_manifest_fields_do_not_become_save_arguments():
    manifest, _ = package()
    manifest['notes'] = {'author': 'new exporter'}
    manifest['skill']['future_option'] = True
    manifest['recording']['codec_note'] = 'new optional description'
    manifest['compatibility']['labels'] = ['G1']
    parsed, data = unpack(archive(manifest))
    assert data == b'recording'
    assert set(parsed) == {'schema', 'skill', 'compatibility', 'recording'}
    assert 'future_option' not in parsed['skill'] and 'codec_note' not in parsed['recording']
    assert parsed['compatibility']['labels'] == ['G1']


@pytest.mark.parametrize('section,key', [(None, 'schema'), (None, 'skill'),
    ('skill', 'name'), ('skill', 'conditions'), ('recording', 'sha256'), ('recording', 'bytes')])
def test_additive_compatibility_does_not_make_required_fields_optional(section, key):
    manifest, _ = package()
    del (manifest[section] if section else manifest)[key]
    with pytest.raises(ValueError):
        unpack(archive(manifest))


@pytest.mark.parametrize('extra', ['../outside', '..\\escape.bin', '/outside',
    'x/../../escape.bin', 'C:\\escape.bin', 'trajectory.jsonl', 'code.py'])
def test_archive_names_remain_an_exact_allowlist(extra):
    manifest, _ = package()
    with pytest.raises(ValueError, match='unexpected_package_files'):
        unpack(archive(manifest, extra=extra))


def test_checksum_size_and_archive_limits_are_still_enforced(tmp_path):
    manifest, data = package()
    with pytest.raises(ValueError, match='checksum'):
        unpack(archive(manifest, b'modified!'))
    manifest['recording']['bytes'] = MAX_RECORDING + 1
    with pytest.raises(ValueError, match='invalid_package_recording'):
        unpack(archive(manifest))
    path = tmp_path/'large.motus-skill'
    with path.open('wb') as stream:
        stream.truncate(MAX_ARCHIVE + 1)
    with pytest.raises(ValueError, match='invalid_skill_package_file'):
        read_package(str(path))


def test_uploaded_absolute_regular_file_is_read_without_transfer_state(tmp_path):
    manifest, data = package()
    path = tmp_path/'动作.motus-skill'; path.write_bytes(data)
    assert read_package(str(path)) == (manifest, b'recording')
    with pytest.raises(ValueError, match='绝对路径'):
        read_package(path.name)
    link = tmp_path/'link'; link.symlink_to(path)
    with pytest.raises(OSError):
        read_package(str(link))
    pipe = tmp_path/'pipe'; os.mkfifo(pipe)
    with pytest.raises(ValueError, match='invalid_skill_package_file'):
        read_package(str(pipe))


def test_card_file_argument_and_only_per_skill_execution_are_advertised(tmp_path):
    # Schema generation does not need a controller or a robot model.
    adapter = SimpleNamespace(title='设备', compatibility=lambda: {}, execution_binding=lambda: {})
    service = TeachingPlugin(adapter, tmp_path, 'teaching')
    tool = service.get_tools()[0]; schema = tool['inputSchema']
    assert schema['properties']['package']['format'] == 'file'
    assert schema['properties']['package']['uploadTo'] == 'mcp'
    assert schema['x-action-params']['import']['required'] == ['package']
    assert schema['x-skill-package'] == {'schema': 'motus.teaching.package/1', 'persistent_execution': True}
    assert 'configSchema' not in tool
    for action in ['package_begin', 'package_read', 'package_write', 'package_finish',
                   'session_challenge', 'session_authorize', 'session_revoke', 'config']:
        with pytest.raises(ValueError, match='unknown_teaching_action'):
            service.dispatch(action, {})
    with pytest.raises(ValueError, match='invalid_teaching_argument_auto_execute_verified'):
        service.dispatch('info', {'auto_execute_verified': True})
