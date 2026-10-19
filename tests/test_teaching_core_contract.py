"""Actual merged Core policy API over loopback HTTP; no robot or SDK involved."""
import asyncio
import os
from pathlib import Path
import socket
import threading
import time

import pytest

from common.teaching.service import read_execution_policy
from common.teaching.package import pack, read_package
from test_file_upload_endpoint import server

CORE = os.environ.get('TEACHING_CORE_ROOT')
pytestmark = pytest.mark.skipif(not CORE, reason='set TEACHING_CORE_ROOT to merged Core checkout')


@pytest.fixture
def core_policy(tmp_path, monkeypatch):
    import fastapi
    import uvicorn
    monkeypatch.syspath_prepend(str(Path(CORE)/'agent-core/src'))
    monkeypatch.setenv('DB_PATH', str(tmp_path/'core.db'))
    import config
    from api import skill_execution
    tool = {'name': 'g1_teaching', 'inputSchema': {'x-skill-package': {
        'schema': 'motus.teaching.package/1', 'persistent_execution': True}}}
    state = {
        'services': {'mcp': [{'id': 'g1', 'transport': 'http', 'tools': [tool]}]},
        'canvas_layout': {'cards': [{'id': 'teach', 'mcpId': 'g1', 'toolName': 'g1_teaching'}]},
        'skills': {'installed': [{'slug': 'a-name-without-taught-prefix',
            'attachment': {'schema': 'motus.teaching.package/1', 'tool': 'g1_teaching', 'asset_id': 'a'*32},
            'origin': {'mcp_id': 'g1', 'instance_id': 'teach'}}]},
        'skill_execution_policies': {'g1:g1_teaching:teach:'+('a'*32): {
            'enabled': True, 'device_binding': 'device-v1', 'asset_binding': 'recording-v1'}}}
    monkeypatch.setattr(config, 'main', state)
    app = fastapi.FastAPI()
    app.include_router(skill_execution.router, prefix='/api/skills')
    sock = socket.socket(); sock.bind(('127.0.0.1', 0)); sock.listen(16)
    monkeypatch.setenv('AGENT_CORE_URL', f'http://127.0.0.1:{sock.getsockname()[1]}')
    server = uvicorn.Server(uvicorn.Config(app, log_level='error', lifespan='off'))
    thread = threading.Thread(target=server.run, kwargs={'sockets': [sock]}, daemon=True)
    thread.start()
    deadline = time.monotonic()+5
    try:
        while not server.started:
            if time.monotonic() > deadline: raise AssertionError('Core did not start')
            time.sleep(.01)
        yield state
    finally:
        server.should_exit = True; thread.join(5); sock.close()


def read(**changes):
    args = dict(tool='g1_teaching', instance_id='teach', device_binding='device-v1',
                asset_id='a'*32, asset_binding='recording-v1')
    return read_execution_policy(**(args | changes))


def test_merged_core_policy_shape_and_attachment_identity(core_policy):
    assert read() == {'enabled': True, 'source': 'skill', 'binding_changed': False}
    assert read(asset_binding='different')['binding_changed']
    assert not read(device_binding='another-device')['enabled']
    assert not read(instance_id='another-card')['enabled']
    assert not read(asset_id='b'*32)['enabled']
    core_policy['skills']['installed'][0].pop('attachment')
    assert not read()['enabled']  # no inference from a slug or a remembered grant


def test_each_read_observes_current_setting_and_removed_card_policy_is_ignored(core_policy):
    assert read()['enabled']
    core_policy['skill_execution_policies']['g1:g1_teaching:teach:'+('a'*32)]['enabled'] = False
    core_policy['canvas_layout']['toolConfigs'] = {'teach': {'auto_execute_verified': True}}
    assert read() == {'enabled': False, 'source': 'manual', 'binding_changed': False}


def test_policy_client_refuses_non_loopback_core(monkeypatch):
    monkeypatch.setenv('AGENT_CORE_URL', 'https://example.com')
    with pytest.raises(ValueError, match='requires_local_core'):
        read()


@pytest.mark.parametrize('token,accepted', [('upload-token', True), ('invalid-token', False)])
def test_core_transfers_package_to_driver_path_over_http(server, tmp_path, monkeypatch, token, accepted):
    """Real Core transfer + shared Driver intake + package reader; no robot."""
    monkeypatch.syspath_prepend(str(Path(CORE)/'agent-core/src'))
    monkeypatch.setenv('DB_PATH', str(tmp_path/'core.db'))
    import auth
    import mcp_client
    monkeypatch.setattr(auth, '_token', token)
    manifest, data = pack(dict(name='动作', description='描述', conditions='条件', phrase='动作'),
                          {'model': 'G1'}, b'recording', 'test/1')
    local = tmp_path/'core-only'/'skill.motus-skill'
    local.parent.mkdir(); local.write_bytes(data)
    base, uploads = server
    schema = {'properties': {'package': {'type': 'string', 'format': 'file', 'uploadTo': 'mcp'}}}
    args, error = asyncio.run(mcp_client._transfer_file_args('g1', base+'/mcp', schema,
                              {'action': 'import', 'package': str(local)}))
    if not accepted:
        assert error and '401' in error
        assert not list(uploads.rglob('*.motus-skill'))
        return
    assert error is None
    remote = Path(args['package'])
    assert remote != local and remote.is_relative_to(uploads)
    assert read_package(str(remote)) == (manifest, b'recording')
