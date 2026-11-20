"""Verified motor/model mapping, independent freshness and joint-card lifecycle."""
import copy
import json
from pathlib import Path

import pytest

from rokae.helios.feedback import GROUPS, JOINT_NAMES, ScopeFeedbackClient, parse_scope, parse_state
from rokae.helios.device import HeliosJoints
from test_helios_joints import frame, vendor_server, wait_for


def scope_frame():
    return json.loads(Path(__file__).with_name('fixtures').joinpath('helios/joint_scope.json').read_text())


def test_scope_mapping_zero_negative_and_missing_gravity():
    msg = scope_frame()
    msg['params']['left_arm'][0] = -2.5
    rows = parse_scope(msg)
    assert [j['name'] for j in rows] == list(JOINT_NAMES)
    assert rows[0]['motor_torque_nm'] == -2.5
    assert rows[1]['motor_torque_nm'] == 0
    assert rows[0]['gravity_torque_nm'] == msg['params']['left_arm_gravity'][0]
    assert rows[-1]['gravity_torque_nm'] is None
    del msg['params']['waist_gravity']
    assert parse_scope(msg)[14]['gravity_torque_nm'] is None
    assert parse_scope(frame()) is None


@pytest.mark.parametrize('value', [True, None, '2', float('nan'), float('inf')])
def test_bad_torque_rejected(value):
    msg = scope_frame()
    msg['params']['right_arm'][0] = value
    with pytest.raises(ValueError):
        parse_scope(msg)


def test_bad_scope_shape_rejected():
    for key in ('head', 'left_arm', 'waist_gravity'):
        msg = scope_frame()
        msg['params'][key].pop()
        with pytest.raises(ValueError):
            parse_scope(msg)


class SnapshotClient:
    def __init__(self, key, rows):
        self.url = 'ws://127.0.0.1/ws'
        self.key, self.rows = key, rows
        self.online = True
        self.started = self.stopped = 0
    def start(self):
        self.started += 1
        self.online = True
    def stop(self):
        self.stopped += 1
        self.online = False
    def snapshot(self):
        return {'state': 'online' if self.online else 'idle', 'available': self.online,
                'age_ms': 50, 'received_at_ms': 1, self.key: copy.deepcopy(self.rows) if self.online else []}


def test_independent_freshness_and_lifecycle():
    positions = SnapshotClient('joints', parse_state(frame()))
    scope = SnapshotClient('torques', parse_scope(scope_frame()))
    plugin = HeliosJoints({'state_url': positions.url}, client=positions, scope_client=scope)
    skeleton = plugin.payload()
    assert skeleton['joints'][0]['motor_torque_nm'] == 0
    # The single skeleton topic carries angle, velocity and both torques per joint.
    assert all({'q', 'motor_torque_nm', 'gravity_torque_nm'} <= set(j) for j in skeleton['joints'])
    assert skeleton['angle_unit'] == 'rad' and skeleton['torque_status']['unit'] == 'Nm'
    assert skeleton['message'] == '实时关节数据正常'
    assert skeleton['torque_status']['feedback_source'] == 'motor_torque'
    assert not skeleton['torque_status']['physical_validity_verified']
    scope.online = False
    skeleton = plugin.payload()
    assert skeleton['available'] and len(skeleton['joints']) == 20
    assert all('motor_torque_nm' not in j for j in skeleton['joints'])
    scope.online = True
    positions.online = False
    assert plugin.payload()['joints'] == []
    plugin.dispatch('start', {'_tool_name':'joints'})
    assert positions.started == scope.started == 1
    plugin.dispatch('stop', {'_tool_name':'joints'})
    assert positions.stopped == scope.stopped == 1
    assert plugin.payload()['joints'] == []


def test_scope_ignores_other_messages_and_only_subscribes_readonly():
    # This endpoint keeps publishing joint/state. It must never refresh scope age.
    with vendor_server() as server:
        client = ScopeFeedbackClient(server['url'], stale_timeout_s=.15, reconnect_delay_s=.03)
        try:
            client.start()
            wait_for(lambda: server['connections'] >= 2)
            snap = client.snapshot()
            assert not snap['available'] and snap['torques'] == [] and snap['frames_received'] == 0
            client.stop()
            assert client.snapshot()['state'] == 'idle'
            assert all(r == {'jsonrpc':'2.0','method':'subscribe','params':['joint/scope']} for r in server['requests'])
        finally:
            client.stop()
