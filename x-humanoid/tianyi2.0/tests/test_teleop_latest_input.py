"""Real local socket backlog and arm-only feedback admission; no robot I/O."""
import json
import socket
import pytest
from teleop_executor import TeleopExecutor
from test_motion_stream import rig


@pytest.mark.parametrize('latest_bad', [None, 'expired', 'signature', 'joint_limit'])
def test_socket_discards_intermediate_targets_but_validates_latest(latest_bad):
    gate, _, writes, advance, packet = rig()
    stale = packet(seq=0)
    advance(110)
    latest = packet(seq=1, q=[.2]*14)
    if latest_bad == 'expired':
        latest = packet(seq=1, generated_ns=1)
    elif latest_bad == 'signature':
        latest['mac'] = '0'*64
    elif latest_bad == 'joint_limit':
        latest = packet(seq=1, q=[2.]*14)
    item = TeleopExecutor({}, 'test', None, None, None, [])
    item.gate = gate
    receive, send = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        receive.setblocking(False)
        item._bus_socket = receive
        for value in (stale, latest):
            send.send(json.dumps(value).encode())
        item._receive_latest_command()
        assert not writes  # receiving alone never writes to an actuator
        if latest_bad:
            assert gate.state == 'hold' and gate.latest is None
        else:
            assert gate.seq == 1 and gate.state == 'ready'
            advance(20)
            gate.tick()
            assert gate.state == 'active' and gate.applied_seq == 1
            assert len(writes) == 1
    finally:
        receive.close()
        send.close()


def test_unbounded_backlog_cannot_starve_watchdog_or_apply_intermediate_packet():
    gate, _, writes, _, packet = rig()
    item = TeleopExecutor({}, 'test', None, None, None, [])
    item.gate = gate
    class Flood:
        count = 0
        def recv(self, size):
            self.count += 1
            return json.dumps(packet(seq=self.count)).encode()
    item._bus_socket = Flood()
    item._receive_latest_command()
    assert item._bus_socket.count == 128 and not writes
    assert gate.latest is None and gate.reason == 'local_dds_command_backlog'


@pytest.mark.parametrize('hands_enabled', [False, True])
def test_only_enabled_hand_requires_freshness(hands_enabled):
    gate, state, writes, advance, packet = rig()
    gate.hands_enabled = hands_enabled
    state['hand_ns'] = 0
    gate.accept(packet())
    gate.tick()
    assert (gate.state == 'fault') == hands_enabled
    if hands_enabled:
        assert gate.reason == 'hand_ns_stale' and not writes
    else:
        assert gate.state == 'active'
        # Arm-only must still fail closed on missing power or arm feedback.
        state['power_ns'] = 0
        gate.tick()
        assert gate.state == 'fault' and gate.reason == 'power_ns_stale'
