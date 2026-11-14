"""Source-time chassis integration, independent of ROS and robot hardware."""
import importlib.util
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).parents[1]


def module():
    path = ROOT / 'loco_odometry.py'
    assert path.exists(), 'source-timestamped chassis odometry is not implemented'
    spec = importlib.util.spec_from_file_location('tianyi_loco_odometry', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


EPOCH = 1_790_000_000_000_000_000


def ready_clock(m):
    clock = m.ClockMapping()
    for i in range(5):
        stamp = 1000 + i * 10
        clock.observe(stamp, EPOCH + stamp * 1_000_000,
                      EPOCH + stamp * 1_000_000 + 2_000_000, stamp * 1_000_000)
    return clock


def delta(stamp, x=0., y=0., yaw=0.):
    return {'timestamp': stamp, 'payload': {
        'positionDifference': {'x': x, 'y': y}, 'angularDifference': yaw}}


def test_clock_requires_multiple_bounded_observations():
    m = module()
    clock = m.ClockMapping()
    clock.observe(1000, EPOCH + 1_000_000_000, EPOCH + 1_002_000_000, 1_000_000_000)
    with pytest.raises(ValueError, match='clock'):
        clock.map(1000, EPOCH + 1_010_000_000, 1_010_000_000)
    clock = ready_clock(m)
    stamp, mapping = clock.map(1050, EPOCH + 1_060_000_000, 1_060_000_000)
    assert stamp == EPOCH + 1_051_000_000
    assert mapping['verified'] is True
    assert mapping['uncertainty_ns'] <= 10_000_000


def test_clock_does_not_use_receive_time_or_change_mapping_within_segment():
    m = module(); clock = ready_clock(m)
    a, mapping = clock.map(1050, EPOCH + 1_060_000_000, 1_060_000_000)
    b, again = clock.map(1050, EPOCH + 1_150_000_000, 1_150_000_000)
    assert a == b and mapping == again
    clock.observe(1150, EPOCH + 1_150_500_000, EPOCH + 1_152_000_000, 1_150_500_000)
    assert clock.map(1160, EPOCH + 1_170_000_000, 1_170_000_000)[1] == mapping


@pytest.mark.parametrize('change', ['wide_rtt', 'wall_jump', 'device_reset', 'expired', 'future', 'old'])
def test_clock_rejects_untrustworthy_time(change):
    m = module(); clock = ready_clock(m)
    with pytest.raises(ValueError):
        if change == 'wide_rtt':
            clock.observe(1100, EPOCH+1_100_000_000, EPOCH+1_150_000_000, 1_100_000_000)
        elif change == 'wall_jump':
            clock.map(1050, EPOCH+2_060_000_000, 1_060_000_000)
        elif change == 'device_reset':
            clock.observe(10, EPOCH+1_100_000_000, EPOCH+1_102_000_000, 1_100_000_000)
        elif change == 'expired':
            clock.map(4000, EPOCH+4_010_000_000, 4_010_000_000)
        elif change == 'future':
            clock.map(1200, EPOCH+1_060_000_000, 1_060_000_000)
        else:
            clock.map(1000, EPOCH+1_500_000_000, 1_500_000_000)


def test_integrates_every_increment_and_uses_source_stamp_and_body_frame():
    m = module(); odom = m.IncrementalOdometry(ready_clock(m))
    samples = odom.feed([delta(1040), delta(1050, yaw=math.pi/2), delta(1060, x=1)],
                        EPOCH+1_070_000_000, 1_070_000_000)
    last = samples[-1]
    assert last['pose']['position'] == pytest.approx([0, 1, 0], abs=1e-12)
    assert last['pose']['quaternion'] == pytest.approx([0, 0, math.sqrt(.5), math.sqrt(.5)])
    assert last['stamp_ms'] == (EPOCH+1_061_000_000)//1_000_000
    assert last['pose']['covariance'] is None
    assert last['vendor']['motus_slam']['status'] == 'tracking'
    measurement = last['vendor']['motus_slam']['measurement']
    assert measurement['source_stamp'] == 1060
    assert measurement['source_stamp']*1_000_000 + measurement['mapping']['offset_ns'] == measurement['stamp_ns']
    assert samples[0]['twist'] == [None]*6


@pytest.mark.parametrize('bad', [delta(1050, x=float('nan')), delta(1050, yaw=True),
                                delta(1040), delta(1030), delta(1200),
                                {'timestamp': 1050, 'payload': {}}, delta(True)])
def test_bad_batch_never_leaves_partial_integration(bad):
    m = module(); odom = m.IncrementalOdometry(ready_clock(m))
    odom.feed([delta(1040)], EPOCH+1_045_000_000, 1_045_000_000)
    with pytest.raises(ValueError):
        odom.feed([delta(1045, x=1), bad], EPOCH+1_210_000_000, 1_210_000_000)
    # No partial pose should be usable after malformed/discontinuous input.
    assert odom.status()['status'] != 'tracking'


def test_new_session_does_not_join_old_path():
    m = module()
    a = m.IncrementalOdometry(ready_clock(m)); b = m.IncrementalOdometry(ready_clock(m))
    old = a.feed([delta(1040), delta(1050, x=1)], EPOCH+1_060_000_000, 1_060_000_000)[-1]
    new = b.feed([delta(1050, x=9)], EPOCH+1_060_000_000, 1_060_000_000)[-1]
    assert new['pose']['position'] == [0, 0, 0]
    assert old['vendor']['motus_slam']['segment_id'] != new['vendor']['motus_slam']['segment_id']
    assert old['pose']['frame_id'] != new['pose']['frame_id']


def test_diagnostic_transport_preserves_sample_time_and_unsubscribes(monkeypatch):
    import json
    import socket
    import threading
    m = module()
    assert hasattr(m, 'DiagnosticClient'), 'read-only diagnostic transport is missing'
    left, right = socket.socketpair()
    commands = []
    def server():
        with right, right.makefile('rb') as f:
            for expected in ('topic/subscribe', 'subscription/read', 'subscription/unsubscribe'):
                while True:
                    line = f.readline()
                    if line.strip(): break
                request = json.loads(line)
                commands.append(request['args']['command'])
                response = ({'subscriptionId': 7} if expected == 'topic/subscribe' else
                            [delta(1050)] if expected == 'subscription/read' else {})
                right.sendall((json.dumps({'request_id': request['request_id'], 'result': {
                    'code': 1, 'result': 'success', 'response': response, 'timestamp': 9999}})+'\n\r\n\r\n').encode())
    thread = threading.Thread(target=server); thread.start()
    monkeypatch.setattr(m.socket, 'create_connection', lambda *a, **kw: left)
    with m.DiagnosticClient('http://robot:1448') as client:
        client.subscribe()
        assert client.read() == [delta(1050)]
    thread.join(1)
    assert not thread.is_alive()
    assert commands == ['topic/subscribe', 'subscription/read', 'subscription/unsubscribe']


def test_worker_integrates_batches_then_publishes_lost_on_disconnect(monkeypatch):
    import time
    m = module()
    assert hasattr(m, 'OdometrySource'), 'diagnostic lifecycle is missing'
    closed = []; published = []; calls = [0]
    class Client:
        def __init__(self, url): pass
        def __enter__(self): return self
        def __exit__(self, *args): closed.append(True)
        def observe(self, clock):
            # Use actual time to exercise worker freshness/lifecycle decisions.
            a=time.time_ns(); mono=time.monotonic_ns()
            device=(a-EPOCH)//1_000_000
            clock.observe(device,a,a+1_000_000,mono)
        def subscribe(self): pass
        def read(self):
            calls[0] += 1
            if calls[0] > 1: raise ConnectionError('disconnected')
            stamp=(time.time_ns()-EPOCH)//1_000_000
            return [delta(stamp-20),delta(stamp-10,x=.1),delta(stamp,x=.2)]
    monkeypatch.setattr(m, 'DiagnosticClient', Client)
    source=m.OdometrySource('http://robot:1448',20,published.append)
    source.start()
    deadline=time.monotonic()+2
    try:
        while time.monotonic()<deadline and not any(s['vendor']['motus_slam']['status']=='lost' for s in published):
            time.sleep(.01)
    finally: source.stop()
    tracking=[s for s in published if s['vendor']['motus_slam']['status']=='tracking']
    assert tracking and tracking[0]['pose']['position'] == pytest.approx([.3,0,0])
    assert published[-1]['pose'] is None
    assert closed
    assert not source.running


@pytest.mark.parametrize('response', [None, [], {'request_id': 1, 'result': []}])
def test_malformed_rpc_is_a_recoverable_protocol_error(response):
    import io, json
    m = module()
    client = m.DiagnosticClient('http://robot:1448')
    class Socket:
        def sendall(self, value): pass
    client._socket = Socket()
    client._reader = io.BytesIO((json.dumps(response)+'\n').encode())
    with pytest.raises(ValueError):
        client._rpc('subscription/read', {})


def test_malformed_subscription_is_rejected(monkeypatch):
    m = module(); client = m.DiagnosticClient('http://robot:1448')
    monkeypatch.setattr(client, '_rpc', lambda *args: None)
    with pytest.raises(ValueError):
        client.subscribe()


def test_low_publish_rate_still_drains_and_checks_source(monkeypatch):
    import time
    m = module(); reads = []; published = []
    class Client:
        def __init__(self, url): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def observe(self, clock):
            a = time.time_ns(); mono = time.monotonic_ns()
            clock.observe((a-EPOCH)//1_000_000, a, a+1_000_000, mono)
        def subscribe(self): pass
        def read(self):
            stamp = (time.time_ns()-EPOCH)//1_000_000
            reads.append(stamp)
            return [delta(stamp)]
    monkeypatch.setattr(m, 'DiagnosticClient', Client)
    source = m.OdometrySource('http://robot:1448', 1, published.append)
    source.start()
    try:
        time.sleep(1.3)
    finally:
        source.stop()
    assert len(reads) >= 15
    assert 1 <= len(published) <= 2
    assert all(s['vendor']['motus_slam']['status']=='tracking' for s in published)


def test_lost_is_not_control_feedback():
    sys.path.insert(0, str(ROOT.parents[1]))
    from common.odom import is_fresh, axis
    m = module(); odom = m.IncrementalOdometry(ready_clock(m))
    sample = odom.feed([delta(1040), delta(1050, x=.001)],
                       EPOCH+1_060_000_000, 1_060_000_000)[-1]
    lost = odom.lost('disconnected')
    assert not is_fresh(lost, sample['stamp_ms'], 250)
    assert axis(lost, 'vx') is None


def test_wide_round_trip_can_be_discarded_without_refreshing_mapping():
    m = module(); clock = ready_clock(m)
    assert hasattr(m, 'ClockObservationRejected')
    with pytest.raises(m.ClockObservationRejected):
        clock.observe(1100, EPOCH+1_100_000_000, EPOCH+1_120_000_000, 1_100_000_000)
    assert clock.map(1100, EPOCH+1_120_000_000, 1_120_000_000)[1]['offset_ns'] == EPOCH+1_000_000
    with pytest.raises(ValueError, match='expired'):
        clock.map(3100, EPOCH+3_110_000_000, 3_110_000_000)


def test_rejected_observation_does_not_count_toward_calibration():
    m = module(); clock = m.ClockMapping()
    for stamp in (1000, 1010, 1020, 1030):
        clock.observe(stamp, EPOCH+stamp*1_000_000,
                      EPOCH+stamp*1_000_000+2_000_000, stamp*1_000_000)
    with pytest.raises(m.ClockObservationRejected):
        clock.observe(1040, EPOCH+1_040_000_000, EPOCH+1_070_000_000, 1_040_000_000)
    assert clock.offset is None
    clock.observe(1050, EPOCH+1_050_000_000, EPOCH+1_052_000_000, 1_050_000_000)
    assert clock.offset == EPOCH+1_000_000


def test_persistently_rejected_clock_probes_eventually_publish_loss(monkeypatch):
    import time
    m = module(); published = []; observations = [0]
    class Client:
        def __init__(self, url): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def subscribe(self): pass
        def observe(self, clock):
            observations[0] += 1
            a=time.time_ns(); mono=time.monotonic_ns()
            clock.observe((a-EPOCH)//1_000_000, a,
                          a+(1_000_000 if observations[0]<=5 else 20_000_000), mono)
        def read(self): return [delta((time.time_ns()-EPOCH)//1_000_000)]
    monkeypatch.setattr(m, 'DiagnosticClient', Client)
    source=m.OdometrySource('http://robot:1448',20,published.append)
    source.start(); deadline=time.monotonic()+3
    try:
        while time.monotonic()<deadline and not any(s['pose'] is None for s in published):
            time.sleep(.01)
    finally: source.stop()
    tracked=[s for s in published if s['pose'] is not None]
    assert len(tracked)>20
    assert len({s['vendor']['motus_slam']['segment_id'] for s in tracked})==1
    assert published[-1]['pose'] is None
    assert published[-1]['vendor']['motus_slam']['reason']=='clock observation expired'
