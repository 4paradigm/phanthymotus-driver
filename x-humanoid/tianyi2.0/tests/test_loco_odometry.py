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
    sys.path.insert(0, str(ROOT))
    try:
        spec.loader.exec_module(result)
    finally:
        sys.path.pop(0)
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
    assert mapping['uncertainty_ns'] == 50_000_000


def test_clock_does_not_use_receive_time_and_preserves_already_mapped_captures():
    m = module(); clock = ready_clock(m)
    a, mapping = clock.map(1050, EPOCH + 1_060_000_000, 1_060_000_000)
    b, again = clock.map(1050, EPOCH + 1_150_000_000, 1_150_000_000)
    assert a == b and mapping == again
    clock.observe(1150, EPOCH + 1_150_500_000, EPOCH + 1_152_000_000, 1_150_500_000)
    assert clock.map(1050, EPOCH + 1_170_000_000, 1_170_000_000) == (a, mapping)
    assert clock.map(1160, EPOCH + 1_170_000_000, 1_170_000_000)[1]['offset_ns'] > mapping['offset_ns']


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
    samples = odom.feed([delta(1040), delta(1050, yaw=math.pi/2), delta(1060, x=1), delta(1070)],
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
    old = a.feed([delta(1040), delta(1050, x=1), delta(1060)], EPOCH+1_060_000_000, 1_060_000_000)[-1]
    new = b.feed([delta(1050, x=9), delta(1060)], EPOCH+1_060_000_000, 1_060_000_000)[-1]
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
            return [delta(stamp-30),delta(stamp-20,x=.1),delta(stamp-10,x=.2),delta(stamp)]
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


def test_publish_deadline_does_not_skip_frames_due_to_read_latency_jitter(monkeypatch):
    import types
    m = module(); now = [1000.]; reads = []; published = []
    monkeypatch.setattr(m, 'time', types.SimpleNamespace(
        monotonic=lambda: now[0], monotonic_ns=lambda: int(now[0]*1e9),
        time_ns=lambda: EPOCH+int(now[0]*1e9)))
    class Stop:
        def is_set(self): return now[0] >= 1002.
        def wait(self, seconds):
            now[0] += seconds
            return self.is_set()
    class Client:
        def __init__(self, url): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def observe(self, clock):
            ns=int(now[0]*1e9)
            clock.observe(ns//1_000_000, EPOCH+ns, EPOCH+ns+1_000_000, ns)
        def subscribe(self): pass
        def read(self):
            now[0] += .003 if len(reads)%2==0 else .001
            reads.append(now[0])
            return [delta(int(now[0]*1000))]
    monkeypatch.setattr(m, 'DiagnosticClient', Client)
    source=m.OdometrySource('http://robot:1448',20,published.append)
    source._stop=Stop()
    source._run()
    assert len(reads)>=35
    assert len(published)>=len(reads)-2
    assert all(s['pose'] is not None for s in published)


def test_uncertain_clock_interval_is_not_evidence_of_clock_drift():
    m=module(); clock=ready_clock(m)
    # This interval overlaps the 25 ms verification budget but is not
    # contained by it; it cannot prove either validity or excessive drift.
    with pytest.raises(m.ClockObservationRejected):
        clock.observe(1100, EPOCH+1_120_000_000, EPOCH+1_126_000_000, 1_120_000_000)
    assert clock._observed==1_040_000_000
    clock.observe(1150, EPOCH+1_150_000_000, EPOCH+1_152_000_000, 1_150_000_000)
    assert clock.offset==EPOCH+1_000_000


def test_clock_interval_proving_drift_is_fatal():
    m=module(); clock=ready_clock(m)
    with pytest.raises(ValueError, match='drift') as error:
        clock.observe(1140, EPOCH+1_100_000_000, EPOCH+1_102_000_000, 1_100_000_000)
    assert not isinstance(error.value,m.ClockObservationRejected)


def test_rejected_clock_probe_retries_on_next_drain_without_extending_expiry(monkeypatch):
    import types
    m=module(); now=[1000.]; observed=[]; published=[]
    monkeypatch.setattr(m,'time',types.SimpleNamespace(
        monotonic=lambda:now[0],monotonic_ns=lambda:int(now[0]*1e9),
        time_ns=lambda:EPOCH+int(now[0]*1e9)))
    class Stop:
        def is_set(self): return now[0]>=1002.
        def wait(self,seconds): now[0]+=seconds; return self.is_set()
    class Client:
        def __init__(self,url): pass
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def subscribe(self): pass
        def observe(self,clock):
            observed.append(now[0]);ns=int(now[0]*1e9)
            clock.observe(ns//1_000_000,EPOCH+ns,
                          EPOCH+ns+(20_000_000 if len(observed)==6 else 1_000_000),ns)
        def read(self): return [delta(int(now[0]*1000))]
    monkeypatch.setattr(m,'DiagnosticClient',Client)
    source=m.OdometrySource('http://robot:1448',20,published.append);source._stop=Stop()
    source._run()
    assert 0 < observed[6]-observed[5] <= .1
    assert all(s['pose'] is not None for s in published)


def test_equal_stamps_across_reads_preserve_all_increments_without_duplicate_output():
    m=module(); odom=m.IncrementalOdometry(ready_clock(m))
    first=odom.feed([delta(1040),delta(1050,x=1,yaw=math.pi/2)],
                    EPOCH+1_060_000_000,1_060_000_000)
    assert [s['vendor']['source_stamp_ms'] for s in first]==[1040]
    second=odom.feed([delta(1050,x=2),delta(1060)],EPOCH+1_070_000_000,1_070_000_000)
    assert len(second)==1
    assert second[0]['pose']['position']==pytest.approx([1,2,0])
    assert second[0]['vendor']['source_stamp_ms']==1050
    assert second[0]['vendor']['motus_slam']['source_events_in_group']==2
    assert second[0]['twist'][:2]==pytest.approx([100,200])
    assert second[0]['vendor']['motus_slam']['measurement']['mapping']['uncertainty_ns']==50_000_000
    assert odom.feed([],EPOCH+1_080_000_000,1_080_000_000)==[]


def test_fifty_ms_budget_accepts_observed_offset_change_but_rejects_excess():
    m=module(); clock=ready_clock(m)
    clock.observe(1100,EPOCH+1_110_000_000,EPOCH+1_113_000_000,1_110_000_000)
    assert clock.map(1100,EPOCH+1_114_000_000,1_114_000_000)[1]['uncertainty_ns']==50_000_000
    with pytest.raises(ValueError,match='drift'):
        clock.observe(1150,EPOCH+1_210_000_000,EPOCH+1_213_000_000,1_210_000_000)


def test_pending_group_is_revalidated_before_emitting_after_a_delayed_read():
    m=module(); odom=m.IncrementalOdometry(ready_clock(m))
    odom.feed([delta(1040),delta(1050,x=1)],EPOCH+1_060_000_000,1_060_000_000)
    with pytest.raises(ValueError,match='stale'):
        odom.feed([delta(1150)],EPOCH+1_310_000_000,1_310_000_000)
    assert odom.status()['status']=='lost'
    assert odom._pose==(0.,0.,0.)


def test_identical_nonzero_events_are_composed_and_closed_groups_cannot_be_replayed():
    m=module(); odom=m.IncrementalOdometry(ready_clock(m))
    samples=odom.feed([delta(1040),delta(1050,x=.1),delta(1050,x=.1),delta(1060)],
                      EPOCH+1_070_000_000,1_070_000_000)
    assert samples[-1]['pose']['position']==pytest.approx([.2,0,0])
    assert samples[-1]['vendor']['motus_slam']['source_events_in_group']==2
    with pytest.raises(ValueError,match='reorder'):
        odom.feed([delta(1050,x=.1)],EPOCH+1_080_000_000,1_080_000_000)
    assert odom.status()['status']=='lost'


def test_normalized_pose_declares_actual_host_clock_reference(monkeypatch):
    m = module()
    reference = dict(clock_id='host_realtime', domain_id='actual-boot', timestamp_basis='host_realtime')
    monkeypatch.setattr(m, 'host_clock_reference', lambda: reference)
    odom = m.IncrementalOdometry(ready_clock(m))
    poses = odom.feed([delta(1050), delta(1060)], EPOCH+1_070_000_000, 1_070_000_000)
    q = poses[0]['vendor']['motus_slam']
    assert q['clock_reference'] == reference
    assert q['pose_reference'] == 'body'
    assert q['pose_source'] == 'chassis_odometry'
    assert q['measurement']['mapping']['uncertainty_ns'] == 50_000_000


def test_clock_probe_reports_transport_phases_without_refreshing_rejected_clock(monkeypatch):
    import types
    m = module(); now = [1_100_000_000]
    monkeypatch.setattr(m, 'time', types.SimpleNamespace(
        time_ns=lambda: EPOCH+now[0], monotonic_ns=lambda: now[0]))
    class Response:
        status = 200
        def read(self, limit):
            now[0] += 20_000_000
            return b'1100'
    class Http:
        def request(self, *args): now[0] += 1_000_000
        def getresponse(self):
            now[0] += 2_000_000
            return Response()
    probe_module = sys.modules['clock_probe']
    monkeypatch.setattr(probe_module, 'time', m.time)
    client = m.DiagnosticClient('http://robot:1448')
    client._clock_probe = types.SimpleNamespace(read=lambda: probe_module.measure(Http(), ''))
    clock = ready_clock(m); before = clock._observed
    source = m.OdometrySource('http://robot:1448', 20, lambda s: None)
    with pytest.raises(m.ClockObservationRejected, match='round trip'):
        source._observe(client, clock)
    info = source.info()['timing']
    assert info['clock_attempts'] == 1 and info['clock_accepted'] == 0
    assert info['clock_rejections'] == {'clock observation has excessive round trip': 1}
    assert info['last_clock_probe']['request_ms'] == 1
    assert info['last_clock_probe']['headers_ms'] == 2
    assert info['last_clock_probe']['body_ms'] == 20
    assert info['last_clock_probe']['rtt_ms'] == 23
    assert clock._observed == before
    info['clock_rejections'].clear()
    assert source.info()['timing']['clock_rejections']


def test_blocked_publish_is_visible_without_rewriting_measurement_time(monkeypatch):
    import types
    m = module(); now = [0]
    monkeypatch.setattr(m, 'time', types.SimpleNamespace(monotonic_ns=lambda: now[0]))
    def publish(sample): now[0] += 300_000_000
    source = m.OdometrySource('http://robot:1448', 20, publish)
    sample = {'stamp_ms': 1234, 'vendor': {'motus_slam': {'status': 'tracking'}}}
    source._emit(sample)
    assert source.info()['timing']['publish_ms'] == 300
    assert sample['stamp_ms'] == 1234


def test_recovery_keeps_last_loss_timing_evidence():
    m = module(); source = m.OdometrySource('http://robot:1448',20,lambda s: None)
    source._timing['read_ms'] = 260
    source._emit({'vendor': {'motus_slam': dict(status='lost', reason='clock observation expired', segment_id='old')}})
    source._timing['read_ms'] = 2
    source._emit({'vendor': {'motus_slam': dict(status='tracking', reason='', segment_id='new')}})
    info = source.info()
    assert info['status'] == 'tracking'
    assert info['last_loss']['segment_id'] == 'old'
    assert info['last_loss']['timing']['read_ms'] == 260
    assert info['timing']['read_ms'] == 2


def test_fast_loss_notification_preserves_preceding_blocked_tracking_publish(monkeypatch):
    import types
    m = module(); now = [0]
    monkeypatch.setattr(m, 'time', types.SimpleNamespace(monotonic_ns=lambda: now[0]))
    def publish(s):
        if s['vendor']['motus_slam']['status'] == 'tracking': now[0] += 2_500_000_000
    source = m.OdometrySource('http://robot:1448',20,publish)
    source._emit({'vendor': {'motus_slam': {'status': 'tracking'}}})
    source._emit({'vendor': {'motus_slam': {'status': 'lost', 'reason': 'clock observation expired'}}})
    assert source.info()['last_loss']['timing']['publish_ms'] == 2500


def test_late_ipc_reply_does_not_refresh_clock_observation(monkeypatch):
    import types
    m=module(); clock=ready_clock(m)
    client=m.DiagnosticClient('http://robot:1448')
    client._clock_probe=types.SimpleNamespace(read=lambda:dict(
        device_ms=1100, before_ns=EPOCH+1_100_000_000,
        after_ns=EPOCH+1_102_000_000, monotonic_ns=1_100_000_000))
    client.observe(clock)
    assert clock._observed == 1_100_000_000
    with pytest.raises(ValueError,match='expired'):
        clock.map(3200,EPOCH+3_200_000_000,3_200_000_000)


def test_failed_ipc_does_not_reuse_previous_http_timing():
    import types
    m=module();client=m.DiagnosticClient('http://robot:1448')
    client.last_clock_probe={'rtt_ms':1}
    def broken():raise ValueError('clock probe IPC timeout')
    client._clock_probe=types.SimpleNamespace(read=broken)
    with pytest.raises(ValueError,match='timeout'):client.observe(ready_clock(m))
    assert client.last_clock_probe == {}

@pytest.mark.parametrize('direction', [-1, 1])
def test_continuous_clock_tracks_one_hour_of_drift(direction):
    m = module(); clock = ready_clock(m)
    # One hour at 1500 ppm: far beyond a frozen offset's 25 ms verification window.
    for i in range(1, 7201):
        host_ns = 1_040_000_000 + i*500_000_000
        drift = direction*i*750_000
        device_ms = (host_ns-drift)//1_000_000
        clock.observe(device_ms, EPOCH+host_ns, EPOCH+host_ns+2_000_000, host_ns)
        stamp, evidence = clock.map(device_ms, EPOCH+host_ns+2_000_000, host_ns+2_000_000)
        assert abs(stamp-(EPOCH+host_ns)) <= 25_000_000
        assert evidence['uncertainty_ns'] == 50_000_000
    assert clock._observed == host_ns


def test_clock_slew_preserves_past_and_monotonic_millisecond_captures():
    m = module(); clock = ready_clock(m)
    past, metadata = clock.map(1100, EPOCH+1_110_000_000, 1_110_000_000)
    # Device clock moves ahead; correction must not step timestamps backward.
    clock.observe(1140, EPOCH+1_120_000_000, EPOCH+1_122_000_000, 1_120_000_000)
    assert clock.map(1100,EPOCH+1_125_000_000,1_125_000_000) == (past, metadata)
    stamps = [clock.map(s,EPOCH+(s-20)*1_000_000,(s-20)*1_000_000)[0]
              for s in range(1140, 1300)]
    assert all(b>a for a,b in zip(stamps,stamps[1:]))
    offset = stamps[-1]-1299_000_000
    assert offset < metadata['offset_ns']  # actually follows the new clock


def test_drifting_clock_keeps_integrated_pose_and_equal_stamp_groups_continuous():
    m = module(); clock = ready_clock(m); odom = m.IncrementalOdometry(clock)
    segment = odom.segment
    odom.feed([delta(1040)], EPOCH+1_042_000_000, 1_042_000_000)
    last_stamp = 0
    for i in range(1, 1201):
        host = 1_040_000_000+i*50_000_000
        device_ms = (host-i*75_000)//1_000_000
        if i % 10 == 0:
            clock.observe(device_ms,EPOCH+host,EPOCH+host+2_000_000,host)
        samples=odom.feed([delta(device_ms,x=.004),delta(device_ms,x=.006)],
                          EPOCH+host+2_000_000,host+2_000_000)
        assert len(samples)==1 and samples[0]['vendor']['motus_slam']['segment_id']==segment
        stamp=samples[0]['vendor']['motus_slam']['measurement']['stamp_ns']
        assert stamp>last_stamp
        last_stamp=stamp
    assert odom._pose==pytest.approx((11.99,0,0))
    assert len(clock._curves)<=16


def test_probe_behind_latest_mapped_capture_does_not_rewrite_it():
    m=module(); clock=ready_clock(m)
    first=clock.map(1200,EPOCH+1_180_000_000,1_180_000_000)
    clock.observe(1180,EPOCH+1_190_000_000,EPOCH+1_192_000_000,1_190_000_000)
    assert clock.map(1200,EPOCH+1_195_000_000,1_195_000_000)==first
    assert clock.map(1201,EPOCH+1_196_000_000,1_196_000_000)[0]>first[0]


def test_unfinished_slew_can_reverse_without_rewriting_or_extending_expiry():
    m=module(); clock=ready_clock(m)
    clock.observe(1140,EPOCH+1_120_000_000,EPOCH+1_122_000_000,1_120_000_000)
    old=clock.map(1500,EPOCH+1_480_000_000,1_480_000_000)
    pivot=clock.map(1640,EPOCH+1_655_000_000,1_655_000_000)
    clock.observe(1640,EPOCH+1_655_000_000,EPOCH+1_657_000_000,1_655_000_000)
    assert clock.map(1640,EPOCH+1_657_000_000,1_657_000_000)==pivot
    assert clock.map(1500,EPOCH+1_660_000_000,1_660_000_000)==old
    curves, observed=list(clock._curves),clock._observed
    with pytest.raises(m.ClockObservationRejected):
        clock.observe(1700,EPOCH+1_715_000_000,EPOCH+1_724_000_000,1_715_000_000)
    assert list(clock._curves)==curves and clock._observed==observed
    stamps=[clock.map(s,EPOCH+(s+15)*1_000_000,(s+15)*1_000_000)[0]
            for s in range(1640,1801)]
    assert all(b>a for a,b in zip(stamps,stamps[1:]))
    assert stamps[-1]-1800_000_000>pivot[1]['offset_ns']
    with pytest.raises(ValueError,match='expired'):
        clock.map(4000,EPOCH+4_015_000_000,4_015_000_000)
