"""Offline startup diagnostics must explain missing real source evidence."""
import json
import sys
from types import SimpleNamespace as NS

import pytest

from test_tianyi_body_mount import source, emit, m
from test_tianyi_rgbd_camera import info


def waiting(source):
    source.registered = False
    source.geometry = None
    source.count = 0
    source.infos.clear()
    return source


def test_source_and_cards_explain_parameter_discovery_without_claiming_running(source):
    waiting(source)
    source._parameters()
    status = source.dispatch('info', {})
    assert status['state'] == 'unavailable'
    assert status['source_status']['stage'] == 'parameter_service'
    assert 'get_parameters' in status['reason']
    for kind in ('color', 'depth', 'imu'):
        card = m.RgbdInputCard(source, kind)
        card.running = True
        assert card.get_tool()['source_status'] == status['source_status']
        assert card.dispatch('info', {})['source_status'] == status['source_status']


def test_pending_parameter_response_is_distinct_from_service_discovery(source, monkeypatch):
    waiting(source)
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=NS(Request=lambda: NS())))
    source.client = NS(service_is_ready=lambda: True,
                       call_async=lambda request: NS(done=lambda: False))
    source._parameters()
    status = source.dispatch('info', {})['source_status']
    assert status['stage'] == 'parameter_response'
    assert status['parameters']['stage'] == 'response'
    assert 'response' in status['reason']


def test_parameter_request_failure_is_bounded_diagnostic_and_can_retry(source, monkeypatch):
    waiting(source)
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=NS(Request=lambda: NS())))
    def fail(request):
        raise RuntimeError('request transport failure ' + 'x' * 1000)
    source.client = NS(service_is_ready=lambda: True, call_async=fail)
    source._parameters()
    status = source.dispatch('info', {})['source_status']
    assert status['stage'] == 'parameter_request'
    assert 'request transport failure' in status['reason']
    assert len(status['reason']) <= 256
    assert source.future is None and source.running


def test_rejected_parameter_values_identify_registration_and_global_clock_gates(source):
    waiting(source)
    source.future = NS(done=lambda: True, result=lambda: NS(values=[
        NS(bool_value=False), NS(string_value='device'), NS(bool_value=False)]))
    source._parameters()
    status = source.dispatch('info', {})['source_status']
    assert status['stage'] == 'parameter_validation'
    assert status['parameters'] == dict(stage='validation', depth_registration=False,
                                         time_domain='device', enable_sync_host_time=False)
    assert not status['first_calibrated_output']


def test_parameter_service_rediscovery_does_not_repeat_old_validation_error(source):
    waiting(source)
    source.future = NS(done=lambda: True, result=lambda: NS(values=[
        NS(bool_value=False), NS(string_value='device'), NS(bool_value=False)]))
    source._parameters()
    assert source.error
    source._parameters()  # The fixture service is unavailable on the next query.
    status = source.dispatch('info', {})['source_status']
    assert status['stage'] == 'parameter_service'
    assert 'get_parameters' in status['reason']


def test_stream_diagnostics_distinguish_received_paired_and_forwarded_captures(source):
    source.registered = False
    stamp = 12_345_678_901
    image = NS(header=NS(stamp=NS(sec=12, nanosec=345_678_901)))
    source._image('color', image)
    streams = source.dispatch('info', {})['source_status']['streams']
    assert streams['color']['received'] == 1
    assert streams['color']['last_capture_ns'] == stamp
    assert streams['depth']['received'] == 0
    assert streams['paired_capture_ns'] == 0
    assert streams['forwarded'] == 0 and streams['last_publish_ns'] is None
    source.registered = True
    emit(source)
    streams = source.dispatch('info', {})['source_status']['streams']
    assert streams['forwarded'] == 1 and streams['last_publish_ns'] > 0


@pytest.mark.parametrize('received,missing', [('color', 'depth'), ('depth', 'color')])
def test_missing_camera_info_is_named_and_geometry_wait_is_distinct(source, received, missing):
    waiting(source)
    source.registered = True
    source._info(received, info(frame='head_optical'))
    status = source.dispatch('info', {})['source_status']
    assert status['stage'] == 'camera_info'
    assert status['camera_info'] == {received: True, missing: False}
    assert missing in status['reason']
    source._info(missing, info(frame='head_optical'))
    status = source.dispatch('info', {})['source_status']
    assert status['stage'] == 'rectified_geometry'
    assert not status['rectified_geometry_ready']


def test_geometry_does_not_claim_ready_before_first_calibrated_output(source):
    status = source.dispatch('info', {})
    assert status['state'] == 'unavailable'
    assert status['source_status']['stage'] == 'first_calibrated_output'
    assert status['source_status']['rectified_geometry_ready']
    emit(source)
    status = source.dispatch('info', {})
    assert status['state'] == 'running'
    assert status['source_status']['state'] == 'available'
    assert status['source_status']['stage'] == 'ready'
    assert status['source_status']['first_calibrated_output']
    assert len(json.dumps(status['source_status'])) < 2000


def test_full_restart_discards_previous_camera_info_and_stopped_callbacks(source):
    source._info('color', info(frame='head_optical'))
    source._info('depth', info(frame='head_optical'))
    source.stop()
    source._info('color', info(frame='old_optical'))
    assert source.infos['color'].header.frame_id == 'head_optical'
    source._work = lambda: None
    source.start()
    source.worker.join()
    assert source.infos == {}
    source.registered = True
    image = NS(width=640, height=480, header=NS(frame_id='head_optical', stamp=NS(sec=0, nanosec=0)))
    source._image('color', image)
    assert source.geometry is None
    status = source.dispatch('info', {})['source_status']
    assert status['camera_info'] == {'color': False, 'depth': False}
    assert 'color' in status['reason'] and 'depth' in status['reason']


@pytest.mark.parametrize('kind', ['source', 'color', 'depth'])
def test_start_deadline_fails_with_latest_stage_and_can_later_become_ready(source, monkeypatch, kind):
    waiting(source)
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=0))
    source.registered = True
    source._info('color', info(frame='head_optical'))
    card = source if kind == 'source' else m.RgbdInputCard(source, kind)
    now = [0.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(m.time, 'sleep', lambda delay: now.__setitem__(0, now[0] + delay))
    result = card.dispatch('start', {})
    assert 10. <= now[0] < 10.1
    assert result['state'] == 'error'
    assert 'depth' in result['error'] and 'CameraInfo' in result['error']
    assert result['source_status']['stage'] == 'camera_info'
    assert 'depth' in result['reason']
    source._info('depth', info(frame='head_optical'))
    source.geometry = m.make_geometry(source.infos['color'], source.infos['depth'], registered=True)
    emit(source)
    assert card.dispatch('info', {})['state'] == 'running'


@pytest.mark.parametrize('kind', ['color', 'depth'])
def test_card_start_succeeds_only_after_real_calibrated_output(source, monkeypatch, kind):
    waiting(source)
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=0))
    source.registered = True
    card = m.RgbdInputCard(source, kind)
    now = [0.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: now[0])
    def capture(delay):
        now[0] += delay
        source._info('color', info(frame='head_optical'))
        source._info('depth', info(frame='head_optical'))
        source.geometry = m.make_geometry(source.infos['color'], source.infos['depth'], registered=True)
        emit(source)
    monkeypatch.setattr(m.time, 'sleep', capture)
    result = card.dispatch('start', {})
    assert result['state'] == 'running'
    assert result['camera_info'] and 'error' not in result
    assert result['source_status']['first_calibrated_output']
