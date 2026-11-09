"""Production camera loop wiring: snapshot/VO demands do not share a gate."""
import importlib.util
from pathlib import Path
import socket
import sys
from types import SimpleNamespace as NS

import pytest

spec = importlib.util.spec_from_file_location('snapshot_hook_fixture', Path(__file__).with_name('test_bundle_hook.py'))
fixture_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture_module)
capture = fixture_module.capture


@pytest.mark.parametrize('snapshot_fault', [False, True])
def test_one_pipeline_and_independent_optional_branches(capture, monkeypatch, snapshot_fault):
    device, state = capture
    state.frame_limit = 3
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    rs = sys.modules['pyrealsense2']
    original_pipeline = rs.pipeline
    pipelines = []
    def pipeline():
        p = original_pipeline()
        original = p.get_active_profile
        def profile():
            result = original()
            result.get_device = lambda: NS(get_info=lambda _: 'actual-serial',
                first_depth_sensor=lambda: NS(get_depth_scale=lambda: .001))
            return result
        p.get_active_profile = profile
        pipelines.append(p)
        return p
    rs.pipeline = pipeline
    rs.camera_info = NS(serial_number='serial_number')
    seen, raw_calls, instances = [], [], []
    class Raw:
        enabled = True
        def __init__(self, *args, **kwargs): instances.append(self)
        def publish(self, *_args): raw_calls.append(True)
    monkeypatch.setitem(sys.modules, 'visual_inputs', NS(RawCameraPublisher=Raw))
    class Snapshot:
        pending = True
        def __init__(self, sock, source, scale):
            self.sock = sock
            assert source == 'actual-serial' and scale == .001
        def poll(self): pass
        def observe(self, source, aligned, *, wait_window_ns):
            assert source is aligned  # One frameset passed through existing align hook.
            start_ns, end_ns = wait_window_ns
            assert isinstance(start_ns, int) and isinstance(end_ns, int)
            assert 0 < start_ns <= end_ns
            seen.append((instances[0].enabled, self.pending))
            if snapshot_fault:
                raise RuntimeError('snapshot failure')
            if len(seen) == 2:
                self.pending = False
        def close(self): self.sock.close()
    monkeypatch.setitem(sys.modules, 'rgbd_snapshot', NS(SnapshotProducer=Snapshot))
    def on_spin(node):
        node.subscriptions['/robot/odometry_input/raw_enabled'](NS(data=state.spin_calls != 2))
    state.on_spin = on_spin
    parent, child = socket.socketpair()
    try:
        device._camera_subprocess('robot', child.detach())
    finally:
        parent.close()
    assert len(pipelines) == 1 and state.stopped
    assert len(state.published['/robot/camera/color']) == 3
    assert len(state.published['/robot/camera/depth']) == 3
    assert len(raw_calls) == 2  # VO's stop only gates VO, not snapshots or media.
    if not snapshot_fault:
        assert seen == [(True, True), (False, True), (True, False)]
